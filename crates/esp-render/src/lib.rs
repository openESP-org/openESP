use anyhow::{Context, Result};
use bytemuck::{Pod, Zeroable};
use esp_assets::{Aircraft, Model, texture};
use glam::{DVec3, Mat4, Vec3};
use std::{collections::HashMap, path::Path, sync::Arc};
use wgpu::util::DeviceExt;
use winit::window::Window;

pub mod camera;
pub use camera::Camera;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
    uv: [f32; 2],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct DrawUniform {
    model: [[f32; 4]; 4],
    normal: [[f32; 4]; 4],
    diffuse: [f32; 4],
    settings: [f32; 4],
}

struct Draw {
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    count: u32,
    transform: Mat4,
    uniform: DrawUniform,
    buffer: wgpu::Buffer,
    bind: wgpu::BindGroup,
    blended: bool,
    centre: Vec3,
}

struct DrawData<'a> {
    vertices: &'a [Vertex],
    indices: &'a [u16],
    transform: Mat4,
    uniform: DrawUniform,
    blended: bool,
}

pub struct Renderer {
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    depth: wgpu::TextureView,
    opaque: wgpu::RenderPipeline,
    blended: wgpu::RenderPipeline,
    camera_buffer: wgpu::Buffer,
    camera_bind: wgpu::BindGroup,
    exterior: Vec<Draw>,
    interior: Vec<Draw>,
    ground: Draw,
    pub centre: Vec3,
    pub radius: f32,
    pub aircraft_origin: DVec3,
    pub cockpit_eye: Option<DVec3>,
}

impl Renderer {
    pub async fn new(window: Arc<Window>, aircraft: &Aircraft) -> Result<Self> {
        let instance = wgpu::Instance::default();
        let surface = instance.create_surface(window.clone())?;
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                compatible_surface: Some(&surface),
                power_preference: wgpu::PowerPreference::HighPerformance,
                ..Default::default()
            })
            .await?;
        eprintln!(
            "GPU: {} ({:?})",
            adapter.get_info().name,
            adapter.get_info().backend
        );
        let compressed = adapter
            .features()
            .contains(wgpu::Features::TEXTURE_COMPRESSION_BC);
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                required_features: if compressed {
                    wgpu::Features::TEXTURE_COMPRESSION_BC
                } else {
                    wgpu::Features::empty()
                },
                ..Default::default()
            })
            .await?;
        let size = window.inner_size();
        let mut config = surface
            .get_default_config(&adapter, size.width.max(1), size.height.max(1))
            .context("surface has no supported configuration")?;
        if surface
            .get_capabilities(&adapter)
            .usages
            .contains(wgpu::TextureUsages::COPY_SRC)
        {
            config.usage |= wgpu::TextureUsages::COPY_SRC;
        }
        surface.configure(&device, &config);
        let camera_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("camera"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let draw_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("draw"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[Some(&camera_layout), Some(&draw_layout)],
            immediate_size: 0,
        });
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("aircraft"),
            source: wgpu::ShaderSource::Wgsl(include_str!("aircraft.wgsl").into()),
        });
        let pipeline = |blend: bool| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some(if blend { "transparent" } else { "opaque" }), layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &shader, entry_point: Some("vs_main"), compilation_options: Default::default(),
                buffers: &[Some(wgpu::VertexBufferLayout { array_stride: size_of::<Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x2] })],
            },
            primitive: wgpu::PrimitiveState { front_face: wgpu::FrontFace::Cw, cull_mode: None, ..Default::default() },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float, depth_write_enabled: Some(!blend),
                depth_compare: Some(wgpu::CompareFunction::GreaterEqual), stencil: Default::default(), bias: Default::default(),
            }),
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &shader, entry_point: Some("fs_main"), compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState { format: config.format,
                    blend: blend.then_some(wgpu::BlendState::ALPHA_BLENDING), write_mask: wgpu::ColorWrites::ALL })],
            }),
            multiview_mask: None, cache: None,
        })
        };
        let opaque = pipeline(false);
        let blended = pipeline(true);
        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("camera"),
            contents: bytemuck::cast_slice(&Mat4::IDENTITY.to_cols_array()),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let camera_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &camera_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
        });
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::Repeat,
            address_mode_v: wgpu::AddressMode::Repeat,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::MipmapFilterMode::Linear,
            ..Default::default()
        });
        let mut textures: HashMap<String, (wgpu::TextureView, bool)> = HashMap::new();
        let mut upload = |model: &Model| -> Result<Vec<Draw>> {
            let transforms = model.transforms()?;
            let lod = model.lods.first().context("model has no LODs")?;
            let mut draws = Vec::new();
            for mesh in &lod.meshes {
                let material = &model.materials[mesh.material];
                let mut flip_v = false;
                let view = if let Some(name) = &material.diffuse_texture {
                    if let Some((view, flip)) = textures.get(name) {
                        flip_v = *flip;
                        view.clone()
                    } else {
                        let texture = match aircraft.texture_path(name) {
                            Ok(Some(path)) => texture::Texture::load(&path, compressed)?,
                            Ok(None) => texture::Texture::rgba(1, 1, vec![0, 0, 0, 255], false),
                            Err(error) => {
                                eprintln!("{error}");
                                texture::Texture::rgba(1, 1, vec![255, 0, 255, 255], false)
                            }
                        };
                        flip_v = texture.flip_v;
                        let view = upload_texture(&device, &queue, &texture)?;
                        textures.insert(name.clone(), (view.clone(), flip_v));
                        view
                    }
                } else {
                    upload_texture(
                        &device,
                        &queue,
                        &texture::Texture::rgba(1, 1, vec![255; 4], false),
                    )?
                };
                let vertices: Vec<_> = mesh
                    .vertices
                    .iter()
                    .map(|v| Vertex {
                        position: v.position,
                        normal: v.normal,
                        uv: v.uv,
                    })
                    .collect();
                let uniform = DrawUniform {
                    model: Mat4::IDENTITY.to_cols_array_2d(),
                    normal: Mat4::IDENTITY.to_cols_array_2d(),
                    diffuse: material.source.diffuse,
                    settings: [
                        if flip_v { 1.0 } else { 0.0 },
                        0.0,
                        if material.source.flags & 0x00400000 != 0 {
                            material.source.alpha_test_threshold / 255.0
                        } else {
                            0.0
                        },
                        0.0,
                    ],
                };
                draws.push(make_draw(
                    &device,
                    &draw_layout,
                    &sampler,
                    &view,
                    DrawData {
                        vertices: &vertices,
                        indices: &mesh.indices,
                        transform: transforms[mesh.node],
                        uniform,
                        blended: material.source.source_blend == 5
                            || material.source.destination_blend != 1,
                    },
                ));
            }
            Ok(draws)
        };
        let exterior = upload(&aircraft.exterior)?;
        let interior = aircraft
            .interior
            .as_ref()
            .map(upload)
            .transpose()?
            .unwrap_or_default();
        let mut min = Vec3::splat(f32::INFINITY);
        let mut max = Vec3::splat(f32::NEG_INFINITY);
        let transforms = aircraft.exterior.transforms()?;
        for mesh in &aircraft.exterior.lods[0].meshes {
            for vertex in &mesh.vertices {
                let p = transforms[mesh.node].transform_point3(Vec3::from_array(vertex.position));
                min = min.min(p);
                max = max.max(p);
            }
        }
        anyhow::ensure!(
            min.is_finite() && max.is_finite(),
            "aircraft has no finite geometry bounds"
        );
        let aircraft_origin = DVec3::new(0.0, -f64::from(min.y), 0.0);
        let centre = (min + max) * 0.5 + aircraft_origin.as_vec3();
        let radius = ((max - min).length() * 0.5).max(1.0);
        let extent = (radius * 4.0).max(50.0);
        let vertices = [
            (-extent, -extent),
            (-extent, extent),
            (extent, extent),
            (extent, -extent),
        ]
        .map(|(x, z)| Vertex {
            position: [x, 0.0, z],
            normal: [0.0, 1.0, 0.0],
            uv: [x, z],
        });
        let white = upload_texture(
            &device,
            &queue,
            &texture::Texture::rgba(1, 1, vec![255; 4], false),
        )?;
        let ground = make_draw(
            &device,
            &draw_layout,
            &sampler,
            &white,
            DrawData {
                vertices: &vertices,
                indices: &[0, 1, 2, 0, 2, 3],
                transform: Mat4::IDENTITY,
                uniform: DrawUniform {
                    model: Mat4::IDENTITY.to_cols_array_2d(),
                    normal: Mat4::IDENTITY.to_cols_array_2d(),
                    diffuse: [0.32, 0.32, 0.32, 1.0],
                    settings: [0.0, 1.0, 0.0, 0.0],
                },
                blended: false,
            },
        );
        let cockpit_eye = aircraft
            .eyepoint
            .map(|eye| aircraft_origin + eye.as_dvec3());
        let depth = depth_target(&device, &config);
        Ok(Self {
            surface,
            device,
            queue,
            config,
            depth,
            opaque,
            blended,
            camera_buffer,
            camera_bind,
            exterior,
            interior,
            ground,
            centre,
            radius,
            aircraft_origin,
            cockpit_eye,
        })
    }

    pub fn resize(&mut self, width: u32, height: u32) {
        if width == 0 || height == 0 {
            return;
        }
        self.config.width = width;
        self.config.height = height;
        self.surface.configure(&self.device, &self.config);
        self.depth = depth_target(&self.device, &self.config);
    }

    pub fn render(
        &mut self,
        camera: &Camera,
        cockpit: bool,
        screenshot: Option<&Path>,
    ) -> Result<bool> {
        let frame = match self.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(frame)
            | wgpu::CurrentSurfaceTexture::Suboptimal(frame) => frame,
            wgpu::CurrentSurfaceTexture::Outdated => {
                self.surface.configure(&self.device, &self.config);
                return Ok(false);
            }
            wgpu::CurrentSurfaceTexture::Timeout | wgpu::CurrentSurfaceTexture::Occluded => {
                return Ok(false);
            }
            error => anyhow::bail!("surface acquisition failed: {error:?}"),
        };
        let matrix = camera.view_projection(self.config.width as f32 / self.config.height as f32);
        self.queue.write_buffer(
            &self.camera_buffer,
            0,
            bytemuck::cast_slice(&matrix.to_cols_array()),
        );
        let draws = if cockpit {
            &mut self.interior
        } else {
            &mut self.exterior
        };
        for draw in draws.iter_mut().chain(std::iter::once(&mut self.ground)) {
            let origin = if draw.uniform.settings[1] > 0.0 {
                DVec3::ZERO
            } else {
                self.aircraft_origin
            };
            let transform =
                Mat4::from_translation((origin - camera.position).as_vec3()) * draw.transform;
            draw.uniform.model = transform.to_cols_array_2d();
            draw.uniform.normal = draw.transform.inverse().transpose().to_cols_array_2d();
            self.queue
                .write_buffer(&draw.buffer, 0, bytemuck::bytes_of(&draw.uniform));
        }
        let mut transparent: Vec<_> = draws.iter().filter(|d| d.blended).collect();
        transparent.sort_by(|a, b| {
            let distance = |d: &Draw| {
                (self.aircraft_origin + d.transform.transform_point3(d.centre).as_dvec3()
                    - camera.position)
                    .length_squared()
            };
            distance(b).total_cmp(&distance(a))
        });
        let view = frame.texture.create_view(&Default::default());
        let mut encoder = self.device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("aircraft scene"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    depth_slice: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.56,
                            g: 0.56,
                            b: 0.56,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(0.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_bind_group(0, &self.camera_bind, &[]);
            pass.set_pipeline(&self.opaque);
            let draw_one = |pass: &mut wgpu::RenderPass<'_>, draw: &Draw| {
                pass.set_bind_group(1, &draw.bind, &[]);
                pass.set_vertex_buffer(0, draw.vertices.slice(..));
                pass.set_index_buffer(draw.indices.slice(..), wgpu::IndexFormat::Uint16);
                pass.draw_indexed(0..draw.count, 0, 0..1);
            };
            draw_one(&mut pass, &self.ground);
            for draw in draws.iter().filter(|d| !d.blended) {
                draw_one(&mut pass, draw);
            }
            pass.set_pipeline(&self.blended);
            for draw in transparent {
                draw_one(&mut pass, draw);
            }
        }
        let capture = if screenshot.is_some() {
            anyhow::ensure!(
                self.config.usage.contains(wgpu::TextureUsages::COPY_SRC),
                "surface does not support screenshots"
            );
            let stride =
                (self.config.width * 4).next_multiple_of(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT);
            let buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("screenshot"),
                size: u64::from(stride) * u64::from(self.config.height),
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });
            encoder.copy_texture_to_buffer(
                wgpu::TexelCopyTextureInfo {
                    texture: &frame.texture,
                    mip_level: 0,
                    origin: wgpu::Origin3d::ZERO,
                    aspect: wgpu::TextureAspect::All,
                },
                wgpu::TexelCopyBufferInfo {
                    buffer: &buffer,
                    layout: wgpu::TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(stride),
                        rows_per_image: None,
                    },
                },
                wgpu::Extent3d {
                    width: self.config.width,
                    height: self.config.height,
                    depth_or_array_layers: 1,
                },
            );
            Some((buffer, stride))
        } else {
            None
        };
        self.queue.submit([encoder.finish()]);
        if let Some((buffer, stride)) = capture {
            let (sender, receiver) = std::sync::mpsc::sync_channel(1);
            buffer
                .slice(..)
                .map_async(wgpu::MapMode::Read, move |result| {
                    let _ = sender.send(result);
                });
            self.device.poll(wgpu::PollType::wait_indefinitely())?;
            receiver.recv()??;
            let mapped = buffer.slice(..).get_mapped_range()?;
            let mut pixels: Vec<_> = mapped
                .chunks_exact(stride as usize)
                .flat_map(|row| row[..self.config.width as usize * 4].iter().copied())
                .collect();
            if matches!(
                self.config.format,
                wgpu::TextureFormat::Bgra8Unorm | wgpu::TextureFormat::Bgra8UnormSrgb
            ) {
                for pixel in pixels.as_chunks_mut::<4>().0 {
                    pixel.swap(0, 2);
                }
            } else {
                anyhow::ensure!(
                    matches!(
                        self.config.format,
                        wgpu::TextureFormat::Rgba8Unorm | wgpu::TextureFormat::Rgba8UnormSrgb
                    ),
                    "unsupported screenshot surface format"
                );
            }
            if let Some(path) = screenshot {
                image::save_buffer(
                    path,
                    &pixels,
                    self.config.width,
                    self.config.height,
                    image::ColorType::Rgba8,
                )?;
            }
        }
        self.queue.present(frame);
        Ok(true)
    }
}

fn depth_target(device: &wgpu::Device, config: &wgpu::SurfaceConfiguration) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some("depth"),
            size: wgpu::Extent3d {
                width: config.width,
                height: config.height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        })
        .create_view(&Default::default())
}

fn upload_texture(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    image: &texture::Texture,
) -> Result<wgpu::TextureView> {
    let format = match image.format {
        texture::Format::Rgba8 => wgpu::TextureFormat::Rgba8UnormSrgb,
        texture::Format::Bc1 => wgpu::TextureFormat::Bc1RgbaUnormSrgb,
        texture::Format::Bc2 => wgpu::TextureFormat::Bc2RgbaUnormSrgb,
        texture::Format::Bc3 => wgpu::TextureFormat::Bc3RgbaUnormSrgb,
    };
    let base = image.mips.first().context("texture has no mips")?;
    anyhow::ensure!(
        base.width <= device.limits().max_texture_dimension_2d
            && base.height <= device.limits().max_texture_dimension_2d,
        "texture exceeds GPU dimension limit"
    );
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: None,
        size: wgpu::Extent3d {
            width: base.width,
            height: base.height,
            depth_or_array_layers: 1,
        },
        mip_level_count: image.mips.len() as u32,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let (bw, bh) = format.block_dimensions();
    let block = format
        .block_copy_size(None)
        .context("texture has no copy layout")?;
    for (level, mip) in image.mips.iter().enumerate() {
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &texture,
                mip_level: level as u32,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &mip.data,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(mip.width.div_ceil(bw) * block),
                rows_per_image: None,
            },
            wgpu::Extent3d {
                width: mip.width.next_multiple_of(bw),
                height: mip.height.next_multiple_of(bh),
                depth_or_array_layers: 1,
            },
        );
    }
    Ok(texture.create_view(&Default::default()))
}

fn make_draw(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    sampler: &wgpu::Sampler,
    texture: &wgpu::TextureView,
    data: DrawData<'_>,
) -> Draw {
    let DrawData {
        vertices,
        indices,
        transform,
        uniform,
        blended,
    } = data;
    let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("draw"),
        contents: bytemuck::bytes_of(&uniform),
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
    });
    let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: None,
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(texture),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::Sampler(sampler),
            },
        ],
    });
    let centre = vertices
        .iter()
        .map(|v| Vec3::from_array(v.position))
        .sum::<Vec3>()
        / vertices.len().max(1) as f32;
    Draw {
        vertices: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("vertices"),
            contents: bytemuck::cast_slice(vertices),
            usage: wgpu::BufferUsages::VERTEX,
        }),
        indices: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("indices"),
            contents: bytemuck::cast_slice(indices),
            usage: wgpu::BufferUsages::INDEX,
        }),
        count: indices.len() as u32,
        transform,
        uniform,
        buffer,
        bind,
        blended,
        centre,
    }
}

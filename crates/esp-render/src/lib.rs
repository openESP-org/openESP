//! esp-render owns the wgpu renderer.
use anyhow::{Context, Result};
use bytemuck::{Pod, Zeroable};
use esp_assets::{Aircraft, Model, texture};
use glam::{DVec3, Mat4, Vec3};
use std::{collections::HashMap, path::Path, sync::Arc};
use wgpu::util::DeviceExt;
use winit::window::Window;

pub mod camera;
mod draw;
mod ground;
mod material;
mod sky;
mod tangent;
pub use camera::Camera;
use draw::make_draw;
use material::{PipelineKey, texture_format};
pub use sky::{CloudLayer, Weather};

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
    uv: [f32; 2],
    tangent: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct DrawUniform {
    model: [[f32; 4]; 4],
    normal: [[f32; 4]; 4],
    diffuse: [f32; 4],
    settings: [f32; 4],
    pbr: [f32; 4],
    normal_settings: [f32; 4],
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
    node: Option<usize>,
    visible: bool,
    pipeline: usize,
}

struct DrawData<'a> {
    vertices: &'a [Vertex],
    indices: &'a [u32],
    transform: Mat4,
    uniform: DrawUniform,
    blended: bool,
    node: Option<usize>,
    pipeline: usize,
    metallic_map: Option<&'a wgpu::TextureView>,
    normal_map: Option<&'a wgpu::TextureView>,
    emissive_map: Option<&'a wgpu::TextureView>,
}

pub struct Renderer {
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    depth: wgpu::TextureView,
    multisampled_color: Option<wgpu::TextureView>,
    sample_count: u32,
    pipelines: Vec<wgpu::RenderPipeline>,
    sky: wgpu::RenderPipeline,
    camera_buffer: wgpu::Buffer,
    camera_bind: wgpu::BindGroup,
    exterior: Vec<Draw>,
    interior: Vec<Draw>,
    panels: HashMap<String, wgpu::Texture>,
    ground: Draw,
    terrain_draws: Vec<Draw>,
    draw_layout: wgpu::BindGroupLayout,
    terrain_sampler: wgpu::Sampler,
    terrain_blend_pipeline: usize,
    ground_tracks_camera: bool,
    photos: Vec<Draw>,
    runway: Vec<Draw>,
    objects: Vec<Draw>,
    pub centre: Vec3,
    pub radius: f32,
    pub aircraft_origin: DVec3,
    pub aircraft_rotation: Mat4,
    pub cockpit_eye: Option<DVec3>,
}

#[derive(Clone, Copy, Debug)]
pub struct Runway {
    pub threshold: DVec3,
    /// Clockwise from true north, in radians.
    pub heading: f64,
    pub length: f32,
    pub width: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Taxiway {
    pub start: DVec3,
    pub end: DVec3,
    pub width: f32,
    pub centerline: bool,
}

#[derive(Clone, Debug)]
pub struct Apron {
    pub vertices: Vec<DVec3>,
    pub indices: Vec<u16>,
}

pub struct SceneryObject {
    pub model: Arc<Model>,
    pub transform: Mat4,
    pub textures: std::path::PathBuf,
}

/// A geographically projected photo tile draped over terrain.
pub struct PhotoTile {
    pub vertices: Vec<DVec3>,
    pub uvs: Vec<[f32; 2]>,
    pub indices: Vec<u32>,
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
}

pub struct TerrainMesh {
    pub vertices: Vec<DVec3>,
    pub normals: Vec<DVec3>,
    pub land_classes: Vec<Option<u8>>,
    pub texture_sets: Vec<Option<u32>>,
    pub textures: HashMap<u32, texture::Texture>,
    pub indices: Vec<u32>,
}

pub struct Scene<'a> {
    pub runways: &'a [Runway],
    pub taxiways: &'a [Taxiway],
    pub aprons: &'a [Apron],
    pub objects: &'a [SceneryObject],
    pub photos: &'a [PhotoTile],
    pub terrain: Option<&'a TerrainMesh>,
}

fn apron_mesh(aprons: &[Apron]) -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for apron in aprons {
        let base = vertices.len() as u32;
        vertices.extend(apron.vertices.iter().map(|point| Vertex {
            position: [point.x as f32, 0.01, point.z as f32],
            normal: [0.0, 1.0, 0.0],
            uv: [0.0, 0.0],
            tangent: [0.0; 4],
        }));
        for triangle in apron.indices.as_chunks::<3>().0 {
            let [a, b, c] = triangle.map(|index| apron.vertices[usize::from(index)]);
            let winding = (b.z - a.z) * (c.x - a.x) - (b.x - a.x) * (c.z - a.z);
            if winding > 0.0 {
                indices.extend(triangle.map(|i| base + u32::from(i)));
            } else if winding < 0.0 {
                indices.extend([
                    base + u32::from(triangle[0]),
                    base + u32::from(triangle[2]),
                    base + u32::from(triangle[1]),
                ]);
            }
        }
    }
    (vertices, indices)
}

fn taxiway_mesh(taxiways: &[Taxiway], markings: bool) -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for taxiway in taxiways {
        if markings && !taxiway.centerline {
            continue;
        }
        let start = taxiway.start.as_vec3();
        let end = taxiway.end.as_vec3();
        let Some(direction) = (end - start).try_normalize() else {
            continue;
        };
        let side = Vec3::new(direction.z, 0.0, -direction.x)
            * if markings { 0.08 } else { taxiway.width * 0.5 };
        let height = if markings { 0.025 } else { 0.02 };
        let base = vertices.len() as u32;
        vertices.extend(
            [start - side, end - side, end + side, start + side].map(|point| Vertex {
                position: [point.x, height, point.z],
                normal: [0.0, 1.0, 0.0],
                uv: [0.0, 0.0],
                tangent: [0.0; 4],
            }),
        );
        indices.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    (vertices, indices)
}

fn runway_mesh(markings: bool, length: f32, width: f32) -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut rectangle = |left: f32, right: f32, start: f32, end: f32, height: f32| {
        let base = vertices.len() as u32;
        vertices.extend(
            [(left, start), (left, end), (right, end), (right, start)].map(|(x, z)| Vertex {
                position: [x, height, z],
                normal: [0.0, 1.0, 0.0],
                uv: [0.0, 0.0],
                tangent: [0.0; 4],
            }),
        );
        indices.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    };
    if markings {
        let edge = (width * 0.5 - 1.0).max(0.0);
        rectangle(-edge - 0.5, -edge, 0.0, length, 0.06);
        rectangle(edge, edge + 0.5, 0.0, length, 0.06);
        for step in 0..((length - 150.0).max(0.0) / 60.0).floor() as u32 {
            let start = 150.0 + step as f32 * 60.0;
            rectangle(-0.45, 0.45, start, (start + 30.0).min(length), 0.06);
        }
        for side in [-1.0, 1.0] {
            for stripe in 0..4 {
                let center = side * (width * (0.08 + stripe as f32 * 0.09));
                rectangle(
                    center - 0.75,
                    center + 0.75,
                    6.0_f32.min(length),
                    36.0_f32.min(length),
                    0.06,
                );
            }
        }
    } else {
        rectangle(-width * 0.5, width * 0.5, 0.0, length, 0.03);
    }
    (vertices, indices)
}

impl Renderer {
    pub async fn new(
        window: Arc<Window>,
        aircraft: &Aircraft,
        scene: Scene<'_>,
        requested_samples: u32,
    ) -> Result<Self> {
        let Scene {
            runways,
            taxiways,
            aprons,
            objects,
            photos,
            terrain,
        } = scene;
        // Avoid initializing secondary GL drivers when using native GPU APIs.
        // Some Windows GL drivers can fail even when DX12 was selected for
        // rendering.
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::PRIMARY,
            ..wgpu::InstanceDescriptor::new_without_display_handle()
        });
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
        let color_flags = adapter.get_texture_format_features(config.format).flags;
        let depth_flags = adapter
            .get_texture_format_features(wgpu::TextureFormat::Depth32Float)
            .flags;
        let sample_count = [8, 4, 2, 1]
            .into_iter()
            .find(|&count| {
                count <= requested_samples
                    && color_flags.sample_count_supported(count)
                    && depth_flags.sample_count_supported(count)
                    && (count == 1
                        || color_flags
                            .contains(wgpu::TextureFormatFeatureFlags::MULTISAMPLE_RESOLVE))
            })
            .unwrap_or(1);
        eprintln!("anti-aliasing: {sample_count}x MSAA");
        config.present_mode = wgpu::PresentMode::AutoVsync;
        surface.configure(&device, &config);
        let camera_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("camera"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
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
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 4,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 5,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
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
        let pipeline = |key: PipelineKey| -> Result<wgpu::RenderPipeline> {
            Ok(device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("aircraft material"), layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &shader, entry_point: Some("vs_main"), compilation_options: Default::default(),
                buffers: &[Some(wgpu::VertexBufferLayout { array_stride: size_of::<Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x2, 3 => Float32x4] })],
            },
            primitive: wgpu::PrimitiveState { front_face: wgpu::FrontFace::Cw, cull_mode: if key.double_sided {None}else{Some(wgpu::Face::Back)}, ..Default::default() },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float, depth_write_enabled: Some(key.depth_write),
                depth_compare: Some(wgpu::CompareFunction::GreaterEqual), stencil: Default::default(), bias: Default::default(),
            }),
            multisample: wgpu::MultisampleState { count: sample_count, ..Default::default() },
            fragment: Some(wgpu::FragmentState {
                module: &shader, entry_point: Some("fs_main"), compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState { format: config.format,
                    blend: key.blend()?, write_mask: wgpu::ColorWrites::ALL })],
            }),
            multiview_mask: None, cache: None,
        }))
        };
        let ground_key = PipelineKey {
            source: 2,
            destination: 1,
            depth_write: true,
            double_sided: true,
        };
        let mut pipelines = vec![pipeline(ground_key)?];
        let mut pipeline_indices = HashMap::from([(ground_key, 0)]);
        let sky = sky::pipeline(&device, &camera_layout, config.format, sample_count);
        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("camera"),
            contents: bytemuck::bytes_of(&sky::CameraUniform {
                view_projection: Mat4::IDENTITY.to_cols_array_2d(),
                inverse_view_projection: Mat4::IDENTITY.to_cols_array_2d(),
                sun: [0.0, 1.0, 0.0, 0.0],
                lighting: [0.0; 4],
                clouds: [[0.0; 4]; 3],
                world: [0.0; 4],
            }),
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
        let mut textures: HashMap<String, (wgpu::TextureView, bool, bool)> = HashMap::new();
        let mut panels = HashMap::new();
        let mut upload = |model: &Model,
                          texture_root: Option<&std::path::Path>|
         -> Result<Vec<Draw>> {
            let transforms = model.transforms()?;
            let lod = model.lods.first().context("model has no LODs")?;
            let mut draws = Vec::new();
            for mesh in &lod.meshes {
                let material = &model.materials[mesh.material];
                if material.source.flags & 0x00400000 != 0 {
                    anyhow::ensure!(
                        (1..=8).contains(&material.source.alpha_test_function),
                        "unsupported FSX alpha test function {}",
                        material.source.alpha_test_function
                    );
                }
                let key = PipelineKey::new(
                    material.source.source_blend,
                    material.source.destination_blend,
                    material.source.flags,
                    material.source.flags2,
                );
                let pipeline_index = if let Some(&index) = pipeline_indices.get(&key) {
                    index
                } else {
                    let index = pipelines.len();
                    pipelines.push(pipeline(key)?);
                    pipeline_indices.insert(key, index);
                    index
                };
                let mut flip_v = false;
                let view = if let Some(name) = &material.diffuse_texture {
                    let cache_key = format!(
                        "{}|{name}",
                        texture_root.map_or_else(String::new, |p| p.to_string_lossy().into_owned())
                    );
                    if let Some((view, flip, _)) = textures.get(&cache_key) {
                        flip_v = *flip;
                        view.clone()
                    } else {
                        let mut is_panel = false;
                        let resolved = if let Some(root) = texture_root {
                            esp_assets::resolve_texture(root, name).map(Some)
                        } else {
                            aircraft.texture_path(name)
                        };
                        let texture = match resolved {
                            Ok(Some(path)) => texture::Texture::load(&path, compressed)?,
                            Ok(None) => {
                                if let Some(panel) =
                                    aircraft.panel_images.get(&name.to_ascii_uppercase())
                                {
                                    is_panel = true;
                                    eprintln!(
                                        "{name}: partial gauge rendering ({} missing gauges, {} omitted elements)",
                                        panel.omitted_gauges, panel.omitted_elements
                                    );
                                    texture::Texture::rgba(
                                        panel.image.width(),
                                        panel.image.height(),
                                        panel.image.as_raw().clone(),
                                        false,
                                    )
                                } else {
                                    eprintln!("missing cockpit panel texture {name}");
                                    texture::Texture::rgba(1, 1, vec![0, 0, 0, 255], false)
                                }
                            }
                            Err(error) => {
                                eprintln!("{error}");
                                texture::Texture::rgba(1, 1, vec![255, 0, 255, 255], false)
                            }
                        };
                        flip_v = texture.flip_v;
                        let uploaded = upload_texture(&device, &queue, &texture)?;
                        let view = uploaded.create_view(&Default::default());
                        if is_panel {
                            panels.insert(name.to_ascii_uppercase(), uploaded);
                        }
                        textures.insert(cache_key, (view.clone(), flip_v, false));
                        view
                    }
                } else {
                    upload_texture(
                        &device,
                        &queue,
                        &texture::Texture::rgba(1, 1, vec![255; 4], false),
                    )?
                    .create_view(&Default::default())
                };
                let metallic = material::linear_texture(
                    &device,
                    &queue,
                    aircraft,
                    texture_root,
                    compressed,
                    &mut textures,
                    material.metallic_texture.as_deref(),
                )?;
                let normal_map = material::linear_texture(
                    &device,
                    &queue,
                    aircraft,
                    texture_root,
                    compressed,
                    &mut textures,
                    material.normal_texture.as_deref(),
                )?;
                let emissive = material::texture(
                    &device,
                    &queue,
                    aircraft,
                    texture_root,
                    compressed,
                    &mut textures,
                    material.emissive_texture.as_deref(),
                    true,
                )?;
                let frames = normal_map.as_ref().map(|_| tangent::frames(mesh));
                let vertices: Vec<_> = mesh
                    .vertices
                    .iter()
                    .enumerate()
                    .map(|(i, v)| Vertex {
                        position: v.position,
                        normal: v.normal,
                        uv: v.uv,
                        tangent: frames.as_ref().map_or([0.0; 4], |values| values[i]),
                    })
                    .collect();
                let uniform = DrawUniform {
                    model: Mat4::IDENTITY.to_cols_array_2d(),
                    normal: Mat4::IDENTITY.to_cols_array_2d(),
                    diffuse: if material.diffuse_texture.is_some() && material.pbr.is_none() {
                        [1.0, 1.0, 1.0, material.source.diffuse[3]]
                    } else {
                        material.source.diffuse
                    },
                    settings: [
                        if flip_v { 1.0 } else { 0.0 },
                        0.0,
                        if material.source.flags & 0x00400000 != 0 {
                            material.source.alpha_test_threshold / 255.0
                        } else {
                            0.0
                        },
                        if material.source.flags & 0x00400000 != 0 {
                            material.source.alpha_test_function as f32
                        } else {
                            8.0
                        },
                    ],
                    pbr: material.pbr.as_ref().map_or([0.0; 4], |pbr| {
                        [
                            pbr.metallic.clamp(0.0, 1.0),
                            1.0 - pbr.smoothness.clamp(0.0, 1.0),
                            1.0,
                            if let Some((_, metallic_flip, _)) = &metallic {
                                if *metallic_flip == flip_v { 1.0 } else { 2.0 }
                            } else {
                                0.0
                            },
                        ]
                    }),
                    normal_settings: {
                        let mut settings =
                            normal_map
                                .as_ref()
                                .map_or([0.0; 4], |(_, normal_flip, alpha)| {
                                    let scale = if material.pbr.is_none()
                                        && material.source.bump_scale.is_finite()
                                        && material.source.bump_scale > 0.0
                                    {
                                        material.source.bump_scale
                                    } else {
                                        1.0
                                    };
                                    [
                                        1.0,
                                        if *normal_flip == flip_v {
                                            -scale
                                        } else {
                                            scale
                                        },
                                        if *alpha { 1.0 } else { 0.0 },
                                        0.0,
                                    ]
                                });
                        settings[3] = emissive.as_ref().map_or(0.0, |(_, emissive_flip, _)| {
                            material::emissive_mode(
                                material.source.flags,
                                material.source.flags2,
                                material.pbr.is_some(),
                                *emissive_flip != flip_v,
                            )
                        });
                        settings
                    },
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
                        blended: material.source.source_blend != 2
                            || material.source.destination_blend != 1,
                        node: Some(mesh.node),
                        pipeline: pipeline_index,
                        metallic_map: metallic.as_ref().map(|(view, _, _)| view),
                        normal_map: normal_map.as_ref().map(|(view, _, _)| view),
                        emissive_map: emissive.as_ref().map(|(view, _, _)| view),
                    },
                ));
            }
            Ok(draws)
        };
        let exterior = upload(&aircraft.exterior, None)?;
        let interior = if let Some(interior) = &aircraft.interior {
            upload(interior, None)?
        } else {
            Vec::new()
        };
        let mut scenery_draws = Vec::new();
        for object in objects {
            anyhow::ensure!(object.transform.is_finite(), "invalid scenery transform");
            for mut draw in upload(&object.model, Some(&object.textures))? {
                draw.transform = object.transform * draw.transform;
                draw.uniform.settings[1] = 2.0;
                draw.node = None;
                scenery_draws.push(draw);
            }
        }
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
        let centre = (min + max) * 0.5;
        let radius = ((max - min).length() * 0.5).max(1.0);
        let (vertices, indices) = ground::mesh(terrain)?;
        let white = upload_texture(
            &device,
            &queue,
            &texture::Texture::rgba(1, 1, vec![255; 4], false),
        )?
        .create_view(&Default::default());
        let ground = make_draw(
            &device,
            &draw_layout,
            &sampler,
            &white,
            DrawData {
                vertices: &vertices,
                indices: &indices,
                transform: Mat4::IDENTITY,
                uniform: DrawUniform {
                    model: Mat4::IDENTITY.to_cols_array_2d(),
                    normal: Mat4::IDENTITY.to_cols_array_2d(),
                    diffuse: [0.32, 0.32, 0.32, 1.0],
                    settings: [0.0, if terrain.is_some() { 3.0 } else { 1.0 }, 0.0, 0.0],
                    pbr: [0.0; 4],
                    normal_settings: [0.0; 4],
                },
                blended: false,
                node: None,
                pipeline: 0,
                metallic_map: None,
                normal_map: None,
                emissive_map: None,
            },
        );
        let photo_key = PipelineKey {
            source: 5,
            destination: 6,
            depth_write: false,
            double_sided: true,
        };
        let photo_pipeline = if let Some(&index) = pipeline_indices.get(&photo_key) {
            index
        } else {
            let index = pipelines.len();
            pipelines.push(pipeline(photo_key)?);
            index
        };
        let terrain_draws = terrain.map_or(Ok(Vec::new()), |terrain| {
            ground::textured_draws(
                &device,
                &queue,
                &draw_layout,
                &sampler,
                terrain,
                photo_pipeline,
            )
        })?;
        let photo_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        let mut photo_draws = Vec::with_capacity(photos.len());
        for photo in photos {
            anyhow::ensure!(
                photo.width > 0
                    && photo.height > 0
                    && photo.vertices.len() == photo.uvs.len()
                    && photo.vertices.iter().all(|vertex| vertex.is_finite())
                    && photo.uvs.iter().flatten().all(|value| value.is_finite())
                    && !photo.indices.is_empty()
                    && photo.indices.len().is_multiple_of(3)
                    && photo
                        .indices
                        .iter()
                        .all(|index| (*index as usize) < photo.vertices.len())
                    && (photo.width as usize)
                        .checked_mul(photo.height as usize)
                        .and_then(|n| n.checked_mul(4))
                        == Some(photo.rgba.len()),
                "invalid photo tile"
            );
            let vertices: Vec<_> = photo
                .vertices
                .iter()
                .zip(&photo.uvs)
                .map(|(position, uv)| Vertex {
                    position: [
                        position.x as f32,
                        position.y as f32 + 0.1,
                        position.z as f32,
                    ],
                    normal: [0.0, 1.0, 0.0],
                    uv: *uv,
                    tangent: [0.0; 4],
                })
                .collect();
            let image =
                texture::Texture::rgba(photo.width, photo.height, photo.rgba.clone(), false);
            let view = upload_texture(&device, &queue, &image)?.create_view(&Default::default());
            photo_draws.push(make_draw(
                &device,
                &draw_layout,
                &photo_sampler,
                &view,
                DrawData {
                    vertices: &vertices,
                    indices: &photo.indices,
                    transform: Mat4::IDENTITY,
                    uniform: DrawUniform {
                        model: Mat4::IDENTITY.to_cols_array_2d(),
                        normal: Mat4::IDENTITY.to_cols_array_2d(),
                        diffuse: [1.0; 4],
                        settings: [0.0, 2.0, 0.0, 0.0],
                        pbr: [0.0; 4],
                        normal_settings: [0.0; 4],
                    },
                    blended: true,
                    node: None,
                    pipeline: photo_pipeline,
                    metallic_map: None,
                    normal_map: None,
                    emissive_map: None,
                },
            ));
        }
        let mut runway_draws = Vec::new();
        anyhow::ensure!(
            aprons
                .iter()
                .all(|apron| apron.vertices.iter().all(|p| p.is_finite())
                    && apron.indices.len().is_multiple_of(3)
                    && apron
                        .indices
                        .iter()
                        .all(|i| usize::from(*i) < apron.vertices.len())),
            "invalid apron geometry"
        );
        let (vertices, indices) = apron_mesh(aprons);
        if !indices.is_empty() {
            runway_draws.push(make_draw(
                &device,
                &draw_layout,
                &sampler,
                &white,
                DrawData {
                    vertices: &vertices,
                    indices: &indices,
                    transform: Mat4::IDENTITY,
                    uniform: DrawUniform {
                        model: Mat4::IDENTITY.to_cols_array_2d(),
                        normal: Mat4::IDENTITY.to_cols_array_2d(),
                        diffuse: [0.23, 0.23, 0.23, 1.0],
                        settings: [0.0, 2.0, 0.0, 0.0],
                        pbr: [0.0; 4],
                        normal_settings: [0.0; 4],
                    },
                    blended: false,
                    node: None,
                    pipeline: 0,
                    metallic_map: None,
                    normal_map: None,
                    emissive_map: None,
                },
            ));
        }
        for (markings, color) in [
            (false, [0.18, 0.18, 0.18, 1.0]),
            (true, [0.8, 0.62, 0.04, 1.0]),
        ] {
            anyhow::ensure!(
                taxiways.iter().all(|t| t.start.is_finite()
                    && t.end.is_finite()
                    && t.width.is_finite()
                    && (0.1..=1000.0).contains(&t.width)),
                "invalid taxiway geometry"
            );
            let (vertices, indices) = taxiway_mesh(taxiways, markings);
            if indices.is_empty() {
                continue;
            }
            runway_draws.push(make_draw(
                &device,
                &draw_layout,
                &sampler,
                &white,
                DrawData {
                    vertices: &vertices,
                    indices: &indices,
                    transform: Mat4::IDENTITY,
                    uniform: DrawUniform {
                        model: Mat4::IDENTITY.to_cols_array_2d(),
                        normal: Mat4::IDENTITY.to_cols_array_2d(),
                        diffuse: color,
                        settings: [0.0, 2.0, 0.0, 0.0],
                        pbr: [0.0; 4],
                        normal_settings: [0.0; 4],
                    },
                    blended: false,
                    node: None,
                    pipeline: 0,
                    metallic_map: None,
                    normal_map: None,
                    emissive_map: None,
                },
            ));
        }
        for runway in runways {
            anyhow::ensure!(
                runway.threshold.is_finite()
                    && runway.heading.is_finite()
                    && runway.length.is_finite()
                    && (1.0..=20_000.0).contains(&runway.length)
                    && runway.width.is_finite()
                    && (1.0..=1_000.0).contains(&runway.width),
                "invalid runway position"
            );
            let transform = Mat4::from_translation(runway.threshold.as_vec3())
                * Mat4::from_rotation_y(runway.heading as f32);
            anyhow::ensure!(
                transform.is_finite(),
                "runway is outside the renderable scene"
            );
            for (markings, color) in [
                (false, [0.12, 0.12, 0.12, 1.0]),
                (true, [0.92, 0.92, 0.92, 1.0]),
            ] {
                let (vertices, indices) = runway_mesh(markings, runway.length, runway.width);
                runway_draws.push(make_draw(
                    &device,
                    &draw_layout,
                    &sampler,
                    &white,
                    DrawData {
                        vertices: &vertices,
                        indices: &indices,
                        transform,
                        uniform: DrawUniform {
                            model: Mat4::IDENTITY.to_cols_array_2d(),
                            normal: Mat4::IDENTITY.to_cols_array_2d(),
                            diffuse: color,
                            settings: [0.0, 2.0, 0.0, 0.0],
                            pbr: [0.0; 4],
                            normal_settings: [0.0; 4],
                        },
                        blended: false,
                        node: None,
                        pipeline: 0,
                        metallic_map: None,
                        normal_map: None,
                        emissive_map: None,
                    },
                ));
            }
        }
        let cockpit_eye = aircraft
            .eyepoint
            .map(|eye| aircraft_origin + eye.as_dvec3());
        let depth = depth_target(&device, &config, sample_count);
        let multisampled_color = color_target(&device, &config, sample_count);
        Ok(Self {
            surface,
            device,
            queue,
            config,
            depth,
            multisampled_color,
            sample_count,
            pipelines,
            sky,
            camera_buffer,
            camera_bind,
            exterior,
            interior,
            panels,
            ground,
            terrain_draws,
            draw_layout,
            terrain_sampler: sampler,
            terrain_blend_pipeline: photo_pipeline,
            ground_tracks_camera: terrain.is_none(),
            photos: photo_draws,
            runway: runway_draws,
            objects: scenery_draws,
            centre,
            radius,
            aircraft_origin,
            aircraft_rotation: Mat4::IDENTITY,
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
        self.depth = depth_target(&self.device, &self.config, self.sample_count);
        self.multisampled_color = color_target(&self.device, &self.config, self.sample_count);
    }

    pub fn update_panel(&self, name: &str, width: u32, height: u32, pixels: &[u8]) -> Result<()> {
        let Some(texture) = self.panels.get(name) else {
            return Ok(());
        };
        anyhow::ensure!(
            texture.width() == width
                && texture.height() == height
                && pixels.len() == width as usize * height as usize * 4,
            "invalid panel texture update"
        );
        self.queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            pixels,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(width * 4),
                rows_per_image: None,
            },
            wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
        );
        Ok(())
    }

    pub fn update_model(
        &mut self,
        cockpit: bool,
        transforms: &[Mat4],
        visible: &[bool],
    ) -> Result<()> {
        let draws = if cockpit {
            &mut self.interior
        } else {
            &mut self.exterior
        };
        for draw in draws {
            if let Some(node) = draw.node {
                draw.transform = *transforms.get(node).context("missing scene transform")?;
                draw.visible = *visible.get(node).context("missing scene visibility")?;
            }
        }
        Ok(())
    }

    pub fn render(
        &mut self,
        camera: &Camera,
        cockpit: bool,
        sun: Vec3,
        panel_lights: bool,
        weather: Weather,
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
        anyhow::ensure!(
            sun.is_finite() && sun.length_squared() > 0.0,
            "invalid sun direction"
        );
        let sun = sun.normalize();
        let camera_uniform = sky::uniform(camera, matrix, sun, panel_lights, weather);
        self.queue
            .write_buffer(&self.camera_buffer, 0, bytemuck::bytes_of(&camera_uniform));
        let draws = if cockpit {
            &mut self.interior
        } else {
            &mut self.exterior
        };
        if self.ground_tracks_camera {
            self.ground.transform = Mat4::from_translation(Vec3::new(
                camera.position.x.round() as f32,
                0.0,
                camera.position.z.round() as f32,
            ));
        }
        for draw in draws
            .iter_mut()
            .chain(std::iter::once(&mut self.ground))
            .chain(self.terrain_draws.iter_mut())
            .chain(self.photos.iter_mut())
            .chain(self.runway.iter_mut())
            .chain(self.objects.iter_mut())
        {
            let (origin, rotation) = if draw.uniform.settings[1] > 0.0 {
                (DVec3::ZERO, Mat4::IDENTITY)
            } else {
                (self.aircraft_origin, self.aircraft_rotation)
            };
            let model = rotation * draw.transform;
            let transform = Mat4::from_translation((origin - camera.position).as_vec3()) * model;
            draw.uniform.model = transform.to_cols_array_2d();
            draw.uniform.normal = model.inverse().transpose().to_cols_array_2d();
            self.queue
                .write_buffer(&draw.buffer, 0, bytemuck::bytes_of(&draw.uniform));
        }
        let mut transparent: Vec<_> = draws.iter().filter(|d| d.visible && d.blended).collect();
        transparent.sort_by(|a, b| {
            let distance = |d: &Draw| {
                (self.aircraft_origin
                    + (self.aircraft_rotation * d.transform)
                        .transform_point3(d.centre)
                        .as_dvec3()
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
                    view: self.multisampled_color.as_ref().unwrap_or(&view),
                    resolve_target: self.multisampled_color.as_ref().map(|_| &view),
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
            pass.set_pipeline(&self.sky);
            pass.draw(0..3, 0..1);
            let draw_one = |pass: &mut wgpu::RenderPass<'_>, draw: &Draw| {
                pass.set_pipeline(&self.pipelines[draw.pipeline]);
                pass.set_bind_group(1, &draw.bind, &[]);
                pass.set_vertex_buffer(0, draw.vertices.slice(..));
                pass.set_index_buffer(draw.indices.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..draw.count, 0, 0..1);
            };
            draw_one(&mut pass, &self.ground);
            for draw in &self.terrain_draws {
                draw_one(&mut pass, draw);
            }
            for draw in &self.photos {
                draw_one(&mut pass, draw);
            }
            for draw in &self.runway {
                draw_one(&mut pass, draw);
            }
            for draw in &self.objects {
                draw_one(&mut pass, draw);
            }
            for draw in draws.iter().filter(|d| d.visible && !d.blended) {
                draw_one(&mut pass, draw);
            }
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

fn depth_target(
    device: &wgpu::Device,
    config: &wgpu::SurfaceConfiguration,
    sample_count: u32,
) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some("depth"),
            size: wgpu::Extent3d {
                width: config.width,
                height: config.height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        })
        .create_view(&Default::default())
}

fn color_target(
    device: &wgpu::Device,
    config: &wgpu::SurfaceConfiguration,
    sample_count: u32,
) -> Option<wgpu::TextureView> {
    if sample_count == 1 {
        return None;
    }
    Some(
        device
            .create_texture(&wgpu::TextureDescriptor {
                label: Some("multisampled scene color"),
                size: wgpu::Extent3d {
                    width: config.width,
                    height: config.height,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count,
                dimension: wgpu::TextureDimension::D2,
                format: config.format,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            })
            .create_view(&Default::default()),
    )
}
fn upload_texture(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    image: &texture::Texture,
) -> Result<wgpu::Texture> {
    upload_texture_with_color_space(device, queue, image, true)
}

fn upload_texture_with_color_space(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    image: &texture::Texture,
    srgb: bool,
) -> Result<wgpu::Texture> {
    let format = texture_format(image.format, srgb);
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
    Ok(texture)
}

#[cfg(test)]
mod runway_tests {
    use super::*;

    #[test]
    fn apron_triangles_are_oriented_upward_and_degenerates_skipped() {
        let apron = Apron {
            vertices: vec![DVec3::ZERO, DVec3::X, DVec3::Z],
            indices: vec![0, 1, 2, 0, 2, 1, 0, 0, 1],
        };
        let (vertices, indices) = apron_mesh(&[apron]);
        assert_eq!(vertices.len(), 3);
        assert_eq!(indices, [0, 2, 1, 0, 2, 1]);
    }

    #[test]
    fn taxiway_mesh_preserves_width_and_centerline_flags() {
        let segment = Taxiway {
            start: DVec3::ZERO,
            end: DVec3::Z * 100.0,
            width: 20.0,
            centerline: true,
        };
        let (vertices, indices) = taxiway_mesh(&[segment], false);
        assert_eq!(vertices.len(), 4);
        assert_eq!(indices.len(), 6);
        assert_eq!(
            (vertices[0].position[0] - vertices[3].position[0]).abs(),
            20.0
        );
        assert_eq!(taxiway_mesh(&[segment], true).1.len(), 6);
        assert!(
            taxiway_mesh(
                &[Taxiway {
                    centerline: false,
                    ..segment
                }],
                true
            )
            .1
            .is_empty()
        );
        assert!(
            taxiway_mesh(
                &[Taxiway {
                    end: DVec3::ZERO,
                    ..segment
                }],
                false
            )
            .1
            .is_empty()
        );
    }

    #[test]
    fn runway_surface_and_markings_have_valid_triangles() {
        for marked in [false, true] {
            let (vertices, indices) = runway_mesh(marked, 2000.0, 45.0);
            assert!(!vertices.is_empty());
            assert_eq!(indices.len() % 3, 0);
            assert!(indices.iter().all(|&i| (i as usize) < vertices.len()));
            assert!(
                vertices
                    .iter()
                    .all(|v| v.position.iter().all(|x| x.is_finite()))
            );
        }
    }
}

//! Camera-facing aircraft light sprites for static effect particles.
use std::collections::HashMap;

use anyhow::Result;
use esp_assets::{effects::LightEffects, texture::Texture};
use glam::{DVec3, Mat4, Vec3};
use wgpu::util::DeviceExt;

impl super::Renderer {
    pub fn set_light_effects(&mut self, content: &LightEffects) -> Result<()> {
        self.effects = Some(Effects::new(
            &self.device,
            &self.queue,
            &self.sky.get_bind_group_layout(0),
            self.config.format,
            self.sample_count,
            content,
        )?);
        Ok(())
    }

    pub fn set_light_switches(&mut self, switches: [bool; 14]) {
        if let Some(effects) = &mut self.effects {
            effects.switches(switches);
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Uniform {
    position: [f32; 4],
    right: [f32; 4],
    up: [f32; 4],
    colour: [f32; 4],
    uv: [f32; 4],
}

struct Sprite {
    kind: u32,
    anchor: Vec3,
    size: [f32; 2],
    delay: f64,
    rate: f64,
    lifetime: f64,
    uniform: Uniform,
    buffer: wgpu::Buffer,
    bind: wgpu::BindGroup,
    additive: bool,
}

pub(super) struct Effects {
    sprites: Vec<Sprite>,
    additive: wgpu::RenderPipeline,
    alpha: wgpu::RenderPipeline,
    switches: [bool; 14],
}

fn active(time: f64, delay: f64, rate: f64, lifetime: f64) -> bool {
    if lifetime <= 0.0 {
        return true;
    }
    if rate <= 0.0 || time < delay {
        return false;
    }
    let period = 1.0 / rate;
    (time - delay).rem_euclid(period) < lifetime.max(1.0 / 30.0).min(period)
}

impl Effects {
    pub fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        camera_layout: &wgpu::BindGroupLayout,
        format: wgpu::TextureFormat,
        samples: u32,
        content: &LightEffects,
    ) -> Result<Self> {
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("effect sprite"),
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
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("effect sprite"),
            source: wgpu::ShaderSource::Wgsl(include_str!("effects.wgsl").into()),
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("effect sprite"),
            bind_group_layouts: &[Some(camera_layout), Some(&layout)],
            immediate_size: 0,
        });
        let pipeline = |blend: wgpu::BlendState| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("effect sprite"),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some("vs_main"),
                    compilation_options: Default::default(),
                    buffers: &[],
                },
                primitive: wgpu::PrimitiveState {
                    cull_mode: None,
                    ..Default::default()
                },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: wgpu::TextureFormat::Depth32Float,
                    depth_write_enabled: Some(false),
                    depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
                    stencil: Default::default(),
                    bias: Default::default(),
                }),
                multisample: wgpu::MultisampleState {
                    count: samples,
                    ..Default::default()
                },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some("fs_main"),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format,
                        blend: Some(blend),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let additive = pipeline(wgpu::BlendState {
            color: wgpu::BlendComponent {
                src_factor: wgpu::BlendFactor::SrcAlpha,
                dst_factor: wgpu::BlendFactor::One,
                operation: wgpu::BlendOperation::Add,
            },
            alpha: wgpu::BlendComponent::OVER,
        });
        let alpha = pipeline(wgpu::BlendState::ALPHA_BLENDING);
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        let mut views = HashMap::new();
        let mut sprites = Vec::new();
        for light in &content.lights {
            let Some(particles) = content.effects.get(&light.effect.to_ascii_lowercase()) else {
                continue;
            };
            let [longitudinal, lateral, vertical] = light.position_feet;
            let [dx, dy, dz] = content.datum_feet;
            let anchor = Vec3::new(
                (lateral + dy) as f32,
                (vertical + dz) as f32,
                (longitudinal + dx) as f32,
            ) * 0.3048;
            for particle in particles
                .iter()
                .filter(|p| p.kind == 19 && p.is_static && matches!(p.blend_mode, 1 | 2))
            {
                let Some(name) = &particle.texture else {
                    continue;
                };
                let key = name.to_ascii_lowercase();
                if !views.contains_key(&key) {
                    let Some(bytes) = content.textures.get(&key) else {
                        continue;
                    };
                    let texture = match Texture::from_bytes(
                        bytes,
                        device
                            .features()
                            .contains(wgpu::Features::TEXTURE_COMPRESSION_BC),
                    ) {
                        Ok(texture) => texture,
                        Err(error) => {
                            eprintln!("effect texture {name}: {error:#}");
                            continue;
                        }
                    };
                    let view = super::upload_texture(device, queue, &texture)?
                        .create_view(&Default::default());
                    views.insert(key.clone(), (view, texture.flip_v));
                }
                let (view, flip) = &views[&key];
                let average = |v: [f64; 2]| (v[0] + v[1]) * 0.5;
                let offset = particle.emitter_offset.map(average);
                let uniform = Uniform {
                    position: [0.0, 0.0, 0.0, f32::from(*flip)],
                    right: [0.0; 4],
                    up: [0.0; 4],
                    colour: particle
                        .color_start
                        .map(|x| (x / 255.0).clamp(0.0, 1.0) as f32),
                    uv: [
                        particle.uv1[0] as f32,
                        particle.uv1[1] as f32,
                        particle.uv2[0] as f32,
                        particle.uv2[1] as f32,
                    ],
                };
                let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("effect sprite"),
                    contents: bytemuck::bytes_of(&uniform),
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                });
                let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("effect sprite"),
                    layout: &layout,
                    entries: &[
                        wgpu::BindGroupEntry {
                            binding: 0,
                            resource: buffer.as_entire_binding(),
                        },
                        wgpu::BindGroupEntry {
                            binding: 1,
                            resource: wgpu::BindingResource::TextureView(view),
                        },
                        wgpu::BindGroupEntry {
                            binding: 2,
                            resource: wgpu::BindingResource::Sampler(&sampler),
                        },
                    ],
                });
                sprites.push(Sprite {
                    kind: light.kind,
                    anchor: anchor
                        + Vec3::new(offset[0] as f32, offset[1] as f32, offset[2] as f32),
                    size: [
                        average(particle.scale[0]).max(0.0) as f32,
                        average(particle.scale[1]).max(0.0) as f32,
                    ],
                    delay: average(particle.delay).max(0.0),
                    rate: average(particle.rate).max(0.0),
                    lifetime: average(particle.lifetime).max(0.0),
                    uniform,
                    buffer,
                    bind,
                    additive: particle.blend_mode == 2,
                });
            }
        }
        Ok(Self {
            sprites,
            additive,
            alpha,
            switches: [false; 14],
        })
    }

    pub fn switches(&mut self, switches: [bool; 14]) {
        self.switches = switches;
    }
    pub fn update(
        &mut self,
        queue: &wgpu::Queue,
        camera: &super::Camera,
        origin: DVec3,
        rotation: Mat4,
    ) {
        let forward = camera.forward();
        let up = camera.frame * Vec3::Y;
        let right = up.cross(forward).normalize_or_zero();
        let up = forward.cross(right).normalize_or_zero();
        for sprite in &mut self.sprites {
            let point =
                (origin - camera.position).as_vec3() + rotation.transform_point3(sprite.anchor);
            sprite.uniform.position[..3].copy_from_slice(&point.to_array());
            sprite.uniform.right[..3].copy_from_slice(&(right * sprite.size[0] * 0.5).to_array());
            sprite.uniform.up[..3].copy_from_slice(&(up * sprite.size[1] * 0.5).to_array());
            queue.write_buffer(&sprite.buffer, 0, bytemuck::bytes_of(&sprite.uniform));
        }
    }
    pub fn draw<'a>(&'a self, pass: &mut wgpu::RenderPass<'a>, time: f64) {
        for sprite in &self.sprites {
            if !self
                .switches
                .get(sprite.kind as usize)
                .copied()
                .unwrap_or(false)
                || !active(time, sprite.delay, sprite.rate, sprite.lifetime)
            {
                continue;
            }
            pass.set_pipeline(if sprite.additive {
                &self.additive
            } else {
                &self.alpha
            });
            pass.set_bind_group(1, &sprite.bind, &[]);
            pass.draw(0..6, 0..1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wgpu::util::DeviceExt;

    #[test]
    fn sprite_shader_is_valid_wgsl() {
        let shader = naga::front::wgsl::parse_str(include_str!("effects.wgsl")).unwrap();
        naga::valid::Validator::new(
            naga::valid::ValidationFlags::all(),
            naga::valid::Capabilities::all(),
        )
        .validate(&shader)
        .unwrap();
    }
    #[test]
    fn steady_and_delayed_strobe_particles_follow_authored_timing() {
        assert!(active(100.0, 0.0, 1.0, 0.0));
        assert!(!active(0.1, 0.2, 0.5, 0.01));
        assert!(active(0.21, 0.2, 0.5, 0.01));
        assert!(!active(0.3, 0.2, 0.5, 0.01));
        assert!(active(2.21, 0.2, 0.5, 0.01));
    }

    #[test]
    #[ignore = "requires GPU, OPENESP_AIRCRAFT_PATHS and OPENESP_P3D_ROOT"]
    fn renders_real_aircraft_light_effects_offscreen() {
        pollster::block_on(async {
            let paths = std::env::var_os("OPENESP_AIRCRAFT_PATHS").unwrap();
            let aircraft = std::env::split_paths(&paths).next().unwrap();
            let root = std::path::PathBuf::from(std::env::var_os("OPENESP_P3D_ROOT").unwrap());
            let content = LightEffects::load(&aircraft, &root).unwrap();
            let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
                backends: wgpu::Backends::PRIMARY,
                ..wgpu::InstanceDescriptor::new_without_display_handle()
            });
            let adapter = instance
                .request_adapter(&wgpu::RequestAdapterOptions::default())
                .await
                .unwrap();
            let (device, queue) = adapter
                .request_device(&wgpu::DeviceDescriptor {
                    required_features: adapter.features() & wgpu::Features::TEXTURE_COMPRESSION_BC,
                    ..Default::default()
                })
                .await
                .unwrap();
            let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: None,
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
            let sky =
                crate::sky::pipeline(&device, &layout, wgpu::TextureFormat::Rgba8UnormSrgb, 1);
            let mut effects = Effects::new(
                &device,
                &queue,
                &sky.get_bind_group_layout(0),
                wgpu::TextureFormat::Rgba8UnormSrgb,
                1,
                &content,
            )
            .unwrap();
            assert!(effects.sprites.len() >= 8);
            let camera =
                crate::Camera::looking_at(DVec3::new(0.0, 2.0, -45.0), DVec3::new(0.0, 2.0, 0.0));
            let matrix = camera.view_projection(2.0);
            let weather = crate::Weather {
                visibility_m: 100000.0,
                origin_msl: 0.0,
                clouds: [None; 3],
                cloud_offset: [0.0; 2],
                precipitation: [0.0; 2],
                simulation_seconds: 0.0,
            };
            let uniforms = crate::sky::uniform(&camera, matrix, Vec3::Y, false, weather, false);
            let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: None,
                contents: bytemuck::bytes_of(&uniforms),
                usage: wgpu::BufferUsages::UNIFORM,
            });
            let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: None,
                layout: &layout,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: buffer.as_entire_binding(),
                }],
            });
            let target = device.create_texture(&wgpu::TextureDescriptor {
                label: None,
                size: wgpu::Extent3d {
                    width: 256,
                    height: 128,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rgba8UnormSrgb,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
                view_formats: &[],
            });
            let depth = device.create_texture(&wgpu::TextureDescriptor {
                label: None,
                size: target.size(),
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Depth32Float,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            });
            let output = device.create_buffer(&wgpu::BufferDescriptor {
                label: None,
                size: 256 * 128 * 4,
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });
            effects.update(&queue, &camera, DVec3::ZERO, Mat4::IDENTITY);
            let mut images = Vec::new();
            for (nav, strobe, time) in [
                (false, false, 0.0),
                (true, false, 0.0),
                (false, true, 0.1),
                (false, true, 0.21),
            ] {
                let mut switches = [false; 14];
                switches[3] = nav;
                switches[2] = strobe;
                effects.switches(switches);
                let mut encoder = device.create_command_encoder(&Default::default());
                {
                    let view = target.create_view(&Default::default());
                    let depth_view = depth.create_view(&Default::default());
                    let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                        label: None,
                        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                            view: &view,
                            resolve_target: None,
                            depth_slice: None,
                            ops: wgpu::Operations {
                                load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                                store: wgpu::StoreOp::Store,
                            },
                        })],
                        depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                            view: &depth_view,
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
                    pass.set_bind_group(0, &bind, &[]);
                    effects.draw(&mut pass, time);
                }
                encoder.copy_texture_to_buffer(
                    target.as_image_copy(),
                    wgpu::TexelCopyBufferInfo {
                        buffer: &output,
                        layout: wgpu::TexelCopyBufferLayout {
                            offset: 0,
                            bytes_per_row: Some(1024),
                            rows_per_image: Some(128),
                        },
                    },
                    target.size(),
                );
                queue.submit([encoder.finish()]);
                let (sender, receiver) = std::sync::mpsc::channel();
                output
                    .slice(..)
                    .map_async(wgpu::MapMode::Read, move |result| {
                        let _ = sender.send(result);
                    });
                device.poll(wgpu::PollType::wait_indefinitely()).unwrap();
                receiver.recv().unwrap().unwrap();
                images.push(output.slice(..).get_mapped_range().unwrap().to_vec());
                output.unmap();
            }
            assert!(
                images[0]
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .all(|pixel| pixel[..3] == [0, 0, 0])
            );
            assert!(
                images[1]
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .any(|pixel| pixel[..3].iter().any(|value| *value > 10)),
                "navigation lights are not visible"
            );
            assert!(
                images[2]
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .all(|pixel| pixel[..3] == [0, 0, 0]),
                "strobe remained on between flashes"
            );
            assert!(
                images[3]
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .any(|pixel| pixel[..3].iter().any(|value| *value > 10)),
                "delayed strobe flash is not visible"
            );
            image::save_buffer(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../target/b737-lights-headless.png"),
                &images[1],
                256,
                128,
                image::ColorType::Rgba8,
            )
            .unwrap();
            image::save_buffer(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../target/b737-strobe-headless.png"),
                &images[3],
                256,
                128,
                image::ColorType::Rgba8,
            )
            .unwrap();
        });
    }
}

use crate::{
    Draw, DrawData, DrawUniform, Renderer, TerrainMesh, Vertex, make_draw, upload_texture,
};
use anyhow::{Result, ensure};
use glam::Mat4;
use std::collections::BTreeMap;
use wgpu::util::DeviceExt;

impl Renderer {
    /// Replace the terrain geometry without rebuilding the rest of the scene.
    pub fn update_terrain(&mut self, terrain: &TerrainMesh) -> Result<()> {
        let (vertices, indices) = mesh(Some(terrain))?;
        let draws = textured_draws(
            &self.device,
            &self.queue,
            &self.draw_layout,
            &self.terrain_sampler,
            terrain,
            self.terrain_blend_pipeline,
        )?;
        let count = u32::try_from(indices.len())?;
        self.ground.vertices = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("terrain vertices"),
                contents: bytemuck::cast_slice(&vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });
        self.ground.indices = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("terrain indices"),
                contents: bytemuck::cast_slice(&indices),
                usage: wgpu::BufferUsages::INDEX,
            });
        self.ground.count = count;
        self.terrain_draws = draws;
        self.ground_tracks_camera = false;
        Ok(())
    }
}

pub(super) fn mesh(terrain: Option<&TerrainMesh>) -> Result<(Vec<Vertex>, Vec<u32>)> {
    if let Some(terrain) = terrain {
        ensure!(
            terrain.vertices.len() == terrain.normals.len()
                && terrain.vertices.len() == terrain.land_classes.len()
                && terrain.vertices.len() == terrain.texture_sets.len()
                && !terrain.indices.is_empty()
                && terrain.indices.len().is_multiple_of(3)
                && terrain.vertices.iter().all(|vertex| vertex.is_finite())
                && terrain.normals.iter().all(|normal| normal.is_finite())
                && terrain
                    .indices
                    .iter()
                    .all(|index| (*index as usize) < terrain.vertices.len()),
            "invalid terrain mesh"
        );
        let vertices = terrain
            .vertices
            .iter()
            .zip(&terrain.normals)
            .zip(&terrain.land_classes)
            .map(|((position, normal), class)| Vertex {
                position: position.as_vec3().to_array(),
                normal: normal.as_vec3().to_array(),
                uv: [0.0; 2],
                tangent: {
                    let [r, g, b] = land_colour(*class);
                    [r, g, b, 1.0]
                },
            })
            .collect();
        return Ok((vertices, terrain.indices.clone()));
    }
    // Keep grid UVs precise near the camera while the surrounding fill reaches the horizon.
    let extent = 40_000.0;
    let pattern = 1_024.0;
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut quad = |points: [(f64, f64); 4]| {
        let base = vertices.len() as u32;
        for (x, z) in points {
            vertices.push(Vertex {
                position: [x as f32, 0.0, z as f32],
                normal: [0.0, 1.0, 0.0],
                uv: [x as f32, z as f32],
                tangent: [0.0; 4],
            });
        }
        indices.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    };
    quad([
        (-pattern, -pattern),
        (-pattern, pattern),
        (pattern, pattern),
        (pattern, -pattern),
    ]);
    quad([
        (-extent, pattern),
        (-extent, extent),
        (extent, extent),
        (extent, pattern),
    ]);
    quad([
        (-extent, -extent),
        (-extent, -pattern),
        (extent, -pattern),
        (extent, -extent),
    ]);
    quad([
        (-extent, -pattern),
        (-extent, pattern),
        (-pattern, pattern),
        (-pattern, -pattern),
    ]);
    quad([
        (pattern, -pattern),
        (pattern, pattern),
        (extent, pattern),
        (extent, -pattern),
    ]);
    Ok((vertices, indices))
}

/// Build opaque base draws and blended transition draws for the terrain.
pub(super) fn textured_draws(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    layout: &wgpu::BindGroupLayout,
    sampler: &wgpu::Sampler,
    terrain: &TerrainMesh,
    blend_pipeline: usize,
) -> Result<Vec<Draw>> {
    let groups = texture_batches(terrain)?;
    let compressed = device
        .features()
        .contains(wgpu::Features::TEXTURE_COMPRESSION_BC);
    let mut result = Vec::new();
    for ((overlay, key), (vertices, indices)) in groups {
        let image = &terrain.textures[&key];
        if !compressed && image.format != esp_assets::texture::Format::Rgba8 {
            continue;
        }
        let view = upload_texture(device, queue, image)?.create_view(&Default::default());
        result.push(make_draw(
            device,
            layout,
            sampler,
            &view,
            DrawData {
                vertices: &vertices,
                indices: &indices,
                transform: Mat4::IDENTITY,
                uniform: DrawUniform {
                    model: Mat4::IDENTITY.to_cols_array_2d(),
                    normal: Mat4::IDENTITY.to_cols_array_2d(),
                    diffuse: [1.0; 4],
                    settings: [
                        f32::from(image.flip_v),
                        if overlay { 5.0 } else { 4.0 },
                        0.0,
                        0.0,
                    ],
                    pbr: [0.0; 4],
                    normal_settings: [0.0; 4],
                },
                blended: false,
                node: None,
                pipeline: if overlay { blend_pipeline } else { 0 },
                metallic_map: None,
                normal_map: None,
                emissive_map: None,
            },
        ));
    }
    Ok(result)
}

type TextureBatches = BTreeMap<(bool, u32), (Vec<Vertex>, Vec<u32>)>;

fn texture_batches(terrain: &TerrainMesh) -> Result<TextureBatches> {
    let (vertices, _) = mesh(Some(terrain))?;
    let mut groups = TextureBatches::new();
    for triangle in terrain.indices.as_chunks::<3>().0 {
        let keys = triangle.map(|index| {
            terrain.texture_sets[index as usize].filter(|key| terrain.textures.contains_key(key))
        });
        let Some(base) = keys.iter().flatten().next().copied() else {
            continue;
        };
        let mut add = |key, overlay| -> Result<()> {
            let (group_vertices, group_indices) = groups.entry((overlay, key)).or_default();
            for (index, vertex_key) in triangle.iter().zip(keys) {
                let mut vertex = vertices[*index as usize];
                vertex.uv = [vertex.position[0] / 1200.0, -vertex.position[2] / 1200.0];
                vertex.tangent[3] = if !overlay || vertex_key == Some(key) {
                    1.0
                } else {
                    0.0
                };
                group_indices.push(u32::try_from(group_vertices.len())?);
                group_vertices.push(vertex);
            }
            Ok(())
        };
        add(base, false)?;
        let mut added = [None; 2];
        let mut count = 0;
        for key in keys.into_iter().flatten().filter(|key| *key != base) {
            if added.contains(&Some(key)) {
                continue;
            }
            add(key, true)?;
            added[count] = Some(key);
            count += 1;
        }
    }
    Ok(groups)
}

// Approximate colours until region/season texture lookup and water rendering are available.
// Class meanings are defined by the Prepar3D Terrain SDK's Olson table.
fn land_colour(class: Option<u8>) -> [f32; 3] {
    match class {
        Some(0 | 100 | 123..=127) => [0.07, 0.17, 0.23],
        Some(1 | 101..=121 | 136) => [0.36, 0.35, 0.32],
        Some(
            3..=6 | 13 | 19..=29 | 32..=34 | 48 | 54 | 60..=62 | 72 | 78 | 89 | 90 | 144 | 145,
        ) => [0.16, 0.26, 0.13],
        Some(8 | 11 | 50..=52 | 69 | 99 | 129 | 137 | 146) => [0.48, 0.39, 0.25],
        Some(9 | 42 | 53 | 63 | 130 | 131 | 133 | 138 | 141) => [0.32, 0.31, 0.26],
        Some(122 | 142 | 143) => [0.72, 0.75, 0.74],
        Some(
            2
            | 7
            | 10
            | 12
            | 16
            | 17
            | 30
            | 31
            | 35..=38
            | 40
            | 41
            | 43..=47
            | 55..=59
            | 64
            | 76
            | 91..=98
            | 128
            | 134
            | 135
            | 139
            | 140
            | 147,
        ) => [0.29, 0.35, 0.19],
        _ => [0.32, 0.32, 0.32],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::DVec3;

    #[test]
    fn terrain_mesh_keeps_heights_and_rejects_bad_indices() {
        let mut terrain = TerrainMesh {
            vertices: vec![DVec3::ZERO, DVec3::X, DVec3::new(0.0, 10.0, 1.0)],
            normals: vec![DVec3::Y; 3],
            land_classes: vec![Some(0), Some(3), None],
            texture_sets: vec![None; 3],
            textures: Default::default(),
            indices: vec![0, 1, 2],
        };
        let (vertices, indices) = mesh(Some(&terrain)).unwrap();
        assert_eq!(vertices[2].position, [0.0, 10.0, 1.0]);
        assert_eq!(vertices[0].tangent, [0.07, 0.17, 0.23, 1.0]);
        assert_ne!(vertices[0].tangent, vertices[1].tangent);
        assert_eq!(indices, [0, 1, 2]);
        terrain.indices[2] = 3;
        assert!(mesh(Some(&terrain)).is_err());
        assert!(!mesh(None).unwrap().1.is_empty());
    }

    #[test]
    fn blends_texture_transitions_and_keeps_uvs_anchored() {
        let mut terrain = TerrainMesh {
            vertices: vec![
                DVec3::ZERO,
                DVec3::new(1200.0, 0.0, 0.0),
                DVec3::new(0.0, 0.0, 1200.0),
                DVec3::new(1200.0, 0.0, 1200.0),
            ],
            normals: vec![DVec3::Y; 4],
            land_classes: vec![None; 4],
            texture_sets: vec![Some(7), Some(9), Some(7), Some(9)],
            textures: [7, 9]
                .map(|key| {
                    (
                        key,
                        esp_assets::texture::Texture::rgba(1, 1, vec![255; 4], false),
                    )
                })
                .into(),
            indices: vec![0, 2, 3, 1, 3, 2],
        };
        let batches = texture_batches(&terrain).unwrap();
        assert_eq!(batches.len(), 4);
        let (vertices, indices) = &batches[&(false, 7)];
        assert_eq!(indices, &[0, 1, 2]);
        assert_eq!(vertices[1].uv, [0.0, -1.0]);
        assert_eq!(vertices[2].uv, [1.0, -1.0]);
        let (overlay, _) = &batches[&(true, 9)];
        assert_eq!(
            overlay
                .iter()
                .map(|vertex| vertex.tangent[3])
                .collect::<Vec<_>>(),
            [0.0, 0.0, 1.0]
        );
        terrain.textures.remove(&9);
        let fallback = texture_batches(&terrain).unwrap();
        assert_eq!(fallback.len(), 1);
        assert_eq!(fallback[&(false, 7)].1.len(), 6);
    }

    #[test]
    fn terrain_shader_is_valid_wgsl() {
        let shader = naga::front::wgsl::parse_str(include_str!("aircraft.wgsl")).unwrap();
        naga::valid::Validator::new(
            naga::valid::ValidationFlags::all(),
            naga::valid::Capabilities::all(),
        )
        .validate(&shader)
        .unwrap();
    }

    #[test]
    #[ignore = "requires a GPU adapter"]
    fn renders_textured_terrain_offscreen() {
        pollster::block_on(async {
            let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
                backends: wgpu::Backends::PRIMARY,
                ..wgpu::InstanceDescriptor::new_without_display_handle()
            });
            let adapter = instance
                .request_adapter(&wgpu::RequestAdapterOptions::default())
                .await
                .unwrap();
            let bc = adapter.features() & wgpu::Features::TEXTURE_COMPRESSION_BC;
            let (device, queue) = adapter
                .request_device(&wgpu::DeviceDescriptor {
                    required_features: bc,
                    ..Default::default()
                })
                .await
                .unwrap();
            let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: None,
                source: wgpu::ShaderSource::Wgsl(include_str!("aircraft.wgsl").into()),
            });
            let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: None, layout: None,
                vertex: wgpu::VertexState {
                    module: &shader, entry_point: Some("vs_main"), compilation_options: Default::default(),
                    buffers: &[Some(wgpu::VertexBufferLayout {
                        array_stride: std::mem::size_of::<Vertex>() as u64, step_mode: wgpu::VertexStepMode::Vertex,
                        attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x2, 3 => Float32x4],
                    })],
                },
                primitive: wgpu::PrimitiveState { cull_mode: None, ..Default::default() },
                depth_stencil: None, multisample: Default::default(),
                fragment: Some(wgpu::FragmentState {
                    module: &shader, entry_point: Some("fs_main"), compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState { format: wgpu::TextureFormat::Rgba8UnormSrgb, blend: Some(wgpu::BlendState::ALPHA_BLENDING), write_mask: wgpu::ColorWrites::ALL })],
                }), multiview_mask: None, cache: None,
            });
            let emissive_path = std::env::var_os("OPENESP_EMISSIVE_TEXTURE");
            let normal_path = std::env::var_os("OPENESP_NORMAL_TEXTURE");
            let emissive_day = std::env::var_os("OPENESP_EMISSIVE_DAY").is_some();
            let panel_on = std::env::var_os("OPENESP_PANEL_LIGHT_ON").is_some();
            let encoded_mode = std::env::var("OPENESP_EMISSIVE_MODE")
                .ok()
                .map(|value| value.parse::<f32>().unwrap())
                .unwrap_or(1.0);
            let camera = crate::sky::CameraUniform {
                view_projection: Mat4::from_cols_array_2d(&[
                    [1.0 / 600.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0],
                    [0.0, -1.0 / 600.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ])
                .to_cols_array_2d(),
                inverse_view_projection: Mat4::IDENTITY.to_cols_array_2d(),
                sun: if normal_path.is_some() {
                    [0.6, 0.8, 0.0, 0.0]
                } else if emissive_path.is_some() && !emissive_day {
                    [0.0, -1.0, 0.0, 0.0]
                } else {
                    [0.0, 1.0, 0.0, 0.0]
                },
                lighting: [f32::from(panel_on), 0.0, 0.0, 0.0],
            };
            let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: None,
                contents: bytemuck::bytes_of(&camera),
                usage: wgpu::BufferUsages::UNIFORM,
            });
            let camera_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: None,
                layout: &pipeline.get_bind_group_layout(0),
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
                ..Default::default()
            });
            let external = std::env::var_os("OPENESP_TERRAIN_TEXTURE");
            let external_second = std::env::var_os("OPENESP_TERRAIN_TEXTURE_2");
            let texture = external.as_ref().map_or_else(
                || esp_assets::texture::Texture::rgba(1, 1, vec![255, 0, 0, 255], false),
                |path| {
                    esp_assets::texture::Texture::load(std::path::Path::new(path), !bc.is_empty())
                        .unwrap()
                },
            );
            let second_texture = external_second.as_ref().map_or_else(
                || esp_assets::texture::Texture::rgba(1, 1, vec![0, 0, 255, 255], false),
                |path| {
                    esp_assets::texture::Texture::load(std::path::Path::new(path), !bc.is_empty())
                        .unwrap()
                },
            );
            let two_textures = external.is_none() || external_second.is_some();
            let terrain = TerrainMesh {
                vertices: vec![
                    glam::DVec3::new(-600.0, 0.0, 600.0),
                    glam::DVec3::new(1800.0, 0.0, 600.0),
                    glam::DVec3::new(-600.0, 0.0, -1800.0),
                ],
                normals: vec![glam::DVec3::Y; 3],
                land_classes: vec![Some(3); 3],
                texture_sets: if two_textures {
                    vec![Some(7), Some(9), Some(7)]
                } else {
                    vec![Some(7); 3]
                },
                textures: [(7, texture), (9, second_texture)].into(),
                indices: vec![0, 1, 2],
            };
            let mut draws = textured_draws(
                &device,
                &queue,
                &pipeline.get_bind_group_layout(1),
                &sampler,
                &terrain,
                0,
            )
            .unwrap();
            if let Some(path) = &normal_path {
                let image =
                    esp_assets::texture::Texture::load(std::path::Path::new(path), !bc.is_empty())
                        .unwrap();
                let normal = crate::upload_texture_with_color_space(&device, &queue, &image, false)
                    .unwrap()
                    .create_view(&Default::default());
                let white = crate::upload_texture(
                    &device,
                    &queue,
                    &esp_assets::texture::Texture::rgba(1, 1, vec![255; 4], false),
                )
                .unwrap()
                .create_view(&Default::default());
                let vertices = [
                    Vertex {
                        position: [-600.0, 0.0, 600.0],
                        normal: [0.0, 1.0, 0.0],
                        uv: [0.0, 0.0],
                        tangent: [1.0, 0.0, 0.0, 1.0],
                    },
                    Vertex {
                        position: [1800.0, 0.0, 600.0],
                        normal: [0.0, 1.0, 0.0],
                        uv: [2.0, 0.0],
                        tangent: [1.0, 0.0, 0.0, 1.0],
                    },
                    Vertex {
                        position: [-600.0, 0.0, -1800.0],
                        normal: [0.0, 1.0, 0.0],
                        uv: [0.0, 2.0],
                        tangent: [1.0, 0.0, 0.0, 1.0],
                    },
                ];
                draws = vec![make_draw(
                    &device,
                    &pipeline.get_bind_group_layout(1),
                    &sampler,
                    &white,
                    DrawData {
                        vertices: &vertices,
                        indices: &[0, 1, 2],
                        transform: Mat4::IDENTITY,
                        uniform: DrawUniform {
                            model: Mat4::IDENTITY.to_cols_array_2d(),
                            normal: Mat4::IDENTITY.to_cols_array_2d(),
                            diffuse: [1.0; 4],
                            settings: [0.0; 4],
                            pbr: [0.0; 4],
                            normal_settings: [1.0, -1.0, f32::from(image.normal_x_in_alpha), 0.0],
                        },
                        blended: false,
                        node: None,
                        pipeline: 0,
                        metallic_map: None,
                        normal_map: Some(&normal),
                        emissive_map: None,
                    },
                )];
            }
            if let Some(path) = &emissive_path {
                let image =
                    esp_assets::texture::Texture::load(std::path::Path::new(path), !bc.is_empty())
                        .unwrap();
                let emission = crate::upload_texture(&device, &queue, &image)
                    .unwrap()
                    .create_view(&Default::default());
                let black = crate::upload_texture(
                    &device,
                    &queue,
                    &esp_assets::texture::Texture::rgba(1, 1, vec![0, 0, 0, 255], false),
                )
                .unwrap()
                .create_view(&Default::default());
                for draw in &mut draws {
                    draw.uniform.normal_settings[3] = encoded_mode;
                    queue.write_buffer(&draw.buffer, 0, bytemuck::bytes_of(&draw.uniform));
                    draw.bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
                        label: None,
                        layout: &pipeline.get_bind_group_layout(1),
                        entries: &[
                            wgpu::BindGroupEntry {
                                binding: 0,
                                resource: draw.buffer.as_entire_binding(),
                            },
                            wgpu::BindGroupEntry {
                                binding: 1,
                                resource: wgpu::BindingResource::TextureView(&black),
                            },
                            wgpu::BindGroupEntry {
                                binding: 2,
                                resource: wgpu::BindingResource::Sampler(&sampler),
                            },
                            wgpu::BindGroupEntry {
                                binding: 3,
                                resource: wgpu::BindingResource::TextureView(&black),
                            },
                            wgpu::BindGroupEntry {
                                binding: 4,
                                resource: wgpu::BindingResource::TextureView(&black),
                            },
                            wgpu::BindGroupEntry {
                                binding: 5,
                                resource: wgpu::BindingResource::TextureView(&emission),
                            },
                        ],
                    });
                }
            }
            assert_eq!(
                draws.len(),
                if normal_path.is_some() {
                    1
                } else if two_textures {
                    2
                } else {
                    1
                }
            );
            let target = device.create_texture(&wgpu::TextureDescriptor {
                label: None,
                size: wgpu::Extent3d {
                    width: 64,
                    height: 64,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rgba8UnormSrgb,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
                view_formats: &[],
            });
            let output = device.create_buffer(&wgpu::BufferDescriptor {
                label: None,
                size: 64 * 64 * 4,
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });
            let mut encoder = device.create_command_encoder(&Default::default());
            {
                let view = target.create_view(&Default::default());
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
                    depth_stencil_attachment: None,
                    timestamp_writes: None,
                    occlusion_query_set: None,
                    multiview_mask: None,
                });
                pass.set_pipeline(&pipeline);
                pass.set_bind_group(0, &camera_bind, &[]);
                for draw in &draws {
                    pass.set_bind_group(1, &draw.bind, &[]);
                    pass.set_vertex_buffer(0, draw.vertices.slice(..));
                    pass.set_index_buffer(draw.indices.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..draw.count, 0, 0..1);
                }
            }
            encoder.copy_texture_to_buffer(
                target.as_image_copy(),
                wgpu::TexelCopyBufferInfo {
                    buffer: &output,
                    layout: wgpu::TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(256),
                        rows_per_image: Some(64),
                    },
                },
                wgpu::Extent3d {
                    width: 64,
                    height: 64,
                    depth_or_array_layers: 1,
                },
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
            let pixels = output.slice(..).get_mapped_range().unwrap();
            let center = &pixels[(32 * 64 + 32) * 4..][..4];
            if normal_path.is_some() {
                let pixels = pixels.as_chunks::<4>().0;
                let minimum = pixels.iter().map(|pixel| pixel[0]).min().unwrap();
                let maximum = pixels.iter().map(|pixel| pixel[0]).max().unwrap();
                assert!(
                    maximum.saturating_sub(minimum) > 5,
                    "normal map did not change lighting"
                );
            } else if emissive_path.is_some() {
                let lit = pixels
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .any(|pixel| pixel[..3].iter().any(|channel| *channel > 20));
                assert_eq!(
                    lit,
                    std::env::var_os("OPENESP_EMISSIVE_EXPECT_DARK").is_none()
                );
            } else if external.is_none() {
                assert!(
                    center[0] > 40 && center[1] < 5 && center[2] > 40,
                    "pixel: {center:?}"
                );
            } else {
                assert!(
                    pixels
                        .as_chunks::<4>()
                        .0
                        .iter()
                        .any(|pixel| pixel[..3] != center[..3])
                );
            }
            let path =
                std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(if normal_path.is_some() {
                    "../../target/legacy-bump-headless.png"
                } else if emissive_path.is_some() {
                    "../../target/emissive-headless.png"
                } else if external.is_some() {
                    "../../target/terrain-texture-headless.png"
                } else {
                    "../../target/terrain-blend-headless.png"
                });
            image::save_buffer(path, &pixels, 64, 64, image::ColorType::Rgba8).unwrap();
        });
    }
}

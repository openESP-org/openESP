//! Upload per-mesh draw resources and material bindings.
use super::*;

pub(super) fn make_draw(
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
        node,
        pipeline,
        metallic_map,
        normal_map,
        emissive_map,
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
            wgpu::BindGroupEntry {
                binding: 3,
                resource: wgpu::BindingResource::TextureView(metallic_map.unwrap_or(texture)),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: wgpu::BindingResource::TextureView(normal_map.unwrap_or(texture)),
            },
            wgpu::BindGroupEntry {
                binding: 5,
                resource: wgpu::BindingResource::TextureView(emissive_map.unwrap_or(texture)),
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
        node,
        visible: true,
        pipeline,
    }
}

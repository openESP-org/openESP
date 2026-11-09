use super::*;
use wgpu::util::DeviceExt;

#[test]
fn camera_samples_clouds_relative_to_advected_world() {
    let camera = crate::Camera::looking_at(
        glam::DVec3::new(120.0, 100.0, -40.0),
        glam::DVec3::new(120.0, 100.0, 0.0),
    );
    let weather = Weather {
        visibility_m: 10_000.0,
        origin_msl: 300.0,
        clouds: [None; 3],
        cloud_offset: [70.0, -60.0],
    };
    let uniforms = uniform(&camera, glam::Mat4::IDENTITY, glam::Vec3::Y, false, weather);
    assert_eq!(uniforms.world[..2], [50.0, 20.0]);
    assert_eq!(uniforms.sun[3], 400.0);
}

#[test]
fn sky_shader_is_valid_wgsl() {
    let shader = naga::front::wgsl::parse_str(crate::SKY_SHADER).unwrap();
    naga::valid::Validator::new(
        naga::valid::ValidationFlags::all(),
        naga::valid::Capabilities::all(),
    )
    .validate(&shader)
    .unwrap();
}

#[test]
#[ignore = "requires a GPU adapter"]
fn renders_reported_cloud_layer_offscreen() {
    pollster::block_on(async {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::PRIMARY,
            ..wgpu::InstanceDescriptor::new_without_display_handle()
        });
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions::default())
            .await
            .unwrap();
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor::default())
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
        let pipeline = pipeline(&device, &layout, wgpu::TextureFormat::Rgba8UnormSrgb, 1);
        let camera = crate::Camera::looking_at(
            glam::DVec3::new(0.0, 50.0, 0.0),
            glam::DVec3::new(0.0, 0.3, 1.0) + glam::DVec3::new(0.0, 50.0, 0.0),
        );
        let matrix = camera.view_projection(2.0);
        let mut uniform = CameraUniform {
            view_projection: matrix.to_cols_array_2d(),
            inverse_view_projection: matrix.inverse().to_cols_array_2d(),
            sun: [0.2, 0.85, 0.48, 50.0],
            lighting: [0.0, 100_000.0, 0.0, 0.0],
            clouds: [[0.0; 4]; 3],
            world: [0.0; 4],
        };
        let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: None,
            contents: bytemuck::bytes_of(&uniform),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
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
        let mut images = Vec::new();
        for phase in 0..3 {
            if phase > 0 {
                uniform.clouds[0] = [660.0, 600.0, 0.75, 0.0];
            }
            if phase == 2 {
                uniform.world[0] = -2000.0;
            }
            queue.write_buffer(&buffer, 0, bytemuck::bytes_of(&uniform));
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
                pass.set_pipeline(&pipeline);
                pass.set_bind_group(0, &bind, &[]);
                pass.draw(0..3, 0..1);
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
        let changed = images[0]
            .as_chunks::<4>()
            .0
            .iter()
            .zip(images[1].as_chunks::<4>().0.iter())
            .filter(|(a, b)| a[..3].iter().zip(&b[..3]).any(|(a, b)| a.abs_diff(*b) > 10))
            .count();
        assert!(
            changed > 1000,
            "reported cloud layer did not change the sky: {changed} pixels"
        );
        let moved = images[1]
            .as_chunks::<4>()
            .0
            .iter()
            .zip(images[2].as_chunks::<4>().0.iter())
            .filter(|(a, b)| a[..3].iter().zip(&b[..3]).any(|(a, b)| a.abs_diff(*b) > 10))
            .count();
        assert!(
            moved > 1000,
            "cloud advection did not change the sky: {moved} pixels"
        );
        image::save_buffer(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../target/metar-clouds-headless.png"),
            &images[1],
            256,
            128,
            image::ColorType::Rgba8,
        )
        .unwrap();
        image::save_buffer(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../target/metar-clouds-moved-headless.png"),
            &images[2],
            256,
            128,
            image::ColorType::Rgba8,
        )
        .unwrap();
    });
}

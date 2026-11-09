use wgpu::{Device, RenderPipeline, TextureFormat};

#[cfg(test)]
mod tests;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub(super) struct CameraUniform {
    pub view_projection: [[f32; 4]; 4],
    pub inverse_view_projection: [[f32; 4]; 4],
    pub sun: [f32; 4],
    pub lighting: [f32; 4],
    pub clouds: [[f32; 4]; 3],
    pub world: [f32; 4],
}

#[derive(Clone, Copy, Debug)]
pub struct CloudLayer {
    pub base_msl: f64,
    pub coverage: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Weather {
    pub visibility_m: f64,
    pub origin_msl: f64,
    pub clouds: [Option<CloudLayer>; 3],
}

pub(super) fn uniform(
    camera: &crate::Camera,
    matrix: glam::Mat4,
    sun: glam::Vec3,
    panel_lights: bool,
    weather: Weather,
) -> CameraUniform {
    CameraUniform {
        view_projection: matrix.to_cols_array_2d(),
        inverse_view_projection: matrix.inverse().to_cols_array_2d(),
        sun: [
            sun.x,
            sun.y,
            sun.z,
            (camera.position.y + weather.origin_msl).max(0.0) as f32,
        ],
        lighting: [
            f32::from(panel_lights),
            if weather.visibility_m.is_finite() {
                weather.visibility_m.clamp(1.0, 1_000_000.0) as f32
            } else {
                100_000.0
            },
            0.0,
            0.0,
        ],
        clouds: weather.clouds.map(|layer| {
            layer
                .filter(|layer| layer.base_msl.is_finite() && layer.coverage.is_finite())
                .map_or([0.0; 4], |layer| {
                    [
                        layer.base_msl as f32,
                        600.0,
                        layer.coverage.clamp(0.0, 1.0) as f32,
                        0.0,
                    ]
                })
        }),
        world: [camera.position.x as f32, camera.position.z as f32, 0.0, 0.0],
    }
}

pub(super) fn pipeline(
    device: &Device,
    camera_layout: &wgpu::BindGroupLayout,
    format: TextureFormat,
    samples: u32,
) -> RenderPipeline {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("atmospheric scattering"),
        source: wgpu::ShaderSource::Wgsl(crate::SKY_SHADER.into()),
    });
    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("sky"),
        bind_group_layouts: &[Some(camera_layout)],
        immediate_size: 0,
    });
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("sky"),
        layout: Some(&layout),
        vertex: wgpu::VertexState {
            module: &shader,
            entry_point: Some("vs_main"),
            compilation_options: Default::default(),
            buffers: &[],
        },
        primitive: wgpu::PrimitiveState::default(),
        depth_stencil: Some(wgpu::DepthStencilState {
            format: TextureFormat::Depth32Float,
            depth_write_enabled: Some(false),
            depth_compare: Some(wgpu::CompareFunction::Always),
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
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    })
}

use anyhow::{Result, bail};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(super) struct PipelineKey {
    pub source: u32,
    pub destination: u32,
    pub depth_write: bool,
    pub double_sided: bool,
}

impl PipelineKey {
    pub fn new(source: u32, destination: u32, flags: u32, flags2: u32) -> Self {
        let opaque = source == 2 && destination == 1;
        Self {
            source,
            destination,
            depth_write: flags & 0x20000 == 0 && (opaque || flags & 0x10000 != 0),
            double_sided: flags2 & 8 != 0,
        }
    }

    pub fn blend(self) -> Result<Option<wgpu::BlendState>> {
        let factor = |value| {
            Ok(match value {
                1 => wgpu::BlendFactor::Zero,
                2 => wgpu::BlendFactor::One,
                3 => wgpu::BlendFactor::Src,
                4 => wgpu::BlendFactor::OneMinusSrc,
                5 => wgpu::BlendFactor::SrcAlpha,
                6 => wgpu::BlendFactor::OneMinusSrcAlpha,
                7 => wgpu::BlendFactor::DstAlpha,
                8 => wgpu::BlendFactor::OneMinusDstAlpha,
                9 => wgpu::BlendFactor::Dst,
                10 => wgpu::BlendFactor::OneMinusDst,
                _ => bail!("unsupported FSX material blend factor {value}"),
            })
        };
        let component = wgpu::BlendComponent {
            src_factor: factor(self.source)?,
            dst_factor: factor(self.destination)?,
            operation: wgpu::BlendOperation::Add,
        };
        Ok(
            (self.source != 2 || self.destination != 1).then_some(wgpu::BlendState {
                color: component,
                alpha: component,
            }),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn preserves_colour_blending_and_depth_flags() {
        let key = PipelineKey::new(5, 4, 0, 0);
        let blend = key.blend().unwrap().unwrap();
        assert_eq!(blend.color.src_factor, wgpu::BlendFactor::SrcAlpha);
        assert_eq!(blend.color.dst_factor, wgpu::BlendFactor::OneMinusSrc);
        assert!(!key.depth_write);
        assert!(PipelineKey::new(5, 6, 0x10000, 8).depth_write);
        assert!(PipelineKey::new(5, 6, 0, 8).double_sided);
        assert!(!PipelineKey::new(2, 1, 0x20000, 0).depth_write);
        assert!(PipelineKey::new(2, 1, 0, 0).blend().unwrap().is_none());
        assert!(PipelineKey::new(99, 1, 0, 0).blend().is_err());
    }
}

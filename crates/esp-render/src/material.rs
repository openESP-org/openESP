use anyhow::{Result, bail};
use esp_assets::texture;
use std::{collections::HashMap, path::Path};

pub(super) fn linear_texture(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    aircraft: &esp_assets::Aircraft,
    root: Option<&Path>,
    compressed: bool,
    cache: &mut HashMap<String, (wgpu::TextureView, bool, bool)>,
    name: Option<&str>,
) -> Result<Option<(wgpu::TextureView, bool, bool)>> {
    let Some(name) = name else { return Ok(None) };
    let key = format!(
        "linear|{}|{name}",
        root.map_or_else(String::new, |p| p.to_string_lossy().into_owned())
    );
    if let Some((view, flip, alpha)) = cache.get(&key) {
        return Ok(Some((view.clone(), *flip, *alpha)));
    }
    let resolved = if let Some(root) = root {
        esp_assets::resolve_path(root, name).map(Some)
    } else {
        aircraft.texture_path(name)
    };
    match resolved {
        Ok(Some(path)) => {
            let image = texture::Texture::load(&path, compressed)?;
            let flip = image.flip_v;
            let alpha = image.normal_x_in_alpha;
            let view = super::upload_texture_with_color_space(device, queue, &image, false)?
                .create_view(&Default::default());
            cache.insert(key, (view.clone(), flip, alpha));
            Ok(Some((view, flip, alpha)))
        }
        Ok(None) => Ok(None),
        Err(error) => {
            eprintln!("material data texture: {error}");
            Ok(None)
        }
    }
}

pub(super) fn texture_format(format: texture::Format, srgb: bool) -> wgpu::TextureFormat {
    match (format, srgb) {
        (texture::Format::Rgba8, true) => wgpu::TextureFormat::Rgba8UnormSrgb,
        (texture::Format::Bc1, true) => wgpu::TextureFormat::Bc1RgbaUnormSrgb,
        (texture::Format::Bc2, true) => wgpu::TextureFormat::Bc2RgbaUnormSrgb,
        (texture::Format::Bc3, true) => wgpu::TextureFormat::Bc3RgbaUnormSrgb,
        (texture::Format::Rgba8, false) => wgpu::TextureFormat::Rgba8Unorm,
        (texture::Format::Bc1, false) => wgpu::TextureFormat::Bc1RgbaUnorm,
        (texture::Format::Bc2, false) => wgpu::TextureFormat::Bc2RgbaUnorm,
        (texture::Format::Bc3, false) => wgpu::TextureFormat::Bc3RgbaUnorm,
    }
}

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
    fn color_textures_and_material_data_use_different_transfer_functions() {
        for format in [
            texture::Format::Rgba8,
            texture::Format::Bc1,
            texture::Format::Bc2,
            texture::Format::Bc3,
        ] {
            assert!(texture_format(format, true).is_srgb());
            assert!(!texture_format(format, false).is_srgb());
        }
    }
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

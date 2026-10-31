use anyhow::{Context, Result, bail};
use ddsfile::{D3DFormat, FourCC};
use esp_formats::texture::{bmp, dds};
use std::path::Path;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    Rgba8,
    Bc1,
    Bc2,
    Bc3,
}

pub struct Mip {
    pub width: u32,
    pub height: u32,
    pub data: Vec<u8>,
}

pub struct Texture {
    pub format: Format,
    pub mips: Vec<Mip>,
    pub flip_v: bool,
}

impl Texture {
    pub fn load(path: &Path, compressed: bool) -> Result<Self> {
        let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
        Self::from_bytes(&bytes, compressed)
            .with_context(|| format!("loading texture {}", path.display()))
    }

    pub fn from_bytes(bytes: &[u8], compressed: bool) -> Result<Self> {
        if bytes.starts_with(b"DDS ") {
            let texture = dds::Texture::parse(bytes)?;
            if texture.layer_count() != 1 || texture.header().depth.unwrap_or(1) != 1 {
                bail!("expected a 2D texture");
            }
            let format = match texture.header().spf.fourcc.as_ref().map(|f| f.0) {
                Some(FourCC::DXT1) => Some(Format::Bc1),
                Some(FourCC::DXT3) => Some(Format::Bc2),
                Some(FourCC::DXT5) => Some(Format::Bc3),
                _ => None,
            };
            if compressed && let Some(format) = format {
                let mips = (0..texture.mip_count())
                    .map(|mip| {
                        let surface = texture.surface(0, mip).context("missing DDS mip")?;
                        Ok(Mip {
                            width: surface.width,
                            height: surface.height,
                            data: surface.data.to_vec(),
                        })
                    })
                    .collect::<Result<Vec<_>>>()?;
                return Ok(Self {
                    format,
                    mips,
                    flip_v: false,
                });
            }
            let mut decoder = dds_codec::Decoder::new(std::io::Cursor::new(bytes))?;
            let size = decoder.main_size();
            let length = u64::from(size.width) * u64::from(size.height) * 4;
            anyhow::ensure!(
                length <= 512 * 1024 * 1024,
                "decoded texture exceeds 512 MiB"
            );
            let mut data = Vec::new();
            data.try_reserve_exact(length as usize)?;
            data.resize(length as usize, 0);
            let view =
                dds_codec::ImageViewMut::new(&mut data, size, dds_codec::ColorFormat::RGBA_U8)
                    .context("invalid DDS output dimensions")?;
            decoder.read_surface(view)?;
            return Ok(Self::rgba(size.width, size.height, data, false));
        }
        let bitmap = bmp::Bitmap::parse(bytes)?;
        if let Some(surface) = bitmap.compressed_surface()? {
            let format = match surface.format {
                D3DFormat::DXT1 => Format::Bc1,
                D3DFormat::DXT3 => Format::Bc2,
                D3DFormat::DXT5 => Format::Bc3,
                _ => bail!("unsupported compressed BMP"),
            };
            if !compressed {
                bail!("compressed BMP requires GPU BC texture support");
            }
            return Ok(Self {
                format,
                mips: vec![Mip {
                    width: surface.width,
                    height: surface.stored_height.unsigned_abs(),
                    data: surface.data.to_vec(),
                }],
                flip_v: true,
            });
        }
        let image = bitmap.decode(image::Limits::default())?.into_rgba8();
        Ok(Self::rgba(
            image.width(),
            image.height(),
            image.into_raw(),
            true,
        ))
    }

    pub fn rgba(width: u32, height: u32, data: Vec<u8>, flip_v: bool) -> Self {
        Self {
            format: Format::Rgba8,
            mips: vec![Mip {
                width,
                height,
                data,
            }],
            flip_v,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn dds(format: D3DFormat, width: u32, height: u32) -> ddsfile::Dds {
        ddsfile::Dds::new_d3d(ddsfile::NewD3dParams {
            width,
            height,
            depth: None,
            format,
            mipmap_levels: None,
            caps2: None,
        })
        .unwrap()
    }
    fn bytes(dds: &ddsfile::Dds) -> Vec<u8> {
        let mut bytes = Vec::new();
        dds.write(&mut bytes).unwrap();
        bytes
    }
    #[test]
    fn decodes_uncompressed_bgra_and_preserves_alpha() {
        let mut dds = dds(D3DFormat::A8R8G8B8, 1, 1);
        dds.data.copy_from_slice(&[30, 20, 10, 40]);
        let bytes = bytes(&dds);
        let texture = Texture::from_bytes(&bytes, true).unwrap();
        assert_eq!(texture.format, Format::Rgba8);
        assert_eq!(texture.mips[0].data, [10, 20, 30, 40]);
        assert!(!texture.flip_v);
        for end in 0..bytes.len() {
            assert!(Texture::from_bytes(&bytes[..end], true).is_err());
        }
    }
    #[test]
    fn retains_bc_blocks_or_decodes_them_on_request() {
        let mut dds = dds(D3DFormat::DXT1, 4, 4);
        dds.data.copy_from_slice(&[0, 248, 0, 0, 0, 0, 0, 0]);
        let bytes = bytes(&dds);
        let compressed = Texture::from_bytes(&bytes, true).unwrap();
        assert_eq!(compressed.format, Format::Bc1);
        assert_eq!(compressed.mips[0].data, dds.data);
        let decoded = Texture::from_bytes(&bytes, false).unwrap();
        assert!(
            decoded.mips[0]
                .data
                .as_chunks::<4>()
                .0
                .iter()
                .all(|p| p == &[255, 0, 0, 255])
        );
    }
}

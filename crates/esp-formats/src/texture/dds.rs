use ddsfile::{
    Caps2, D3D10ResourceDimension, D3DFormat, DataFormat, DxgiFormat, FourCC, Header, Header10,
    MiscFlag,
};
use std::io::Cursor;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("invalid DDS header: {0}")]
    Header(#[from] ddsfile::Error),
    #[error("invalid DDS dimensions, layer count, or mipmap count")]
    Dimensions,
    #[error("unsupported DDS pixel layout")]
    UnsupportedFormat,
    #[error("DDS surface sizes overflow")]
    SizeOverflow,
    #[error("truncated DDS surfaces: need {expected} bytes, found {actual}")]
    Truncated { expected: u64, actual: usize },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CubeFace {
    PositiveX,
    NegativeX,
    PositiveY,
    NegativeY,
    PositiveZ,
    NegativeZ,
}

const FACES: [(CubeFace, Caps2); 6] = [
    (CubeFace::PositiveX, Caps2::CUBEMAP_POSITIVEX),
    (CubeFace::NegativeX, Caps2::CUBEMAP_NEGATIVEX),
    (CubeFace::PositiveY, Caps2::CUBEMAP_POSITIVEY),
    (CubeFace::NegativeY, Caps2::CUBEMAP_NEGATIVEY),
    (CubeFace::PositiveZ, Caps2::CUBEMAP_POSITIVEZ),
    (CubeFace::NegativeZ, Caps2::CUBEMAP_NEGATIVEZ),
];

#[derive(Debug)]
struct Mip {
    width: u32,
    height: u32,
    depth: u32,
    row_pitch: usize,
    slice_pitch: usize,
    offset: usize,
    size: usize,
}

#[derive(Debug)]
pub struct Surface<'a> {
    pub width: u32,
    pub height: u32,
    pub depth: u32,
    pub row_pitch: usize,
    pub slice_pitch: usize,
    pub data: &'a [u8],
}

#[derive(Debug)]
pub struct Texture<'a> {
    header: Header,
    header10: Option<Header10>,
    raw_header: &'a [u8],
    data: &'a [u8],
    trailing_data: &'a [u8],
    mips: Vec<Mip>,
    stride: usize,
    layer_count: u32,
    faces: Vec<CubeFace>,
}

impl<'a> Texture<'a> {
    pub fn parse(bytes: &'a [u8]) -> Result<Self, Error> {
        if bytes.get(..4) != Some(b"DDS ") {
            return Err(ddsfile::Error::BadMagicNumber.into());
        }
        let mut cursor = Cursor::new(bytes);
        cursor.set_position(4);
        let header = Header::read(&mut cursor)?;
        let header10 = if header.spf.fourcc == Some(FourCC(FourCC::DX10)) {
            Some(Header10::read(&mut cursor)?)
        } else {
            None
        };
        let (arrays, volume, cube) = if let Some(h) = &header10 {
            let cube = h.misc_flag.contains(MiscFlag::TEXTURECUBE);
            let volume = h.resource_dimension == D3D10ResourceDimension::Texture3D;
            if h.array_size == 0
                || (volume && h.array_size != 1)
                || (cube && h.resource_dimension != D3D10ResourceDimension::Texture2D)
                || (h.resource_dimension == D3D10ResourceDimension::Texture1D && header.height != 1)
                || matches!(
                    h.resource_dimension,
                    D3D10ResourceDimension::Unknown | D3D10ResourceDimension::Buffer
                )
            {
                return Err(Error::Dimensions);
            }
            (h.array_size, volume, cube)
        } else {
            (
                1,
                header.caps2.contains(Caps2::VOLUME),
                header.caps2.contains(Caps2::CUBEMAP),
            )
        };
        let faces: Vec<_> = if cube {
            FACES
                .iter()
                .filter(|(_, flag)| header10.is_some() || header.caps2.contains(*flag))
                .map(|(face, _)| *face)
                .collect()
        } else {
            Vec::new()
        };
        let depth = if volume { header.depth.unwrap_or(0) } else { 1 };
        let count = header.mip_map_count.unwrap_or(1);
        let largest = header.width.max(header.height).max(depth);
        if header.width == 0
            || header.height == 0
            || depth == 0
            || count == 0
            || count > u32::BITS - largest.leading_zeros()
            || (cube && (volume || faces.is_empty() || header.width != header.height))
            || (!volume && header.depth.is_some_and(|d| d > 1))
        {
            return Err(Error::Dimensions);
        }
        let layer_count = arrays
            .checked_mul(if cube { faces.len() as u32 } else { 1 })
            .ok_or(Error::SizeOverflow)?;
        let layout = if let Some(h) = &header10 {
            match h.dxgi_format {
                DxgiFormat::R8G8_B8G8_UNorm | DxgiFormat::G8R8_G8B8_UNorm | DxgiFormat::YUY2 => {
                    Layout::Packed(4)
                }
                DxgiFormat::Y210 | DxgiFormat::Y216 => Layout::Packed(8),
                format => Layout::from_format(&format)?,
            }
        } else {
            match D3DFormat::try_from_pixel_format(&header.spf).ok_or(Error::UnsupportedFormat)? {
                D3DFormat::R8G8_B8G8 | D3DFormat::G8R8_G8B8 | D3DFormat::UYVY | D3DFormat::YUY2 => {
                    Layout::Packed(4)
                }
                format => Layout::from_format(&format)?,
            }
        };
        let mut mips = Vec::new();
        let mut stride = 0u64;
        for level in 0..count {
            let width = (header.width >> level).max(1);
            let height = (header.height >> level).max(1);
            let depth = (depth >> level).max(1);
            let (row, rows) = layout.pitch(width, height);
            let slice = row.checked_mul(rows).ok_or(Error::SizeOverflow)?;
            let size = slice
                .checked_mul(u64::from(depth))
                .ok_or(Error::SizeOverflow)?;
            mips.push(Mip {
                width,
                height,
                depth,
                row_pitch: as_usize(row)?,
                slice_pitch: as_usize(slice)?,
                offset: as_usize(stride)?,
                size: as_usize(size)?,
            });
            stride = stride.checked_add(size).ok_or(Error::SizeOverflow)?;
        }
        let expected = stride
            .checked_mul(u64::from(layer_count))
            .ok_or(Error::SizeOverflow)?;
        let start = cursor.position() as usize;
        let data = &bytes[start..];
        if expected > data.len() as u64 {
            return Err(Error::Truncated {
                expected,
                actual: data.len(),
            });
        }
        let (data, trailing_data) = data.split_at(as_usize(expected)?);
        Ok(Self {
            header,
            header10,
            raw_header: &bytes[..start],
            data,
            trailing_data,
            mips,
            stride: as_usize(stride)?,
            layer_count,
            faces,
        })
    }

    pub fn header(&self) -> &Header {
        &self.header
    }
    pub fn header10(&self) -> Option<&Header10> {
        self.header10.as_ref()
    }
    pub fn raw_header(&self) -> &'a [u8] {
        self.raw_header
    }
    pub fn trailing_data(&self) -> &'a [u8] {
        self.trailing_data
    }
    pub fn mip_count(&self) -> u32 {
        self.mips.len() as u32
    }
    pub fn layer_count(&self) -> u32 {
        self.layer_count
    }

    /// Cubemap faces occupy consecutive layers within each array element.
    pub fn layer(&self, index: u32) -> Option<(u32, Option<CubeFace>)> {
        if index >= self.layer_count {
            return None;
        }
        if self.faces.is_empty() {
            Some((index, None))
        } else {
            let count = self.faces.len() as u32;
            Some((index / count, Some(self.faces[(index % count) as usize])))
        }
    }

    pub fn surface(&self, layer: u32, mip: u32) -> Option<Surface<'a>> {
        self.layer(layer)?;
        let mip = self.mips.get(mip as usize)?;
        let start = layer as usize * self.stride + mip.offset;
        Some(Surface {
            width: mip.width,
            height: mip.height,
            depth: mip.depth,
            row_pitch: mip.row_pitch,
            slice_pitch: mip.slice_pitch,
            data: &self.data[start..start + mip.size],
        })
    }
}

fn as_usize(value: u64) -> Result<usize, Error> {
    usize::try_from(value).map_err(|_| Error::SizeOverflow)
}

enum Layout {
    Block(u32),
    Pixels(u8),
    Packed(u32),
}

impl Layout {
    fn from_format(format: &dyn DataFormat) -> Result<Self, Error> {
        if let Some(size) = format.get_block_size() {
            Ok(Self::Block(size))
        } else {
            format
                .get_bits_per_pixel()
                .filter(|bits| *bits > 0)
                .map(Self::Pixels)
                .ok_or(Error::UnsupportedFormat)
        }
    }

    fn pitch(&self, width: u32, height: u32) -> (u64, u64) {
        let (width, height) = (u64::from(width), u64::from(height));
        match *self {
            Self::Block(size) => (width.div_ceil(4) * u64::from(size), height.div_ceil(4)),
            Self::Pixels(bits) => ((width * u64::from(bits)).div_ceil(8), height),
            Self::Packed(size) => (width.div_ceil(2) * u64::from(size), height),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn word(bytes: &mut [u8], offset: usize, value: u32) {
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn dxt1(width: u32, height: u32, mips: u32, size: usize) -> Vec<u8> {
        let mut bytes = vec![0; 128 + size];
        bytes[..4].copy_from_slice(b"DDS ");
        word(&mut bytes, 4, 124);
        word(&mut bytes, 8, 0x000a1007);
        word(&mut bytes, 12, height);
        word(&mut bytes, 16, width);
        word(&mut bytes, 28, mips);
        word(&mut bytes, 76, 32);
        word(&mut bytes, 80, 4);
        bytes[84..88].copy_from_slice(b"DXT1");
        word(&mut bytes, 108, 0x401008);
        bytes
    }

    fn dx10(width: u32, height: u32, mips: u32, format: u32, size: usize) -> Vec<u8> {
        let mut bytes = dxt1(width, height, mips, 20 + size);
        bytes[84..88].copy_from_slice(b"DX10");
        word(&mut bytes, 128, format);
        word(&mut bytes, 132, 3);
        word(&mut bytes, 140, 1);
        bytes
    }

    #[test]
    fn reads_rectangular_mips_without_copying() {
        let mut bytes = dxt1(8, 4, 4, 40);
        bytes[144..152].fill(7);
        bytes.extend([91, 92]);
        let texture = Texture::parse(&bytes).unwrap();
        assert_eq!(texture.mip_count(), 4);
        assert_eq!(texture.layer_count(), 1);
        let mip = texture.surface(0, 1).unwrap();
        assert_eq!((mip.width, mip.height, mip.depth), (4, 2, 1));
        assert_eq!((mip.row_pitch, mip.slice_pitch), (8, 8));
        assert_eq!(mip.data, &[7; 8]);
        assert_eq!(mip.data.as_ptr(), bytes[144..].as_ptr());
        assert_eq!(texture.surface(0, 3).unwrap().data.len(), 8);
        assert!(texture.surface(1, 0).is_none());
        assert!(texture.surface(0, 4).is_none());
        assert_eq!(texture.trailing_data(), &[91, 92]);
        assert_eq!(texture.raw_header(), &bytes[..128]);
    }

    #[test]
    fn reads_sparse_cubemap_faces() {
        let mut bytes = dxt1(4, 4, 1, 24);
        word(&mut bytes, 112, 0x200 | 0x400 | 0x2000 | 0x4000);
        bytes[136..144].fill(5);
        let texture = Texture::parse(&bytes).unwrap();
        assert_eq!(texture.layer_count(), 3);
        assert_eq!(texture.layer(0), Some((0, Some(CubeFace::PositiveX))));
        assert_eq!(texture.layer(1), Some((0, Some(CubeFace::NegativeY))));
        assert_eq!(texture.layer(2), Some((0, Some(CubeFace::PositiveZ))));
        assert_eq!(texture.surface(1, 0).unwrap().data, &[5; 8]);
    }

    #[test]
    fn reads_dx10_cube_arrays() {
        let mut bytes = dx10(4, 4, 1, 98, 12 * 16);
        word(&mut bytes, 136, 4);
        word(&mut bytes, 140, 2);
        bytes[148 + 11 * 16..].fill(11);
        let texture = Texture::parse(&bytes).unwrap();
        assert_eq!(texture.layer_count(), 12);
        assert_eq!(texture.layer(6), Some((1, Some(CubeFace::PositiveX))));
        assert_eq!(texture.layer(11), Some((1, Some(CubeFace::NegativeZ))));
        assert_eq!(texture.surface(11, 0).unwrap().data, &[11; 16]);
        assert_eq!(
            texture.header10().unwrap().dxgi_format,
            DxgiFormat::BC7_UNorm
        );
    }

    #[test]
    fn reads_volume_depth_at_each_mip() {
        let mut bytes = dx10(4, 4, 3, 61, 64 + 8 + 1);
        word(&mut bytes, 8, 0x0082100f);
        word(&mut bytes, 24, 4);
        word(&mut bytes, 112, 0x200000);
        word(&mut bytes, 132, 4);
        let texture = Texture::parse(&bytes).unwrap();
        let mip = texture.surface(0, 1).unwrap();
        assert_eq!((mip.width, mip.height, mip.depth), (2, 2, 2));
        assert_eq!((mip.row_pitch, mip.slice_pitch, mip.data.len()), (2, 4, 8));
        assert_eq!(texture.surface(0, 2).unwrap().data.len(), 1);
    }

    #[test]
    fn packed_rows_round_up_pixel_pairs() {
        let bytes = dx10(3, 2, 1, 107, 16);
        let texture = Texture::parse(&bytes).unwrap();
        assert_eq!(texture.surface(0, 0).unwrap().row_pitch, 8);
    }

    #[test]
    fn reads_legacy_color_and_alpha_blocks() {
        for fourcc in [b"DXT3", b"DXT5"] {
            let mut bytes = dxt1(4, 4, 3, 48);
            bytes[84..88].copy_from_slice(fourcc);
            let texture = Texture::parse(&bytes).unwrap();
            assert_eq!(texture.surface(0, 2).unwrap().data.len(), 16);
        }
        let mut bytes = dxt1(3, 2, 2, 28);
        word(&mut bytes, 80, 0x41);
        word(&mut bytes, 84, 0);
        word(&mut bytes, 88, 32);
        word(&mut bytes, 92, 0x00ff0000);
        word(&mut bytes, 96, 0x0000ff00);
        word(&mut bytes, 100, 0x000000ff);
        word(&mut bytes, 104, 0xff000000);
        let texture = Texture::parse(&bytes).unwrap();
        let mip = texture.surface(0, 0).unwrap();
        assert_eq!((mip.row_pitch, mip.slice_pitch), (12, 24));
        assert_eq!(texture.surface(0, 1).unwrap().data.len(), 4);
    }

    #[test]
    fn rejects_truncation_and_invalid_dimensions() {
        let bytes = dxt1(8, 4, 4, 40);
        for end in 0..bytes.len() {
            assert!(Texture::parse(&bytes[..end]).is_err(), "length {end}");
        }
        for (offset, value) in [(12, 0), (16, 0), (28, 0), (28, 33), (16, u32::MAX)] {
            let mut invalid = bytes.clone();
            word(&mut invalid, offset, value);
            assert!(Texture::parse(&invalid).is_err());
        }
        let mut bytes = dx10(4, 4, 1, 98, 16);
        word(&mut bytes, 140, u32::MAX);
        word(&mut bytes, 136, 4);
        assert!(matches!(Texture::parse(&bytes), Err(Error::SizeOverflow)));
        word(&mut bytes, 140, 0);
        assert!(matches!(Texture::parse(&bytes), Err(Error::Dimensions)));
    }

    #[test]
    fn rejects_unsupported_planar_format() {
        let bytes = dx10(4, 4, 1, 103, 24);
        assert!(matches!(
            Texture::parse(&bytes),
            Err(Error::UnsupportedFormat)
        ));
    }
}

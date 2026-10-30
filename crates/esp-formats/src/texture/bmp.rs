use binrw::BinRead;
use image::{DynamicImage, ImageDecoder, Limits, codecs::bmp::BmpDecoder};
use std::io::Cursor;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("invalid BMP header: {0}")]
    Header(#[from] binrw::Error),
    #[error("invalid BMP file size, header size, or pixel offset")]
    Layout,
    #[error("BMP decoding failed: {0}")]
    Decode(#[from] image::ImageError),
}

#[derive(Debug, Clone, PartialEq, Eq, BinRead)]
#[br(little, magic = b"BM")]
pub struct FileHeader {
    pub size: u32,
    pub reserved: [u16; 2],
    pub pixel_offset: u32,
}

#[derive(Debug)]
pub struct Bitmap<'a> {
    header: FileHeader,
    data: &'a [u8],
    dib_header: &'a [u8],
    metadata: &'a [u8],
    pixels: &'a [u8],
    trailing_data: &'a [u8],
}

impl<'a> Bitmap<'a> {
    pub fn parse(bytes: &'a [u8]) -> Result<Self, Error> {
        let mut cursor = Cursor::new(bytes);
        let header = FileHeader::read(&mut cursor)?;
        let dib_size = u32::read_le(&mut cursor)?;
        let size = usize::try_from(header.size).map_err(|_| Error::Layout)?;
        let offset = usize::try_from(header.pixel_offset).map_err(|_| Error::Layout)?;
        let dib_end = usize::try_from(dib_size)
            .ok()
            .and_then(|size| 14usize.checked_add(size))
            .ok_or(Error::Layout)?;
        if dib_size < 12 || dib_end > offset || offset > size || size > bytes.len() {
            return Err(Error::Layout);
        }
        Ok(Self {
            header,
            data: &bytes[..size],
            dib_header: &bytes[14..dib_end],
            metadata: &bytes[dib_end..offset],
            pixels: &bytes[offset..size],
            trailing_data: &bytes[size..],
        })
    }

    pub fn header(&self) -> &FileHeader {
        &self.header
    }

    pub fn dib_header(&self) -> &'a [u8] {
        self.dib_header
    }

    pub fn metadata(&self) -> &'a [u8] {
        self.metadata
    }

    pub fn pixels(&self) -> &'a [u8] {
        self.pixels
    }

    pub fn trailing_data(&self) -> &'a [u8] {
        self.trailing_data
    }

    pub fn decode(&self, mut limits: Limits) -> Result<DynamicImage, Error> {
        let mut decoder = BmpDecoder::new(Cursor::new(self.data))?;
        decoder.set_limits(limits.clone())?;
        limits.reserve(decoder.total_bytes())?;
        Ok(DynamicImage::from_decoder(decoder)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bitmap(
        width: i32,
        height: i32,
        bits: u16,
        compression: u32,
        metadata: &[u8],
        pixels: &[u8],
    ) -> Vec<u8> {
        let mut bytes = b"BM".to_vec();
        bytes.extend_from_slice(&(54 + metadata.len() as u32 + pixels.len() as u32).to_le_bytes());
        bytes.extend_from_slice(&[0; 4]);
        bytes.extend_from_slice(&(54 + metadata.len() as u32).to_le_bytes());
        bytes.extend_from_slice(&40u32.to_le_bytes());
        bytes.extend_from_slice(&width.to_le_bytes());
        bytes.extend_from_slice(&height.to_le_bytes());
        bytes.extend_from_slice(&1u16.to_le_bytes());
        bytes.extend_from_slice(&bits.to_le_bytes());
        bytes.extend_from_slice(&compression.to_le_bytes());
        bytes.extend_from_slice(&(pixels.len() as u32).to_le_bytes());
        bytes.extend_from_slice(&[0; 8]);
        bytes.extend_from_slice(
            &(if bits <= 8 {
                metadata.len() as u32 / 4
            } else {
                0
            })
            .to_le_bytes(),
        );
        bytes.extend_from_slice(&0u32.to_le_bytes());
        bytes.extend_from_slice(metadata);
        bytes.extend_from_slice(pixels);
        bytes
    }

    #[test]
    fn decodes_padded_rows_and_both_orientations() {
        for height in [-2, 2] {
            let bytes = bitmap(1, height, 24, 0, &[], &[0, 0, 255, 99, 0, 255, 0, 99]);
            let bmp = Bitmap::parse(&bytes).unwrap();
            let image = bmp.decode(Limits::default()).unwrap().to_rgb8();
            assert_eq!(image.dimensions(), (1, 2));
            assert_eq!(
                image.as_raw(),
                if height < 0 {
                    &[255, 0, 0, 0, 255, 0]
                } else {
                    &[0, 255, 0, 255, 0, 0]
                }
            );
            assert_eq!(bmp.dib_header().len(), 40);
            assert!(bmp.metadata().is_empty());
        }
    }

    #[test]
    fn decodes_palettes_bitfields_and_rle() {
        let palette = [0, 0, 255, 0, 0, 255, 0, 255];
        for (bits, compression, pixels) in [
            (1, 0, &[0b01000000, 0, 0, 0][..]),
            (4, 0, &[0x01, 0, 0, 0][..]),
            (8, 0, &[0, 1, 0, 0][..]),
            (8, 1, &[1, 0, 1, 1, 0, 0, 0, 1][..]),
            (4, 2, &[2, 0x01, 0, 0, 0, 1][..]),
        ] {
            let bytes = bitmap(2, 1, bits, compression, &palette, pixels);
            let bmp = Bitmap::parse(&bytes).unwrap();
            assert_eq!(bmp.metadata(), palette);
            assert_eq!(
                bmp.decode(Limits::default()).unwrap().to_rgb8().as_raw(),
                &[255, 0, 0, 0, 255, 0]
            );
        }
        let masks: Vec<_> = [0xf800u32, 0x07e0, 0x001f]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect();
        let bytes = bitmap(2, 1, 16, 3, &masks, &[0, 248, 224, 7]);
        assert_eq!(
            Bitmap::parse(&bytes)
                .unwrap()
                .decode(Limits::default())
                .unwrap()
                .to_rgb8()
                .as_raw(),
            &[255, 0, 0, 0, 255, 0]
        );
    }

    #[test]
    fn bounds_files_and_decoded_allocations() {
        let bytes = bitmap(1, 1, 24, 0, &[], &[0, 0, 0, 0]);
        for end in 0..bytes.len() {
            assert!(Bitmap::parse(&bytes[..end]).is_err());
        }
        for (field, value) in [(2, 0u32), (10, 53), (10, u32::MAX), (14, u32::MAX)] {
            let mut corrupt = bytes.clone();
            corrupt[field..field + 4].copy_from_slice(&value.to_le_bytes());
            assert!(Bitmap::parse(&corrupt).is_err());
        }
        let mut limits = Limits::default();
        limits.max_alloc = Some(2);
        assert!(matches!(
            Bitmap::parse(&bytes).unwrap().decode(limits),
            Err(Error::Decode(image::ImageError::Limits(_)))
        ));
        let mut limits = Limits::default();
        limits.max_image_width = Some(0);
        assert!(Bitmap::parse(&bytes).unwrap().decode(limits).is_err());
        let mut trailing = bytes.clone();
        trailing.extend_from_slice(b"extra");
        let bmp = Bitmap::parse(&trailing).unwrap();
        assert_eq!(bmp.trailing_data(), b"extra");
        assert_eq!(bmp.pixels(), &[0, 0, 0, 0]);
        let bytes = bitmap(1, 1, 8, 1, &[0; 8], &[0]);
        assert!(
            Bitmap::parse(&bytes)
                .unwrap()
                .decode(Limits::default())
                .is_err()
        );
        let bytes = bitmap(0, 1, 24, 0, &[], &[]);
        assert!(
            Bitmap::parse(&bytes)
                .unwrap()
                .decode(Limits::default())
                .is_err()
        );
    }

    #[test]
    fn preserves_unsupported_compression_and_header_extensions() {
        let metadata = b"FS70\x14\0\0\0\0\x04\0\0\0\0\0\0\0\0\0\0";
        let bytes = bitmap(4, 4, 16, u32::from_le_bytes(*b"DXT3"), metadata, &[0; 16]);
        let bmp = Bitmap::parse(&bytes).unwrap();
        assert_eq!(bmp.metadata(), metadata);
        assert_eq!(bmp.pixels(), [0; 16]);
        assert!(matches!(
            bmp.decode(Limits::default()),
            Err(Error::Decode(image::ImageError::Decoding(_)))
        ));
    }
}

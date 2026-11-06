//! FSX/P3D TRQ1 terrain raster records, with borrowed compressed payloads.
use super::{Bgl, Directory, Error, range};
pub mod codec;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Compression {
    None,
    Lz1,
    Delta,
    DeltaLz1,
    Lz2,
    DeltaLz2,
    BitPack,
    BitPackLz1,
    SolidBlock,
    BitPackLz2,
    Ptc,
    Dxt1,
    Dxt3,
    Dxt5,
    Other(u8),
}

impl From<u8> for Compression {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::None,
            1 => Self::Lz1,
            2 => Self::Delta,
            3 => Self::DeltaLz1,
            4 => Self::Lz2,
            5 => Self::DeltaLz2,
            6 => Self::BitPack,
            7 => Self::BitPackLz1,
            8 => Self::SolidBlock,
            9 => Self::BitPackLz2,
            10 => Self::Ptc,
            11 => Self::Dxt1,
            12 => Self::Dxt3,
            13 => Self::Dxt5,
            other => Self::Other(other),
        }
    }
}

#[derive(Debug)]
pub struct Raster<'a> {
    pub data_type: u16,
    pub data_compression: Compression,
    pub mask_compression: Compression,
    pub qmid: [u32; 2],
    pub variations: u32,
    pub columns: u16,
    pub rows: u16,
    pub dimension_flags: [u16; 2],
    pub header_extension: &'a [u8],
    pub data: &'a [u8],
    pub mask: &'a [u8],
    pub trailing_data: &'a [u8],
}

impl<'a> Raster<'a> {
    pub fn decode_mask(&self) -> Result<Option<Vec<u8>>, codec::Error> {
        if self.mask.is_empty() {
            return Ok(None);
        }
        codec::decode_mask(self.mask_compression, self.mask, self.columns, self.rows).map(Some)
    }
    pub fn decode_data(&self) -> Result<Vec<u8>, codec::Error> {
        if self.data_compression == Compression::Ptc
            && (self.data_type != 1 || self.columns != 256 || self.rows != 256)
        {
            return Err(codec::Error::Unsupported(Compression::Ptc));
        }
        let bytes_per_pixel = match self.data_type {
            1 | 2 | 9 | 10 => 2,
            3..=7 => 1,
            11 => 4,
            _ => return Err(codec::Error::InvalidData),
        };
        let size = usize::from(self.columns)
            .checked_mul(usize::from(self.rows))
            .and_then(|size| size.checked_mul(bytes_per_pixel))
            .ok_or(codec::Error::Limit)?;
        codec::decode(self.data_compression, self.data, size)
    }

    pub fn parse(bytes: &'a [u8]) -> Result<Self, Error> {
        range(bytes, 0, 40)?;
        if &bytes[..4] != b"TRQ1" {
            return Err(Error::Header);
        }
        let word = |offset| u16::from_le_bytes(bytes[offset..offset + 2].try_into().unwrap());
        let dword = |offset| u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap());
        let header_size = u64::from(dword(4));
        if header_size < 40 || word(24) == 0 || word(28) == 0 {
            return Err(Error::Header);
        }
        let header_extension = range(bytes, 40, header_size - 40)?;
        let size_data = u64::from(dword(32));
        let size_mask = u64::from(dword(36));
        let data = range(bytes, header_size, size_data)?;
        let mask = range(bytes, header_size + size_data, size_mask)?;
        Ok(Self {
            data_type: word(8),
            data_compression: bytes[10].into(),
            mask_compression: bytes[11].into(),
            qmid: [dword(12), dword(16)],
            variations: dword(20),
            columns: word(24),
            rows: word(28),
            dimension_flags: [word(26), word(30)],
            header_extension,
            data,
            mask,
            trailing_data: &bytes[(header_size + size_data + size_mask) as usize..],
        })
    }
}

impl Bgl<'_> {
    pub fn raster_tiles(&self) -> Result<Vec<(u32, Raster<'_>)>, Error> {
        let mut tiles = Vec::new();
        for section in self.sections.iter().filter(|s| matches!(s.kind, 0x67..=0x6a | 0x6c | 0x6e | 0x78..=0x83 | 0x8c..=0x98 | 0xa0..=0xac)) {
            let records: Vec<_> = match &section.directory {
                Directory::Direct(subsections) => subsections.iter().map(|s| &s.records).collect(),
                Directory::Indirect { records, .. } => records.iter().collect(),
                Directory::Unknown => continue,
            };
            for record in records {
                // Section IDs 0xa0/0xa1 also identify TACAN records in P3D.
                if section.kind >= 0xa0 && !record.data.starts_with(b"TRQ1") { continue; }
                tiles.push((section.kind, Raster::parse(record.data)?));
            }
        }
        Ok(tiles)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn separates_dimensions_flags_and_bounded_payloads() {
        let mut bytes = vec![0u8; 45];
        bytes[..4].copy_from_slice(b"TRQ1");
        for (offset, value) in [
            (4, 40u32),
            (24, 0x1200_0100),
            (28, 0x3400_0080),
            (32, 3),
            (36, 2),
        ] {
            bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        bytes[10] = 10;
        let tile = Raster::parse(&bytes).unwrap();
        assert_eq!((tile.columns, tile.rows), (256, 128));
        assert_eq!(tile.dimension_flags, [0x1200, 0x3400]);
        assert_eq!(tile.data_compression, Compression::Ptc);
        assert_eq!((tile.data.len(), tile.mask.len()), (3, 2));
        for end in 0..bytes.len() {
            assert!(Raster::parse(&bytes[..end]).is_err());
        }
        bytes[4..8].copy_from_slice(&39u32.to_le_bytes());
        assert!(Raster::parse(&bytes).is_err());
    }
}

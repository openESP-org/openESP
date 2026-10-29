use binrw::BinRead;
use std::{fmt, io::Cursor};

#[derive(BinRead)]
#[br(little)]
struct Header {
    id: [u8; 4],
    size: u32,
}

#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    Truncated { offset: usize },
    InvalidRiff,
    InvalidSize,
    UnsupportedFormat([u8; 4]),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Truncated { offset } => write!(f, "truncated data at offset {offset}"),
            Self::InvalidRiff => write!(f, "missing RIFF signature"),
            Self::InvalidSize => write!(f, "invalid RIFF size"),
            Self::UnsupportedFormat(id) => write!(f, "unsupported RIFF form {id:?}"),
        }
    }
}

impl std::error::Error for Error {}

#[derive(Debug, PartialEq, Eq)]
pub struct Chunk<'a> {
    pub id: [u8; 4],
    pub data: &'a [u8],
}

#[derive(Debug, PartialEq, BinRead)]
#[br(little)]
pub struct BoundingBox {
    pub min: [f32; 3],
    pub max: [f32; 3],
}

impl Chunk<'_> {
    pub fn bounding_box(&self) -> Result<Option<BoundingBox>, Error> {
        if &self.id != b"BBOX" {
            return Ok(None);
        }
        if self.data.len() != 24 {
            return Err(Error::InvalidSize);
        }
        BoundingBox::read(&mut Cursor::new(self.data))
            .map(Some)
            .map_err(|_| Error::InvalidSize)
    }

    pub fn radius(&self) -> Result<Option<f32>, Error> {
        if &self.id != b"RADI" {
            return Ok(None);
        }
        if self.data.len() != 4 {
            return Err(Error::InvalidSize);
        }
        f32::read_le(&mut Cursor::new(self.data))
            .map(Some)
            .map_err(|_| Error::InvalidSize)
    }
}

#[derive(Debug)]
pub struct Model<'a> {
    pub chunks: Vec<Chunk<'a>>,
    pub trailing_data: &'a [u8],
}

impl<'a> Model<'a> {
    pub fn parse(data: &'a [u8]) -> Result<Self, Error> {
        let header =
            Header::read(&mut Cursor::new(data)).map_err(|_| Error::Truncated { offset: 0 })?;
        if &header.id != b"RIFF" {
            return Err(Error::InvalidRiff);
        }
        let size = usize::try_from(header.size).map_err(|_| Error::InvalidSize)?;
        if size < 4 {
            return Err(Error::InvalidSize);
        }
        let end = 8usize.checked_add(size).ok_or(Error::InvalidSize)?;
        let body = data.get(8..end).ok_or(Error::Truncated { offset: 8 })?;
        let form: [u8; 4] = body[..4].try_into().map_err(|_| Error::InvalidSize)?;
        if &form != b"MDLX" {
            return Err(Error::UnsupportedFormat(form));
        }
        let mut offset = 12;
        let mut chunks = Vec::new();
        while offset < end {
            let remaining = &data[offset..end];
            let chunk = Header::read(&mut Cursor::new(remaining))
                .map_err(|_| Error::Truncated { offset })?;
            let length = usize::try_from(chunk.size).map_err(|_| Error::InvalidSize)?;
            let start = offset + 8;
            let chunk_end = start.checked_add(length).ok_or(Error::InvalidSize)?;
            let next = chunk_end
                .checked_add(length % 2)
                .ok_or(Error::InvalidSize)?;
            if next > end {
                return Err(Error::Truncated { offset: start });
            }
            chunks.push(Chunk {
                id: chunk.id,
                data: &data[start..chunk_end],
            });
            offset = next;
        }
        Ok(Self {
            chunks,
            trailing_data: &data[end..],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_chunks_and_padding() {
        let data = b"RIFF\x16\0\0\0MDLXTEST\x01\0\0\0\x42\0NEXT\0\0\0\0tail";
        let model = Model::parse(data).unwrap();
        assert_eq!(
            model.chunks,
            [
                Chunk {
                    id: *b"TEST",
                    data: b"B"
                },
                Chunk {
                    id: *b"NEXT",
                    data: b""
                }
            ]
        );
        assert_eq!(model.trailing_data, b"tail");
    }

    #[test]
    fn rejects_invalid_and_truncated_inputs() {
        assert!(matches!(
            Model::parse(b"RIFF"),
            Err(Error::Truncated { .. })
        ));
        assert_eq!(
            Model::parse(b"NOPE\x04\0\0\0MDLX").unwrap_err(),
            Error::InvalidRiff
        );
        assert_eq!(
            Model::parse(b"RIFF\x03\0\0\0MDL").unwrap_err(),
            Error::InvalidSize
        );
        assert_eq!(
            Model::parse(b"RIFF\x04\0\0\0PV20").unwrap_err(),
            Error::UnsupportedFormat(*b"PV20")
        );
        assert!(matches!(
            Model::parse(b"RIFF\x0d\0\0\0MDLXTEST\x01\0\0\0X"),
            Err(Error::Truncated { .. })
        ));
        assert!(matches!(
            Model::parse(b"RIFF\x05\0\0\0MDLXX"),
            Err(Error::Truncated { .. })
        ));
        assert!(matches!(
            Model::parse(b"RIFF\xff\xff\xff\xffMDLX"),
            Err(Error::Truncated { .. })
        ));
    }

    #[test]
    fn reads_bounds_and_radius() {
        let mut data = Vec::new();
        for value in [-1.0f32, -2.0, -3.0, 4.0, 5.0, 6.0] {
            data.extend_from_slice(&value.to_le_bytes());
        }
        let bounds = Chunk {
            id: *b"BBOX",
            data: &data,
        };
        assert_eq!(
            bounds.bounding_box().unwrap(),
            Some(BoundingBox {
                min: [-1.0, -2.0, -3.0],
                max: [4.0, 5.0, 6.0]
            })
        );
        assert_eq!(bounds.radius().unwrap(), None);
        let radius_data = 7.5f32.to_le_bytes();
        let radius = Chunk {
            id: *b"RADI",
            data: &radius_data,
        };
        assert_eq!(radius.radius().unwrap(), Some(7.5));
        assert_eq!(
            Chunk {
                id: *b"BBOX",
                data: &data[..23]
            }
            .bounding_box(),
            Err(Error::InvalidSize)
        );
        assert_eq!(
            Chunk {
                id: *b"RADI",
                data: b"12345"
            }
            .radius(),
            Err(Error::InvalidSize)
        );
    }
}

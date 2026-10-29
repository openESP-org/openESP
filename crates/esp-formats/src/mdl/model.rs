use super::{BoundingBox, Chunk, Error, Model, crash::CrashTree, data::ModelData};
use binrw::BinRead;
use std::{collections::HashSet, io::Cursor};
use uuid::Uuid;

#[derive(Debug)]
pub struct ModelHeader<'a> {
    pub marker: u32,
    pub version: f32,
    pub trailing_data: &'a [u8],
}

#[derive(Debug, Default)]
pub struct DecodedModel<'a> {
    pub header: Option<ModelHeader<'a>>,
    pub guid: Option<Uuid>,
    pub name: Option<&'a [u8]>,
    pub bounds: Option<BoundingBox>,
    pub radius: Option<f32>,
    pub crash: Option<CrashTree<'a>>,
    pub exterior: Option<ModelData<'a>>,
    pub shadow: Option<ModelData<'a>>,
    pub unknown: Vec<Chunk<'a>>,
}

impl<'a> Model<'a> {
    pub fn decode(&self) -> Result<DecodedModel<'a>, Error> {
        let mut model = DecodedModel::default();
        let mut seen = HashSet::new();
        for chunk in &self.chunks {
            let known = matches!(
                &chunk.id,
                b"MDLH" | b"MDLG" | b"MDLN" | b"BBOX" | b"RADI" | b"MDLD" | b"SHAM" | b"CRAS"
            );
            if known && !seen.insert(chunk.id) {
                return Err(Error::DuplicateSection(chunk.id));
            }
            match &chunk.id {
                b"MDLH" => {
                    let mut cursor = Cursor::new(chunk.data);
                    let marker =
                        u32::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 0 })?;
                    let version =
                        f32::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 4 })?;
                    model.header = Some(ModelHeader {
                        marker,
                        version,
                        trailing_data: &chunk.data[8..],
                    });
                }
                b"MDLG" => {
                    let bytes = chunk.data.try_into().map_err(|_| Error::InvalidSize)?;
                    model.guid = Some(Uuid::from_bytes_le(bytes));
                }
                b"MDLN" => {
                    let end = chunk
                        .data
                        .iter()
                        .position(|&byte| byte == 0)
                        .unwrap_or(chunk.data.len());
                    model.name = Some(&chunk.data[..end]);
                }
                b"BBOX" => model.bounds = chunk.bounding_box()?,
                b"RADI" => model.radius = chunk.radius()?,
                b"CRAS" => model.crash = Some(CrashTree::parse(chunk.data)?),
                b"MDLD" => model.exterior = Some(ModelData::parse(chunk)?),
                b"SHAM" => model.shadow = Some(ModelData::parse(chunk)?),
                _ => model.unknown.push(Chunk {
                    id: chunk.id,
                    data: chunk.data,
                }),
            }
        }
        Ok(model)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_mixed_endian_guid_and_model_metadata() {
        let guid = [
            0x33, 0x22, 0x11, 0x00, 0x55, 0x44, 0x77, 0x66, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd,
            0xee, 0xff,
        ];
        let mut header = 123456u32.to_le_bytes().to_vec();
        header.extend(10.0f32.to_le_bytes());
        header.extend([7, 8]);
        let model = Model {
            chunks: vec![
                Chunk {
                    id: *b"MDLH",
                    data: &header,
                },
                Chunk {
                    id: *b"MDLG",
                    data: &guid,
                },
                Chunk {
                    id: *b"MDLN",
                    data: b"Object\0",
                },
                Chunk {
                    id: *b"MDLD",
                    data: &[],
                },
                Chunk {
                    id: *b"SMAP",
                    data: &[42],
                },
            ],
            trailing_data: &[],
        };
        let decoded = model.decode().unwrap();
        assert_eq!(
            decoded.guid.unwrap().to_string(),
            "00112233-4455-6677-8899-aabbccddeeff"
        );
        assert_eq!(decoded.name, Some(b"Object".as_slice()));
        let header = decoded.header.unwrap();
        assert_eq!(header.marker, 123456);
        assert_eq!(header.version, 10.0);
        assert_eq!(header.trailing_data, &[7, 8]);
        assert!(decoded.exterior.is_some());
        assert_eq!(decoded.unknown[0].data, &[42]);
    }

    #[test]
    fn rejects_truncated_and_duplicate_metadata() {
        for (id, length) in [(b"MDLH", 8), (b"MDLG", 16)] {
            for end in 0..length {
                let data = vec![0; end];
                let model = Model {
                    chunks: vec![Chunk {
                        id: *id,
                        data: &data,
                    }],
                    trailing_data: &[],
                };
                assert!(model.decode().is_err());
            }
        }
        let model = Model {
            chunks: vec![
                Chunk {
                    id: *b"MDLN",
                    data: b"one",
                },
                Chunk {
                    id: *b"MDLN",
                    data: b"two",
                },
            ],
            trailing_data: &[],
        };
        assert!(matches!(model.decode(), Err(Error::DuplicateSection(_))));
    }
}

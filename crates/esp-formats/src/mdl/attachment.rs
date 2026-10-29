use super::{Error, geometry::records};
use binrw::BinRead;
use std::io::Cursor;
use uuid::Uuid;

#[derive(Debug)]
pub struct AttachedObject<'a> {
    pub marker: u32,
    pub kind: u16,
    pub raw: &'a [u8],
    pub definition: Option<ObjectDefinition<'a>>,
}

#[derive(Debug)]
pub struct ObjectDefinition<'a> {
    pub attachpoint: &'a [u8],
    pub unknown: u16,
    pub reserved: [u32; 4],
    pub value: ObjectValue<'a>,
}

#[derive(Debug, PartialEq)]
pub enum ObjectValue<'a> {
    Library {
        guid: Uuid,
        scale: f32,
        extra: &'a [u8],
    },
    Effect {
        filename: &'a [u8],
        parameters: &'a [u8],
    },
}

pub fn attached_objects(data: &[u8]) -> Result<Vec<AttachedObject<'_>>, Error> {
    let mut offset = 0;
    let mut result = Vec::new();
    while offset < data.len() {
        let mut cursor = Cursor::new(&data[offset..]);
        let marker = u32::read_le(&mut cursor).map_err(|_| Error::Truncated { offset })?;
        let kind =
            u16::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: offset + 4 })?;
        let length =
            u16::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: offset + 6 })?;
        let start = offset + 8;
        let end = start
            .checked_add(usize::from(length))
            .ok_or(Error::InvalidSize)?;
        let raw = data
            .get(start..end)
            .ok_or(Error::Truncated { offset: start })?;
        let definition = if matches!(kind, 2 | 4) {
            Some(ObjectDefinition::parse(kind, raw)?)
        } else {
            None
        };
        result.push(AttachedObject {
            marker,
            kind,
            raw,
            definition,
        });
        offset = end;
    }
    Ok(result)
}

impl<'a> ObjectDefinition<'a> {
    fn parse(kind: u16, data: &'a [u8]) -> Result<Self, Error> {
        let mut cursor = Cursor::new(data);
        let name_offset = u16::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 0 })?;
        let unknown = u16::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 2 })?;
        let reserved =
            <[u32; 4]>::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 4 })?;
        let name_start = usize::from(name_offset);
        let name_bytes = data.get(name_start..).ok_or(Error::InvalidReference {
            field: "attachpoint name offset",
            value: i64::from(name_offset),
            available: data.len(),
        })?;
        let name_end = name_bytes
            .iter()
            .position(|&byte| byte == 0)
            .ok_or(Error::InvalidSize)?;
        let attachpoint = &name_bytes[..name_end];
        let value = match kind {
            2 => {
                let guid = <[u8; 16]>::read_le(&mut cursor)
                    .map_err(|_| Error::Truncated { offset: 20 })?;
                let scale =
                    f32::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 36 })?;
                let extra = data.get(40..name_start).ok_or(Error::InvalidSize)?;
                ObjectValue::Library {
                    guid: Uuid::from_bytes_le(guid),
                    scale,
                    extra,
                }
            }
            4 => {
                let filename = data.get(20..100).ok_or(Error::Truncated { offset: 20 })?;
                let filename = &filename[..filename
                    .iter()
                    .position(|&byte| byte == 0)
                    .unwrap_or(filename.len())];
                let parameters = data.get(100..name_start).ok_or(Error::InvalidSize)?;
                let parameters = &parameters[..parameters
                    .iter()
                    .position(|&byte| byte == 0)
                    .unwrap_or(parameters.len())];
                ObjectValue::Effect {
                    filename,
                    parameters,
                }
            }
            _ => return Err(Error::InvalidSize),
        };
        Ok(Self {
            attachpoint,
            unknown,
            reserved,
            value,
        })
    }
}

#[derive(Debug, PartialEq)]
pub struct Platform<'a> {
    pub surface: u32,
    pub scene_node: i32,
    pub vertices: Vec<[f32; 3]>,
    pub trailing_data: &'a [u8],
}

#[derive(Debug, PartialEq)]
pub struct AttachPoint<'a> {
    pub scene_node: i32,
    pub name: &'a [u8],
    pub trailing_data: &'a [u8],
}

impl<'a> Platform<'a> {
    pub fn parse(data: &'a [u8]) -> Result<Self, Error> {
        let mut cursor = Cursor::new(data);
        let surface = u32::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 0 })?;
        let scene_node = i32::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 4 })?;
        let count = u32::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 8 })?;
        let size = usize::try_from(count)
            .ok()
            .and_then(|count| count.checked_mul(12))
            .and_then(|size| size.checked_add(12))
            .ok_or(Error::InvalidSize)?;
        let vertices = data.get(12..size).ok_or(Error::Truncated { offset: 12 })?;
        Ok(Self {
            surface,
            scene_node,
            vertices: records(vertices, 12)?,
            trailing_data: &data[size..],
        })
    }
}

impl<'a> AttachPoint<'a> {
    pub fn parse(data: &'a [u8]) -> Result<Self, Error> {
        let mut cursor = Cursor::new(data);
        let scene_node = i32::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 0 })?;
        let length = u32::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 4 })?;
        let end = usize::try_from(length)
            .ok()
            .and_then(|length| length.checked_add(8))
            .ok_or(Error::InvalidSize)?;
        let bytes = data.get(8..end).ok_or(Error::Truncated { offset: 8 })?;
        let name = bytes.strip_suffix(&[0]).unwrap_or(bytes);
        Ok(Self {
            scene_node,
            name,
            trailing_data: &data[end..],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn object_record(kind: u16, payload: &[u8]) -> Vec<u8> {
        [
            0x41000u32.to_le_bytes().as_slice(),
            &kind.to_le_bytes(),
            &(payload.len() as u16).to_le_bytes(),
            payload,
        ]
        .concat()
    }

    #[test]
    fn reads_attached_library_and_effect_records() {
        let mut library = vec![0; 20];
        library[..2].copy_from_slice(&40u16.to_le_bytes());
        library.extend([1; 16]);
        library.extend(2.5f32.to_le_bytes());
        library.extend(b"attachpt_library\0");
        let mut effect = vec![0; 100];
        effect[..2].copy_from_slice(&105u16.to_le_bytes());
        effect[20..28].copy_from_slice(b"fx_test\0");
        effect.extend(b"TEST\0attachpt_effect\0");
        let bytes = [
            object_record(2, &library),
            object_record(4, &effect),
            object_record(99, &[42]),
        ]
        .concat();
        let objects = attached_objects(&bytes).unwrap();
        assert_eq!(objects.len(), 3);
        let library = objects[0].definition.as_ref().unwrap();
        assert_eq!(library.attachpoint, b"attachpt_library");
        assert_eq!(
            library.value,
            ObjectValue::Library {
                guid: Uuid::from_bytes([1; 16]),
                scale: 2.5,
                extra: &[]
            }
        );
        let effect = objects[1].definition.as_ref().unwrap();
        assert_eq!(effect.attachpoint, b"attachpt_effect");
        assert_eq!(
            effect.value,
            ObjectValue::Effect {
                filename: b"fx_test",
                parameters: b"TEST"
            }
        );
        assert!(objects[2].definition.is_none());
        assert_eq!(objects[2].raw, &[42]);
    }

    #[test]
    fn rejects_truncated_objects_and_invalid_name_offsets() {
        let mut payload = vec![0; 100];
        payload[..2].copy_from_slice(&101u16.to_le_bytes());
        let bytes = object_record(4, &payload);
        assert!(attached_objects(&bytes).is_err());
        payload[..2].copy_from_slice(&99u16.to_le_bytes());
        assert!(attached_objects(&object_record(4, &payload)).is_err());
        payload[..2].copy_from_slice(&100u16.to_le_bytes());
        payload.extend(b"attachpt\0");
        let bytes = object_record(4, &payload);
        for end in 1..bytes.len() {
            assert!(attached_objects(&bytes[..end]).is_err());
        }
        assert!(attached_objects(&[]).unwrap().is_empty());
    }

    #[test]
    fn reads_floating_point_platform_vertices() {
        let mut bytes: Vec<_> = [99u32, 2, 3]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect();
        for coordinate in [-1.5f32, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 2.5, 0.0] {
            bytes.extend(coordinate.to_le_bytes());
        }
        let platform = Platform::parse(&bytes).unwrap();
        assert_eq!(platform.surface, 99);
        assert_eq!(platform.scene_node, 2);
        assert_eq!(platform.vertices[0], [-1.5, 0.0, 0.0]);
        for end in 0..bytes.len() {
            assert!(Platform::parse(&bytes[..end]).is_err());
        }
        bytes[8..12].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(Platform::parse(&bytes).is_err());
    }

    #[test]
    fn bounds_attachpoint_names_and_retains_extra_bytes() {
        let bytes = b"\x02\0\0\0\x05\0\0\0gear\0extra";
        let point = AttachPoint::parse(bytes).unwrap();
        assert_eq!(point.scene_node, 2);
        assert_eq!(point.name, b"gear");
        assert_eq!(point.trailing_data, b"extra");
        for end in 0..13 {
            assert!(AttachPoint::parse(&bytes[..end]).is_err());
        }
        assert!(AttachPoint::parse(b"\0\0\0\0\xff\xff\xff\xff").is_err());
    }
}

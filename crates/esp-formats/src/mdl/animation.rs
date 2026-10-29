use super::{Chunk, Error, Header, parse_chunks};
use binrw::BinRead;
use std::io::Cursor;
use uuid::Uuid;

#[derive(Debug)]
pub struct AnimationBlock<'a> {
    pub version: Option<f32>,
    pub animations: Vec<Animation<'a>>,
    pub unknown: Vec<Chunk<'a>>,
    pub unknown_list_sections: Vec<Chunk<'a>>,
    pub trailing_data: &'a [u8],
}

#[derive(Debug)]
pub struct Animation<'a> {
    pub guid: Uuid,
    pub length: f32,
    pub type_name: &'a [u8],
    pub type_parameter: &'a [u8],
    pub tracks: Vec<Track<'a>>,
    pub unknown: Vec<Chunk<'a>>,
}

#[derive(Debug)]
pub struct Track<'a> {
    pub kind: u32,
    pub id: u32,
    pub length: f32,
    pub keys: Vec<Key<'a>>,
    pub unknown: Vec<Chunk<'a>>,
}

#[derive(Debug, PartialEq)]
pub struct Key<'a> {
    pub time: f32,
    pub value: KeyValue<'a>,
}

#[derive(Debug, PartialEq)]
pub enum KeyValue<'a> {
    Translation([f32; 3]),
    Rotation([f32; 4]),
    Unknown { kind: u8, data: &'a [u8] },
}

impl<'a> AnimationBlock<'a> {
    pub fn parse(data: &'a [u8]) -> Result<Self, Error> {
        let header =
            Header::read(&mut Cursor::new(data)).map_err(|_| Error::Truncated { offset: 0 })?;
        if &header.id != b"RIFF" {
            return Err(Error::InvalidRiff);
        }
        let end = usize::try_from(header.size)
            .ok()
            .and_then(|size| size.checked_add(8))
            .ok_or(Error::InvalidSize)?;
        let body = data.get(8..end).ok_or(Error::Truncated { offset: 8 })?;
        let mut result = Self {
            version: None,
            animations: Vec::new(),
            unknown: Vec::new(),
            unknown_list_sections: Vec::new(),
            trailing_data: &data[end..],
        };
        // The animation RIFF starts with XANH, without a form-type field.
        for chunk in parse_chunks(body)? {
            match &chunk.id {
                b"XANH" => {
                    if result.version.is_some() {
                        return Err(Error::DuplicateSection(chunk.id));
                    }
                    if chunk.data.len() != 4 {
                        return Err(Error::InvalidSize);
                    }
                    result.version = Some(
                        f32::read_le(&mut Cursor::new(chunk.data))
                            .map_err(|_| Error::InvalidSize)?,
                    );
                }
                b"XANL" => {
                    for entry in parse_chunks(chunk.data)? {
                        if &entry.id == b"XANI" {
                            result.animations.push(Animation::parse(entry.data)?);
                        } else {
                            result.unknown_list_sections.push(entry);
                        }
                    }
                }
                _ => result.unknown.push(chunk),
            }
        }
        Ok(result)
    }
}

impl<'a> Animation<'a> {
    fn parse(data: &'a [u8]) -> Result<Self, Error> {
        let prefix = data.get(..45).ok_or(Error::Truncated { offset: 0 })?;
        let mut cursor = Cursor::new(prefix);
        let guid = <[u8; 16]>::read_le(&mut cursor).map_err(|_| Error::InvalidSize)?;
        let length = f32::read_le(&mut cursor).map_err(|_| Error::InvalidSize)?;
        let mut result = Self {
            guid: Uuid::from_bytes_le(guid),
            length,
            type_name: c_string(&prefix[20..36]),
            type_parameter: c_string(&prefix[36..45]),
            tracks: Vec::new(),
            unknown: Vec::new(),
        };
        for chunk in parse_chunks(&data[45..])? {
            if &chunk.id == b"XANS" {
                result.tracks.push(Track::parse(chunk.data)?);
            } else {
                result.unknown.push(chunk);
            }
        }
        Ok(result)
    }
}

impl<'a> Track<'a> {
    fn parse(data: &'a [u8]) -> Result<Self, Error> {
        let mut cursor = Cursor::new(data);
        let kind = u32::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 0 })?;
        let id = u32::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 4 })?;
        let length = f32::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 8 })?;
        let mut result = Self {
            kind,
            id,
            length,
            keys: Vec::new(),
            unknown: Vec::new(),
        };
        for chunk in parse_chunks(&data[12..])? {
            if &chunk.id == b"XANK" {
                result.keys.push(Key::parse(chunk.data)?);
            } else {
                result.unknown.push(chunk);
            }
        }
        Ok(result)
    }
}

impl<'a> Key<'a> {
    fn parse(data: &'a [u8]) -> Result<Self, Error> {
        let mut cursor = Cursor::new(data);
        let kind = u8::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 0 })?;
        let time = f32::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 1 })?;
        let value = match kind {
            3 => {
                if data.len() != 17 {
                    return Err(Error::InvalidSize);
                }
                KeyValue::Translation(
                    <[f32; 3]>::read_le(&mut cursor).map_err(|_| Error::InvalidSize)?,
                )
            }
            4 => {
                if data.len() != 21 {
                    return Err(Error::InvalidSize);
                }
                KeyValue::Rotation(
                    <[f32; 4]>::read_le(&mut cursor).map_err(|_| Error::InvalidSize)?,
                )
            }
            _ => KeyValue::Unknown {
                kind,
                data: &data[5..],
            },
        };
        Ok(Self { time, value })
    }
}

fn c_string(data: &[u8]) -> &[u8] {
    &data[..data
        .iter()
        .position(|&byte| byte == 0)
        .unwrap_or(data.len())]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn chunk(id: &[u8; 4], payload: &[u8]) -> Vec<u8> {
        [
            id.as_slice(),
            &(payload.len() as u32).to_le_bytes(),
            payload,
        ]
        .concat()
    }

    fn sample() -> Vec<u8> {
        let mut translation = vec![3];
        for value in [2.0f32, 1.0, 2.0, 3.0] {
            translation.extend(value.to_le_bytes());
        }
        let mut rotation = vec![4];
        for value in [3.0f32, 0.0, 0.0, 0.0, 1.0] {
            rotation.extend(value.to_le_bytes());
        }
        let mut track = [
            2u32.to_le_bytes(),
            17u32.to_le_bytes(),
            100.0f32.to_le_bytes(),
        ]
        .concat();
        track.extend(chunk(b"XANK", &translation));
        track.extend(chunk(b"XANK", &rotation));
        let mut animation = vec![0; 16];
        animation.extend(100.0f32.to_le_bytes());
        let mut type_name = [0; 16];
        type_name[..3].copy_from_slice(b"Sim");
        animation.extend(type_name);
        animation.extend(b"AutoPlay\0");
        animation.extend(chunk(b"XAPS", b"example#percent\0"));
        animation.extend(chunk(b"XANS", &track));
        let mut body = chunk(b"XANH", &9.2f32.to_le_bytes());
        body.extend(chunk(b"XANL", &chunk(b"XANI", &animation)));
        chunk(b"RIFF", &body)
    }

    #[test]
    fn reads_animation_hierarchy_and_unaligned_keys() {
        let bytes = sample();
        let block = AnimationBlock::parse(&bytes).unwrap();
        assert_eq!(block.version, Some(9.2));
        let animation = &block.animations[0];
        assert_eq!(animation.type_name, b"Sim");
        assert_eq!(animation.type_parameter, b"AutoPlay");
        assert_eq!(animation.unknown[0].id, *b"XAPS");
        assert_eq!(animation.unknown[0].data, b"example#percent\0");
        let track = &animation.tracks[0];
        assert_eq!(track.id, 17);
        assert_eq!(
            track.keys[0],
            Key {
                time: 2.0,
                value: KeyValue::Translation([1.0, 2.0, 3.0])
            }
        );
        assert_eq!(
            track.keys[1].value,
            KeyValue::Rotation([0.0, 0.0, 0.0, 1.0])
        );
        for end in 0..bytes.len() {
            assert!(AnimationBlock::parse(&bytes[..end]).is_err());
        }
        let bytes = [bytes, vec![0; 32]].concat();
        assert_eq!(
            AnimationBlock::parse(&bytes).unwrap().trailing_data.len(),
            32
        );
    }

    #[test]
    fn rejects_incomplete_keys_and_preserves_unknown_types() {
        for length in 0..21 {
            let mut bytes = vec![0; length];
            if let Some(first) = bytes.first_mut() {
                *first = 4;
            }
            assert!(Key::parse(&bytes).is_err());
        }
        assert_eq!(
            Key::parse(&[99, 0, 0, 0, 0, 42]).unwrap().value,
            KeyValue::Unknown {
                kind: 99,
                data: &[42]
            }
        );
    }
}

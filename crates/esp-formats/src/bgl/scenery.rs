use super::{Error, range};
use binrw::BinRead;
use std::io::Cursor;
use uuid::Uuid;

#[derive(Debug)]
pub struct Record<'a> {
    pub kind: u16,
    pub data: &'a [u8],
    pub raw_data: &'a [u8],
}

#[derive(Debug, BinRead)]
#[br(little)]
pub struct Placement {
    pub longitude: u32,
    pub latitude: u32,
    pub altitude: i32,
    pub flags: u16,
    pub pitch: u16,
    pub bank: u16,
    pub heading: u16,
    pub image_complexity: u16,
    pub unknown: u16,
    #[br(map = Uuid::from_bytes_le)]
    pub instance: Uuid,
}

impl Placement {
    pub fn longitude_degrees(&self) -> f64 {
        f64::from(self.longitude) * (360.0 / 805306368.0) - 180.0
    }

    pub fn latitude_degrees(&self) -> f64 {
        90.0 - f64::from(self.latitude) * (180.0 / 536870912.0)
    }

    pub fn altitude_meters(&self) -> f64 {
        f64::from(self.altitude) / 1000.0
    }

    pub fn orientation_degrees(&self) -> [f64; 3] {
        [self.pitch, self.bank, self.heading].map(|angle| f64::from(angle) * 360.0 / 65536.0)
    }
}

#[derive(Debug)]
pub struct PlacedObject<'a> {
    pub placement: Placement,
    pub definition: Definition<'a>,
    pub trailing_data: &'a [u8],
}

#[derive(Debug)]
pub enum Definition<'a> {
    Library {
        guid: Uuid,
        scale: f32,
    },
    Windsock(Windsock),
    Effect {
        name: &'a [u8],
        parameters: &'a [u8],
    },
    Beacon {
        kind: u8,
        unknown: [u8; 3],
    },
}

#[derive(Debug, BinRead)]
#[br(little)]
pub struct Windsock {
    pub pole_height: f32,
    pub sock_length: f32,
    pub pole_color: [u8; 4],
    pub sock_color: [u8; 4],
    pub flags: u16,
}

pub fn parse(data: &[u8]) -> Result<Vec<Record<'_>>, Error> {
    let mut records = Vec::new();
    let mut offset = 0;
    while offset < data.len() {
        let mut cursor = Cursor::new(&data[offset..]);
        let kind = u16::read_le(&mut cursor)?;
        let size = u16::read_le(&mut cursor)?;
        if size < 4 {
            return Err(Error::RecordSize {
                offset,
                size: size.into(),
            });
        }
        let mut length = u64::from(size);
        if kind == 0x1002 && size == 4 {
            // Attachment payloads reuse object IDs but have a different placement header.
            let nested = range(data, offset as u64 + 4, 4)?;
            let size = u16::from_le_bytes([nested[2], nested[3]]);
            if size < 4 {
                return Err(Error::RecordSize {
                    offset: offset + 4,
                    size: size.into(),
                });
            }
            length += u64::from(size) + 4;
            let end = range(data, offset as u64 + length - 4, 4)?;
            let kind = u16::from_le_bytes([end[0], end[1]]);
            if !matches!(kind, 0x1001 | 0x1003) || end[2..] != [4, 0] {
                return Err(Error::AttachmentDelimiter(offset + length as usize - 4));
            }
        }
        let raw_data = range(data, offset as u64, length)?;
        records.push(Record {
            kind,
            data: &raw_data[4..],
            raw_data,
        });
        offset += raw_data.len();
    }
    Ok(records)
}

impl<'a> Record<'a> {
    pub fn decode(&self) -> Result<Option<PlacedObject<'a>>, Error> {
        if !matches!(self.kind, 0x0b..=0x0d | 0x11) {
            return Ok(None);
        }
        let mut cursor = Cursor::new(self.data);
        let placement = Placement::read(&mut cursor)?;
        let definition = match self.kind {
            0x0b => Definition::Library {
                guid: Uuid::from_bytes_le(<[u8; 16]>::read_le(&mut cursor)?),
                scale: f32::read_le(&mut cursor)?,
            },
            0x0c => Definition::Windsock(Windsock::read(&mut cursor)?),
            0x0d => {
                let start = cursor.position();
                let name = range(self.data, start, 80)?;
                let parameters = &self.data[start as usize + 80..];
                let end = parameters
                    .iter()
                    .position(|&b| b == 0)
                    .map(|n| n + 1)
                    .ok_or(Error::UnterminatedString)?;
                cursor.set_position(start + 80 + end as u64);
                Definition::Effect {
                    name: trim_nul(name),
                    parameters: trim_nul(&parameters[..end]),
                }
            }
            0x11 => Definition::Beacon {
                kind: u8::read_le(&mut cursor)?,
                unknown: <[u8; 3]>::read_le(&mut cursor)?,
            },
            _ => return Ok(None),
        };
        Ok(Some(PlacedObject {
            placement,
            definition,
            trailing_data: &self.data[cursor.position() as usize..],
        }))
    }
}

fn trim_nul(bytes: &[u8]) -> &[u8] {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    &bytes[..end]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(kind: u16, payload: &[u8]) -> Vec<u8> {
        let mut bytes = vec![0; 44];
        bytes[..2].copy_from_slice(&kind.to_le_bytes());
        bytes[2..4].copy_from_slice(&(44u16 + payload.len() as u16).to_le_bytes());
        bytes[4..8].copy_from_slice(&0x18000000u32.to_le_bytes());
        bytes[8..12].copy_from_slice(&0x10000000u32.to_le_bytes());
        bytes[12..16].copy_from_slice(&(-1234i32).to_le_bytes());
        bytes[16..18].copy_from_slice(&0x8001u16.to_le_bytes());
        bytes[22..24].copy_from_slice(&0x4000u16.to_le_bytes());
        bytes.extend(payload);
        bytes
    }

    #[test]
    fn decodes_library_position_and_retains_unknown_metadata() {
        let guid = Uuid::parse_str("12345678-9abc-def0-1234-56789abcdef0").unwrap();
        let mut payload = guid.to_bytes_le().to_vec();
        payload.extend(1.5f32.to_le_bytes());
        payload.extend([7, 8]);
        let bytes = record(0x0b, &payload);
        let records = parse(&bytes).unwrap();
        let object = records[0].decode().unwrap().unwrap();
        assert_eq!(object.placement.longitude_degrees(), 0.0);
        assert_eq!(object.placement.latitude_degrees(), 0.0);
        assert_eq!(object.placement.altitude_meters(), -1.234);
        assert_eq!(object.placement.orientation_degrees(), [0.0, 0.0, 90.0]);
        assert_eq!(object.placement.flags, 0x8001);
        assert_eq!(object.trailing_data, &[7, 8]);
        let Definition::Library {
            guid: actual,
            scale,
        } = object.definition
        else {
            panic!()
        };
        assert_eq!(actual, guid);
        assert_eq!(scale, 1.5);
    }

    #[test]
    fn decodes_windsocks_beacons_and_effects() {
        let mut payload = [3.0f32.to_le_bytes(), 2.0f32.to_le_bytes()].concat();
        payload.extend([10, 20, 30, 255, 40, 50, 60, 255, 1, 0]);
        let mut bytes = record(0x0c, &payload);
        bytes.extend(record(0x11, &[0xf5, 1, 0, 0]));
        let mut payload = vec![0; 80];
        payload[..6].copy_from_slice(b"fx_nav");
        payload.extend(b"DAY=1\0\0");
        bytes.extend(record(0x0d, &payload));
        let records = parse(&bytes).unwrap();
        let Definition::Windsock(sock) = records[0].decode().unwrap().unwrap().definition else {
            panic!()
        };
        assert_eq!((sock.pole_height, sock.sock_length), (3.0, 2.0));
        assert_eq!(sock.pole_color, [10, 20, 30, 255]);
        assert_eq!(sock.flags, 1);
        assert!(matches!(
            records[1].decode().unwrap().unwrap().definition,
            Definition::Beacon { kind: 0xf5, .. }
        ));
        let effect = records[2].decode().unwrap().unwrap();
        assert!(matches!(
            effect.definition,
            Definition::Effect {
                name: b"fx_nav",
                parameters: b"DAY=1"
            }
        ));
        assert_eq!(effect.trailing_data, &[0]);
    }

    #[test]
    fn keeps_attachment_headers_out_of_placement_decoders() {
        let mut bytes = record(0x0b, &[0; 20]);
        bytes.extend([2, 16, 4, 0, 11, 0, 4, 0, 3, 16, 4, 0]);
        bytes.extend(record(0x11, &[0xf5, 1, 0, 0]));
        let records = parse(&bytes).unwrap();
        assert_eq!(records.len(), 3);
        assert_eq!(records[1].kind, 0x1002);
        assert_eq!(records[1].raw_data.len(), 12);
        assert!(records[1].decode().unwrap().is_none());
        assert!(records[2].decode().unwrap().is_some());
        bytes[72] = 0;
        assert!(matches!(parse(&bytes), Err(Error::AttachmentDelimiter(_))));
    }

    #[test]
    fn rejects_malformed_records_and_incomplete_payloads() {
        let bytes = record(0x0b, &[0; 20]);
        for end in 1..bytes.len() {
            assert!(parse(&bytes[..end]).is_err());
        }
        for size in 0..4 {
            assert!(parse(&[11, 0, size, 0]).is_err());
        }
        let bytes = record(0x0b, &[0; 19]);
        assert!(parse(&bytes).unwrap()[0].decode().is_err());
        let bytes = record(0x0d, &[b'x'; 81]);
        assert!(matches!(
            parse(&bytes).unwrap()[0].decode(),
            Err(Error::UnterminatedString)
        ));
        let bytes = [0xff, 0xff, 5, 0, 9];
        let records = parse(&bytes).unwrap();
        assert_eq!(records[0].data, &[9]);
        assert!(records[0].decode().unwrap().is_none());
    }
}

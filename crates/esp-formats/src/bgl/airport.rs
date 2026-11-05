//! Airport and runway geometry in FSX and Prepar3D airport BGL sections.
use super::{Bgl, Directory, Error, range};

#[derive(Debug, Clone, PartialEq)]
pub struct Airport {
    pub ident: String,
    pub latitude: f64,
    pub longitude: f64,
    pub elevation_m: f64,
    pub runways: Vec<Runway>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Runway {
    pub primary_number: u8,
    pub primary_designator: u8,
    pub secondary_number: u8,
    pub secondary_designator: u8,
    pub surface: u16,
    pub latitude: f64,
    pub longitude: f64,
    pub elevation_m: f64,
    pub length_m: f32,
    pub width_m: f32,
    pub heading_deg: f32,
}

fn u16_at(data: &[u8], offset: usize) -> Result<u16, Error> {
    Ok(u16::from_le_bytes(
        range(data, offset as u64, 2)?.try_into().unwrap(),
    ))
}
fn u32_at(data: &[u8], offset: usize) -> Result<u32, Error> {
    Ok(u32::from_le_bytes(
        range(data, offset as u64, 4)?.try_into().unwrap(),
    ))
}
fn f32_at(data: &[u8], offset: usize) -> Result<f32, Error> {
    Ok(f32::from_bits(u32_at(data, offset)?))
}
fn position(data: &[u8], offset: usize) -> Result<(f64, f64, f64), Error> {
    let longitude = f64::from(u32_at(data, offset)?) * (360.0 / 805_306_368.0) - 180.0;
    let latitude = 90.0 - f64::from(u32_at(data, offset + 4)?) * (180.0 / 536_870_912.0);
    let altitude = i32::from_le_bytes(range(data, (offset + 8) as u64, 4)?.try_into().unwrap());
    Ok((latitude, longitude, f64::from(altitude) / 1000.0))
}
fn ident(encoded: u32) -> String {
    let mut value = encoded >> 5;
    let mut bytes = Vec::new();
    while value != 0 && bytes.len() < 5 {
        let digit = value % 38;
        match digit {
            2..=11 => bytes.push(b'0' + (digit - 2) as u8),
            12..=37 => bytes.push(b'A' + (digit - 12) as u8),
            _ => break,
        }
        value /= 38;
    }
    bytes.reverse();
    String::from_utf8(bytes).unwrap()
}

fn runway(data: &[u8]) -> Result<Runway, Error> {
    range(data, 0, 44)?;
    let (latitude, longitude, elevation_m) = position(data, 20)?;
    Ok(Runway {
        surface: u16_at(data, 6)?,
        primary_number: data[8],
        primary_designator: data[9],
        secondary_number: data[10],
        secondary_designator: data[11],
        latitude,
        longitude,
        elevation_m,
        length_m: f32_at(data, 32)?,
        width_m: f32_at(data, 36)?,
        heading_deg: f32_at(data, 40)?,
    })
}

/// Parse consecutive airport records in one BGL subsection.
pub fn parse(data: &[u8]) -> Result<Vec<Airport>, Error> {
    let mut airports = Vec::new();
    let mut offset = 0;
    while offset < data.len() {
        let kind = u16_at(data, offset)?;
        let size = u32_at(data, offset + 2)? as usize;
        if size < 6 {
            return Err(Error::RecordSize {
                offset,
                size: size as u32,
            });
        }
        let record = range(data, offset as u64, size as u64)?;
        let subrecords = match kind {
            0x003c => Some(56),
            0x00ab => Some(60),
            _ => None,
        };
        if let Some(mut suboffset) = subrecords {
            range(record, 0, suboffset as u64)?;
            let (latitude, longitude, elevation_m) = position(record, 12)?;
            let mut airport = Airport {
                ident: ident(u32_at(record, 40)?),
                latitude,
                longitude,
                elevation_m,
                runways: Vec::new(),
            };
            while suboffset < record.len() {
                let child_kind = u16_at(record, suboffset)?;
                let child_size = u32_at(record, suboffset + 2)? as usize;
                if child_size < 6 {
                    return Err(Error::RecordSize {
                        offset: offset + suboffset,
                        size: child_size as u32,
                    });
                }
                let child = range(record, suboffset as u64, child_size as u64)?;
                if matches!(child_kind, 0x0004 | 0x003e) {
                    airport.runways.push(runway(child)?);
                }
                suboffset += child_size;
            }
            airports.push(airport);
        }
        offset += size;
    }
    Ok(airports)
}

impl Bgl<'_> {
    pub fn airports(&self) -> Result<Vec<Airport>, Error> {
        let mut airports = Vec::new();
        for section in self.sections.iter().filter(|section| section.kind == 3) {
            match &section.directory {
                Directory::Direct(subsections) => {
                    for subsection in subsections {
                        airports.extend(parse(subsection.records.data)?);
                    }
                }
                Directory::Indirect { records, .. } => {
                    for record in records {
                        airports.extend(parse(record.data)?);
                    }
                }
                Directory::Unknown => {}
            }
        }
        Ok(airports)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(kind: u16, body: &[u8]) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend(kind.to_le_bytes());
        bytes.extend(((body.len() + 6) as u32).to_le_bytes());
        bytes.extend(body);
        bytes
    }

    #[test]
    fn reads_runway_geometry_and_rejects_corrupt_records() {
        let mut runway_body = vec![0u8; 38];
        runway_body[2..6].copy_from_slice(&[9, 1, 27, 2]);
        runway_body[26..30].copy_from_slice(&3000f32.to_le_bytes());
        runway_body[30..34].copy_from_slice(&45f32.to_le_bytes());
        runway_body[34..38].copy_from_slice(&90f32.to_le_bytes());
        let runway = record(0x3e, &runway_body);
        let mut body = vec![0; 54];
        body.extend(&runway);
        let bytes = record(0xab, &body);
        let airports = parse(&bytes).unwrap();
        assert_eq!(airports.len(), 1);
        assert_eq!(airports[0].runways.len(), 1);
        assert_eq!(airports[0].runways[0].primary_number, 9);
        assert_eq!(airports[0].runways[0].length_m, 3000.0);
        assert_eq!(airports[0].runways[0].width_m, 45.0);
        for end in 1..bytes.len() {
            assert!(parse(&bytes[..end]).is_err());
        }
        let mut malformed = bytes.clone();
        malformed[62..66].copy_from_slice(&5u32.to_le_bytes());
        assert!(parse(&malformed).is_err());
    }
}

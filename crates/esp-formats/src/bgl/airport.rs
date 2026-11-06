//! Airport and runway geometry in FSX and Prepar3D airport BGL sections.
use super::{Bgl, Directory, Error, range};

#[derive(Debug, Clone, PartialEq)]
pub struct Airport {
    pub ident: String,
    pub latitude: f64,
    pub longitude: f64,
    pub elevation_m: f64,
    pub runways: Vec<Runway>,
    pub helipads: Vec<Helipad>,
    pub taxi_points: Vec<TaxiPoint>,
    pub taxi_parkings: Vec<TaxiParking>,
    pub taxi_paths: Vec<TaxiPath>,
    pub aprons: Vec<Apron>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Apron {
    pub surface: u8,
    pub draw_surface: bool,
    pub vertices: Vec<(f64, f64)>,
    pub indices: Vec<u16>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TaxiPoint {
    pub latitude: f64,
    pub longitude: f64,
    pub kind: u8,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TaxiParking {
    pub latitude: f64,
    pub longitude: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TaxiPath {
    pub start: u16,
    pub end: u16,
    pub kind: u8,
    pub surface: u8,
    pub width_m: f32,
    pub draw_surface: bool,
    pub centerline: bool,
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

#[derive(Debug, Clone, PartialEq)]
pub struct Helipad {
    pub surface: u8,
    pub kind: u8,
    pub transparent: bool,
    pub closed: bool,
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

fn helipad(data: &[u8]) -> Result<Helipad, Error> {
    range(data, 0, 36)?;
    let (latitude, longitude, elevation_m) = position(data, 12)?;
    Ok(Helipad {
        surface: data[6],
        kind: data[7] & 0x0f,
        transparent: data[7] & 0x10 != 0,
        closed: data[7] & 0x20 != 0,
        latitude,
        longitude,
        elevation_m,
        length_m: f32_at(data, 24)?,
        width_m: f32_at(data, 28)?,
        heading_deg: f32_at(data, 32)?,
    })
}

fn taxi_points(data: &[u8], stride: usize) -> Result<Vec<TaxiPoint>, Error> {
    let count = usize::from(u16_at(data, 6)?);
    range(data, 8, (count * stride) as u64)?;
    (0..count)
        .map(|i| {
            let offset = 8 + i * stride;
            let longitude = f64::from(u32_at(data, offset + 4)?) * (360.0 / 805_306_368.0) - 180.0;
            let latitude = 90.0 - f64::from(u32_at(data, offset + 8)?) * (180.0 / 536_870_912.0);
            Ok(TaxiPoint {
                latitude,
                longitude,
                kind: data[offset],
            })
        })
        .collect()
}

fn taxi_paths(data: &[u8], stride: usize) -> Result<Vec<TaxiPath>, Error> {
    let count = usize::from(u16_at(data, 6)?);
    range(data, 8, (count * stride) as u64)?;
    (0..count)
        .map(|i| {
            let offset = 8 + i * stride;
            let flags = data[offset + 4];
            Ok(TaxiPath {
                start: u16_at(data, offset)?,
                end: u16_at(data, offset + 2)? & 0x0fff,
                kind: flags & 0x0f,
                draw_surface: flags & 0x20 != 0,
                centerline: data[offset + 6] & 1 != 0,
                surface: data[offset + 7],
                width_m: f32_at(data, offset + 8)?,
            })
        })
        .collect()
}

fn taxi_parkings(data: &[u8], p3d_v5: bool) -> Result<Vec<TaxiParking>, Error> {
    let count = usize::from(u16_at(data, 6)?);
    let mut offset = 8;
    let mut parkings = Vec::with_capacity(count);
    for _ in 0..count {
        let flags = u32_at(data, offset)?;
        let longitude = f64::from(u32_at(data, offset + 28)?) * (360.0 / 805_306_368.0) - 180.0;
        let latitude = 90.0 - f64::from(u32_at(data, offset + 32)?) * (180.0 / 536_870_912.0);
        let size = 36 + ((flags >> 24) as usize) * 4 + if p3d_v5 { 4 } else { 0 };
        range(data, offset as u64, size as u64)?;
        parkings.push(TaxiParking {
            latitude,
            longitude,
        });
        offset += size;
    }
    Ok(parkings)
}

fn apron(data: &[u8], counts_offset: usize) -> Result<Apron, Error> {
    let vertex_count = usize::from(u16_at(data, counts_offset)?);
    let triangle_count = usize::from(u16_at(data, counts_offset + 2)?);
    let vertex_offset = counts_offset + 4;
    let index_offset = vertex_offset + vertex_count * 8;
    range(data, vertex_offset as u64, (vertex_count * 8) as u64)?;
    range(data, index_offset as u64, (triangle_count * 6) as u64)?;
    let mut vertices = Vec::with_capacity(vertex_count);
    for index in 0..vertex_count {
        let offset = vertex_offset + index * 8;
        let longitude = f64::from(u32_at(data, offset)?) * (360.0 / 805_306_368.0) - 180.0;
        let latitude = 90.0 - f64::from(u32_at(data, offset + 4)?) * (180.0 / 536_870_912.0);
        vertices.push((latitude, longitude));
    }
    let mut indices = Vec::with_capacity(triangle_count * 3);
    for index in 0..triangle_count * 3 {
        let vertex = u16_at(data, index_offset + index * 2)?;
        if usize::from(vertex) >= vertex_count {
            return Err(Error::Range {
                offset: u64::from(vertex),
                size: vertex_count as u64,
            });
        }
        indices.push(vertex);
    }
    Ok(Apron {
        surface: data[6],
        draw_surface: data[7] & 1 != 0,
        vertices,
        indices,
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
                helipads: Vec::new(),
                taxi_points: Vec::new(),
                taxi_parkings: Vec::new(),
                taxi_paths: Vec::new(),
                aprons: Vec::new(),
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
                match child_kind {
                    0x0026 => airport.helipads.push(helipad(child)?),
                    0x001a => airport.taxi_points.extend(taxi_points(child, 12)?),
                    0x00ac => airport.taxi_points.extend(taxi_points(child, 16)?),
                    0x003d => airport.taxi_parkings.extend(taxi_parkings(child, false)?),
                    0x00ad => airport.taxi_parkings.extend(taxi_parkings(child, true)?),
                    0x001c => airport.taxi_paths.extend(taxi_paths(child, 20)?),
                    0x0040 => airport.taxi_paths.extend(taxi_paths(child, 36)?),
                    0x00ae => airport.taxi_paths.extend(taxi_paths(child, 40)?),
                    0x0030 => airport.aprons.push(apron(child, 8)?),
                    0x0041 => airport.aprons.push(apron(child, 24)?),
                    0x00b0 => airport.aprons.push(apron(child, 28)?),
                    _ => {}
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
    fn reads_taxiway_record_versions_and_checks_count_bounds() {
        for (point_kind, point_stride) in [(0x1a, 12), (0xac, 16)] {
            let mut body = vec![0u8; 2 + point_stride];
            body[..2].copy_from_slice(&1u16.to_le_bytes());
            body[2] = 1;
            let raw = record(point_kind, &body);
            let points = taxi_points(&raw, point_stride).unwrap();
            assert_eq!(points.len(), 1);
            assert_eq!(points[0].kind, 1);
            assert!(taxi_points(&raw[..raw.len() - 1], point_stride).is_err());
        }
        for (path_kind, stride) in [(0x1c, 20), (0x40, 36), (0xae, 40)] {
            let mut body = vec![0u8; 2 + stride];
            body[..2].copy_from_slice(&1u16.to_le_bytes());
            body[2..4].copy_from_slice(&3u16.to_le_bytes());
            body[4..6].copy_from_slice(&0xf005u16.to_le_bytes());
            body[6] = 0x21;
            body[8] = 1;
            body[10..14].copy_from_slice(&23.0f32.to_le_bytes());
            let raw = record(path_kind, &body);
            let paths = taxi_paths(&raw, stride).unwrap();
            assert_eq!(
                (paths[0].start, paths[0].end, paths[0].width_m),
                (3, 5, 23.0)
            );
            assert!(paths[0].draw_surface && paths[0].centerline);
            assert!(taxi_paths(&raw[..raw.len() - 1], stride).is_err());
        }
    }

    #[test]
    fn reads_variable_length_parking_records_and_checks_bounds() {
        for (kind, extra) in [(0x3d, 0), (0xad, 4)] {
            let mut body = 2u16.to_le_bytes().to_vec();
            let mut first = vec![0u8; 36 + 8 + extra];
            first[..4].copy_from_slice(&(2u32 << 24).to_le_bytes());
            first[28..32].copy_from_slice(&402_653_184u32.to_le_bytes());
            first[32..36].copy_from_slice(&268_435_456u32.to_le_bytes());
            body.extend(first);
            let mut second = vec![0u8; 36 + extra];
            second[28..32].copy_from_slice(&402_653_184u32.to_le_bytes());
            second[32..36].copy_from_slice(&268_435_456u32.to_le_bytes());
            body.extend(second);
            let raw = record(kind, &body);
            let parsed = taxi_parkings(&raw, extra != 0).unwrap();
            assert_eq!(parsed.len(), 2);
            assert_eq!((parsed[1].latitude, parsed[1].longitude), (0.0, 0.0));
            assert!(taxi_parkings(&raw[..raw.len() - 1], extra != 0).is_err());
        }
    }

    #[test]
    fn reads_helipad_position_flags_and_dimensions() {
        let mut body = vec![0u8; 30];
        body[0] = 4;
        body[1] = 0x32;
        body[6..10].copy_from_slice(&402_653_184u32.to_le_bytes());
        body[10..14].copy_from_slice(&268_435_456u32.to_le_bytes());
        body[14..18].copy_from_slice(&12_345i32.to_le_bytes());
        body[18..22].copy_from_slice(&30f32.to_le_bytes());
        body[22..26].copy_from_slice(&20f32.to_le_bytes());
        body[26..30].copy_from_slice(&90f32.to_le_bytes());
        let raw = record(0x26, &body);
        let pad = helipad(&raw).unwrap();
        assert_eq!(
            (pad.latitude, pad.longitude, pad.elevation_m),
            (0.0, 0.0, 12.345)
        );
        assert_eq!((pad.kind, pad.surface), (2, 4));
        assert!(pad.transparent && pad.closed);
        assert_eq!(
            (pad.length_m, pad.width_m, pad.heading_deg),
            (30.0, 20.0, 90.0)
        );
        assert!(helipad(&raw[..35]).is_err());
        let mut airport_body = vec![0; 50];
        airport_body.extend(raw);
        assert_eq!(
            parse(&record(0x3c, &airport_body)).unwrap()[0]
                .helipads
                .len(),
            1
        );
    }

    #[test]
    fn reads_triangulated_aprons_and_rejects_bad_indices() {
        for (kind, counts_offset) in [(0x30, 8), (0x41, 24), (0xb0, 28)] {
            let mut body = vec![0u8; counts_offset - 6];
            body[1] = 1;
            body.extend(3u16.to_le_bytes());
            body.extend(1u16.to_le_bytes());
            body.extend([0u8; 24]);
            for index in [0u16, 1, 2] {
                body.extend(index.to_le_bytes());
            }
            let mut raw = record(kind, &body);
            let parsed = apron(&raw, counts_offset).unwrap();
            assert!(parsed.draw_surface);
            assert_eq!(parsed.vertices.len(), 3);
            assert_eq!(parsed.indices, [0, 1, 2]);
            assert!(apron(&raw[..raw.len() - 1], counts_offset).is_err());
            let end = raw.len();
            raw[end - 2..].copy_from_slice(&3u16.to_le_bytes());
            assert!(apron(&raw, counts_offset).is_err());
        }
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

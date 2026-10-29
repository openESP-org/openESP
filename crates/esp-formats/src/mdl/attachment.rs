use super::{Error, geometry::records};
use binrw::BinRead;
use std::io::Cursor;

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

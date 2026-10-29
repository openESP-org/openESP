use super::{Error, geometry::records};
use binrw::BinRead;
use std::io::Cursor;

#[derive(Debug, PartialEq)]
pub struct CrashTree<'a> {
    pub unknown: [u32; 2],
    pub origin: [f32; 3],
    pub size: [f32; 3],
    pub nodes: Vec<[i32; 8]>,
    pub trailing_data: &'a [u8],
}

impl<'a> CrashTree<'a> {
    pub fn parse(data: &'a [u8]) -> Result<Self, Error> {
        let mut cursor = Cursor::new(data);
        let unknown =
            <[u32; 2]>::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 0 })?;
        let origin =
            <[f32; 3]>::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 8 })?;
        let size = <[f32; 3]>::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 20 })?;
        let count = u32::read_le(&mut cursor).map_err(|_| Error::Truncated { offset: 32 })?;
        let end = usize::try_from(count)
            .ok()
            .and_then(|count| count.checked_mul(32))
            .and_then(|length| length.checked_add(36))
            .ok_or(Error::InvalidSize)?;
        let nodes = data.get(36..end).ok_or(Error::Truncated { offset: 36 })?;
        Ok(Self {
            unknown,
            origin,
            size,
            nodes: records(nodes, 32)?,
            trailing_data: &data[end..],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tree() -> Vec<u8> {
        let mut bytes = [1u32.to_le_bytes(), 8u32.to_le_bytes()].concat();
        for value in [-1.5f32, -2.0, -3.0, 3.0, 4.0, 6.0] {
            bytes.extend(value.to_le_bytes());
        }
        bytes.extend(1u32.to_le_bytes());
        for value in [-1i32, -2, -1, -2, -1, -2, -1, -2] {
            bytes.extend(value.to_le_bytes());
        }
        bytes
    }

    #[test]
    fn reads_octants_and_preserves_extensions() {
        let mut bytes = tree();
        bytes.extend([7, 8]);
        let tree = CrashTree::parse(&bytes).unwrap();
        assert_eq!(tree.unknown, [1, 8]);
        assert_eq!(tree.origin, [-1.5, -2.0, -3.0]);
        assert_eq!(tree.size, [3.0, 4.0, 6.0]);
        assert_eq!(tree.nodes[0], [-1, -2, -1, -2, -1, -2, -1, -2]);
        assert_eq!(tree.trailing_data, &[7, 8]);
    }

    #[test]
    fn bounds_node_count_before_allocating() {
        let mut bytes = tree();
        for end in 0..bytes.len() {
            assert!(CrashTree::parse(&bytes[..end]).is_err());
        }
        bytes[32..36].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(CrashTree::parse(&bytes).is_err());
        bytes[32..36].copy_from_slice(&0u32.to_le_bytes());
        assert!(CrashTree::parse(&bytes[..36]).unwrap().nodes.is_empty());
    }
}

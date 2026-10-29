use super::{Error, geometry::records};
use binrw::BinRead;

#[derive(Debug, Clone, PartialEq, Eq, BinRead)]
#[br(little)]
pub struct Node {
    pub child: i16,
    pub peer: i16,
    pub animation_map: i16,
    pub unknown: i16,
}

#[derive(Debug, Clone, PartialEq, Eq, BinRead)]
#[br(little)]
pub struct AnimationMap {
    pub transformation_type: u32,
    pub transform: i32,
}

pub fn nodes(data: &[u8]) -> Result<Vec<Node>, Error> {
    records(data, 8)
}

pub fn animation_map(data: &[u8]) -> Result<Vec<AnimationMap>, Error> {
    records(data, 8)
}

pub fn animation_links(data: &[u8]) -> Result<Vec<i32>, Error> {
    records(data, 4)
}

pub fn bone_references(data: &[u8]) -> Result<Vec<i16>, Error> {
    records(data, 2)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn preserves_node_indices_and_unknown_fields() {
        let bytes: Vec<_> = [1i16, -1, 2, 123]
            .into_iter()
            .flat_map(i16::to_le_bytes)
            .collect();
        assert_eq!(
            nodes(&bytes).unwrap(),
            [Node {
                child: 1,
                peer: -1,
                animation_map: 2,
                unknown: 123
            }]
        );
        for end in 1..8 {
            assert_eq!(nodes(&bytes[..end]), Err(Error::InvalidSize));
        }
    }

    #[test]
    fn reads_maps_and_signed_links() {
        let bytes: Vec<_> = [2i32, 15, 99, -1]
            .into_iter()
            .flat_map(i32::to_le_bytes)
            .collect();
        assert_eq!(
            animation_map(&bytes).unwrap(),
            [
                AnimationMap {
                    transformation_type: 2,
                    transform: 15
                },
                AnimationMap {
                    transformation_type: 99,
                    transform: -1
                }
            ]
        );
        assert_eq!(animation_links(&bytes).unwrap(), [2, 15, 99, -1]);
        assert_eq!(bone_references(&[255, 255, 2, 0]).unwrap(), [-1, 2]);
        assert_eq!(animation_links(&bytes[..3]), Err(Error::InvalidSize));
        assert_eq!(bone_references(&[0]), Err(Error::InvalidSize));
        assert!(nodes(&[]).unwrap().is_empty());
    }
}

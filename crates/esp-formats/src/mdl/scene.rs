use super::{Error, geometry::records};
use binrw::BinRead;
use petgraph::{algo::toposort, graphmap::DiGraphMap};

#[derive(Debug, Clone, PartialEq, Eq, BinRead)]
#[br(little)]
pub struct Node {
    pub child: i16,
    pub peer: i16,
    pub animation_map_offset: i16,
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

pub fn animation_links(data: &[u8]) -> Result<Vec<i16>, Error> {
    records(data, 2)
}

pub fn bone_references(data: &[u8]) -> Result<Vec<i16>, Error> {
    records(data, 2)
}

pub fn visibility_links(data: &[u8]) -> Result<Vec<i16>, Error> {
    records(data, 2)
}

pub fn parents(nodes: &[Node]) -> Result<Vec<Option<usize>>, Error> {
    let mut graph = DiGraphMap::<usize, ()>::new();
    for (index, node) in nodes.iter().enumerate() {
        graph.add_node(index);
        for (field, link) in [("scene child", node.child), ("scene peer", node.peer)] {
            if link == -1 {
                continue;
            }
            let target = usize::try_from(link)
                .ok()
                .filter(|&target| target < nodes.len())
                .ok_or(Error::InvalidReference {
                    field,
                    value: i64::from(link),
                    available: nodes.len(),
                })?;
            graph.add_edge(index, target, ());
        }
    }
    let order = toposort(&graph, None).map_err(|cycle| Error::CyclicScene {
        node: cycle.node_id(),
    })?;
    let mut parents = vec![None; nodes.len()];
    let mut assigned = vec![false; nodes.len()];
    for index in order {
        let node = &nodes[index];
        for (link, parent) in [(node.child, Some(index)), (node.peer, parents[index])] {
            if link == -1 {
                continue;
            }
            let link = link as usize;
            if assigned[link] && parents[link] != parent {
                return Err(Error::AmbiguousParent { node: link });
            }
            parents[link] = parent;
            assigned[link] = true;
        }
    }
    Ok(parents)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(child: i16, peer: i16) -> Node {
        Node {
            child,
            peer,
            animation_map_offset: 0,
            unknown: 0,
        }
    }

    #[test]
    fn resolves_parents_without_assuming_node_order() {
        let nodes = [node(-1, -1), node(3, -1), node(0, 1), node(-1, -1)];
        assert_eq!(parents(&nodes).unwrap(), [Some(2), None, None, Some(1)]);
        let nodes = [node(1, -1), node(3, 2), node(-1, -1), node(-1, -1)];
        assert_eq!(parents(&nodes).unwrap(), [None, Some(0), Some(0), Some(1)]);
        assert!(parents(&[]).unwrap().is_empty());
    }

    #[test]
    fn rejects_cycles_and_invalid_node_links() {
        for nodes in [
            vec![node(0, -1)],
            vec![node(1, -1), node(-1, 0)],
            vec![node(-1, 1), node(-1, 0)],
        ] {
            assert!(matches!(parents(&nodes), Err(Error::CyclicScene { .. })));
        }
        for node in [node(-2, -1), node(1, -1), node(-1, -2), node(-1, 1)] {
            assert!(matches!(
                parents(&[node]),
                Err(Error::InvalidReference { .. })
            ));
        }
        assert!(matches!(
            parents(&[node(2, -1), node(2, -1), node(-1, -1)]),
            Err(Error::AmbiguousParent { node: 2 })
        ));
        let nodes: Vec<_> = (0..10000)
            .map(|index| node(if index == 9999 { -1 } else { index + 1 }, -1))
            .collect();
        let hierarchy = parents(&nodes).unwrap();
        assert_eq!(hierarchy[9999], Some(9998));
    }

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
                animation_map_offset: 2,
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
        assert_eq!(animation_links(&[255, 255, 2, 0]).unwrap(), [-1, 2]);
        assert_eq!(bone_references(&[255, 255, 2, 0]).unwrap(), [-1, 2]);
        assert_eq!(animation_links(&bytes[..3]), Err(Error::InvalidSize));
        assert_eq!(bone_references(&[0]), Err(Error::InvalidSize));
        assert!(nodes(&[]).unwrap().is_empty());
    }

    #[test]
    fn reads_one_animation_link_per_node_with_an_odd_node_count() {
        let mut bytes = vec![255; 710];
        bytes[708..].copy_from_slice(&123i16.to_le_bytes());
        let links = animation_links(&bytes).unwrap();
        assert_eq!(links.len(), 355);
        assert_eq!(links[0], -1);
        assert_eq!(links[354], 123);
        assert_eq!(animation_links(&bytes[..709]), Err(Error::InvalidSize));
    }
}

use super::{
    Chunk, Error, animation, attachment, geometry, material, parse_chunks, scene, visibility,
};
use std::collections::HashSet;

#[derive(Debug, Default)]
pub struct ModelData<'a> {
    pub textures: Vec<&'a [u8]>,
    pub materials: Vec<material::Material>,
    pub indices: Vec<u16>,
    pub vertices: Vec<Vec<geometry::Vertex>>,
    pub tangents: Vec<geometry::VertexBufferData<geometry::Tangent>>,
    pub bone_maps: Vec<geometry::VertexBufferData<i32>>,
    pub skin_weights: Vec<geometry::VertexBufferData<geometry::SkinWeight>>,
    pub transforms: Vec<[f32; 16]>,
    pub nodes: Vec<scene::Node>,
    pub animation_map: Vec<scene::AnimationMap>,
    pub animation_links: Vec<i16>,
    pub animations: Option<animation::AnimationBlock<'a>>,
    pub visibility: Vec<visibility::Condition<'a>>,
    pub visibility_links: Vec<i16>,
    pub bone_references: Vec<i16>,
    pub lods: Vec<Lod<'a>>,
    pub platforms: Vec<attachment::Platform<'a>>,
    pub attachpoints: Vec<attachment::AttachPoint<'a>>,
    pub attached_objects: Vec<attachment::AttachedObject<'a>>,
    pub unknown_platform_sections: Vec<Chunk<'a>>,
    pub unknown_attachpoint_sections: Vec<Chunk<'a>>,
    pub unknown: Vec<Chunk<'a>>,
    pub unknown_vertex_sections: Vec<Chunk<'a>>,
    pub unknown_lod_sections: Vec<Chunk<'a>>,
}

#[derive(Debug)]
pub struct Lod<'a> {
    pub level: i32,
    pub parts: Vec<geometry::Part>,
    pub unknown: Vec<Chunk<'a>>,
}

#[derive(Debug)]
pub struct PartGeometry<'a> {
    pub vertices: &'a [geometry::Vertex],
    pub tangents: Option<&'a [geometry::Tangent]>,
    pub skin_weights: Option<&'a [geometry::SkinWeight]>,
    pub bone_map: Option<&'a [i32]>,
    pub indices: &'a [u16],
    pub material: &'a material::Material,
    pub scene_node: &'a scene::Node,
}

#[derive(Debug)]
pub struct VertexAttributes<'a> {
    pub vertices: &'a [geometry::Vertex],
    pub tangents: Option<&'a [geometry::Tangent]>,
    pub skin_weights: Option<&'a [geometry::SkinWeight]>,
    pub bone_map: Option<&'a [i32]>,
}

impl<'a> ModelData<'a> {
    pub fn vertex_attributes(&self, index: i32) -> Result<VertexAttributes<'_>, Error> {
        let vertices = reference(&self.vertices, index, "vertex buffer")?;
        let index = index as u32;
        let tangents = buffer(&self.tangents, index, *b"TANS")?;
        let skin_weights = buffer(&self.skin_weights, index, *b"SKIN")?;
        let bone_map = buffer(&self.bone_maps, index, *b"BMAP")?;
        for (section, actual) in [
            (*b"TANS", tangents.map(<[_]>::len)),
            (*b"SKIN", skin_weights.map(<[_]>::len)),
        ] {
            if let Some(actual) = actual.filter(|&actual| actual != vertices.len()) {
                return Err(Error::AttributeCount {
                    section,
                    index,
                    expected: vertices.len(),
                    actual,
                });
            }
        }
        Ok(VertexAttributes {
            vertices,
            tangents,
            skin_weights,
            bone_map,
        })
    }

    pub fn node_transform(&self, index: i32) -> Result<&[f32; 16], Error> {
        let node = reference(&self.nodes, index, "scene node")?;
        let offset = node.animation_map_offset;
        if offset < 0 || offset % 8 != 0 {
            return Err(Error::InvalidReference {
                field: "animation map byte offset",
                value: i64::from(offset),
                available: self.animation_map.len().saturating_mul(8),
            });
        }
        let map = reference(&self.animation_map, i32::from(offset / 8), "animation map")?;
        reference(&self.transforms, map.transform, "transform")
    }

    pub fn part_geometry(&self, part: &geometry::Part) -> Result<PartGeometry<'_>, Error> {
        let attributes = self.vertex_attributes(part.vertex_buffer)?;
        let vertices = range(
            attributes.vertices,
            part.vertex_offset,
            part.vertex_count,
            "vertices",
        )?;
        let tangents = attributes
            .tangents
            .map(|entries| range(entries, part.vertex_offset, part.vertex_count, "tangents"))
            .transpose()?;
        let skin_weights = attributes
            .skin_weights
            .map(|entries| {
                range(
                    entries,
                    part.vertex_offset,
                    part.vertex_count,
                    "skin weights",
                )
            })
            .transpose()?;
        let indices = range(
            &self.indices,
            part.index_offset,
            part.index_count,
            "indices",
        )?;
        for &index in indices {
            if usize::from(index) >= vertices.len() {
                return Err(Error::InvalidReference {
                    field: "vertex index",
                    value: i64::from(index),
                    available: vertices.len(),
                });
            }
        }
        let material = reference(&self.materials, part.material, "material")?;
        let scene_node = reference(&self.nodes, part.scene_node, "scene node")?;
        Ok(PartGeometry {
            vertices,
            tangents,
            skin_weights,
            bone_map: attributes.bone_map,
            indices,
            material,
            scene_node,
        })
    }

    pub fn parse(chunk: &Chunk<'a>) -> Result<Self, Error> {
        if !matches!(&chunk.id, b"MDLD" | b"SHAM") {
            return Err(Error::UnexpectedSection(chunk.id));
        }
        let mut result = Self::default();
        let mut seen = HashSet::new();
        for child in parse_chunks(chunk.data)? {
            let known = matches!(
                &child.id,
                b"TEXT"
                    | b"MATE"
                    | b"INDE"
                    | b"VERB"
                    | b"TRAN"
                    | b"SCEN"
                    | b"AMAP"
                    | b"SGAL"
                    | b"SGBR"
                    | b"LODT"
                    | b"PLAL"
                    | b"REFL"
                    | b"ANIB"
                    | b"VISL"
                    | b"SGVL"
                    | b"ATTO"
            );
            if known && !seen.insert(child.id) {
                return Err(Error::DuplicateSection(child.id));
            }
            match &child.id {
                b"TEXT" => result.textures = material::textures(child.data)?,
                b"MATE" => result.materials = material::materials(child.data)?,
                b"INDE" => result.indices = geometry::indices(child.data)?,
                b"TRAN" => result.transforms = geometry::transforms(child.data)?,
                b"SCEN" => result.nodes = scene::nodes(child.data)?,
                b"AMAP" => result.animation_map = scene::animation_map(child.data)?,
                b"SGAL" => result.animation_links = scene::animation_links(child.data)?,
                b"SGBR" => result.bone_references = scene::bone_references(child.data)?,
                b"ANIB" => result.animations = Some(animation::AnimationBlock::parse(child.data)?),
                b"VISL" => result.visibility = visibility::conditions(child.data)?,
                b"SGVL" => result.visibility_links = scene::visibility_links(child.data)?,
                b"ATTO" => result.attached_objects = attachment::attached_objects(child.data)?,
                b"VERB" => {
                    for buffer in parse_chunks(child.data)? {
                        match &buffer.id {
                            b"VERT" => result.vertices.push(geometry::vertices(buffer.data)?),
                            b"TANS" => result.tangents.push(geometry::tangents(buffer.data)?),
                            b"BMAP" => result.bone_maps.push(geometry::bone_map(buffer.data)?),
                            b"SKIN" => result
                                .skin_weights
                                .push(geometry::skin_weights(buffer.data)?),
                            _ => result.unknown_vertex_sections.push(buffer),
                        }
                    }
                }
                b"LODT" => {
                    for entry in parse_chunks(child.data)? {
                        if &entry.id == b"LODE" {
                            result.lods.push(Lod::parse(&entry)?);
                        } else {
                            result.unknown_lod_sections.push(entry);
                        }
                    }
                }
                b"PLAL" => {
                    for entry in parse_chunks(child.data)? {
                        if &entry.id == b"PLAT" {
                            result
                                .platforms
                                .push(attachment::Platform::parse(entry.data)?);
                        } else {
                            result.unknown_platform_sections.push(entry);
                        }
                    }
                }
                b"REFL" => {
                    for entry in parse_chunks(child.data)? {
                        if &entry.id == b"REFP" {
                            result
                                .attachpoints
                                .push(attachment::AttachPoint::parse(entry.data)?);
                        } else {
                            result.unknown_attachpoint_sections.push(entry);
                        }
                    }
                }
                _ => result.unknown.push(child),
            }
        }
        Ok(result)
    }
}

fn buffer<T>(
    buffers: &[geometry::VertexBufferData<T>],
    index: u32,
    section: [u8; 4],
) -> Result<Option<&[T]>, Error> {
    let mut matches = buffers.iter().filter(|buffer| buffer.buffer_index == index);
    let entries = matches.next().map(|buffer| buffer.entries.as_slice());
    if matches.next().is_some() {
        return Err(Error::DuplicateBuffer { section, index });
    }
    Ok(entries)
}

fn reference<'a, T>(entries: &'a [T], index: i32, field: &'static str) -> Result<&'a T, Error> {
    usize::try_from(index)
        .ok()
        .and_then(|index| entries.get(index))
        .ok_or(Error::InvalidReference {
            field,
            value: i64::from(index),
            available: entries.len(),
        })
}

fn range<'a, T>(
    entries: &'a [T],
    start: u32,
    count: u32,
    field: &'static str,
) -> Result<&'a [T], Error> {
    let end = u64::from(start) + u64::from(count);
    let bounds = usize::try_from(start).ok().zip(usize::try_from(end).ok());
    bounds
        .and_then(|(start, end)| entries.get(start..end))
        .ok_or(Error::InvalidRange {
            field,
            start,
            count,
            available: entries.len(),
        })
}

impl<'a> Lod<'a> {
    pub fn parse(chunk: &Chunk<'a>) -> Result<Self, Error> {
        let level = chunk.lod()?.ok_or(Error::UnexpectedSection(chunk.id))?;
        let mut parts = Vec::new();
        let mut unknown = Vec::new();
        for child in parse_chunks(&chunk.data[4..])? {
            if &child.id == b"PART" {
                parts.extend(geometry::parts(child.data)?);
            } else {
                unknown.push(child);
            }
        }
        Ok(Self {
            level,
            parts,
            unknown,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mdl::Model;

    fn chunk(id: &[u8; 4], data: &[u8]) -> Vec<u8> {
        let mut bytes = id.to_vec();
        bytes.extend((data.len() as u32).to_le_bytes());
        bytes.extend(data);
        bytes
    }

    #[test]
    fn decodes_geometry_through_the_file_hierarchy() {
        let mut body = chunk(b"VERB", &chunk(b"VERT", &[0; 96]));
        body.extend(chunk(b"INDE", &[0, 0, 1, 0, 2, 0]));
        body.extend(chunk(b"MATE", &[0; 120]));
        body.extend(chunk(b"SCEN", &[255, 255, 255, 255, 0, 0, 0, 0]));
        let mut lod = 100i32.to_le_bytes().to_vec();
        let part: Vec<_> = [1i32, 0, 0, 0, 0, 3, 0, 3, -1]
            .into_iter()
            .flat_map(i32::to_le_bytes)
            .collect();
        lod.extend(chunk(b"PART", &part));
        body.extend(chunk(b"LODT", &chunk(b"LODE", &lod)));
        body.extend(chunk(b"UNKN", &[42, 17]));
        body.extend(chunk(b"REFL", &chunk(b"REFP", b"\0\0\0\0\x05\0\0\0gear\0")));
        let mut payload = b"MDLX".to_vec();
        payload.extend(chunk(b"MDLD", &body));
        let file = chunk(b"RIFF", &payload);
        let model = Model::parse(&file).unwrap();
        let data = ModelData::parse(&model.chunks[0]).unwrap();
        assert_eq!(data.vertices[0].len(), 3);
        assert_eq!(data.indices, [0, 1, 2]);
        assert_eq!(data.lods[0].level, 100);
        assert_eq!(data.lods[0].parts[0].vertex_count, 3);
        let geometry = data.part_geometry(&data.lods[0].parts[0]).unwrap();
        assert_eq!(geometry.vertices.len(), 3);
        assert_eq!(geometry.indices, &[0, 1, 2]);
        assert_eq!(data.unknown[0].data, &[42, 17]);
        assert_eq!(data.attachpoints[0].name, b"gear");
        for end in 0..file.len() {
            assert!(Model::parse(&file[..end]).is_err());
        }
    }

    #[test]
    fn rejects_duplicate_and_truncated_known_sections() {
        let bytes = [chunk(b"INDE", &[]), chunk(b"INDE", &[])].concat();
        let data = Chunk {
            id: *b"MDLD",
            data: &bytes,
        };
        assert!(matches!(
            ModelData::parse(&data),
            Err(Error::DuplicateSection(_))
        ));
        let bytes = chunk(b"VERB", &chunk(b"VERT", &[0; 31]));
        assert!(
            ModelData::parse(&Chunk {
                id: *b"MDLD",
                data: &bytes
            })
            .is_err()
        );
    }

    #[test]
    fn resolves_part_relative_indices_and_rejects_invalid_ranges() {
        let vertices = geometry::vertices(&[0; 128]).unwrap();
        let mut data = ModelData {
            vertices: vec![vertices],
            indices: vec![0, 1, 2],
            materials: material::materials(&[0; 120]).unwrap(),
            nodes: scene::nodes(&[255, 255, 255, 255, 0, 0, 0, 0]).unwrap(),
            ..ModelData::default()
        };
        let mut part = geometry::Part {
            primitive_type: 1,
            scene_node: 0,
            material: 0,
            vertex_buffer: 0,
            vertex_offset: 1,
            vertex_count: 3,
            index_offset: 0,
            index_count: 3,
            mouse_rectangle: -1,
        };
        assert_eq!(data.part_geometry(&part).unwrap().vertices.len(), 3);
        part.vertex_offset = u32::MAX;
        assert!(matches!(
            data.part_geometry(&part),
            Err(Error::InvalidRange { .. })
        ));
        part.vertex_offset = 1;
        part.vertex_buffer = -1;
        assert!(matches!(
            data.part_geometry(&part),
            Err(Error::InvalidReference { .. })
        ));
        part.vertex_buffer = 0;
        part.index_count = u32::MAX;
        assert!(matches!(
            data.part_geometry(&part),
            Err(Error::InvalidRange { .. })
        ));
        part.index_count = 3;
        data.indices[2] = 3;
        assert!(matches!(
            data.part_geometry(&part),
            Err(Error::InvalidReference {
                field: "vertex index",
                ..
            })
        ));
        data.indices[2] = 2;
        part.material = 1;
        assert!(matches!(
            data.part_geometry(&part),
            Err(Error::InvalidReference {
                field: "material",
                ..
            })
        ));
    }

    #[test]
    fn resolves_animation_map_byte_offsets() {
        let mut data = ModelData {
            nodes: vec![scene::Node {
                child: -1,
                peer: -1,
                animation_map_offset: 8,
                unknown: 0,
            }],
            animation_map: vec![
                scene::AnimationMap {
                    transformation_type: 1,
                    transform: 0,
                },
                scene::AnimationMap {
                    transformation_type: 2,
                    transform: 1,
                },
            ],
            transforms: vec![[0.0; 16], [2.0; 16]],
            ..ModelData::default()
        };
        assert_eq!(data.node_transform(0).unwrap(), &[2.0; 16]);
        for offset in [-1, 1, 16] {
            data.nodes[0].animation_map_offset = offset;
            assert!(matches!(
                data.node_transform(0),
                Err(Error::InvalidReference { .. })
            ));
        }
        data.nodes[0].animation_map_offset = 8;
        data.animation_map[1].transform = 2;
        assert!(matches!(
            data.node_transform(0),
            Err(Error::InvalidReference {
                field: "transform",
                ..
            })
        ));
        assert!(data.node_transform(-1).is_err());
    }

    #[test]
    fn resolves_auxiliary_buffers_by_index_and_slices_part_attributes() {
        let mut verb = chunk(b"VERT", &[0; 64]);
        verb.extend(chunk(b"VERT", &[0; 96]));
        let tangents = [
            1u32.to_le_bytes().to_vec(),
            (1..=18)
                .flat_map(|value| (value as f32).to_le_bytes())
                .collect(),
        ]
        .concat();
        verb.extend(chunk(b"TANS", &tangents));
        let skin = [
            1u32.to_le_bytes().to_vec(),
            [0, 1, 255, 255, 128, 127, 0, 0].repeat(3),
        ]
        .concat();
        verb.extend(chunk(b"SKIN", &skin));
        verb.extend(chunk(
            b"BMAP",
            &[1i32, 10, 20]
                .into_iter()
                .flat_map(i32::to_le_bytes)
                .collect::<Vec<_>>(),
        ));
        let body = [
            chunk(b"VERB", &verb),
            chunk(b"MATE", &[0; 120]),
            chunk(b"SCEN", &[255, 255, 255, 255, 0, 0, 0, 0]),
            chunk(b"INDE", &[0, 0, 1, 0]),
        ]
        .concat();
        let mut data = ModelData::parse(&Chunk {
            id: *b"MDLD",
            data: &body,
        })
        .unwrap();
        let first = data.vertex_attributes(0).unwrap();
        assert_eq!(first.vertices.len(), 2);
        assert!(first.tangents.is_none());
        assert!(first.skin_weights.is_none());
        assert!(first.bone_map.is_none());
        let part = geometry::Part {
            primitive_type: 1,
            scene_node: 0,
            material: 0,
            vertex_buffer: 1,
            vertex_offset: 1,
            vertex_count: 2,
            index_offset: 0,
            index_count: 2,
            mouse_rectangle: -1,
        };
        let geometry = data.part_geometry(&part).unwrap();
        assert_eq!(geometry.tangents.unwrap()[0].first, [7.0, 8.0, 9.0]);
        assert_eq!(geometry.tangents.unwrap().len(), 2);
        assert_eq!(geometry.skin_weights.unwrap().len(), 2);
        assert_eq!(geometry.skin_weights.unwrap()[1].bones, [0, 1, 255, 255]);
        assert_eq!(geometry.bone_map.unwrap(), [10, 20]);
        assert!(data.vertex_attributes(-1).is_err());
        assert!(data.vertex_attributes(2).is_err());
        data.skin_weights[0].entries.pop();
        assert!(matches!(
            data.vertex_attributes(1),
            Err(Error::AttributeCount {
                section: [b'S', b'K', b'I', b'N'],
                expected: 3,
                actual: 2,
                ..
            })
        ));
        data.skin_weights.clear();
        data.tangents[0].entries.pop();
        assert!(matches!(
            data.vertex_attributes(1),
            Err(Error::AttributeCount {
                section: [b'T', b'A', b'N', b'S'],
                ..
            })
        ));
    }

    #[test]
    fn rejects_duplicate_auxiliary_buffers() {
        let mut data = ModelData {
            vertices: vec![vec![]],
            ..ModelData::default()
        };
        for section in [*b"TANS", *b"SKIN", *b"BMAP"] {
            match &section {
                b"TANS" => {
                    data.tangents = vec![
                        geometry::VertexBufferData {
                            buffer_index: 0,
                            entries: vec![]
                        };
                        2
                    ]
                }
                b"SKIN" => {
                    data.skin_weights = vec![
                        geometry::VertexBufferData {
                            buffer_index: 0,
                            entries: vec![]
                        };
                        2
                    ]
                }
                b"BMAP" => {
                    data.bone_maps = vec![
                        geometry::VertexBufferData {
                            buffer_index: 0,
                            entries: vec![]
                        };
                        2
                    ]
                }
                _ => unreachable!(),
            }
            assert!(
                matches!(data.vertex_attributes(0), Err(Error::DuplicateBuffer { section: actual, index: 0 }) if actual == section)
            );
            data.tangents.clear();
            data.skin_weights.clear();
            data.bone_maps.clear();
        }
    }
}

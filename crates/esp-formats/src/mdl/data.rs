use super::{Chunk, Error, attachment, geometry, material, parse_chunks, scene};
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
    pub animation_links: Vec<i32>,
    pub bone_references: Vec<i16>,
    pub lods: Vec<Lod<'a>>,
    pub platforms: Vec<attachment::Platform<'a>>,
    pub attachpoints: Vec<attachment::AttachPoint<'a>>,
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

impl<'a> ModelData<'a> {
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
        let mut lod = 100i32.to_le_bytes().to_vec();
        let part: Vec<_> = [1i32, -1, -1, 0, 0, 3, 0, 3, -1]
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
}

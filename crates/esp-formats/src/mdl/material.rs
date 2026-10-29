use super::{Error, geometry::records};
use binrw::BinRead;

#[derive(Debug, Clone, PartialEq, BinRead)]
#[br(little)]
pub struct Material {
    pub flags: u32,
    pub flags2: u32,
    pub diffuse_texture: i32,
    pub detail_texture: i32,
    pub bump_texture: i32,
    pub specular_texture: i32,
    pub emissive_texture: i32,
    pub reflection_texture: i32,
    pub fresnel_texture: i32,
    pub diffuse: [f32; 4],
    pub specular: [f32; 4],
    pub specular_power: f32,
    pub detail_scale: f32,
    pub bump_scale: f32,
    pub reflection_scale: f32,
    pub precipitation_offset: f32,
    pub specular_map_power_scale: f32,
    pub specular_bloom_floor: f32,
    pub ambient_light_scale: f32,
    pub source_blend: u32,
    pub destination_blend: u32,
    pub alpha_test_function: u32,
    pub alpha_test_threshold: f32,
    pub final_alpha_multiply: f32,
}

pub fn materials(data: &[u8]) -> Result<Vec<Material>, Error> {
    records(data, 120)
}

pub fn textures(data: &[u8]) -> Result<Vec<&[u8]>, Error> {
    if !data.len().is_multiple_of(64) {
        return Err(Error::InvalidSize);
    }
    Ok(data
        .as_chunks::<64>()
        .0
        .iter()
        .map(|name| {
            let end = name.iter().position(|&b| b == 0).unwrap_or(name.len());
            &name[..end]
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_all_material_fields() {
        let mut bytes = Vec::new();
        for value in [i32::MIN, 42, -1, 1, 2, 3, 4, 5, 6] {
            bytes.extend(value.to_le_bytes());
        }
        for value in 0..16 {
            bytes.extend((value as f32 + 0.5).to_le_bytes());
        }
        for value in [99u32, 2, 8] {
            bytes.extend(value.to_le_bytes());
        }
        bytes.extend(0.75f32.to_le_bytes());
        bytes.extend(1.0f32.to_le_bytes());
        assert_eq!(bytes.len(), 120);
        let material = materials(&bytes).unwrap().remove(0);
        assert_eq!(material.flags, 0x80000000);
        assert_eq!(material.diffuse_texture, -1);
        assert_eq!(material.fresnel_texture, 6);
        assert_eq!(material.specular, [4.5, 5.5, 6.5, 7.5]);
        assert_eq!(material.ambient_light_scale, 15.5);
        assert_eq!(material.source_blend, 99);
        assert_eq!(material.alpha_test_threshold, 0.75);
        assert_eq!(material.final_alpha_multiply, 1.0);
        for end in 1..120 {
            assert_eq!(materials(&bytes[..end]), Err(Error::InvalidSize));
        }
    }

    #[test]
    fn preserves_texture_bytes() {
        let mut bytes = [0u8; 192];
        bytes[..9].copy_from_slice(b"Cabin.dds");
        bytes[64..128].fill(0xff);
        let names = textures(&bytes).unwrap();
        assert_eq!(names[0], b"Cabin.dds");
        assert_eq!(names[1], &[0xff; 64]);
        assert!(names[2].is_empty());
        assert_eq!(textures(&bytes[..191]), Err(Error::InvalidSize));
        assert!(textures(&[]).unwrap().is_empty());
    }
}

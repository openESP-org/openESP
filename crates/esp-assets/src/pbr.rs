use anyhow::{Result, bail, ensure};
use esp_formats::mdl::material::{Material as Legacy, PbrMaterial};

use crate::Material;

/// Translate the common PBR surface properties to the renderer's material interface.
pub fn load(pbr: &PbrMaterial, textures: &[&[u8]]) -> Result<Material> {
    ensure!(
        pbr.albedo.iter().all(|v| v.is_finite())
            && pbr.metallic.is_finite()
            && pbr.smoothness.is_finite()
            && pbr.masked_threshold.is_finite(),
        "invalid PBR material values"
    );
    let resolve_texture =
        |index, kind| -> Result<Option<String>> {
            Ok(match index {
                -1 => None,
                index if index >= 0 => Some(
                    std::str::from_utf8(textures.get(index as usize).ok_or_else(|| {
                        anyhow::anyhow!("invalid PBR {kind} texture index {index}")
                    })?)?
                    .to_owned(),
                ),
                index => bail!("invalid PBR {kind} texture index {index}"),
            })
        };
    let diffuse_texture = resolve_texture(pbr.albedo_texture, "albedo")?;
    let metallic_texture = resolve_texture(pbr.metallic_texture, "metallic")?;
    let normal_texture = resolve_texture(pbr.normal_texture, "normal")?;
    let emissive_texture = resolve_texture(pbr.emissive_texture, "emissive")?;
    let (source_blend, destination_blend, alpha_flag) = match pbr.render_mode {
        0 => (2, 1, false),
        1 => (2, 1, true),
        2 => (5, 6, false),
        3 => (5, 2, false),
        mode => bail!("unsupported PBR render mode {mode}"),
    };
    let source = Legacy {
        flags: (if diffuse_texture.is_some() { 2 } else { 0 })
            | if alpha_flag { 0x00400000 } else { 0 },
        flags2: 0,
        diffuse_texture: pbr.albedo_texture,
        detail_texture: -1,
        bump_texture: -1,
        specular_texture: -1,
        emissive_texture: -1,
        reflection_texture: -1,
        fresnel_texture: -1,
        diffuse: pbr.albedo,
        specular: [0.0; 4],
        specular_power: 0.0,
        detail_scale: 1.0,
        bump_scale: 1.0,
        reflection_scale: 0.0,
        precipitation_offset: 0.0,
        specular_map_power_scale: 1.0,
        specular_bloom_floor: 0.0,
        ambient_light_scale: 1.0,
        source_blend,
        destination_blend,
        alpha_test_function: 5,
        alpha_test_threshold: pbr.masked_threshold * 255.0,
        final_alpha_multiply: 1.0,
    };
    Ok(Material {
        source,
        diffuse_texture,
        metallic_texture,
        normal_texture,
        emissive_texture,
        pbr: Some(pbr.clone()),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maps_pbr_mask_and_albedo_without_losing_parameters() {
        let mut bytes = [0u8; 184];
        for (word, value) in [
            (1, 1.0f32.to_bits()),
            (4, 1.0f32.to_bits()),
            (5, 0.6f32.to_bits()),
            (6, 0.5f32.to_bits()),
            (7, 1),
            (10, 0.4f32.to_bits()),
            (11, 0),
            (12, 1),
            (13, 2),
            (14, 3),
        ] {
            bytes[word * 4..word * 4 + 4].copy_from_slice(&value.to_le_bytes());
        }
        let pbr = esp_formats::mdl::material::pbr_materials(&bytes)
            .unwrap()
            .remove(0);
        let material = load(
            &pbr,
            &[
                b"albedo.dds",
                b"metallic.dds",
                b"normal.dds",
                b"emissive.dds",
            ],
        )
        .unwrap();
        assert_eq!(material.diffuse_texture.as_deref(), Some("albedo.dds"));
        assert_eq!(material.metallic_texture.as_deref(), Some("metallic.dds"));
        assert_eq!(material.normal_texture.as_deref(), Some("normal.dds"));
        assert_eq!(material.emissive_texture.as_deref(), Some("emissive.dds"));
        assert_eq!(material.source.flags & 0x00400000, 0x00400000);
        assert_eq!(material.source.alpha_test_function, 5);
        assert!((material.source.alpha_test_threshold / 255.0 - 0.4).abs() < 1e-6);
        assert_eq!(material.pbr.unwrap().metallic, 0.6);
        assert!(load(&pbr, &[]).is_err());
        assert!(load(&pbr, &[b"albedo.dds"]).is_err());
    }
}

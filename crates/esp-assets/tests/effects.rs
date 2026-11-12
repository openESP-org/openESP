use esp_assets::effects::LightEffects;

#[test]
#[ignore = "requires OPENESP_AIRCRAFT_PATHS and OPENESP_P3D_ROOT"]
fn loads_b737_lights_and_p3d_effect_textures() {
    let paths = std::env::var_os("OPENESP_AIRCRAFT_PATHS").unwrap();
    let aircraft = std::env::split_paths(&paths).next().unwrap();
    let root = std::path::PathBuf::from(std::env::var_os("OPENESP_P3D_ROOT").unwrap());
    let loaded = LightEffects::load(&aircraft, &root).unwrap();
    assert!(loaded.lights.len() >= 8);
    assert!(loaded.effects.contains_key("fx_strobe"));
    let strobe = &loaded.effects["fx_strobe"];
    assert!(strobe.iter().any(|particle| particle.delay[0] > 0.1));
    for effect in loaded.effects.values() {
        for particle in effect
            .iter()
            .filter(|particle| particle.kind == 19 && particle.is_static)
        {
            if let Some(name) = &particle.texture {
                let bytes = &loaded.textures[&name.to_ascii_lowercase()];
                let texture = esp_assets::texture::Texture::from_bytes(bytes, true).unwrap();
                assert!(texture.mips[0].width > 0 && texture.mips[0].height > 0);
            }
        }
    }
}

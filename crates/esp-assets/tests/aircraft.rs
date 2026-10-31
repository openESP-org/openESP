use esp_assets::Aircraft;

#[path = "../../../tests/support/content.rs"]
mod content;

#[test]
#[ignore = "requires OPENESP_FSX_ROOT or OPENESP_AIRCRAFT_PATHS"]
fn loads_external_aircraft() {
    for path in content::aircraft_paths() {
        let aircraft = Aircraft::load(&path, 0).unwrap();
        let mut textures = 0;
        let mut missing = std::collections::BTreeSet::new();
        for model in std::iter::once(&aircraft.exterior).chain(&aircraft.interior) {
            assert!(!model.lods[0].meshes.is_empty());
            assert!(model.transforms().unwrap().iter().all(|m| m.is_finite()));
            for material in &model.materials {
                if let Some(name) = &material.diffuse_texture {
                    match aircraft.texture_path(name) {
                        Ok(Some(path)) => {
                            let bytes = std::fs::read(&path).unwrap();
                            if bytes.starts_with(b"DDS ") {
                                esp_formats::texture::dds::Texture::parse(&bytes).unwrap();
                            } else {
                                esp_formats::texture::bmp::Bitmap::parse(&bytes).unwrap();
                            }
                            textures += 1;
                        }
                        Ok(None) => {}
                        Err(_) => {
                            missing.insert(name);
                        }
                    }
                }
            }
        }
        assert!(textures > 0);
        eprintln!(
            "{}: {textures} texture references validated; unresolved: {missing:?}",
            aircraft.title
        );
    }
}

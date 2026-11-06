use esp_assets::Model;

#[test]
#[ignore = "requires a Prepar3D PV20 model in OPENESP_P3D_MODEL"]
fn loads_prepar3d_model() {
    let path = std::env::var_os("OPENESP_P3D_MODEL").expect("set OPENESP_P3D_MODEL");
    let model = Model::load(std::path::Path::new(&path)).expect("load Prepar3D model");
    assert!(!model.materials.is_empty());
    assert!(
        model
            .lods
            .iter()
            .any(|lod| lod.meshes.iter().any(|mesh| !mesh.indices.is_empty()))
    );
}

#[test]
#[ignore = "requires a PV44 scenery library in OPENESP_P3D_LIBRARY"]
fn loads_real_pbr_scenery_models() {
    let path = std::env::var_os("OPENESP_P3D_LIBRARY").expect("set OPENESP_P3D_LIBRARY");
    let bytes = std::fs::read(path).unwrap();
    let bgl = esp_formats::bgl::Bgl::parse(&bytes).unwrap();
    let mut found = 0;
    for section in bgl
        .sections
        .iter()
        .filter(|s| s.kind == 0x2b && s.class == 5)
    {
        for record in section.guid_records().unwrap() {
            if record.data.get(8..12) != Some(b"PV44") {
                continue;
            }
            let decoded = esp_formats::mdl::Model::parse(record.data)
                .unwrap()
                .decode()
                .unwrap();
            let exterior = decoded.exterior.unwrap();
            assert!(!exterior.pbr_materials.is_empty());
            assert!(exterior.lods.iter().any(|lod| !lod.parts.is_empty()));
            let model = Model::from_bytes(record.data).expect("load PBR model");
            assert!(
                model
                    .materials
                    .iter()
                    .any(|material| material.pbr.is_some())
            );
            assert!(
                model
                    .materials
                    .iter()
                    .any(|material| material.pbr.is_some() && material.diffuse_texture.is_some())
            );
            found += 1;
        }
    }
    assert!(found > 0, "no PV44 models found");
}

#[test]
#[ignore = "requires a mixed-material P3D aircraft model in OPENESP_P3D_PBR_AIRCRAFT"]
fn loads_mixed_legacy_and_pbr_aircraft_materials() {
    let path = std::env::var_os("OPENESP_P3D_PBR_AIRCRAFT").unwrap();
    let model = Model::load(std::path::Path::new(&path)).unwrap();
    assert!(
        model
            .materials
            .iter()
            .any(|material| material.pbr.is_none())
    );
    assert!(
        model
            .materials
            .iter()
            .any(|material| material.pbr.is_some())
    );
    assert!(model.lods.iter().flat_map(|lod| &lod.meshes).any(|mesh| {
        mesh.vertices.len() > 1000 && model.materials[mesh.material].pbr.is_some()
    }));
    assert!(
        model
            .materials
            .iter()
            .any(|material| material.metallic_texture.is_some())
    );
    assert!(model.lods.iter().flat_map(|lod| &lod.meshes).any(|mesh| {
        model.materials[mesh.material].normal_texture.is_some()
            && mesh
                .tangents
                .as_ref()
                .is_some_and(|basis| basis.len() == mesh.vertices.len())
    }));
}

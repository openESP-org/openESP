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
fn identifies_real_pbr_scenery_models() {
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
            assert!(
                exterior
                    .unknown
                    .iter()
                    .any(|section| &section.id == b"PBRM" && !section.data.is_empty())
            );
            assert!(exterior.lods.iter().any(|lod| !lod.parts.is_empty()));
            let error = Model::from_bytes(record.data)
                .err()
                .expect("PBR rendering must be identified as unsupported");
            assert!(error.to_string().contains("PBR materials"));
            found += 1;
        }
    }
    assert!(found > 0, "no PV44 models found");
}

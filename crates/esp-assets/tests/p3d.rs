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

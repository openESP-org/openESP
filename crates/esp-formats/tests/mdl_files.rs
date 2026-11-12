use esp_formats::mdl::Model;

#[path = "../../../tests/support/models.rs"]
mod models;

#[test]
#[ignore = "requires the original B737 in OPENESP_AIRCRAFT_PATHS"]
fn b737_effect_names_resolve_to_model_attachpoints() {
    let paths = std::env::var_os("OPENESP_AIRCRAFT_PATHS").unwrap();
    let root = std::env::split_paths(&paths).next().unwrap();
    let bytes = std::fs::read(root.join("model/B737_800.mdl")).unwrap();
    let model = Model::parse(&bytes).unwrap();
    let decoded = model.decode().unwrap();
    let data = decoded.exterior.unwrap();
    let effects = data
        .attached_objects
        .iter()
        .filter_map(|object| object.definition.as_ref())
        .filter(|definition| {
            matches!(
                definition.value,
                esp_formats::mdl::attachment::ObjectValue::Effect { .. }
            )
        })
        .collect::<Vec<_>>();
    assert_eq!(effects.len(), 3);
    for effect in effects {
        let point = data
            .attachpoints
            .iter()
            .find(|point| point.name.eq_ignore_ascii_case(effect.attachpoint))
            .expect("effect has no matching attachpoint");
        assert!((point.scene_node as usize) < data.nodes.len());
    }
}

#[test]
#[ignore = "requires ESP_FORMATS_MDL_FILES, OPENESP_FSX_ROOT or OPENESP_AIRCRAFT_PATHS"]
fn decodes_external_models() {
    let mut parts = 0;
    for path in models::model_paths() {
        let bytes =
            std::fs::read(&path).unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        let model =
            Model::parse(&bytes).unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        let decoded = model
            .decode()
            .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        for data in [decoded.exterior, decoded.shadow].into_iter().flatten() {
            for index in 0..data.vertices.len() {
                data.vertex_attributes(index.try_into().unwrap())
                    .unwrap_or_else(|error| {
                        panic!("{} vertex buffer {index}: {error}", path.display())
                    });
            }
            for material in &data.materials {
                material
                    .texture_maps(&data.textures)
                    .unwrap_or_else(|error| {
                        panic!("{} material textures: {error}", path.display())
                    });
            }
            let parents = esp_formats::mdl::scene::parents(&data.nodes)
                .unwrap_or_else(|error| panic!("{} scene hierarchy: {error}", path.display()));
            assert_eq!(parents.len(), data.nodes.len());
            for index in 0..data.nodes.len() {
                data.node_transform(index.try_into().unwrap())
                    .unwrap_or_else(|error| {
                        panic!("{} scene node {index}: {error}", path.display())
                    });
            }
            for lod in &data.lods {
                for part in &lod.parts {
                    data.part_geometry(part).unwrap_or_else(|error| {
                        panic!("{} LOD {}: {error}", path.display(), lod.level)
                    });
                    parts += 1;
                }
            }
        }
    }
    assert!(parts > 0, "no geometry parts were tested");
}

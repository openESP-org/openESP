use esp_formats::mdl::Model;

#[test]
#[ignore = "requires ESP_FORMATS_MDL_FILES containing paths to external MDLX files"]
fn decodes_external_models() {
    let paths = std::env::var_os("ESP_FORMATS_MDL_FILES").expect("set ESP_FORMATS_MDL_FILES");
    let mut parts = 0;
    for path in std::env::split_paths(&paths) {
        let bytes =
            std::fs::read(&path).unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        let model =
            Model::parse(&bytes).unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        let decoded = model
            .decode()
            .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        for data in [decoded.exterior, decoded.shadow].into_iter().flatten() {
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

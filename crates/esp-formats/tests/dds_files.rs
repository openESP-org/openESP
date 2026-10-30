use esp_formats::texture::dds::Texture;

#[test]
#[ignore = "requires ESP_FORMATS_DDS_FILES containing paths to external DDS files"]
fn decodes_external_textures() {
    let paths = std::env::var_os("ESP_FORMATS_DDS_FILES").expect("set ESP_FORMATS_DDS_FILES");
    let mut count = 0;
    for path in std::env::split_paths(&paths) {
        let bytes = std::fs::read(&path).unwrap();
        let texture = Texture::parse(&bytes).unwrap_or_else(|error| panic!("{path:?}: {error}"));
        let mut size = texture.raw_header().len() + texture.trailing_data().len();
        for layer in 0..texture.layer_count() {
            assert!(texture.layer(layer).is_some());
            for mip in 0..texture.mip_count() {
                let surface = texture.surface(layer, mip).unwrap();
                assert_eq!(
                    surface.data.len(),
                    surface.slice_pitch * surface.depth as usize
                );
                size += surface.data.len();
            }
        }
        assert_eq!(size, bytes.len());
        count += 1;
    }
    assert!(count > 0);
}

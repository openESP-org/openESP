use esp_formats::texture::bmp::Bitmap;

#[test]
#[ignore = "requires ESP_FORMATS_COMPRESSED_BITMAP_FILES containing paths to external FS70 DXT textures"]
fn parses_external_compressed_surfaces() {
    let paths = std::env::var_os("ESP_FORMATS_COMPRESSED_BITMAP_FILES")
        .expect("set ESP_FORMATS_COMPRESSED_BITMAP_FILES");
    let mut count = 0;
    for path in std::env::split_paths(&paths) {
        let bytes = std::fs::read(&path).unwrap();
        let bitmap = Bitmap::parse(&bytes).unwrap();
        let surface = bitmap
            .compressed_surface()
            .unwrap_or_else(|error| panic!("{path:?}: {error}"))
            .expect("expected FS70 DXT surface");
        assert_eq!(
            surface.data.len() + surface.remaining_data.len(),
            bitmap.pixels().len()
        );
        assert!(surface.width > 0 && surface.stored_height != 0);
        count += 1;
    }
    assert!(count > 0);
}

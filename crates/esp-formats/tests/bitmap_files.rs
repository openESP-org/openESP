use esp_formats::texture::bmp::Bitmap;
use image::Limits;

#[test]
#[ignore = "requires ESP_FORMATS_BITMAP_FILES containing paths to external standard BMP artwork"]
fn decodes_external_bitmap_artwork() {
    let paths = std::env::var_os("ESP_FORMATS_BITMAP_FILES").expect("set ESP_FORMATS_BITMAP_FILES");
    let mut count = 0;
    for path in std::env::split_paths(&paths) {
        let bytes = std::fs::read(&path).unwrap();
        let bmp = Bitmap::parse(&bytes).unwrap_or_else(|error| panic!("{path:?}: {error}"));
        let image = bmp
            .decode(Limits::default())
            .unwrap_or_else(|error| panic!("{path:?}: {error}"));
        assert!(image.width() > 0 && image.height() > 0);
        assert_eq!(
            14 + bmp.dib_header().len()
                + bmp.metadata().len()
                + bmp.pixels().len()
                + bmp.trailing_data().len(),
            bytes.len()
        );
        count += 1;
    }
    assert!(count > 0);
}

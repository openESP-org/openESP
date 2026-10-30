use esp_formats::cfg::{Config, camera};

#[test]
#[ignore = "requires ESP_FORMATS_CAMERA_FILES containing paths to external camera definitions"]
fn parses_external_camera_definitions() {
    let paths = std::env::var_os("ESP_FORMATS_CAMERA_FILES").expect("set ESP_FORMATS_CAMERA_FILES");
    let mut count = 0;
    for path in std::env::split_paths(&paths) {
        let data = std::fs::read(&path).unwrap();
        let config = Config::from_bytes(&data, encoding_rs::WINDOWS_1252).unwrap();
        let cameras =
            camera::definitions(&config).unwrap_or_else(|error| panic!("{path:?}: {error}"));
        assert!(!cameras.is_empty(), "{path:?}");
        count += 1;
    }
    assert!(count > 0);
}

use esp_formats::cfg::{Config, aircraft::configurations};

#[test]
#[ignore = "requires ESP_FORMATS_AIRCRAFT_FILES containing paths to external aircraft.cfg files"]
fn decodes_external_aircraft_references() {
    let paths =
        std::env::var_os("ESP_FORMATS_AIRCRAFT_FILES").expect("set ESP_FORMATS_AIRCRAFT_FILES");
    let mut count = 0;
    for path in std::env::split_paths(&paths) {
        let bytes = std::fs::read(&path).unwrap();
        let config = Config::from_bytes(&bytes, encoding_rs::WINDOWS_1252).unwrap();
        let variants = configurations(&config).unwrap_or_else(|error| panic!("{path:?}: {error}"));
        assert!(!variants.is_empty());
        for variant in variants {
            assert!(variant.title.is_some());
            assert!(variant.sim.is_some());
        }
        count += 1;
    }
    assert!(count > 0);
}

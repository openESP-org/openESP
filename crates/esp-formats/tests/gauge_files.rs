use esp_formats::gauge::Gauge;

#[test]
#[ignore = "requires ESP_FORMATS_GAUGE_FILES containing paths to external XML gauges"]
fn parses_external_gauge_documents() {
    let paths = std::env::var_os("ESP_FORMATS_GAUGE_FILES").expect("set ESP_FORMATS_GAUGE_FILES");
    let mut count = 0;
    for path in std::env::split_paths(&paths) {
        let bytes = std::fs::read(&path).unwrap();
        let (encoding, skip) =
            encoding_rs::Encoding::for_bom(&bytes).unwrap_or((encoding_rs::UTF_8, 0));
        let text = encoding
            .decode_without_bom_handling_and_without_replacement(&bytes[skip..])
            .unwrap();
        let gauge = Gauge::parse(&text).unwrap_or_else(|error| panic!("{path:?}: {error}"));
        gauge.size().unwrap();
        let images = gauge
            .image_references()
            .collect::<Result<Vec<_>, _>>()
            .unwrap_or_else(|error| panic!("{path:?}: {error}"));
        assert_eq!(images.len(), gauge.images().count());
        for image in gauge.images() {
            assert!(image.attribute("Name").is_some(), "{path:?}: {image:?}");
        }
        count += 1;
    }
    assert!(count > 0);
}

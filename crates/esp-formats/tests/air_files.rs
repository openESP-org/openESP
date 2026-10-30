use esp_formats::air;

#[test]
#[ignore = "requires ESP_FORMATS_AIR_FILES containing paths to external AIR files"]
fn parses_external_flight_dynamics() {
    let paths = std::env::var_os("ESP_FORMATS_AIR_FILES").expect("set ESP_FORMATS_AIR_FILES");
    let mut count = 0;
    for path in std::env::split_paths(&paths) {
        let bytes = std::fs::read(&path).unwrap();
        let records = air::parse(&bytes).unwrap_or_else(|error| panic!("{path:?}: {error}"));
        assert!(!records.is_empty());
        for record in &records {
            record.aerodynamic_parameters().unwrap();
        }
        assert_eq!(
            records
                .iter()
                .map(|record| 8 + record.data.len())
                .sum::<usize>(),
            bytes.len()
        );
        count += 1;
    }
    assert!(count > 0);
}

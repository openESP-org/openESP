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
            record.table().unwrap();
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

#[test]
#[ignore = "requires ESP_FORMATS_AIR_FILES with original piston engine and propeller tables"]
fn decodes_external_piston_and_propeller_tables() {
    use air::table::Table;
    let paths = std::env::var_os("ESP_FORMATS_AIR_FILES").expect("set ESP_FORMATS_AIR_FILES");
    let mut curves = 0;
    let mut surfaces = 0;
    for path in std::env::split_paths(&paths) {
        let data = std::fs::read(&path).unwrap();
        for record in air::parse(&data).unwrap() {
            match (record.id, record.table().unwrap()) {
                (0x506..=0x509, Some(Table::Curve(curve))) => {
                    assert!(curve.points.len() >= 2);
                    assert!(curve.points.windows(2).all(|p| p[0].input < p[1].input));
                    curves += 1;
                }
                (0x511 | 0x512, Some(Table::Surface(surface))) => {
                    let cols = surface.columns as usize;
                    let rows = surface.rows as usize;
                    assert!(cols >= 3 && rows >= 3);
                    assert!(surface.values[1..cols].windows(2).all(|p| p[0] < p[1]));
                    assert!(
                        (1..rows - 1)
                            .all(|r| surface.values[r * cols] < surface.values[(r + 1) * cols])
                    );
                    assert!(surface.values.iter().all(|v| v.is_finite()));
                    eprintln!(
                        "{}: propeller table {:#x}: {rows} rows, {cols} columns",
                        path.display(),
                        record.id
                    );
                    surfaces += 1;
                }
                _ => {}
            }
        }
    }
    assert!(
        curves > 0 && surfaces >= 2,
        "no original piston/propeller data was exercised"
    );
}

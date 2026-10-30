use esp_formats::cabinet::Cabinet;

#[test]
#[ignore = "requires ESP_FORMATS_CABINET_FILES containing paths to external CAB files"]
fn reads_external_gauge_resources() {
    let paths =
        std::env::var_os("ESP_FORMATS_CABINET_FILES").expect("set ESP_FORMATS_CABINET_FILES");
    let mut count = 0;
    for path in std::env::split_paths(&paths) {
        let mut cabinet = Cabinet::new(std::fs::File::open(&path).unwrap()).unwrap();
        let files: Vec<_> = cabinet
            .entries()
            .iter()
            .map(|file| (file.name.to_owned(), file.size))
            .collect();
        assert!(!files.is_empty());
        for (name, size) in files {
            let bytes = cabinet
                .read_file_bytes(&name, 64 * 1024 * 1024)
                .unwrap_or_else(|error| panic!("{path:?}, {name}: {error}"));
            assert_eq!(bytes.len() as u64, u64::from(size));
        }
        count += 1;
    }
    assert!(count > 0);
}

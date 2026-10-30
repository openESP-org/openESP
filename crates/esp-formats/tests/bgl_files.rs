use esp_formats::bgl::{Bgl, Directory};

#[test]
#[ignore = "requires ESP_FORMATS_BGL_FILES containing paths to external BGL files"]
fn decodes_external_containers() {
    let paths = std::env::var_os("ESP_FORMATS_BGL_FILES").expect("set ESP_FORMATS_BGL_FILES");
    let mut count = 0;
    for path in std::env::split_paths(&paths) {
        let bytes = std::fs::read(&path).unwrap();
        let bgl = Bgl::parse(&bytes).unwrap_or_else(|error| panic!("{path:?}: {error}"));
        assert!(!bgl.sections.is_empty());
        for section in &bgl.sections {
            match &section.directory {
                Directory::Direct(subsections) => {
                    assert_eq!(subsections.len(), section.entry_count as usize)
                }
                Directory::Indirect { subsections, .. } => {
                    assert_eq!(subsections.len(), section.entry_count as usize)
                }
                Directory::Unknown => panic!("unexpected directory class {}", section.class),
            }
        }
        count += 1;
    }
    assert!(count > 0);
}

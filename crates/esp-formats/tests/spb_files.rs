use esp_formats::spb::{Bank, propdef::Definitions};
use std::collections::HashMap;

#[test]
#[ignore = "requires ESP_FORMATS_SPB_FILES and ESP_FORMATS_PROPDEF_FILES containing external file paths"]
fn decodes_external_fsx_property_banks() {
    let schemas =
        std::env::var_os("ESP_FORMATS_PROPDEF_FILES").expect("set ESP_FORMATS_PROPDEF_FILES");
    let mut bindings = HashMap::new();
    for path in std::env::split_paths(&schemas) {
        let text = std::fs::read_to_string(&path).unwrap();
        let definitions =
            Definitions::parse(&text).unwrap_or_else(|error| panic!("{path:?}: {error}"));
        for (guid, binding) in definitions.bindings() {
            if let Some(previous) = bindings.insert(guid, binding) {
                assert_eq!(previous, binding, "conflicting definition {guid}");
            }
        }
    }
    let paths = std::env::var_os("ESP_FORMATS_SPB_FILES").expect("set ESP_FORMATS_SPB_FILES");
    let mut count = 0;
    for path in std::env::split_paths(&paths) {
        let bytes = std::fs::read(&path).unwrap();
        let bank = Bank::parse_fsx(&bytes).unwrap_or_else(|error| panic!("{path:?}: {error}"));
        let document = bank
            .decode(&bindings)
            .unwrap_or_else(|error| panic!("{path:?}: {error}"));
        assert!(!document.nodes.is_empty());
        for (index, node) in document.nodes.iter().enumerate() {
            assert!(node.parent.is_none_or(|parent| parent < index));
        }
        count += 1;
    }
    assert!(count > 0);
}

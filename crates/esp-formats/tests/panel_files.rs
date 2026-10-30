use esp_formats::cfg::{Config, panel::Panel};

#[test]
#[ignore = "requires ESP_FORMATS_PANEL_FILES containing paths to external panel.cfg files"]
fn decodes_external_panel_layouts() {
    let paths = std::env::var_os("ESP_FORMATS_PANEL_FILES").expect("set ESP_FORMATS_PANEL_FILES");
    let mut count = 0;
    for path in std::env::split_paths(&paths) {
        let bytes = std::fs::read(&path).unwrap();
        let config = Config::from_bytes(&bytes, encoding_rs::WINDOWS_1252).unwrap();
        let panel = Panel::from_config(&config).unwrap_or_else(|error| panic!("{path:?}: {error}"));
        assert!(panel.alias.is_some() || !panel.windows.is_empty());
        for window in panel.windows {
            for gauge in window.gauges {
                assert!(!gauge.reference.is_empty());
            }
        }
        count += 1;
    }
    assert!(count > 0);
}

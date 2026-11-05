use esp_assets::texture::gauge_bitmap;
use esp_formats::cabinet::Cabinet;

#[path = "../../../tests/support/content.rs"]
mod content;

#[test]
#[ignore = "requires OPENESP_AIRCRAFT_PATHS with the B737_800 gauge CAB"]
fn stock_737_vertical_speed_wheel_dispatches_increment() {
    use esp_script::{Error, Host, Reference, Value};
    #[derive(Default)]
    struct HostState(Vec<String>);
    impl Host for HostState {
        fn read(&mut self, _: &Reference) -> Result<Value, Error> {
            Ok(Value::Number(0.0))
        }
        fn write(&mut self, reference: &Reference, _: Value) -> Result<(), Error> {
            if reference.category == 'K' {
                self.0.push(reference.name.clone());
            }
            Ok(())
        }
    }
    let mut checked = false;
    for path in content::aircraft_paths() {
        if esp_assets::resolve_path(&path, "panel/B737_800.cab").is_err() {
            continue;
        }
        let aircraft = esp_assets::Aircraft::load(&path, 0).unwrap();
        let panel = aircraft.panel.as_ref().unwrap();
        let image = esp_assets::gauge::load_panel(panel, "$737_1")
            .unwrap()
            .unwrap();
        let mut host = HostState::default();
        image.left_click(642.0, 870.0, &mut host).unwrap();
        assert!(host.0.iter().any(|event| event == "AP_VS_VAR_INC"));
        checked = true;
    }
    assert!(checked, "no 737 gauge package found");
}

#[test]
#[ignore = "requires OPENESP_AIRCRAFT_PATHS with the B737_800 gauge CAB"]
fn decodes_real_737_translucent_gauge_art() {
    let mut checked = 0;
    for aircraft in content::aircraft_paths() {
        let Ok(path) = esp_assets::resolve_path(&aircraft, "panel/B737_800.cab") else {
            continue;
        };
        let mut cab = Cabinet::new(std::fs::File::open(path).unwrap()).unwrap();
        for name in [
            "compass_shadow.bmp",
            "compass_highlight.bmp",
            "attitude_backup_shadow.bmp",
        ] {
            let bytes = cab.read_file_bytes(name, 16 * 1024 * 1024).unwrap();
            let image = gauge_bitmap(&bytes, true, true).unwrap();
            assert!(
                image.pixels().any(|p| p[3] == 0),
                "{name} has no transparent background"
            );
            assert!(
                image.pixels().any(|p| p[3] > 0 && p[3] < 255),
                "{name} has no translucent pixels"
            );
            checked += 1;
        }
    }
    assert!(checked > 0, "no 737 alpha artwork found");
}

#[test]
#[ignore = "requires OPENESP_FSX_ROOT or OPENESP_AIRCRAFT_PATHS with a gauge CAB"]
fn decodes_external_gauge_artwork() {
    let mut decoded = 0;
    for aircraft in content::aircraft_paths() {
        let Ok(panel) = esp_assets::resolve_path(&aircraft, "panel") else {
            continue;
        };
        for entry in std::fs::read_dir(panel).unwrap() {
            let path = entry.unwrap().path();
            if !path
                .extension()
                .is_some_and(|ext| ext.eq_ignore_ascii_case("cab"))
            {
                continue;
            }
            let mut cabinet = Cabinet::new(std::fs::File::open(&path).unwrap()).unwrap();
            let names: Vec<_> = cabinet
                .entries()
                .iter()
                .filter(|e| e.name.to_ascii_lowercase().ends_with(".bmp"))
                .map(|e| e.name.clone())
                .collect();
            for name in names.iter().take(5) {
                let bytes = cabinet.read_file_bytes(name, 64 * 1024 * 1024).unwrap();
                let image = gauge_bitmap(&bytes, false, true)
                    .unwrap_or_else(|e| panic!("{}: {name}: {e}", path.display()));
                assert!(image.width() > 0 && image.height() > 0);
                decoded += 1;
            }
        }
    }
    assert!(decoded > 0, "no gauge bitmap artwork found");
    eprintln!("decoded {decoded} gauge bitmaps");
}

#[test]
#[ignore = "requires OPENESP_FSX_ROOT or OPENESP_AIRCRAFT_PATHS with local XML gauges"]
fn resolves_configured_xml_gauges() {
    let mut count = 0;
    for path in content::aircraft_paths() {
        let aircraft = esp_assets::Aircraft::load(&path, 0).unwrap();
        let Some(panel) = aircraft.panel else {
            continue;
        };
        let layout = panel.layout().unwrap();
        for gauge in layout
            .windows
            .iter()
            .filter(|window| window.kind == esp_formats::cfg::panel::WindowKind::VirtualCockpit)
            .flat_map(|window| &window.gauges)
        {
            if let Some(bytes) = panel
                .read_xml_gauge(gauge.reference, 8 * 1024 * 1024)
                .unwrap()
            {
                let text = std::str::from_utf8(&bytes).unwrap();
                let parsed = esp_formats::gauge::Gauge::parse(text).unwrap();
                assert!(parsed.size().unwrap().is_some());
                count += 1;
            }
        }
    }
    assert!(count > 0, "no configured local XML gauges found");
    eprintln!("resolved {count} configured XML gauges");
}

#[test]
#[ignore = "requires OPENESP_FSX_ROOT or OPENESP_AIRCRAFT_PATHS with local XML gauges"]
fn composes_external_cockpit_artwork() {
    let mut count = 0;
    for path in content::aircraft_paths() {
        let aircraft = esp_assets::Aircraft::load(&path, 0).unwrap();
        let Some(panel) = &aircraft.panel else {
            continue;
        };
        for window in panel
            .layout()
            .unwrap()
            .windows
            .iter()
            .filter(|w| w.kind == esp_formats::cfg::panel::WindowKind::VirtualCockpit)
        {
            let Some(name) = window.texture else { continue };
            let image = esp_assets::gauge::load_panel(panel, name).unwrap().unwrap();
            eprintln!(
                "{} {name}: {} omitted gauges, {} omitted elements",
                aircraft.title, image.omitted_gauges, image.omitted_elements
            );
            assert!(image.image.width() > 0 && image.image.height() > 0);
            if image.omitted_gauges < window.gauges.len() {
                count += 1;
            }
        }
    }
    assert!(count > 0, "no local gauges composed");
}

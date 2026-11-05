use esp_formats::cfg::{Config, aircraft::configurations};

#[path = "../../../tests/support/content.rs"]
mod content;

#[test]
#[ignore = "requires OPENESP_AIRCRAFT_PATHS with rotorcraft aircraft.cfg files"]
fn decodes_external_helicopter_rotors() {
    use esp_formats::cfg::helicopter::{
        FuselageAerodynamics, Helicopter, MainRotor, SecondaryRotor,
    };
    let mut configured = 0;
    for path in content::aircraft_paths() {
        let path = path.join("aircraft.cfg");
        let config =
            Config::from_bytes(&std::fs::read(&path).unwrap(), encoding_rs::WINDOWS_1252).unwrap();
        let main = MainRotor::from_config(&config)
            .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        Helicopter::from_config(&config)
            .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        FuselageAerodynamics::from_config(&config)
            .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        SecondaryRotor::from_config(&config)
            .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        if main.radius_feet.is_some() {
            configured += 1;
        }
    }
    assert!(
        configured > 0,
        "no helicopter main rotor configuration found"
    );
}

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

#[test]
#[ignore = "requires ESP_FORMATS_AIRCRAFT_FILES with original wheeled aircraft.cfg files"]
fn decodes_external_landing_gear_and_preserves_malformed_contacts() {
    let paths =
        std::env::var_os("ESP_FORMATS_AIRCRAFT_FILES").expect("set ESP_FORMATS_AIRCRAFT_FILES");
    let mut wheels = 0;
    for path in std::env::split_paths(&paths) {
        let config =
            Config::from_bytes(&std::fs::read(&path).unwrap(), encoding_rs::WINDOWS_1252).unwrap();
        let contacts = esp_formats::cfg::contact::ContactPoints::from_config(&config).unwrap();
        let authored = config
            .sections
            .iter()
            .filter(|(section, _)| {
                section.is_some_and(|s| s.eq_ignore_ascii_case("contact_points"))
            })
            .flat_map(|(_, properties)| properties.iter())
            .filter(|(key, _)| key.to_ascii_lowercase().starts_with("point."))
            .count();
        assert_eq!(
            contacts.points.len() + contacts.rejected.len(),
            authored,
            "contact rows disappeared in {path:?}"
        );
        let count = contacts.points.iter().filter(|p| p.class == 1).count();
        wheels += count;
        for point in &contacts.rejected {
            assert!(!point.source.is_empty());
            eprintln!(
                "{}: contact {}: {}",
                path.display(),
                point.index,
                point.error
            );
        }
        eprintln!(
            "{}: {count} wheels, {} parsed contacts, {} malformed contacts",
            path.display(),
            contacts.points.len(),
            contacts.rejected.len()
        );
    }
    assert!(wheels > 0, "no original landing gear was decoded");
}

use esp_assets::flight::FlightData;
use esp_formats::cfg::electrical::Electrical;
use esp_script::{Host, Reference, Value};
use esp_sim::variables::Variables;

#[path = "../../../tests/support/content.rs"]
mod content;

#[test]
#[ignore = "requires original aircraft with configured electrical circuits"]
fn original_aircraft_battery_load_tracks_lighting_and_generator() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let definition = Electrical::from_config(&data.configuration).unwrap();
        let Some(generator) = definition.generators.iter().find(|g| g.count > 0) else {
            continue;
        };
        let Some(nav) = definition
            .circuits
            .iter()
            .find(|c| c.name == "light_nav" && c.bus == 0 && c.amperes > 0.0)
        else {
            continue;
        };
        if definition.always_available
            || definition.battery_voltage < nav.minimum_voltage
            || definition.generator_voltage < nav.minimum_voltage
        {
            continue;
        }
        let mut variables = Variables::default();
        variables.configure_electrical(&definition, false).unwrap();
        variables.set_number("LIGHT NAV", "bool", 0.0);
        variables
            .dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
            .unwrap();
        variables.update_electrical().unwrap();
        let read = |v: &mut Variables| {
            v.read(&Reference::new(
                'A',
                "ELECTRICAL BATTERY LOAD",
                Some("amps"),
            ))
            .unwrap()
            .number()
            .unwrap()
        };
        let initial = read(&mut variables);
        variables.set_number("LIGHT NAV", "bool", 1.0);
        variables.update_electrical().unwrap();
        assert!((read(&mut variables) - (initial - nav.amperes)).abs() < 1e-9);
        variables.set_number(
            &format!("GENERAL ENG GENERATOR SWITCH:{}", generator.engine),
            "bool",
            1.0,
        );
        variables.set_number(
            &format!("GENERAL ENG COMBUSTION:{}", generator.engine),
            "bool",
            1.0,
        );
        variables.update_electrical().unwrap();
        assert!(read(&mut variables) > initial - nav.amperes);
        verified += 1;
    }
    assert!(verified > 0);
}

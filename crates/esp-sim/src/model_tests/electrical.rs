use crate::electrical::*;
use crate::variables::Variables;
use esp_formats::cfg::Config;
use esp_formats::cfg::electrical::Electrical as Definition;
use esp_physics::variables::VariableStore;
use esp_script::Reference;
use esp_script::Value;

fn number(variables: &mut dyn VariableStore, name: &str, unit: &str) -> f64 {
    variables
        .read(&Reference::new('A', name, Some(unit)))
        .unwrap()
        .number()
        .unwrap()
}

#[test]
fn fuel_pump_and_starter_loads_follow_switches_and_power() {
    let config = Config::parse("[GeneralEngineData]\nengine.0=0,0,0\nengine.1=0,0,0\n[electrical]\nmax_battery_voltage=24\nadditional_system=0,2,17\nfuel_pump=0,5,17\nstarter1=0,20,17").unwrap();
    let mut variables = Variables::default();
    variables.configure_engines([1, 2].into_iter()).unwrap();
    variables
        .configure_electrical(&Definition::from_config(&config).unwrap(), false)
        .unwrap();
    variables
        .dispatch_event("TOGGLE_ELECT_FUEL_PUMP1", Value::Number(0.0))
        .unwrap();
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "GENERAL ENG FUEL PUMP SWITCH:1", "bool"),
        1.0
    );
    assert_eq!(
        number(&mut variables, "GENERAL ENG FUEL PUMP ON:1", "bool"),
        0.0
    );
    assert_eq!(
        number(&mut variables, "ELECTRICAL BATTERY LOAD", "amps"),
        0.0
    );
    variables
        .dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
        .unwrap();
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "GENERAL ENG FUEL PUMP ON:1", "bool"),
        1.0
    );
    assert_eq!(
        number(&mut variables, "GENERAL ENG FUEL PUMP ON:2", "bool"),
        0.0
    );
    assert_eq!(
        number(&mut variables, "ELECTRICAL BATTERY LOAD", "amps"),
        -7.0
    );
    variables.set_number("GENERAL ENG STARTER ACTIVE:1", "bool", 1.0);
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "ELECTRICAL BATTERY LOAD", "amps"),
        -27.0
    );
    variables.set_number("GENERAL ENG STARTER ACTIVE:1", "bool", 0.0);
    variables
        .dispatch_event("TOGGLE_ELECT_FUEL_PUMP1", Value::Number(0.0))
        .unwrap();
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "GENERAL ENG FUEL PUMP ON:1", "bool"),
        0.0
    );
    assert_eq!(
        number(&mut variables, "ELECTRICAL BATTERY LOAD", "amps"),
        -2.0
    );
    variables
        .dispatch_event("TOGGLE_ELECT_FUEL_PUMP", Value::Number(0.0))
        .unwrap();
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "GENERAL ENG FUEL PUMP ON:1", "bool"),
        1.0
    );
    assert_eq!(
        number(&mut variables, "GENERAL ENG FUEL PUMP ON:2", "bool"),
        1.0
    );
    assert!(
        variables
            .dispatch_event("TOGGLE_ELECT_FUEL_PUMP3", Value::Number(0.0))
            .is_err()
    );
    variables
        .dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
        .unwrap();
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "GENERAL ENG FUEL PUMP ON:1", "bool"),
        0.0
    );
}
#[test]
fn generator_output_rises_with_shaft_speed_and_respects_cut_in() {
    let config = Config::parse("[GeneralEngineData]\nengine.0=0,0,0\n[electrical]\nmax_battery_voltage=24\ngenerator_alternator_voltage=28\nmax_generator_alternator_amps=20\nadditional_system=0,10,17").unwrap();
    let definition = Definition::from_config(&config).unwrap();
    let mut variables = Variables::default();
    variables.configure_electrical(&definition, false).unwrap();
    variables
        .dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
        .unwrap();
    variables
        .dispatch_event("TOGGLE_ALTERNATOR1", Value::Number(0.0))
        .unwrap();
    variables.set_number("GENERAL ENG COMBUSTION:1", "bool", 1.0);
    variables.set_number("GENERAL ENG PCT MAX RPM:1", "percent", 10.0);
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "GENERAL ENG GENERATOR ACTIVE:1", "bool"),
        0.0
    );
    assert_eq!(
        number(&mut variables, "ELECTRICAL BATTERY LOAD", "amps"),
        -10.0
    );
    variables.set_number("GENERAL ENG PCT MAX RPM:1", "percent", 20.0);
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "GENERAL ENG GENERATOR ACTIVE:1", "bool"),
        1.0
    );
    assert!((number(&mut variables, "ELECTRICAL BATTERY LOAD", "amps") + 6.0).abs() < 1e-9);
    variables.set_number("GENERAL ENG PCT MAX RPM:1", "percent", 40.0);
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "ELECTRICAL BATTERY LOAD", "amps"),
        0.0
    );
    variables.set_number("GENERAL ENG PCT MAX RPM:1", "percent", 0.0);
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "GENERAL ENG GENERATOR ACTIVE:1", "bool"),
        0.0
    );
    let mut turbine = Variables::default();
    turbine.configure_electrical(&definition, false).unwrap();
    turbine.set_number("GENERAL ENG GENERATOR SWITCH:1", "bool", 1.0);
    turbine.set_number("GENERAL ENG COMBUSTION:1", "bool", 1.0);
    turbine.set_number("TURB ENG N2:1", "percent", 20.0);
    turbine.update_electrical().unwrap();
    assert_eq!(
        number(&mut turbine, "GENERAL ENG GENERATOR ACTIVE:1", "bool"),
        0.0
    );
    turbine.set_number("TURB ENG N2:1", "percent", 55.0);
    turbine.update_electrical().unwrap();
    assert_eq!(
        number(&mut turbine, "GENERAL ENG GENERATOR ACTIVE:1", "bool"),
        1.0
    );
}
#[test]
fn battery_discharge_and_recharge_follow_simulation_time() {
    let config = Config::parse("[GeneralEngineData]\nengine.0=0,0,0\n[electrical]\nmax_battery_voltage=24\nbattery_max_capacity_amph=1\nbattery_charging_current=1\ngenerator_alternator_voltage=28\nmax_generator_alternator_amps=10\nadditional_system=0,2,17").unwrap();
    let mut variables = Variables::default();
    variables
        .configure_electrical(&Definition::from_config(&config).unwrap(), false)
        .unwrap();
    variables
        .dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
        .unwrap();
    variables.update_electrical().unwrap();
    for _ in 0..10 {
        variables.update_electrical().unwrap();
    }
    assert_eq!(
        number(
            &mut variables,
            "ELECTRICAL BATTERY ESTIMATED CAPACITY PCT",
            "percent"
        ),
        100.0
    );
    advance_battery(&mut variables, 900.0).unwrap();
    variables.update_electrical().unwrap();
    assert!(
        (number(
            &mut variables,
            "ELECTRICAL BATTERY ESTIMATED CAPACITY PCT",
            "percent"
        ) - 50.0)
            .abs()
            < 1e-10
    );
    assert!((number(&mut variables, "ELECTRICAL BATTERY VOLTAGE", "volts") - 20.4).abs() < 1e-10);
    variables
        .dispatch_event("TOGGLE_ALTERNATOR1", Value::Number(0.0))
        .unwrap();
    variables.set_number("GENERAL ENG COMBUSTION:1", "bool", 1.0);
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "ELECTRICAL BATTERY LOAD", "amps"),
        1.0
    );
    advance_battery(&mut variables, 900.0).unwrap();
    variables.update_electrical().unwrap();
    assert!(
        (number(
            &mut variables,
            "ELECTRICAL BATTERY ESTIMATED CAPACITY PCT",
            "percent"
        ) - 75.0)
            .abs()
            < 1e-10
    );
    variables.set_number("GENERAL ENG COMBUSTION:1", "bool", 0.0);
    variables.update_electrical().unwrap();
    advance_battery(&mut variables, 3600.0).unwrap();
    variables.update_electrical().unwrap();
    assert_eq!(
        number(
            &mut variables,
            "ELECTRICAL BATTERY ESTIMATED CAPACITY PCT",
            "percent"
        ),
        0.0
    );
    assert_eq!(
        number(&mut variables, "ELECTRICAL MAIN BUS VOLTAGE", "volts"),
        0.0
    );
    variables.set_number("GENERAL ENG COMBUSTION:1", "bool", 1.0);
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "ELECTRICAL BATTERY LOAD", "amps"),
        1.0
    );
    advance_battery(&mut variables, 360.0).unwrap();
    variables.update_electrical().unwrap();
    assert!(
        number(
            &mut variables,
            "ELECTRICAL BATTERY ESTIMATED CAPACITY PCT",
            "percent"
        ) > 0.0
    );
    assert!(advance_battery(&mut variables, f64::NAN).is_err());
}
#[test]
fn estimates_battery_load_from_switched_consumers_and_connected_generation() {
    let config = Config::parse("[GeneralEngineData]\nengine.0=0,0,0\n[electrical]\nmax_battery_voltage=24\ngenerator_alternator_voltage=28\nmax_generator_alternator_amps=5\nadditional_system=0,2,17\nlight_nav=0,5,17\npitot_heat=0,2,17\navionics_bus=0,3,17\navionics=1,4,17").unwrap();
    let mut variables = Variables::default();
    variables
        .configure_electrical(&Definition::from_config(&config).unwrap(), false)
        .unwrap();
    variables.set_number("LIGHT NAV", "bool", 1.0);
    variables.set_number("PITOT HEAT", "bool", 0.0);
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "ELECTRICAL BATTERY LOAD", "amps"),
        0.0
    );
    variables
        .dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
        .unwrap();
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "ELECTRICAL TOTAL LOAD AMPS", "amperes"),
        7.0
    );
    assert_eq!(
        number(&mut variables, "ELECTRICAL BATTERY LOAD", "amps"),
        -7.0
    );
    variables.set_number("PITOT HEAT", "bool", 1.0);
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "ELECTRICAL BATTERY LOAD", "amperes"),
        -9.0
    );
    variables
        .dispatch_event("TOGGLE_ALTERNATOR1", Value::Number(0.0))
        .unwrap();
    variables.set_number("GENERAL ENG COMBUSTION:1", "bool", 1.0);
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "ELECTRICAL BATTERY LOAD", "amperes"),
        -4.0
    );
    variables.set_number("ELECTRICAL GENALT BUS CONTACT SWITCH:1", "bool", 0.0);
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "ELECTRICAL BATTERY LOAD", "amperes"),
        -9.0
    );
    variables.set_number("AVIONICS MASTER SWITCH", "bool", 1.0);
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "ELECTRICAL BATTERY LOAD", "amperes"),
        -16.0
    );
    variables.set_number("ELECTRICAL BATTERY BUS CONTACT SWITCH", "bool", 0.0);
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "ELECTRICAL BATTERY LOAD", "amperes"),
        0.0
    );
}
#[test]
fn routes_sources_contactors_and_circuit_voltage_thresholds() {
    let config = Config::parse("[GeneralEngineData]\nengine.0=0,0,0\nengine.1=0,0,0\n[electrical]\nmax_battery_voltage=24\ngenerator_alternator_voltage=30\nengine_generator_map=1,0\navionics=1,5,17\nadditional_system=0,2,25\nlight_nav=3,5,17\nlight_taxi=4,5,17").unwrap();
    let mut variables = Variables::default();
    variables
        .configure_electrical(&Definition::from_config(&config).unwrap(), false)
        .unwrap();
    assert_eq!(
        number(&mut variables, "CIRCUIT GENERAL PANEL ON", "bool"),
        0.0
    );
    assert_eq!(number(&mut variables, "CIRCUIT NAV LIGHTS ON", "bool"), 1.0);
    variables
        .dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
        .unwrap();
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "ELECTRICAL MAIN BUS VOLTAGE", "volts"),
        24.0
    );
    assert_eq!(
        number(&mut variables, "CIRCUIT GENERAL PANEL ON", "bool"),
        0.0
    );
    assert_eq!(number(&mut variables, "CIRCUIT AVIONICS ON", "bool"), 0.0);
    variables
        .dispatch_event("TOGGLE_AVIONICS_MASTER", Value::Number(0.0))
        .unwrap();
    variables
        .dispatch_event("TOGGLE_ALTERNATOR1", Value::Number(0.0))
        .unwrap();
    variables.set_number("GENERAL ENG COMBUSTION:1", "bool", 1.0);
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "CIRCUIT GENERAL PANEL ON", "bool"),
        1.0
    );
    assert_eq!(
        number(&mut variables, "ELECTRICAL AVIONICS BUS VOLTAGE", "volts"),
        30.0
    );
    variables
        .dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
        .unwrap();
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "CIRCUIT GENERAL PANEL ON", "bool"),
        1.0
    );
    variables
        .dispatch_event("GENALT_BUS1_CONTACT_SET", Value::Number(0.0))
        .unwrap();
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "CIRCUIT GENERAL PANEL ON", "bool"),
        0.0
    );
    assert_eq!(
        number(&mut variables, "CIRCUIT TAXI LIGHTS ON", "bool"),
        1.0
    );
    variables.set_number("GENERAL ENG COMBUSTION:2", "bool", 1.0);
    variables
        .dispatch_event("TOGGLE_ALTERNATOR2", Value::Number(0.0))
        .unwrap();
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "GENERAL ENG GENERATOR ACTIVE:2", "bool"),
        0.0
    );
    variables.set_number("GENERAL ENG COMBUSTION:1", "bool", 0.0);
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "CIRCUIT TAXI LIGHTS ON", "bool"),
        0.0
    );
}

#[test]
fn configured_power_override_bypasses_switches_and_voltage_limits() {
    let config =
        Config::parse("[electrical]\nelectric_always_available=1\nadditional_system=7,5,1000")
            .unwrap();
    let mut variables = Variables::default();
    variables
        .configure_electrical(&Definition::from_config(&config).unwrap(), false)
        .unwrap();
    assert_eq!(
        number(&mut variables, "CIRCUIT GENERAL PANEL ON", "bool"),
        1.0
    );
    assert!(number(&mut variables, "ELECTRICAL AVIONICS BUS VOLTAGE", "volts") > 0.0);
}

#[test]
fn compiled_cockpit_events_control_each_generator_and_bus_contact() {
    let config = Config::parse(
        "[GeneralEngineData]\nengine.0=0,0,0\nengine.1=0,0,0\nengine.2=0,0,0\nengine.3=0,0,0",
    )
    .unwrap();
    let mut variables = Variables::default();
    variables
        .configure_electrical(&Definition::from_config(&config).unwrap(), true)
        .unwrap();
    for index in 1..=4 {
        variables.set_number(&format!("GENERAL ENG COMBUSTION:{index}"), "bool", 1.0);
        variables
            .dispatch_key_event(0x1033a + index, Value::Number(0.0))
            .unwrap();
        variables.update_electrical().unwrap();
        assert_eq!(
            number(
                &mut variables,
                &format!("GENERAL ENG GENERATOR ACTIVE:{index}"),
                "bool"
            ),
            0.0
        );
        variables
            .dispatch_key_event(0x1033a + index, Value::Number(0.0))
            .unwrap();
        variables
            .dispatch_key_event(0x10711 + index, Value::Number(0.0))
            .unwrap();
        variables.update_electrical().unwrap();
        assert_eq!(
            number(
                &mut variables,
                &format!("ELECTRICAL GENALT BUS CONTACT SWITCH:{index}"),
                "bool"
            ),
            0.0
        );
        variables
            .dispatch_key_event(0x10715 + index, Value::Number(0.0))
            .unwrap();
        variables.update_electrical().unwrap();
        assert_eq!(
            number(
                &mut variables,
                &format!("ELECTRICAL GENALT BUS CONTACT SWITCH:{index}"),
                "bool"
            ),
            1.0
        );
    }
    variables
        .dispatch_key_event(0x10346, Value::Number(0.0))
        .unwrap();
    variables.update_electrical().unwrap();
    assert_eq!(
        number(&mut variables, "ELECTRICAL MAIN BUS VOLTAGE", "volts"),
        0.0
    );
}

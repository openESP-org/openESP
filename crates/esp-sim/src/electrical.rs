use anyhow::{Result, ensure};
use esp_formats::cfg::electrical::Electrical as Definition;
use esp_script::{Host, Reference};

use crate::variables::Variables;

pub struct Electrical {
    definition: Definition,
}

impl Electrical {
    pub fn new(definition: Definition) -> Self {
        Self { definition }
    }

    pub fn engine_indices(&self) -> Vec<u32> {
        self.definition
            .generators
            .iter()
            .map(|g| g.engine)
            .collect()
    }

    pub fn initialize(&self, variables: &mut Variables, running: bool) {
        variables.set_number(
            "ELECTRICAL BATTERY ESTIMATED CAPACITY PCT",
            "percent",
            100.0,
        );
        variables.set_number(
            "OPENESP BATTERY CAPACITY",
            "number",
            self.definition.battery_capacity_ah,
        );
        for (name, active) in [
            ("ELECTRICAL MASTER BATTERY", running),
            ("AVIONICS MASTER SWITCH", running),
            ("ELECTRICAL BATTERY BUS CONTACT SWITCH", true),
        ] {
            variables.set_number(name, "bool", f64::from(active));
        }
        for generator in &self.definition.generators {
            let index = generator.engine;
            variables.set_number(
                &format!("GENERAL ENG GENERATOR SWITCH:{index}"),
                "bool",
                f64::from(running),
            );
            variables.set_number(
                &format!("ELECTRICAL GENALT BUS CONTACT SWITCH:{index}"),
                "bool",
                1.0,
            );
        }
    }

    pub fn update(&self, variables: &mut Variables) -> Result<()> {
        let read = |variables: &mut Variables, name: &str| -> Result<bool> {
            Ok(variables
                .read(&Reference::new('A', name, Some("bool")))?
                .number()?
                != 0.0)
        };
        let battery = read(variables, "ELECTRICAL MASTER BATTERY")?;
        let avionics = read(variables, "AVIONICS MASTER SWITCH")?;
        let capacity = variables
            .read(&Reference::new(
                'A',
                "ELECTRICAL BATTERY ESTIMATED CAPACITY PCT",
                Some("percent over 100"),
            ))?
            .number()?
            .clamp(0.0, 1.0);
        let battery_voltage = if capacity <= 0.0 {
            0.0
        } else {
            self.definition.battery_voltage * (0.7 + 0.3 * capacity)
        };
        let mut sources = [0.0_f64; 8];
        // Generator voltage is regulated; battery voltage approximates charge level.
        sources[3] = battery_voltage;
        if battery {
            sources[2] = battery_voltage;
        }
        let mut links = Vec::new();
        let mut generator_capacity = [0.0_f64; 8];
        if read(variables, "ELECTRICAL BATTERY BUS CONTACT SWITCH")? {
            links.push((0, 2));
        }
        for generator in &self.definition.generators {
            let index = generator.engine;
            let switch = read(variables, &format!("GENERAL ENG GENERATOR SWITCH:{index}"))?;
            // An uninstantiated engine has no source of shaft power.
            let combustion = variables
                .read(&Reference::new(
                    'A',
                    &format!("GENERAL ENG COMBUSTION:{index}"),
                    Some("bool"),
                ))
                .ok()
                .map(|v| v.number())
                .transpose()?
                .is_some_and(|v| v != 0.0);
            let active = generator.count > 0
                && self.definition.generator_amperes > 0.0
                && self.definition.generator_voltage > 0.0
                && switch
                && combustion;
            let bus = index as usize + 3;
            ensure!(bus < sources.len(), "generator bus index is out of range");
            if active {
                sources[bus] = self.definition.generator_voltage;
                generator_capacity[bus] =
                    self.definition.generator_amperes * f64::from(generator.count);
            }
            if read(
                variables,
                &format!("ELECTRICAL GENALT BUS CONTACT SWITCH:{index}"),
            )? {
                links.push((0, bus));
            }
            for name in ["GENERAL ENG GENERATOR ACTIVE", "ELECTRICAL GENALT ACTIVE"] {
                variables.set_number(&format!("{name}:{index}"), "bool", f64::from(active));
            }
            variables.set_number(
                &format!("ELECTRICAL GENALT SWITCH:{index}"),
                "bool",
                f64::from(switch),
            );
            variables.set_number(
                &format!("GENERAL ENG MASTER ALTERNATOR:{index}"),
                "bool",
                f64::from(switch),
            );
        }
        let propagate = |voltage: &mut [f64; 8], links: &[(usize, usize)]| {
            for _ in 0..8 {
                for &(a, b) in links {
                    let shared = voltage[a].max(voltage[b]);
                    voltage[a] = shared;
                    voltage[b] = shared;
                }
            }
        };
        propagate(&mut sources, &links);
        let feeder = self
            .definition
            .circuits
            .iter()
            .find(|c| c.name == "avionics_bus")
            .unwrap();
        // The avionics contactor can draw from any source bus except its own
        // output, which would be a self-powered circuit.
        if avionics
            && feeder.bus != 1
            && sources[feeder.bus] > 0.0
            && sources[feeder.bus] >= feeder.minimum_voltage
        {
            sources[1] = sources[feeder.bus];
            links.push((1, feeder.bus));
        }
        if self.definition.always_available {
            sources.fill(
                self.definition
                    .battery_voltage
                    .max(self.definition.generator_voltage),
            );
        }
        for (name, voltage) in [
            ("ELECTRICAL MAIN BUS VOLTAGE", sources[0]),
            ("ELECTRICAL AVIONICS BUS VOLTAGE", sources[1]),
            ("ELECTRICAL BATTERY BUS VOLTAGE", sources[2]),
            ("ELECTRICAL HOT BATTERY BUS VOLTAGE", sources[3]),
            ("ELECTRICAL BATTERY VOLTAGE", battery_voltage),
        ] {
            variables.set_number(name, "volts", voltage);
        }
        for index in 1..=4 {
            variables.set_number(
                &format!("ELECTRICAL GENALT BUS VOLTAGE:{index}"),
                "volts",
                sources[index + 3],
            );
        }
        for circuit in &self.definition.circuits {
            let voltage = sources[circuit.bus];
            let powered = self.definition.always_available
                || (voltage > 0.0 && voltage >= circuit.minimum_voltage);
            variables.set_number(circuit_variable(circuit.name), "bool", f64::from(powered));
        }
        // Legacy circuit amperages are rated loads, so this is an estimate
        // for the switched consumers currently represented by the simulator.
        let mut battery_network = [false; 8];
        battery_network[2] = battery;
        for _ in 0..8 {
            for &(a, b) in &links {
                let connected = battery_network[a] || battery_network[b];
                battery_network[a] = connected;
                battery_network[b] = connected;
            }
        }
        let switch_on = |variables: &mut Variables, name: &str| -> bool {
            variables
                .read(&Reference::new('A', name, Some("bool")))
                .ok()
                .and_then(|value| value.number().ok())
                .is_some_and(|value| value != 0.0)
        };
        let mut total_load = 0.0;
        let mut battery_network_load = 0.0;
        for circuit in &self.definition.circuits {
            let enabled = match circuit.name {
                "additional_system" => true,
                "avionics_bus" | "avionics" => avionics,
                "autopilot" => switch_on(variables, "AUTOPILOT MASTER"),
                "pitot_heat" => switch_on(variables, "PITOT HEAT"),
                "starter1" | "starter2" | "starter3" | "starter4" => {
                    let index = circuit.name.as_bytes()[7] - b'0';
                    switch_on(variables, &format!("GENERAL ENG STARTER ACTIVE:{index}"))
                }
                "light_nav" => switch_on(variables, "LIGHT NAV"),
                "light_beacon" => switch_on(variables, "LIGHT BEACON"),
                "light_landing" => switch_on(variables, "LIGHT LANDING"),
                "light_taxi" => switch_on(variables, "LIGHT TAXI"),
                "light_strobe" => switch_on(variables, "LIGHT STROBE"),
                "light_panel" => switch_on(variables, "LIGHT PANEL"),
                "light_cabin" => switch_on(variables, "LIGHT CABIN"),
                "light_recognition" => switch_on(variables, "LIGHT RECOGNITION"),
                "light_wing" => switch_on(variables, "LIGHT WING"),
                "light_logo" => switch_on(variables, "LIGHT LOGO"),
                _ => false,
            };
            if enabled
                && sources[circuit.bus] > 0.0
                && sources[circuit.bus] >= circuit.minimum_voltage
            {
                total_load += circuit.amperes;
                if battery_network[circuit.bus] {
                    battery_network_load += circuit.amperes;
                }
            }
        }
        let available_generation: f64 = generator_capacity
            .iter()
            .zip(battery_network)
            .filter_map(|(capacity, connected)| connected.then_some(*capacity))
            .sum();
        let battery_load = if battery && !self.definition.always_available {
            let balance = available_generation - battery_network_load;
            if balance < 0.0 {
                if capacity > 0.0 { balance } else { 0.0 }
            } else if self.definition.generator_voltage > battery_voltage {
                balance.min(self.definition.battery_charging_current)
                    * ((1.0 - capacity) * 5.0).clamp(0.0, 1.0)
            } else {
                0.0
            }
        } else {
            0.0
        };
        variables.set_number("ELECTRICAL TOTAL LOAD AMPS", "amperes", total_load);
        variables.set_number("ELECTRICAL BATTERY LOAD", "amperes", battery_load);
        Ok(())
    }
}

/// Integrate battery charge once per simulation step; electrical updates remain
/// instantaneous so repeated gauge or rendering updates cannot drain it.
pub fn advance_battery(variables: &mut Variables, dt: f64) -> Result<()> {
    ensure!(dt.is_finite() && dt >= 0.0, "invalid battery timestep");
    let capacity = match variables.read(&Reference::new(
        'A',
        "OPENESP BATTERY CAPACITY",
        Some("number"),
    )) {
        Ok(value) => value.number()?,
        Err(_) => return Ok(()),
    };
    let charge = variables
        .read(&Reference::new(
            'A',
            "ELECTRICAL BATTERY ESTIMATED CAPACITY PCT",
            Some("percent over 100"),
        ))?
        .number()?;
    let current = variables
        .read(&Reference::new(
            'A',
            "ELECTRICAL BATTERY LOAD",
            Some("amperes"),
        ))?
        .number()?;
    ensure!(
        capacity.is_finite() && capacity > 0.0 && charge.is_finite() && current.is_finite(),
        "invalid battery state"
    );
    let next = (charge + current * dt / (capacity * 3600.0)).clamp(0.0, 1.0);
    variables.set_number(
        "ELECTRICAL BATTERY ESTIMATED CAPACITY PCT",
        "percent over 100",
        next,
    );
    Ok(())
}
fn circuit_variable(name: &str) -> &str {
    match name {
        "additional_system" => "CIRCUIT GENERAL PANEL ON",
        "flap_motor" => "CIRCUIT FLAP MOTOR ON",
        "gear_motor" => "CIRCUIT GEAR MOTOR ON",
        "autopilot" => "CIRCUIT AUTOPILOT ON",
        "avionics_bus" => "CIRCUIT AVIONICS BUS ON",
        "avionics" => "CIRCUIT AVIONICS ON",
        "pitot_heat" => "CIRCUIT PITOT HEAT ON",
        "marker_beacon" => "CIRCUIT MARKER BEACON ON",
        "gear_warning" => "CIRCUIT GEAR WARNING ON",
        "fuel_pump" => "CIRCUIT FUEL PUMP ON",
        "starter1" => "CIRCUIT STARTER ON:1",
        "starter2" => "CIRCUIT STARTER ON:2",
        "starter3" => "CIRCUIT STARTER ON:3",
        "starter4" => "CIRCUIT STARTER ON:4",
        "light_nav" => "CIRCUIT NAV LIGHTS ON",
        "light_beacon" => "CIRCUIT BEACON LIGHTS ON",
        "light_landing" => "CIRCUIT LANDING LIGHTS ON",
        "light_taxi" => "CIRCUIT TAXI LIGHTS ON",
        "light_strobe" => "CIRCUIT STROBE LIGHTS ON",
        "light_panel" => "CIRCUIT PANEL LIGHTS ON",
        "light_cabin" => "CIRCUIT CABIN LIGHTS ON",
        "prop_sync" => "CIRCUIT PROP SYNC ON",
        "auto_feather" => "CIRCUIT AUTO FEATHER ON",
        "auto_brakes" => "CIRCUIT AUTO BRAKES ON",
        "standby_vacuum" => "CIRCUIT STANDBY VACUUM ON",
        "hydraulic_pump" => "CIRCUIT HYDRAULIC PUMP ON",
        "fuel_transfer_pump" => "CIRCUIT FUEL TRANSFER PUMP ON",
        "propeller_deice" => "CIRCUIT PROPELLER DEICE ON",
        "light_recognition" => "CIRCUIT RECOGNITION LIGHTS ON",
        "light_wing" => "CIRCUIT WING LIGHTS ON",
        "light_logo" => "CIRCUIT LOGO LIGHTS ON",
        "directional_gyro" => "CIRCUIT DIRECTIONAL GYRO ON",
        "directional_gyro_slaving" => "CIRCUIT DIRECTIONAL GYRO SLAVING ON",
        _ => unreachable!("circuit definition without simulation variable"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_formats::cfg::Config;
    use esp_script::Value;

    fn number(variables: &mut Variables, name: &str, unit: &str) -> f64 {
        variables
            .read(&Reference::new('A', name, Some(unit)))
            .unwrap()
            .number()
            .unwrap()
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
        assert!(
            (number(&mut variables, "ELECTRICAL BATTERY VOLTAGE", "volts") - 20.4).abs() < 1e-10
        );
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
}

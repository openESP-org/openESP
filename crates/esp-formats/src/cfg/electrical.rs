use super::{Config, ValueError, camera::parse_number, engine};

#[derive(Clone, Debug, PartialEq)]
pub struct Circuit {
    pub name: &'static str,
    pub bus: usize,
    pub amperes: f64,
    pub minimum_voltage: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Generator {
    /// Simulation variable indices start at one, unlike Engine.n in the CFG.
    pub engine: u32,
    pub count: u32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Electrical {
    pub battery_voltage: f64,
    pub battery_capacity_ah: f64,
    pub battery_charging_current: f64,
    pub generator_voltage: f64,
    pub generator_amperes: f64,
    pub always_available: bool,
    pub generators: Vec<Generator>,
    pub circuits: Vec<Circuit>,
}

const CIRCUITS: &[&str] = &[
    "additional_system",
    "flap_motor",
    "gear_motor",
    "autopilot",
    "avionics_bus",
    "avionics",
    "pitot_heat",
    "marker_beacon",
    "gear_warning",
    "fuel_pump",
    "starter1",
    "starter2",
    "starter3",
    "starter4",
    "light_nav",
    "light_beacon",
    "light_landing",
    "light_taxi",
    "light_strobe",
    "light_panel",
    "light_cabin",
    "prop_sync",
    "auto_feather",
    "auto_brakes",
    "standby_vacuum",
    "hydraulic_pump",
    "fuel_transfer_pump",
    "propeller_deice",
    "light_recognition",
    "light_wing",
    "light_logo",
    "directional_gyro",
    "directional_gyro_slaving",
];

impl Electrical {
    pub fn from_config(config: &Config) -> Result<Self, ValueError> {
        let invalid = |key: &str, value: &str| ValueError::Invalid {
            section: "electrical".into(),
            key: key.into(),
            value: value.into(),
        };
        let nonnegative = |key, default| -> Result<f64, ValueError> {
            let value = config.number("electrical", key)?.unwrap_or(default);
            if value < 0.0 {
                return Err(invalid(key, &value.to_string()));
            }
            Ok(value)
        };
        // Fallback hardware for legacy containers without source specifications.
        // These are simulator defaults, not inferred measurements of the aircraft.
        let battery_voltage = nonnegative("max_battery_voltage", 24.0)?;
        let battery_capacity_ah = nonnegative(
            "battery_max_capacity_amph",
            if battery_voltage < 20.0 { 42.0 } else { 24.0 },
        )?;
        if battery_capacity_ah <= 0.0 {
            return Err(invalid(
                "battery_max_capacity_amph",
                &battery_capacity_ah.to_string(),
            ));
        }
        let battery_charging_current =
            nonnegative("battery_charging_current", battery_capacity_ah * 0.1)?;
        let generator_voltage = nonnegative("generator_alternator_voltage", 28.0)?;
        let generator_amperes = nonnegative("max_generator_alternator_amps", 60.0)?;
        let always_available = match config
            .number("electrical", "electric_always_available")?
            .unwrap_or(0.0)
        {
            0.0 => false,
            1.0 => true,
            value => return Err(invalid("electric_always_available", &value.to_string())),
        };
        let engines = engine::engines(config)?;
        let counts = config
            .value("electrical", "engine_generator_map")?
            .map(|value| {
                value
                    .split(',')
                    .map(|v| {
                        v.trim()
                            .parse::<u32>()
                            .map_err(|_| invalid("engine_generator_map", value))
                    })
                    .collect::<Result<Vec<_>, _>>()
            })
            .transpose()?;
        if counts
            .as_ref()
            .is_some_and(|c| c.len() > 4 || c.iter().any(|n| *n > 16))
        {
            return Err(invalid(
                "engine_generator_map",
                "unsupported generator count",
            ));
        }
        let mut generators = Vec::new();
        for engine in engines {
            if engine.index >= 4 {
                return Err(invalid(
                    "engine_generator_map",
                    "engine index exceeds legacy bus range",
                ));
            }
            let count = counts
                .as_ref()
                .map_or(1, |c| c.get(engine.index as usize).copied().unwrap_or(0));
            generators.push(Generator {
                engine: engine.index + 1,
                count,
            });
        }
        let mut circuits = Vec::new();
        for &name in CIRCUITS {
            let mut circuit = Circuit {
                name,
                bus: usize::from(matches!(name, "avionics" | "marker_beacon")),
                amperes: 15.0,
                minimum_voltage: battery_voltage * 0.7,
            };
            if let Some(value) = config.value("electrical", name)? {
                let fields = value.split(',').map(str::trim).collect::<Vec<_>>();
                if !(2..=3).contains(&fields.len()) {
                    return Err(invalid(name, value));
                }
                circuit.bus = fields[0].parse().map_err(|_| invalid(name, value))?;
                circuit.amperes = parse_number("electrical", name, fields[1])?;
                if fields.len() == 3 {
                    circuit.minimum_voltage = parse_number("electrical", name, fields[2])?;
                }
                if circuit.bus > 7 || circuit.amperes < 0.0 || circuit.minimum_voltage < 0.0 {
                    return Err(invalid(name, value));
                }
            }
            circuits.push(circuit);
        }
        Ok(Self {
            battery_voltage,
            battery_capacity_ah,
            battery_charging_current,
            generator_voltage,
            generator_amperes,
            always_available,
            generators,
            circuits,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn retains_circuit_limits_and_generator_mapping() {
        let config = Config::parse("[GeneralEngineData]\nEngine.0=0,0,0\nEngine.2=0,0,0\n[ELECTRICAL]\nmax_battery_voltage=12\ngenerator_alternator_voltage=14\nmax_generator_alternator_amps=80\nengine_generator_map=0,0,2\navionics=1,7,9\nlight_nav=3,2").unwrap();
        let power = Electrical::from_config(&config).unwrap();
        assert_eq!(power.battery_voltage, 12.0);
        assert_eq!(
            power.generators,
            [
                Generator {
                    engine: 1,
                    count: 0
                },
                Generator {
                    engine: 3,
                    count: 2
                }
            ]
        );
        assert_eq!(
            power
                .circuits
                .iter()
                .find(|c| c.name == "avionics")
                .unwrap(),
            &Circuit {
                name: "avionics",
                bus: 1,
                amperes: 7.0,
                minimum_voltage: 9.0
            }
        );
        assert!(
            (power
                .circuits
                .iter()
                .find(|c| c.name == "light_nav")
                .unwrap()
                .minimum_voltage
                - 8.4)
                .abs()
                < 1e-12
        );
    }

    #[test]
    fn rejects_invalid_limits_and_duplicate_parameters() {
        for parameters in [
            "max_battery_voltage=-1",
            "battery_max_capacity_amph=0",
            "battery_charging_current=-1",
            "generator_alternator_voltage=NaN",
            "max_generator_alternator_amps=-2",
            "avionics=8,2,9",
            "avionics=1,-2,9",
            "avionics=1,2,-9",
            "avionics=1,2,9,3",
            "engine_generator_map=1,-1",
            "electric_always_available=2",
            "avionics=1,2,9\nAVIONICS=0,2,9",
        ] {
            assert!(
                Electrical::from_config(
                    &Config::parse(&format!("[electrical]\n{parameters}")).unwrap()
                )
                .is_err(),
                "{parameters}"
            );
        }
    }
}

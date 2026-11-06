use super::*;

impl Flight {
    pub fn publish(state: &State, variables: &mut Variables) -> Result<()> {
        let wind = crate::weather::wind(variables)?;
        crate::weather::publish(state, wind, variables);
        crate::navigation::publish(state, variables)?;
        let air = state.air_velocity(wind);
        let speed = air.length();
        let atmosphere = crate::weather::conditions(variables)?.at(-state.position.z)?;
        let (heading, pitch, bank) = state.orientation.to_euler(EulerRot::ZYX);
        variables.set_number("AIRSPEED TRUE", "meters per second", speed);
        let blocked = |variables: &mut Variables, name| -> Result<bool> {
            Ok(variables
                .read(&Reference::new('A', name, Some("bool")))?
                .number()?
                != 0.0)
        };
        let pitot_blocked = blocked(variables, "PITOT BLOCKED")?;
        let static_blocked = blocked(variables, "STATIC PORT BLOCKED")?;
        let total_pressure = atmosphere.pressure + atmosphere.impact_pressure(speed)?;
        let [pitot_pressure, static_pressure] = variables.air_data_pressures(
            total_pressure,
            atmosphere.pressure,
            pitot_blocked,
            static_blocked,
        )?;
        let indicated_airspeed = Atmosphere::calibrated_airspeed_from_impact_pressure(
            (pitot_pressure - static_pressure).max(0.0),
        )?;
        variables.set_number(
            "AIRSPEED INDICATED",
            "meters per second",
            indicated_airspeed,
        );
        let selected = if variables
            .read(&Reference::new('A', "TRUE AIRSPEED SELECTED", Some("bool")))?
            .number()?
            != 0.0
        {
            speed
        } else {
            indicated_airspeed
        };
        for name in [
            "AIRSPEED SELECT INDICATED OR TRUE",
            "AIRSPEED SELECT INDICATED OR TRUE:2",
        ] {
            variables.set_number(name, "meters per second", selected);
        }
        variables.set_number("AIRSPEED MACH", "mach", speed / atmosphere.sound_speed);
        variables.set_number("AMBIENT PRESSURE", "pascals", atmosphere.pressure);
        variables.set_number(
            "AMBIENT TEMPERATURE",
            "celsius",
            atmosphere.temperature - 273.15,
        );
        variables.set_number(
            "TOTAL AIR TEMPERATURE",
            "celsius",
            atmosphere.temperature * (1.0 + 0.2 * (speed / atmosphere.sound_speed).powi(2))
                - 273.15,
        );
        for name in ["PLANE ALTITUDE", "PLANE ALT ABOVE GROUND"] {
            variables.set_number(name, "meters", -state.position.z);
        }
        for index in 1..=4 {
            let setting = variables
                .read(&Reference::new(
                    'A',
                    &format!("KOHLSMAN SETTING MB:{index}"),
                    Some("pascals"),
                ))?
                .number()?;
            let indicated = Atmosphere::indicated_altitude(static_pressure, setting)?;
            variables.set_number(&format!("INDICATED ALTITUDE:{index}"), "meters", indicated);
            if index == 1 {
                variables.set_number("INDICATED ALTITUDE", "meters", indicated);
            }
        }
        for name in ["PLANE PITCH DEGREES", "ATTITUDE INDICATOR PITCH DEGREES"] {
            variables.set_number(name, "radians", -pitch);
        }
        for name in ["PLANE BANK DEGREES", "ATTITUDE INDICATOR BANK DEGREES"] {
            variables.set_number(name, "radians", -bank);
        }
        variables.set_number(
            "PLANE HEADING DEGREES TRUE",
            "radians",
            heading.rem_euclid(std::f64::consts::TAU),
        );
        variables.set_number("INCIDENCE ALPHA", "radians", air.z.atan2(air.x));
        variables.set_number(
            "INCIDENCE BETA",
            "radians",
            if speed > 0.0 {
                (air.y / speed).clamp(-1.0, 1.0).asin()
            } else {
                0.0
            },
        );
        variables.set_number(
            "VERTICAL SPEED",
            "meters per second",
            if static_blocked {
                0.0
            } else {
                -state.velocity.z
            },
        );
        variables.set_number(
            "GROUND VELOCITY",
            "meters per second",
            state.velocity.truncate().length(),
        );
        Ok(())
    }
}

use anyhow::{Result, ensure};
use esp_formats::cfg::Config;
use esp_sim::{atmosphere::Atmosphere, flight::State, variables::Variables};
use glam::DVec3;

pub struct SpeedLimits {
    stall_clean: Option<f64>,
    stall_landing: Option<f64>,
    cruise: Option<f64>,
    maximum_ias: Option<f64>,
    maximum_mach: Option<f64>,
}

impl SpeedLimits {
    pub fn from_config(config: &Config) -> Result<Self> {
        let get = |key| -> Result<Option<f64>> {
            let value = config.number("reference speeds", key)?;
            ensure!(
                value.is_none_or(|v| v.is_finite() && v >= 0.0),
                "invalid reference speed {key}"
            );
            Ok(value.filter(|v| *v > 0.0))
        };
        Ok(Self {
            stall_clean: get("flaps_up_stall_speed")?,
            stall_landing: get("full_flaps_stall_speed")?,
            cruise: get("cruise_speed")?,
            maximum_ias: get("max_indicated_speed")?,
            maximum_mach: get("max_mach")?,
        })
    }

    pub fn publish(&self, state: &State, variables: &mut Variables) -> Result<()> {
        for (name, speed) in [
            ("DESIGN SPEED VS0", self.stall_landing),
            ("DESIGN SPEED VS1", self.stall_clean),
            ("DESIGN SPEED VC", self.cruise),
            ("ESTIMATED CRUISE SPEED", self.cruise),
        ] {
            if let Some(speed) = speed {
                variables.set_number(name, "knots", speed);
            }
        }
        let atmosphere = Atmosphere::standard(-state.position.z)?;
        let speed = state.air_velocity(DVec3::ZERO).length();
        let indicated = atmosphere.calibrated_airspeed(speed)?;
        if let Some(mach) = self.maximum_mach {
            variables.set_number("MACH MAX OPERATE", "mach", mach);
            variables.set_number("BARBER POLE MACH", "mach", mach);
        }
        let mach_limit = self
            .maximum_mach
            .map(|mach| atmosphere.calibrated_airspeed(mach * atmosphere.sound_speed))
            .transpose()?;
        let limit = match (self.maximum_ias, mach_limit) {
            (Some(knots), Some(mach)) => Some((knots * 1852.0 / 3600.0).min(mach)),
            (Some(knots), None) => Some(knots * 1852.0 / 3600.0),
            (None, Some(mach)) => Some(mach),
            (None, None) => None,
        };
        if let Some(limit) = limit {
            variables.set_number("AIRSPEED BARBER POLE", "meters per second", limit);
            variables.set_number("OVERSPEED WARNING", "bool", f64::from(indicated > limit));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_script::{Host, Reference};

    #[test]
    #[ignore = "requires original aircraft configuration and OPENESP_AIRCRAFT_PATHS"]
    fn original_aircraft_reference_speeds_reach_cockpit_variables() {
        let paths = std::env::var_os("OPENESP_AIRCRAFT_PATHS").expect("set OPENESP_AIRCRAFT_PATHS");
        let paths: Vec<_> = std::env::split_paths(&paths).collect();
        assert!(!paths.is_empty());
        for path in paths {
            let data = esp_assets::flight::FlightData::load(&path, 0).unwrap();
            let limits = SpeedLimits::from_config(&data.configuration).unwrap();
            let mut variables = Variables::default();
            limits.publish(&State::default(), &mut variables).unwrap();
            for (name, key) in [
                ("DESIGN SPEED VS0", "full_flaps_stall_speed"),
                ("DESIGN SPEED VS1", "flaps_up_stall_speed"),
                ("AIRSPEED BARBER POLE", "max_indicated_speed"),
            ] {
                let expected = data
                    .configuration
                    .number("reference speeds", key)
                    .unwrap()
                    .unwrap();
                let actual = variables
                    .read(&Reference::new('A', name, Some("knots")))
                    .unwrap()
                    .number()
                    .unwrap();
                assert!(
                    (actual - expected).abs() < 1e-9,
                    "{name}: {actual} != {expected}"
                );
            }
        }
    }

    #[test]
    fn configured_limits_drive_markers_and_high_altitude_mach_limit() {
        let config = Config::parse("[Reference Speeds]\nflaps_up_stall_speed=142\nfull_flaps_stall_speed=113\ncruise_speed=477\nmax_mach=0.82\nmax_indicated_speed=340").unwrap();
        let limits = SpeedLimits::from_config(&config).unwrap();
        let mut variables = Variables::default();
        let mut state = State {
            velocity: DVec3::X * 100.0,
            ..State::default()
        };
        let read = |variables: &mut Variables, name, unit| {
            variables
                .read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        limits.publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "DESIGN SPEED VS0", "knots"), 113.0);
        assert!((read(&mut variables, "AIRSPEED BARBER POLE", "knots") - 340.0).abs() < 1e-10);
        assert_eq!(read(&mut variables, "OVERSPEED WARNING", "bool"), 0.0);
        state.position.z = -10_000.0;
        let atmosphere = Atmosphere::standard(10_000.0).unwrap();
        state.velocity.x = atmosphere.sound_speed * 0.83;
        limits.publish(&state, &mut variables).unwrap();
        let mach_limit = atmosphere
            .calibrated_airspeed(atmosphere.sound_speed * 0.82)
            .unwrap();
        assert!(
            (read(&mut variables, "AIRSPEED BARBER POLE", "meters per second") - mach_limit).abs()
                < 1e-10
        );
        assert_eq!(read(&mut variables, "OVERSPEED WARNING", "bool"), 1.0);
        state.velocity.x = atmosphere.sound_speed * 0.8;
        limits.publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "OVERSPEED WARNING", "bool"), 0.0);
    }
}

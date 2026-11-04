use anyhow::{Result, ensure};
use esp_formats::{
    air::{self, table::Table},
    cfg::Config,
};
use esp_script::{Host, Reference};
use esp_sim::{atmosphere::Atmosphere, flight::State, variables::Variables};
use glam::DVec3;

pub struct SpeedLimits {
    stall_clean: Option<f64>,
    stall_landing: Option<f64>,
    cruise: Option<f64>,
    maximum_ias: Option<f64>,
    maximum_mach: Option<f64>,
    stall_alpha: Option<f64>,
    stall_warning_type: u8,
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
        let warning = config.number("stall_warning", "type")?.unwrap_or(0.0);
        ensure!(
            warning.fract() == 0.0 && (0.0..=2.0).contains(&warning),
            "invalid stall warning type"
        );
        Ok(Self {
            stall_clean: get("flaps_up_stall_speed")?,
            stall_landing: get("full_flaps_stall_speed")?,
            cruise: get("cruise_speed")?,
            maximum_ias: get("max_indicated_speed")?,
            maximum_mach: get("max_mach")?,
            stall_alpha: None,
            stall_warning_type: warning as u8,
        })
    }

    pub fn from_data(data: &esp_assets::flight::FlightData) -> Result<Self> {
        let mut limits = Self::from_config(&data.configuration)?;
        let records = air::parse(&data.air)?;
        if let Some(record) = records.iter().find(|record| record.id == 0x404)
            && let Some(Table::Curve(curve)) = record.table()?
        {
            limits.stall_alpha = curve
                .points
                .iter()
                .filter(|point| {
                    point.input.is_finite()
                        && point.output.is_finite()
                        && (0.0..=std::f64::consts::FRAC_PI_4).contains(&point.input)
                })
                .max_by(|a, b| a.output.total_cmp(&b.output))
                .map(|point| point.input)
                .filter(|alpha| *alpha > 3.0_f64.to_radians());
        }
        Ok(limits)
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
        let air_velocity = state.air_velocity(DVec3::ZERO);
        let speed = air_velocity.length();
        let sea_level = Atmosphere::standard(0.0)?;
        variables.set_number(
            "DYNAMIC PRESSURE",
            "pascals",
            0.5 * atmosphere.density * speed * speed,
        );
        variables.set_number(
            "SIGMA SQRT",
            "number",
            (atmosphere.density / sea_level.density).sqrt(),
        );
        variables.set_number(
            "TOTAL VELOCITY",
            "meters per second",
            state.velocity.length(),
        );
        variables.set_number(
            "PRESSURE ALTITUDE",
            "meters",
            Atmosphere::indicated_altitude(atmosphere.pressure, 101_325.0)?,
        );
        variables.set_number(
            "ANGLE OF ATTACK INDICATOR",
            "radians",
            air_velocity.z.atan2(air_velocity.x),
        );
        variables.set_number(
            "STALL HORN AVAILABLE",
            "bool",
            f64::from(self.stall_warning_type != 0),
        );
        if let Some(stall_alpha) = self.stall_alpha {
            variables.set_number("STALL ALPHA", "radians", stall_alpha);
            let powered = self.stall_warning_type != 2
                || variables
                    .read(&Reference::new(
                        'A',
                        "CIRCUIT GENERAL PANEL ON",
                        Some("bool"),
                    ))
                    .ok()
                    .and_then(|v| v.number().ok())
                    .is_some_and(|value| value != 0.0);
            let approaching_stall = speed > 10.0 * 1852.0 / 3600.0
                && air_velocity.x > 0.0
                && air_velocity.z.atan2(air_velocity.x) >= stall_alpha - 3.0_f64.to_radians();
            variables.set_number(
                "STALL WARNING",
                "bool",
                f64::from(self.stall_warning_type != 0 && powered && approaching_stall),
            );
        }
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
            let limits = SpeedLimits::from_data(&data).unwrap();
            let mut variables = Variables::default();
            limits.publish(&State::default(), &mut variables).unwrap();
            let critical = limits.stall_alpha.expect("missing positive lift peak");
            assert!((0.1..0.6).contains(&critical));
            variables.set_number("CIRCUIT GENERAL PANEL ON", "bool", 1.0);
            let state = State {
                velocity: DVec3::new(100.0 * critical.cos(), 0.0, 100.0 * critical.sin()),
                ..State::default()
            };
            limits.publish(&state, &mut variables).unwrap();
            assert_eq!(
                variables
                    .read(&Reference::new('A', "STALL WARNING", Some("bool")))
                    .unwrap()
                    .number()
                    .unwrap(),
                1.0
            );
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
    fn stall_warning_uses_angle_of_attack_and_electrical_power() {
        let config = Config::parse("[stall_warning]\ntype=2").unwrap();
        let mut limits = SpeedLimits::from_config(&config).unwrap();
        limits.stall_alpha = Some(15.0_f64.to_radians());
        let mut variables = Variables::default();
        let read = |vars: &mut Variables| {
            vars.read(&Reference::new('A', "STALL WARNING", Some("bool")))
                .unwrap()
                .number()
                .unwrap()
        };
        let velocity = |speed: f64, degrees: f64| {
            let angle = degrees.to_radians();
            DVec3::new(speed * angle.cos(), 0.0, speed * angle.sin())
        };
        let mut state = State {
            velocity: velocity(100.0, 11.0),
            ..State::default()
        };
        variables.set_number("CIRCUIT GENERAL PANEL ON", "bool", 1.0);
        limits.publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables), 0.0);
        state.velocity = velocity(100.0, 13.0);
        limits.publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables), 1.0);
        variables.set_number("CIRCUIT GENERAL PANEL ON", "bool", 0.0);
        limits.publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables), 0.0);
        variables.set_number("CIRCUIT GENERAL PANEL ON", "bool", 1.0);
        state.velocity = velocity(1.0, 13.0);
        limits.publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables), 0.0);
    }

    #[test]
    fn air_data_instruments_follow_velocity_and_atmosphere() {
        let limits = SpeedLimits::from_config(&Config::parse("").unwrap()).unwrap();
        let mut variables = Variables::default();
        let mut state = State {
            position: DVec3::new(0.0, 0.0, -3000.0),
            velocity: DVec3::new(90.0, 0.0, 10.0),
            ..State::default()
        };
        limits.publish(&state, &mut variables).unwrap();
        let read = |vars: &mut Variables, name, unit| {
            vars.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        let atmosphere = Atmosphere::standard(3000.0).unwrap();
        assert!(
            (read(&mut variables, "DYNAMIC PRESSURE", "pascals")
                - 0.5 * atmosphere.density * 8200.0)
                .abs()
                < 1e-9
        );
        assert!(
            (read(&mut variables, "TOTAL VELOCITY", "feet per second")
                - 8200.0_f64.sqrt() / 0.3048)
                .abs()
                < 1e-9
        );
        assert!((read(&mut variables, "PRESSURE ALTITUDE", "meters") - 3000.0).abs() < 1e-8);
        assert!(
            (read(&mut variables, "ANGLE OF ATTACK INDICATOR", "degrees")
                - 10.0_f64.atan2(90.0).to_degrees())
            .abs()
                < 1e-10
        );
        let high_density_ratio = read(&mut variables, "SIGMA SQRT", "number");
        state.position.z = 0.0;
        limits.publish(&state, &mut variables).unwrap();
        assert!(high_density_ratio < 1.0);
        assert_eq!(read(&mut variables, "SIGMA SQRT", "number"), 1.0);
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

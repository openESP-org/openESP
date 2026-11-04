use anyhow::{Result, ensure};
use esp_script::{Host, Reference};
use esp_sim::{atmosphere::Atmosphere, flight::State, variables::Variables};
use glam::DVec3;

#[derive(Clone, Copy)]
pub struct Conditions {
    pub sea_pressure: f64,
    pub sea_temperature: f64,
}

impl Conditions {
    pub fn new(pressure_hpa: f64, temperature_celsius: f64) -> Result<Self> {
        ensure!(
            (800.0..=1100.0).contains(&pressure_hpa)
                && (-60.0..=60.0).contains(&temperature_celsius),
            "invalid sea level weather"
        );
        Ok(Self {
            sea_pressure: pressure_hpa * 100.0,
            sea_temperature: temperature_celsius + 273.15,
        })
    }

    pub fn at(self, altitude: f64) -> Result<Atmosphere> {
        Atmosphere::with_sea_level(altitude, self.sea_pressure, self.sea_temperature)
    }
}

pub fn conditions(variables: &mut Variables) -> Result<Conditions> {
    let pressure = variables
        .read(&Reference::new(
            'A',
            "SEA LEVEL PRESSURE",
            Some("millibars"),
        ))?
        .number()?;
    let temperature = variables
        .read(&Reference::new(
            'A',
            "SEA LEVEL TEMPERATURE",
            Some("celsius"),
        ))?
        .number()?;
    Conditions::new(pressure, temperature)
}

pub fn vector(direction_degrees: f64, speed_knots: f64) -> Result<DVec3> {
    ensure!(
        direction_degrees.is_finite() && speed_knots.is_finite() && speed_knots >= 0.0,
        "invalid wind"
    );
    let direction = direction_degrees.to_radians();
    let speed = speed_knots * 1852.0 / 3600.0;
    Ok(DVec3::new(
        -speed * direction.cos(),
        -speed * direction.sin(),
        0.0,
    ))
}

pub fn wind(variables: &mut Variables) -> Result<DVec3> {
    let direction = variables
        .read(&Reference::new(
            'A',
            "AMBIENT WIND DIRECTION",
            Some("degrees"),
        ))?
        .number()?;
    let speed = variables
        .read(&Reference::new('A', "AMBIENT WIND VELOCITY", Some("knots")))?
        .number()?;
    vector(direction, speed)
}

pub fn publish(state: &State, wind: DVec3, variables: &mut Variables) {
    let body = state.orientation.conjugate() * wind;
    for (name, value) in [
        ("AMBIENT WIND X", wind.y),
        ("AMBIENT WIND Y", -wind.z),
        ("AMBIENT WIND Z", wind.x),
        ("AIRCRAFT WIND X", body.y),
        ("AIRCRAFT WIND Y", -body.z),
        ("AIRCRAFT WIND Z", body.x),
    ] {
        variables.set_number(name, "meters per second", value);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nonstandard_weather_updates_instruments_and_barometric_setting() {
        use crate::{flight::Flight, speed_limits::SpeedLimits};
        use esp_script::Value;
        let mut vars = Variables::default();
        vars.set_number("SEA LEVEL PRESSURE", "millibars", 980.0);
        vars.set_number("SEA LEVEL TEMPERATURE", "celsius", 15.0);
        let state = State {
            position: DVec3::new(0.0, 0.0, -2000.0),
            velocity: DVec3::X * 100.0,
            ..State::default()
        };
        let limits =
            SpeedLimits::from_config(&esp_formats::cfg::Config::parse("").unwrap()).unwrap();
        let read = |vars: &mut Variables, name, unit| {
            vars.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        Flight::publish(&state, &mut vars).unwrap();
        limits.publish(&state, &mut vars).unwrap();
        assert!(read(&mut vars, "PRESSURE ALTITUDE", "meters") > 2000.0);
        vars.dispatch_event("BAROMETRIC", Value::Number(0.0))
            .unwrap();
        Flight::publish(&state, &mut vars).unwrap();
        limits.publish(&state, &mut vars).unwrap();
        assert!((read(&mut vars, "INDICATED ALTITUDE", "meters") - 2000.0).abs() < 1e-8);
        assert!((read(&mut vars, "KOHLSMAN SETTING MB", "millibars") - 980.0).abs() < 1e-10);
        let cool_pressure = read(&mut vars, "DYNAMIC PRESSURE", "pascals");
        vars.set_number("SEA LEVEL TEMPERATURE", "celsius", 45.0);
        Flight::publish(&state, &mut vars).unwrap();
        limits.publish(&state, &mut vars).unwrap();
        assert!(read(&mut vars, "DYNAMIC PRESSURE", "pascals") < cool_pressure);
        assert!(read(&mut vars, "AMBIENT TEMPERATURE", "celsius") > 30.0);
        vars.set_number("PLANE ALTITUDE", "feet", 18_000.0);
        vars.dispatch_event("BAROMETRIC", Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut vars, "KOHLSMAN SETTING MB", "millibars"), 1013.25);
        assert!(Conditions::new(0.0, 15.0).is_err());
        assert!(Conditions::new(1013.25, f64::NAN).is_err());
    }

    #[test]
    fn winds_use_meteorological_from_direction_and_body_axes() {
        let north = vector(0.0, 20.0).unwrap();
        assert!((north.x + 20.0 * 1852.0 / 3600.0).abs() < 1e-12);
        let east = vector(90.0, 20.0).unwrap();
        assert!((east.y + 20.0 * 1852.0 / 3600.0).abs() < 1e-12);
        assert!(vector(0.0, -1.0).is_err());
        assert!(vector(f64::NAN, 1.0).is_err());
        let mut variables = Variables::default();
        let state = State {
            orientation: glam::DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
            ..State::default()
        };
        publish(&state, north, &mut variables);
        let lateral = variables
            .read(&Reference::new('A', "AIRCRAFT WIND X", Some("knots")))
            .unwrap()
            .number()
            .unwrap();
        assert!((lateral - 20.0).abs() < 1e-10);
    }

    #[test]
    #[ignore = "requires original aircraft and OPENESP_AIRCRAFT_PATHS"]
    fn wind_and_temperature_change_flight_forces() {
        use crate::flight::Flight;
        let paths = std::env::var_os("OPENESP_AIRCRAFT_PATHS").expect("set OPENESP_AIRCRAFT_PATHS");
        let paths: Vec<_> = std::env::split_paths(&paths).collect();
        assert!(!paths.is_empty());
        for path in paths {
            let data = esp_assets::flight::FlightData::load(&path, 0).unwrap();
            let setup = || {
                let mut vars = Variables::default();
                Flight::configure_ground(&mut vars);
                vars.configure_flaps(
                    &esp_formats::cfg::aircraft::flaps(&data.configuration).unwrap(),
                )
                .unwrap();
                for name in [
                    "ELEVATOR DEFLECTION PCT",
                    "AILERON LEFT DEFLECTION PCT",
                    "RUDDER DEFLECTION PCT",
                ] {
                    vars.set_number(name, "percent", 0.0);
                }
                vars
            };
            let mut calm_vars = setup();
            let mut wind_vars = setup();
            wind_vars.set_number("AMBIENT WIND DIRECTION", "degrees", 0.0);
            wind_vars.set_number("AMBIENT WIND VELOCITY", "knots", 30.0);
            let mut calm = Flight::new(&data, 130.0, 1500.0, 0.5, true).unwrap();
            let mut headwind = Flight::new(&data, 130.0, 1500.0, 0.5, true).unwrap();
            Flight::publish(&calm.current, &mut calm_vars).unwrap();
            Flight::publish(&headwind.current, &mut wind_vars).unwrap();
            let read = |vars: &mut Variables, name| {
                vars.read(&Reference::new('A', name, Some("knots")))
                    .unwrap()
                    .number()
                    .unwrap()
            };
            assert!((read(&mut calm_vars, "AIRSPEED TRUE") - 130.0).abs() < 1e-9);
            assert!((read(&mut wind_vars, "AIRSPEED TRUE") - 160.0).abs() < 1e-9);
            assert!((read(&mut wind_vars, "GROUND VELOCITY") - 130.0).abs() < 1e-9);
            for _ in 0..60 {
                calm.advance(1.0 / 60.0, &mut calm_vars).unwrap();
                headwind.advance(1.0 / 60.0, &mut wind_vars).unwrap();
            }
            assert!(
                headwind.current.velocity.z < calm.current.velocity.z,
                "headwind did not increase lift"
            );
            let mut cold_vars = setup();
            let mut hot_vars = setup();
            cold_vars.set_number("SEA LEVEL TEMPERATURE", "celsius", -20.0);
            hot_vars.set_number("SEA LEVEL TEMPERATURE", "celsius", 45.0);
            let mut cold = Flight::new(&data, 130.0, 1500.0, 0.5, true).unwrap();
            let mut hot = Flight::new(&data, 130.0, 1500.0, 0.5, true).unwrap();
            for _ in 0..60 {
                cold.advance(1.0 / 60.0, &mut cold_vars).unwrap();
                hot.advance(1.0 / 60.0, &mut hot_vars).unwrap();
            }
            assert!(
                cold.current.velocity.z < hot.current.velocity.z,
                "denser air did not increase lift"
            );
        }
    }
}

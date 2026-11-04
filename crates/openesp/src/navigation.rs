use anyhow::{Result, ensure};
use esp_script::{Host, Reference};
use esp_sim::{flight::State, variables::Variables};
use glam::EulerRot;
use std::f64::consts::{PI, TAU};

const EARTH_RADIUS_METERS: f64 = 6_371_008.8;

pub struct Origin {
    latitude: f64,
    longitude: f64,
}

impl Origin {
    pub fn new(latitude: f64, longitude: f64) -> Result<Self> {
        ensure!(
            latitude.is_finite() && (-90.0..=90.0).contains(&latitude),
            "latitude must be between -90 and 90 degrees"
        );
        ensure!(
            longitude.is_finite() && (-180.0..=180.0).contains(&longitude),
            "longitude must be between -180 and 180 degrees"
        );
        Ok(Self {
            latitude: latitude.to_radians(),
            longitude: longitude.to_radians(),
        })
    }

    fn position(&self, north: f64, east: f64) -> Result<(f64, f64)> {
        ensure!(
            north.is_finite() && east.is_finite(),
            "local position must be finite"
        );
        let distance = north.hypot(east);
        ensure!(distance.is_finite(), "local position is too large");
        if distance == 0.0 {
            return Ok((self.latitude.to_degrees(), self.longitude.to_degrees()));
        }
        let angular_distance = distance / EARTH_RADIUS_METERS;
        let bearing = east.atan2(north);
        let latitude = (self.latitude.sin() * angular_distance.cos()
            + self.latitude.cos() * angular_distance.sin() * bearing.cos())
        .clamp(-1.0, 1.0)
        .asin();
        let longitude = self.longitude
            + (bearing.sin() * angular_distance.sin() * self.latitude.cos())
                .atan2(angular_distance.cos() - self.latitude.sin() * latitude.sin());
        Ok((
            latitude.to_degrees(),
            ((longitude + PI).rem_euclid(TAU) - PI).to_degrees(),
        ))
    }
}

pub fn publish(state: &State, variables: &mut Variables) -> Result<()> {
    let latitude = variables
        .read(&Reference::new(
            'A',
            "WORLD ORIGIN LATITUDE",
            Some("degrees"),
        ))?
        .number()?;
    let longitude = variables
        .read(&Reference::new(
            'A',
            "WORLD ORIGIN LONGITUDE",
            Some("degrees"),
        ))?
        .number()?;
    let (latitude, longitude) =
        Origin::new(latitude, longitude)?.position(state.position.x, state.position.y)?;
    for name in ["PLANE LATITUDE", "GPS POSITION LAT"] {
        variables.set_number(name, "degrees", latitude);
    }
    for name in ["PLANE LONGITUDE", "GPS POSITION LON"] {
        variables.set_number(name, "degrees", longitude);
    }
    let (heading, _, _) = state.orientation.to_euler(EulerRot::ZYX);
    let heading = heading.rem_euclid(TAU);
    let ground_speed = state.velocity.truncate().length();
    let track = if ground_speed > 0.5 {
        state.velocity.y.atan2(state.velocity.x).rem_euclid(TAU)
    } else {
        variables
            .read(&Reference::new(
                'A',
                "GPS GROUND TRUE TRACK",
                Some("radians"),
            ))
            .ok()
            .and_then(|value| value.number().ok())
            .unwrap_or(heading)
    };
    let body = state.orientation.conjugate() * state.velocity;
    for (name, value) in [
        ("GPS GROUND SPEED", ground_speed),
        ("HSI SPEED", ground_speed),
        ("VELOCITY WORLD X", state.velocity.y),
        ("VELOCITY WORLD Y", -state.velocity.z),
        ("VELOCITY WORLD Z", state.velocity.x),
        ("VELOCITY BODY X", body.y),
        ("VELOCITY BODY Y", -body.z),
        ("VELOCITY BODY Z", body.x),
    ] {
        variables.set_number(name, "meters per second", value);
    }
    variables.set_number("GPS GROUND TRUE HEADING", "radians", heading);
    variables.set_number("GPS GROUND TRUE TRACK", "radians", track);
    variables.set_number("GPS POSITION ALT", "meters", -state.position.z);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DQuat, DVec3};

    #[test]
    fn position_tracks_the_local_grid_and_crosses_the_date_line() {
        let origin = Origin::new(0.0, 179.9).unwrap();
        let degree = EARTH_RADIUS_METERS * PI / 180.0;
        let (latitude, longitude) = origin.position(degree, 0.0).unwrap();
        assert!((latitude - 1.0).abs() < 1e-10);
        assert!((longitude - 179.9).abs() < 1e-10);
        let (latitude, longitude) = origin.position(0.0, 0.2 * degree).unwrap();
        assert!(latitude.abs() < 1e-10);
        assert!((longitude + 179.9).abs() < 1e-10);
        assert!(Origin::new(91.0, 0.0).is_err());
        assert!(Origin::new(0.0, -181.0).is_err());
        assert!(Origin::new(f64::NAN, 0.0).is_err());
        assert!(origin.position(f64::INFINITY, 0.0).is_err());
    }

    #[test]
    fn publishes_geographic_position_to_cockpit_variables() {
        let mut variables = Variables::default();
        variables.set_number("WORLD ORIGIN LATITUDE", "degrees", 51.5);
        variables.set_number("WORLD ORIGIN LONGITUDE", "degrees", -0.1);
        let state = State {
            position: DVec3::new(1000.0, 500.0, -2000.0),
            ..State::default()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        let read = |vars: &mut Variables, name| {
            vars.read(&Reference::new('A', name, Some("degrees")))
                .unwrap()
                .number()
                .unwrap()
        };
        let lat = read(&mut variables, "GPS POSITION LAT");
        let lon = read(&mut variables, "GPS POSITION LON");
        assert!(lat > 51.5 && lon > -0.1);
        assert_eq!(lat, read(&mut variables, "PLANE LATITUDE"));
        assert_eq!(lon, read(&mut variables, "PLANE LONGITUDE"));
    }

    #[test]
    fn crosswind_separates_heading_from_ground_track() {
        let mut variables = Variables::default();
        variables.set_number("AMBIENT WIND DIRECTION", "degrees", 90.0);
        variables.set_number("AMBIENT WIND VELOCITY", "knots", 30.0);
        let wind = crate::weather::wind(&mut variables).unwrap();
        let mut state = State {
            position: DVec3::new(0.0, 0.0, -2000.0),
            velocity: DVec3::X * 100.0 + wind + DVec3::Z * -5.0,
            orientation: DQuat::IDENTITY,
            ..State::default()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        let read = |vars: &mut Variables, name, unit| {
            vars.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        let track = read(&mut variables, "GPS GROUND TRUE TRACK", "degrees");
        assert!(track > 350.0 && track < 360.0);
        assert_eq!(
            read(&mut variables, "GPS GROUND TRUE HEADING", "degrees"),
            0.0
        );
        assert!(
            (read(&mut variables, "GPS GROUND SPEED", "meters per second")
                - state.velocity.truncate().length())
            .abs()
                < 1e-10
        );
        assert!(
            (read(&mut variables, "VELOCITY WORLD X", "meters per second") - wind.y).abs() < 1e-10
        );
        assert_eq!(
            read(&mut variables, "VELOCITY WORLD Y", "meters per second"),
            5.0
        );
        assert_eq!(read(&mut variables, "GPS POSITION ALT", "meters"), 2000.0);
        assert!(
            (read(&mut variables, "AIRSPEED TRUE", "meters per second") - 100.125).abs() < 0.001
        );
        state.velocity = DVec3::ZERO;
        state.orientation = DQuat::from_rotation_z(1.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!((read(&mut variables, "GPS GROUND TRUE TRACK", "degrees") - track).abs() < 1e-10);
        assert!((read(&mut variables, "GPS GROUND TRUE HEADING", "radians") - 1.0).abs() < 1e-10);
    }
}

use anyhow::{Result, ensure};
use esp_script::{Host, Reference};
use esp_sim::{flight::State, variables::Variables};
use glam::{DVec3, EulerRot};
use std::f64::consts::{PI, TAU};

const EARTH_RADIUS_METERS: f64 = 6_371_008.8;

pub fn validate_declination(degrees: f64) -> Result<()> {
    ensure!(
        degrees.is_finite() && (-180.0..=180.0).contains(&degrees),
        "magnetic declination must be between -180 and 180 degrees"
    );
    Ok(())
}

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
    // MAGVAR follows the simulator convention: west is positive, east negative.
    let variation = variables
        .read(&Reference::new('A', "MAGVAR", Some("radians")))?
        .number()?;
    validate_declination(variation.to_degrees())?;
    let magnetic_heading = (heading + variation).rem_euclid(TAU);
    let drift = variables
        .read(&Reference::new('A', "GYRO DRIFT ERROR", Some("radians")))?
        .number()?;
    ensure!(drift.is_finite(), "gyro drift error must be finite");
    let gyro_heading = (magnetic_heading + drift).rem_euclid(TAU);
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
    variables.set_number(
        "GPS GROUND MAGNETIC TRACK",
        "radians",
        (track + variation).rem_euclid(TAU),
    );
    variables.set_number("GPS MAGVAR", "radians", variation);
    for name in ["PLANE HEADING DEGREES MAGNETIC", "MAGNETIC COMPASS"] {
        variables.set_number(name, "radians", magnetic_heading);
    }
    for name in ["HEADING INDICATOR", "PLANE HEADING DEGREES GYRO"] {
        variables.set_number(name, "radians", gyro_heading);
    }
    variables.set_number("GPS POSITION ALT", "meters", -state.position.z);
    crate::vor::publish(latitude, longitude, -state.position.z, variation, variables)?;
    publish_direct_to(
        latitude,
        longitude,
        variation,
        ground_speed,
        track,
        variables,
    )?;
    publish_hsi(variables)?;
    Ok(())
}

fn publish_hsi(variables: &mut Variables) -> Result<()> {
    let read = |variables: &mut Variables, name, unit| -> Result<f64> {
        Ok(variables
            .read(&Reference::new('A', name, Some(unit)))?
            .number()?)
    };
    let gps_selected = read(variables, "GPS DRIVES NAV1", "bool")? != 0.0;
    let previously_selected = read(variables, "HSI GPS SOURCE ACTIVE", "bool")? != 0.0;
    variables.set_number("HSI GPS SOURCE ACTIVE", "bool", f64::from(gps_selected));
    if !gps_selected {
        let vor_configured = read(variables, "VOR STATION ACTIVE", "bool")? != 0.0;
        let nav_valid = read(variables, "NAV HAS NAV:1", "bool")? != 0.0;
        if previously_selected && !vor_configured {
            variables.set_number("NAV CDI:1", "number", 0.0);
        }
        for name in [
            "HSI CDI NEEDLE VALID",
            "HSI BEARING VALID",
            "HSI GSI NEEDLE VALID",
            "HSI HAS LOCALIZER",
        ] {
            variables.set_number(name, "bool", 0.0);
        }
        variables.set_number("HSI CDI NEEDLE VALID", "bool", f64::from(nav_valid));
        let needle = if nav_valid {
            read(variables, "NAV RADIO CDI:1", "number")?
        } else {
            0.0
        };
        let distance = if nav_valid {
            read(variables, "NAV DME:1", "meters")?
        } else {
            0.0
        };
        let flag = if nav_valid {
            read(variables, "NAV TOFROM:1", "enum")?
        } else {
            0.0
        };
        variables.set_number("HSI CDI NEEDLE", "number", needle);
        variables.set_number("HSI BEARING", "radians", 0.0);
        variables.set_number("HSI DISTANCE", "meters", distance);
        variables.set_number("HSI TF FLAGS", "enum", flag);
        return Ok(());
    }
    let active = read(variables, "GPS IS ACTIVE WAY POINT", "bool")? != 0.0;
    let leg_valid = active && read(variables, "GPS WP PREV VALID", "bool")? != 0.0;
    let distance = read(variables, "GPS WP DISTANCE", "meters")?;
    let bearing_valid = active && distance > 1e-6;
    let bearing = read(variables, "GPS WP BEARING", "radians")?;
    let desired = read(variables, "GPS WP DESIRED TRACK", "radians")?;
    let needle = if leg_valid {
        read(variables, "GPS CDI NEEDLE", "number")?
    } else {
        0.0
    };
    let to_from = if !leg_valid || !bearing_valid {
        0.0
    } else if (bearing - desired).cos() >= 0.0 {
        1.0
    } else {
        2.0
    };
    variables.set_number("NAV CDI:1", "number", needle);
    variables.set_number("HSI CDI NEEDLE", "number", needle);
    variables.set_number("HSI CDI NEEDLE VALID", "bool", f64::from(leg_valid));
    variables.set_number("HSI BEARING", "radians", bearing);
    variables.set_number("HSI BEARING VALID", "bool", f64::from(bearing_valid));
    variables.set_number("HSI DISTANCE", "meters", distance);
    variables.set_number("HSI TF FLAGS", "enum", to_from);
    variables.set_number("HSI GSI NEEDLE VALID", "bool", 0.0);
    variables.set_number("HSI HAS LOCALIZER", "bool", 0.0);
    Ok(())
}

fn publish_direct_to(
    latitude: f64,
    longitude: f64,
    variation: f64,
    ground_speed: f64,
    track: f64,
    variables: &mut Variables,
) -> Result<()> {
    let active = variables
        .read(&Reference::new(
            'A',
            "GPS IS DIRECTTO FLIGHTPLAN",
            Some("bool"),
        ))?
        .number()?
        != 0.0;
    variables.set_number("GPS IS ACTIVE WAY POINT", "bool", f64::from(active));
    if !active {
        variables.set_number("GPS DIRECT LEG VALID", "bool", 0.0);
        variables.set_number("GPS WP PREV VALID", "bool", 0.0);
        for (name, unit) in [
            ("GPS WP DISTANCE", "meters"),
            ("GPS WP TRUE BEARING", "radians"),
            ("GPS WP BEARING", "radians"),
            ("GPS WP ETE", "seconds"),
            ("GPS WP CROSS TRK", "meters"),
            ("GPS WP DESIRED TRACK", "radians"),
            ("GPS WP TRACK ANGLE ERROR", "radians"),
            ("GPS CDI NEEDLE", "number"),
        ] {
            variables.set_number(name, unit, 0.0);
        }
        return Ok(());
    }
    let destination_latitude = variables
        .read(&Reference::new('A', "GPS WP NEXT LAT", Some("degrees")))?
        .number()?;
    let destination_longitude = variables
        .read(&Reference::new('A', "GPS WP NEXT LON", Some("degrees")))?
        .number()?;
    let destination = Origin::new(destination_latitude, destination_longitude)?;
    let leg_valid = variables
        .read(&Reference::new('A', "GPS DIRECT LEG VALID", Some("bool")))?
        .number()?
        != 0.0;
    let changed = if leg_valid {
        let old_latitude = variables
            .read(&Reference::new(
                'A',
                "GPS DIRECT LEG TARGET LAT",
                Some("degrees"),
            ))?
            .number()?;
        let old_longitude = variables
            .read(&Reference::new(
                'A',
                "GPS DIRECT LEG TARGET LON",
                Some("degrees"),
            ))?
            .number()?;
        old_latitude != destination_latitude || old_longitude != destination_longitude
    } else {
        true
    };
    if changed {
        variables.set_number("GPS WP PREV LAT", "degrees", latitude);
        variables.set_number("GPS WP PREV LON", "degrees", longitude);
        variables.set_number("GPS DIRECT LEG TARGET LAT", "degrees", destination_latitude);
        variables.set_number(
            "GPS DIRECT LEG TARGET LON",
            "degrees",
            destination_longitude,
        );
        variables.set_number("GPS DIRECT LEG VALID", "bool", 1.0);
    }
    let start_latitude = variables
        .read(&Reference::new('A', "GPS WP PREV LAT", Some("degrees")))?
        .number()?;
    let start_longitude = variables
        .read(&Reference::new('A', "GPS WP PREV LON", Some("degrees")))?
        .number()?;
    let start = Origin::new(start_latitude, start_longitude)?;
    let current = Origin::new(latitude, longitude)?;
    let latitude = latitude.to_radians();
    let delta_latitude = destination.latitude - latitude;
    let delta_longitude = destination.longitude - longitude.to_radians();
    let haversine = ((delta_latitude / 2.0).sin().powi(2)
        + latitude.cos() * destination.latitude.cos() * (delta_longitude / 2.0).sin().powi(2))
    .clamp(0.0, 1.0);
    let distance = 2.0 * EARTH_RADIUS_METERS * haversine.sqrt().atan2((1.0 - haversine).sqrt());
    let bearing = if distance < 1e-6 {
        0.0
    } else {
        (delta_longitude.sin() * destination.latitude.cos())
            .atan2(
                latitude.cos() * destination.latitude.sin()
                    - latitude.sin() * destination.latitude.cos() * delta_longitude.cos(),
            )
            .rem_euclid(TAU)
    };
    let vector = |point: &Origin| {
        DVec3::new(
            point.latitude.cos() * point.longitude.cos(),
            point.latitude.cos() * point.longitude.sin(),
            point.latitude.sin(),
        )
    };
    let normal = vector(&start).cross(vector(&destination));
    let valid_leg = normal.length() > 1e-10;
    variables.set_number("GPS WP PREV VALID", "bool", f64::from(valid_leg));
    let (cross_track, desired_track) = if valid_leg {
        let normal = normal.normalize();
        let position = vector(&current);
        // Positive cross-track distance places the aircraft to the right of the course.
        let cross = -normal.dot(position).clamp(-1.0, 1.0).asin() * EARTH_RADIUS_METERS;
        let tangent = normal.cross(position);
        let north = DVec3::new(
            -current.latitude.sin() * current.longitude.cos(),
            -current.latitude.sin() * current.longitude.sin(),
            current.latitude.cos(),
        );
        let east = DVec3::new(-current.longitude.sin(), current.longitude.cos(), 0.0);
        (
            cross,
            tangent.dot(east).atan2(tangent.dot(north)).rem_euclid(TAU),
        )
    } else {
        (0.0, bearing)
    };
    let scale = variables
        .read(&Reference::new('A', "GPS CDI SCALING", Some("meters")))?
        .number()?;
    ensure!(
        scale.is_finite() && scale > 0.0,
        "GPS CDI scaling must be positive and finite"
    );
    variables.set_number("GPS WP CROSS TRK", "meters", cross_track);
    variables.set_number(
        "GPS WP DESIRED TRACK",
        "radians",
        (desired_track + variation).rem_euclid(TAU),
    );
    variables.set_number(
        "GPS WP TRACK ANGLE ERROR",
        "radians",
        (desired_track - track + PI).rem_euclid(TAU) - PI,
    );
    variables.set_number(
        "GPS CDI NEEDLE",
        "number",
        (-127.0 * cross_track / scale).clamp(-127.0, 127.0),
    );
    variables.set_number("GPS WP DISTANCE", "meters", distance);
    variables.set_number("GPS WP TRUE BEARING", "radians", bearing);
    variables.set_number(
        "GPS WP BEARING",
        "radians",
        if distance < 1e-6 {
            0.0
        } else {
            (bearing + variation).rem_euclid(TAU)
        },
    );
    variables.set_number(
        "GPS WP ETE",
        "seconds",
        if ground_speed > 0.5 {
            distance / ground_speed
        } else {
            0.0
        },
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_script::Value;
    use glam::{DQuat, DVec3};

    #[test]
    fn hsi_follows_selected_gps_source_and_clears_when_switched_off() {
        let mut variables = Variables::default();
        variables.set_number("GPS IS DIRECTTO FLIGHTPLAN", "bool", 1.0);
        variables.set_number("GPS WP NEXT LON", "degrees", 1.0);
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        let mut state = State {
            velocity: DVec3::Y * 100.0,
            ..State::default()
        };
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        state.position.x = 500.0;
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE VALID", "bool"), 1.0);
        assert_eq!(read(&mut variables, "HSI BEARING VALID", "bool"), 1.0);
        assert_eq!(read(&mut variables, "HSI TF FLAGS", "enum"), 1.0);
        let cdi = read(&mut variables, "GPS CDI NEEDLE", "number");
        assert!(cdi > 0.0);
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE", "number"), cdi);
        assert_eq!(read(&mut variables, "NAV CDI:1", "number"), cdi);
        assert_eq!(
            read(&mut variables, "HSI DISTANCE", "meters"),
            read(&mut variables, "GPS WP DISTANCE", "meters")
        );
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE VALID", "bool"), 0.0);
        assert_eq!(read(&mut variables, "NAV CDI:1", "number"), 0.0);
        variables.set_number("NAV CDI:1", "number", 42.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV CDI:1", "number"), 42.0);
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        variables.set_number("GPS IS DIRECTTO FLIGHTPLAN", "bool", 0.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE VALID", "bool"), 0.0);
        assert_eq!(read(&mut variables, "HSI BEARING VALID", "bool"), 0.0);
    }

    #[test]
    fn direct_to_course_tracks_cross_track_and_recaptures_changed_destination() {
        let mut variables = Variables::default();
        variables.set_number("GPS IS DIRECTTO FLIGHTPLAN", "bool", 1.0);
        variables.set_number("GPS WP NEXT LON", "degrees", 1.0);
        variables.set_number("GPS CDI SCALING", "meters", 1000.0);
        let mut state = State {
            velocity: DVec3::Y * 100.0,
            ..State::default()
        };
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!((read(&mut variables, "GPS WP DESIRED TRACK", "degrees") - 90.0).abs() < 1e-8);
        state.position = DVec3::new(500.0, 1000.0, 0.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!((read(&mut variables, "GPS WP CROSS TRK", "meters") + 500.0).abs() < 0.01);
        assert!((read(&mut variables, "GPS CDI NEEDLE", "number") - 63.5).abs() < 0.01);
        state.position.x = -2000.0;
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!((read(&mut variables, "GPS WP CROSS TRK", "meters") - 2000.0).abs() < 0.01);
        assert_eq!(read(&mut variables, "GPS CDI NEEDLE", "number"), -127.0);
        variables.set_number("GPS WP NEXT LAT", "degrees", 1.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!(read(&mut variables, "GPS WP CROSS TRK", "meters").abs() < 1e-6);
        variables.set_number("GPS IS DIRECTTO FLIGHTPLAN", "bool", 0.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "GPS WP PREV VALID", "bool"), 0.0);
        assert_eq!(read(&mut variables, "GPS CDI NEEDLE", "number"), 0.0);
    }

    #[test]
    fn direct_to_updates_distance_bearing_and_time_as_aircraft_moves() {
        let mut variables = Variables::default();
        variables.set_number("WORLD ORIGIN LONGITUDE", "degrees", 179.9);
        variables.set_number("MAGVAR", "degrees", -10.0);
        variables.set_number("GPS IS DIRECTTO FLIGHTPLAN", "bool", 1.0);
        variables.set_number("GPS WP NEXT LAT", "degrees", 0.0);
        variables.set_number("GPS WP NEXT LON", "degrees", -179.9);
        let mut state = State {
            velocity: DVec3::Y * 100.0,
            ..State::default()
        };
        let read = |vars: &mut Variables, name, unit| {
            vars.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        let distance = read(&mut variables, "GPS WP DISTANCE", "meters");
        assert!((distance - EARTH_RADIUS_METERS * 0.2_f64.to_radians()).abs() < 1e-6);
        assert!((read(&mut variables, "GPS WP TRUE BEARING", "degrees") - 90.0).abs() < 1e-9);
        assert!((read(&mut variables, "GPS WP BEARING", "degrees") - 80.0).abs() < 1e-9);
        assert!((read(&mut variables, "GPS WP ETE", "seconds") - distance / 100.0).abs() < 1e-8);
        state.position.y = 1000.0;
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!(
            (read(&mut variables, "GPS WP DISTANCE", "meters") - (distance - 1000.0)).abs() < 1e-6
        );
        variables.set_number("GPS IS DIRECTTO FLIGHTPLAN", "bool", 0.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "GPS IS ACTIVE WAY POINT", "bool"), 0.0);
        assert_eq!(read(&mut variables, "GPS WP DISTANCE", "meters"), 0.0);
    }

    #[test]
    fn gyro_adjustment_moves_indicator_without_moving_compass_and_can_be_synchronized() {
        let mut variables = Variables::default();
        variables.set_number("MAGVAR", "degrees", -10.0);
        let state = State {
            orientation: DQuat::from_rotation_z(5_f64.to_radians()),
            ..State::default()
        };
        let read = |vars: &mut Variables, name| {
            vars.read(&Reference::new('A', name, Some("degrees")))
                .unwrap()
                .number()
                .unwrap()
        };
        variables
            .dispatch_event("GYRO_DRIFT_SET", Value::Number(8.0))
            .unwrap();
        variables
            .dispatch_event("GYRO_DRIFT_DEC", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!((read(&mut variables, "MAGNETIC COMPASS") - 355.0).abs() < 1e-10);
        assert!((read(&mut variables, "HEADING INDICATOR") - 2.0).abs() < 1e-10);
        assert!((read(&mut variables, "GYRO DRIFT ERROR") - 7.0).abs() < 1e-10);
        variables
            .dispatch_event("HEADING_GYRO_SET", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!((read(&mut variables, "PLANE HEADING DEGREES GYRO") - 355.0).abs() < 1e-10);
        assert!(
            variables
                .dispatch_event("GYRO_DRIFT_SET", Value::Number(f64::NAN))
                .is_err()
        );
    }

    #[test]
    fn publishes_magnetic_heading_and_track_with_wraparound() {
        let mut variables = Variables::default();
        variables.set_number("MAGVAR", "degrees", -10.0);
        let state = State {
            orientation: DQuat::from_rotation_z(5_f64.to_radians()),
            velocity: DVec3::new(100.0, 0.0, 0.0),
            ..State::default()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        let read = |vars: &mut Variables, name| {
            vars.read(&Reference::new('A', name, Some("degrees")))
                .unwrap()
                .number()
                .unwrap()
        };
        for name in [
            "PLANE HEADING DEGREES MAGNETIC",
            "MAGNETIC COMPASS",
            "HEADING INDICATOR",
            "PLANE HEADING DEGREES GYRO",
        ] {
            assert!((read(&mut variables, name) - 355.0).abs() < 1e-10);
        }
        assert!((read(&mut variables, "GPS GROUND MAGNETIC TRACK") - 350.0).abs() < 1e-10);
        assert!((read(&mut variables, "GPS MAGVAR") + 10.0).abs() < 1e-10);
        assert!(validate_declination(f64::NAN).is_err());
        assert!(validate_declination(181.0).is_err());
    }

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

use anyhow::{Result, ensure};
use esp_physics::{flight::State, variables::Variables};
use esp_script::{Host, Reference};
use glam::{DVec3, EulerRot};
use std::f64::consts::{PI, TAU};

const EARTH_RADIUS_METERS: f64 = 6_371_008.8;

#[derive(Clone, Copy)]
pub struct InitialPlacement {
    north: f64,
    east: f64,
    heading: f64,
    velocity_scale: f64,
    vertical_air_velocity: Option<f64>,
}

impl InitialPlacement {
    pub fn new(north: f64, east: f64, heading_degrees: f64) -> Result<Self> {
        ensure!(
            [north, east, heading_degrees]
                .iter()
                .all(|value| value.is_finite()),
            "starting position and heading must be finite"
        );
        Ok(Self {
            north,
            east,
            heading: heading_degrees.rem_euclid(360.0).to_radians(),
            velocity_scale: 1.0,
            vertical_air_velocity: None,
        })
    }

    pub fn apply(self, state: &mut State) {
        let rotation = glam::DQuat::from_rotation_z(self.heading);
        state.position.x = self.north;
        state.position.y = self.east;
        state.orientation = rotation * state.orientation;
        state.velocity = rotation * state.velocity * self.velocity_scale;
        if let Some(vertical) = self.vertical_air_velocity {
            state.velocity.z = vertical;
        }
    }
}

pub fn approach_start(
    threshold_north: f64,
    threshold_east: f64,
    true_course: f64,
    slope_degrees: f64,
    distance_nm: f64,
    airspeed_knots: f64,
    wind: DVec3,
) -> Result<(InitialPlacement, f64)> {
    ensure!(
        [
            threshold_north,
            threshold_east,
            true_course,
            slope_degrees,
            distance_nm,
            airspeed_knots
        ]
        .iter()
        .all(|value| value.is_finite())
            && wind.is_finite(),
        "approach parameters must be finite"
    );
    ensure!(
        (0.5..=10.0).contains(&distance_nm),
        "approach distance must be between 0.5 and 10 nautical miles"
    );
    ensure!(
        (1.0..=6.0).contains(&slope_degrees),
        "approach glideslope must be between 1 and 6 degrees"
    );
    let speed = airspeed_knots * 1852.0 / 3600.0;
    ensure!(speed > 0.0, "approach airspeed must be positive");
    let course = true_course.to_radians();
    let forward = DVec3::new(course.cos(), course.sin(), 0.0);
    let right = DVec3::new(-course.sin(), course.cos(), 0.0);
    let crosswind = wind.dot(right);
    let along_wind = wind.dot(forward);
    let slope = slope_degrees.to_radians().tan();
    let offset = along_wind * slope - wind.z;
    let a = 1.0 + slope * slope;
    let b = 2.0 * slope * offset;
    let c = crosswind * crosswind + offset * offset - speed * speed;
    let discriminant = b * b - 4.0 * a * c;
    ensure!(
        discriminant >= 0.0 && discriminant.is_finite(),
        "wind exceeds the available approach airspeed"
    );
    let forward_air = (-b + discriminant.sqrt()) / (2.0 * a);
    let ground_speed = forward_air + along_wind;
    ensure!(
        forward_air > 0.0 && ground_speed > 0.0,
        "wind prevents forward progress on approach"
    );
    let correction = (-crosswind).atan2(forward_air);
    let distance = distance_nm * 1852.0;
    let mut placement = InitialPlacement::new(
        threshold_north - forward.x * distance,
        threshold_east - forward.y * distance,
        true_course + correction.to_degrees(),
    )?;
    placement.velocity_scale = forward_air.hypot(crosswind) / speed;
    placement.vertical_air_velocity = Some(ground_speed * slope - wind.z);
    Ok((placement, distance * slope))
}

pub fn validate_declination(degrees: f64) -> Result<()> {
    ensure!(
        degrees.is_finite() && (-180.0..=180.0).contains(&degrees),
        "magnetic declination must be between -180 and 180 degrees"
    );
    Ok(())
}

pub fn publish_acceleration(
    before: &State,
    after: &State,
    dt: f64,
    variables: &mut Variables,
) -> Result<()> {
    ensure!(dt.is_finite() && dt >= 0.0, "invalid acceleration timestep");
    if dt == 0.0 {
        return Ok(());
    }
    let acceleration = (after.velocity - before.velocity) / dt;
    let orientation = before.orientation.slerp(after.orientation, 0.5);
    let specific_force = orientation.conjugate() * (acceleration - glam::DVec3::Z * 9.80665);
    ensure!(specific_force.is_finite(), "invalid specific force");
    let current = variables
        .read(&Reference::new(
            'A',
            "TURN COORDINATOR BALL",
            Some("position 128"),
        ))?
        .number()?;
    ensure!(
        current.is_finite(),
        "invalid turn coordinator ball position"
    );
    let magnitude = specific_force.y.hypot(specific_force.z);
    let target = if magnitude < 0.1 {
        current
    } else {
        (-specific_force.y).atan2(-specific_force.z) / 15_f64.to_radians() * 127.0
    };
    let weight = 1.0 - (-dt / 0.5).exp();
    let ball = (current + (target.clamp(-127.0, 127.0) - current) * weight).clamp(-127.0, 127.0);
    variables.set_number("TURN COORDINATOR BALL", "position 128", ball);
    variables.set_number("TURN COORDINATOR BALL INV", "position 128", -ball);
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

    pub fn position(&self, north: f64, east: f64) -> Result<(f64, f64)> {
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

    pub fn local_position(&self, latitude: f64, longitude: f64) -> Result<(f64, f64)> {
        let target = Self::new(latitude, longitude)?;
        let delta_latitude = target.latitude - self.latitude;
        let delta_longitude = target.longitude - self.longitude;
        let h = ((delta_latitude / 2.0).sin().powi(2)
            + self.latitude.cos() * target.latitude.cos() * (delta_longitude / 2.0).sin().powi(2))
        .clamp(0.0, 1.0);
        let distance = 2.0 * EARTH_RADIUS_METERS * h.sqrt().atan2((1.0 - h).sqrt());
        let bearing = (delta_longitude.sin() * target.latitude.cos()).atan2(
            self.latitude.cos() * target.latitude.sin()
                - self.latitude.sin() * target.latitude.cos() * delta_longitude.cos(),
        );
        Ok((distance * bearing.cos(), distance * bearing.sin()))
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
    let yaw_rate = state.angular_velocity.z;
    // Approximate a coordinator's 30-degree canted gyro, normalized to yaw rate.
    let coordinator_rate = yaw_rate + state.angular_velocity.x * (30_f64.to_radians()).tan();
    for name in ["TURN INDICATOR RATE", "TURN INDICATOR RATE:1"] {
        variables.set_number(name, "radians per second", yaw_rate);
    }
    variables.set_number(
        "TURN COORDINATOR RATE",
        "radians per second",
        coordinator_rate,
    );
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
        let vor_configured = read(variables, "NAV RADIO OWNED:1", "bool")? != 0.0;
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
        let distance = if read(variables, "NAV HAS DME:1", "bool")? != 0.0 {
            read(variables, "NAV DME:1", "meters")?
        } else {
            0.0
        };
        let flag = if nav_valid {
            read(variables, "NAV TOFROM:1", "enum")?
        } else {
            0.0
        };
        let localizer = nav_valid && read(variables, "NAV HAS LOCALIZER:1", "bool")? != 0.0;
        let glide_valid = nav_valid && read(variables, "NAV HAS GLIDE SLOPE:1", "bool")? != 0.0;
        let glide = if glide_valid {
            read(variables, "NAV GSI:1", "number")? * 119.0 / 127.0
        } else {
            0.0
        };
        variables.set_number("HSI CDI NEEDLE", "number", needle);
        variables.set_number("HSI BEARING", "radians", 0.0);
        variables.set_number("HSI DISTANCE", "meters", distance);
        variables.set_number("HSI TF FLAGS", "enum", flag);
        variables.set_number("HSI HAS LOCALIZER", "bool", f64::from(localizer));
        variables.set_number("HSI GSI NEEDLE VALID", "bool", f64::from(glide_valid));
        variables.set_number("HSI GSI NEEDLE", "number", glide);
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
        let (north, east) = origin.local_position(latitude, longitude).unwrap();
        assert!(north.abs() < 1e-6);
        assert!((east - 0.2 * degree).abs() < 1e-6);
        assert!(Origin::new(91.0, 0.0).is_err());
        assert!(Origin::new(0.0, -181.0).is_err());
        assert!(Origin::new(f64::NAN, 0.0).is_err());
        assert!(origin.position(f64::INFINITY, 0.0).is_err());
    }
}

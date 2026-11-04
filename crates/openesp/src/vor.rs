use anyhow::{Result, ensure};
use esp_script::{Host, Reference};
use esp_sim::variables::Variables;
use std::f64::consts::{PI, TAU};

const EARTH_RADIUS: f64 = 6_371_008.8;

pub fn validate(latitude: f64, longitude: f64, frequency: f64, range: f64) -> Result<()> {
    crate::navigation::Origin::new(latitude, longitude)?;
    ensure!(
        frequency.is_finite()
            && (108.0..=117.95).contains(&frequency)
            && (frequency * 20.0 - (frequency * 20.0).round()).abs() < 1e-6,
        "VOR frequency must be a 50 kHz channel between 108.00 and 117.95 MHz"
    );
    ensure!(
        range.is_finite() && range > 0.0,
        "VOR range must be positive and finite"
    );
    Ok(())
}

pub fn validate_ils(course: f64, glideslope: f64, frequency: f64) -> Result<()> {
    ensure!(course.is_finite(), "ILS course must be finite");
    ensure!(
        glideslope.is_finite() && (1.0..=6.0).contains(&glideslope),
        "ILS glideslope must be between 1 and 6 degrees"
    );
    let tenths = (frequency * 10.0 + 1e-7).floor() as i64;
    ensure!(
        frequency.is_finite() && (108.1..=111.95).contains(&frequency) && tenths % 2 == 1,
        "ILS frequency must use an odd-tenth channel from 108.10 to 111.95 MHz"
    );
    Ok(())
}

pub fn publish(
    latitude: f64,
    longitude: f64,
    altitude: f64,
    variation: f64,
    variables: &mut Variables,
) -> Result<()> {
    let read = |v: &mut Variables, name, unit| -> Result<f64> {
        Ok(v.read(&Reference::new('A', name, Some(unit)))?.number()?)
    };
    let station_active = read(variables, "VOR STATION ACTIVE", "bool")? != 0.0;
    if !station_active {
        variables.set_number("NAV HAS NAV:1", "bool", 0.0);
        variables.set_number("NAV HAS DME:1", "bool", 0.0);
        variables.set_number("NAV HAS LOCALIZER:1", "bool", 0.0);
        variables.set_number("NAV HAS GLIDE SLOPE:1", "bool", 0.0);
        return Ok(());
    }
    let station_latitude = read(variables, "VOR STATION LATITUDE", "degrees")?;
    let station_longitude = read(variables, "VOR STATION LONGITUDE", "degrees")?;
    let station_frequency = read(variables, "VOR STATION FREQUENCY", "megahertz")?;
    let range = read(variables, "VOR STATION RANGE", "nautical miles")?;
    validate(
        station_latitude,
        station_longitude,
        station_frequency,
        range,
    )?;
    let tuned = read(variables, "NAV ACTIVE FREQUENCY:1", "megahertz")?;
    let obs = read(variables, "NAV OBS:1", "radians")?;
    ensure!(
        tuned.is_finite() && obs.is_finite(),
        "invalid NAV radio setting"
    );
    let station_latitude = station_latitude.to_radians();
    let latitude = latitude.to_radians();
    let delta_lon = (longitude - station_longitude).to_radians();
    let haversine = (((latitude - station_latitude) / 2.0).sin().powi(2)
        + station_latitude.cos() * latitude.cos() * (delta_lon / 2.0).sin().powi(2))
    .clamp(0.0, 1.0);
    let ground_distance = 2.0 * EARTH_RADIUS * haversine.sqrt().atan2((1.0 - haversine).sqrt());
    let distance = ground_distance.hypot(altitude);
    let mut signal = (tuned - station_frequency).abs() < 0.0005
        && ground_distance <= range * 1852.0
        && ground_distance > 1.0;
    let mut radial = 0.0;
    let mut error = 0.0;
    let mut needle = 0.0;
    let mut flag = 0.0;
    let localizer = read(variables, "VOR STATION LOCALIZER", "bool")? != 0.0;
    let mut glideslope_signal = false;
    let mut glideslope_needle = 0.0;
    if signal {
        let true_radial = (delta_lon.sin() * latitude.cos()).atan2(
            station_latitude.cos() * latitude.sin()
                - station_latitude.sin() * latitude.cos() * delta_lon.cos(),
        );
        radial = (true_radial + variation).rem_euclid(TAU);
        if localizer {
            let course = read(variables, "ILS LOCALIZER COURSE", "degrees")?;
            let slope = read(variables, "ILS GLIDESLOPE ANGLE", "degrees")?;
            validate_ils(course, slope, station_frequency)?;
            let course = course.to_radians().rem_euclid(TAU);
            error = (radial - course).rem_euclid(TAU) - PI;
            let sector = if ground_distance <= 10.0 * 1852.0 {
                35_f64.to_radians()
            } else {
                10_f64.to_radians()
            };
            signal = ground_distance <= 25.0 * 1852.0 && error.abs() <= sector;
            if signal {
                needle = (error / 2.5_f64.to_radians() * 127.0).clamp(-127.0, 127.0);
                flag = 1.0;
                let elevation = altitude.atan2(ground_distance);
                let glide_angle = slope.to_radians();
                glideslope_signal = ground_distance <= 10.0 * 1852.0
                    && error.abs() <= 8_f64.to_radians()
                    && elevation >= 0.45 * glide_angle
                    && elevation <= 1.75 * glide_angle;
                if glideslope_signal {
                    glideslope_needle = ((glide_angle - elevation) / 0.7_f64.to_radians() * 127.0)
                        .clamp(-127.0, 127.0);
                }
            }
        } else {
            let difference = (radial - obs + PI).rem_euclid(TAU) - PI;
            let from = difference.abs() <= PI / 2.0;
            error = if from {
                difference
            } else {
                (difference + TAU).rem_euclid(TAU) - PI
            };
            needle = ((if from { -error } else { error }) / 10_f64.to_radians() * 127.0)
                .clamp(-127.0, 127.0);
            flag = if from { 2.0 } else { 1.0 };
        }
    }
    variables.set_number("NAV HAS NAV:1", "bool", f64::from(signal));
    variables.set_number("NAV HAS DME:1", "bool", f64::from(signal));
    variables.set_number(
        "NAV HAS LOCALIZER:1",
        "bool",
        f64::from(signal && localizer),
    );
    variables.set_number(
        "NAV HAS GLIDE SLOPE:1",
        "bool",
        f64::from(glideslope_signal),
    );
    let localizer_course = if localizer {
        read(variables, "ILS LOCALIZER COURSE", "degrees")?
    } else {
        0.0
    };
    variables.set_number("NAV LOCALIZER:1", "degrees", localizer_course);
    variables.set_number("NAV GSI:1", "number", glideslope_needle);
    variables.set_number("NAV RADIAL:1", "radians", radial);
    variables.set_number("NAV RADIAL ERROR:1", "radians", error);
    variables.set_number(
        "NAV VOR DISTANCE:1",
        "meters",
        if signal { ground_distance } else { 0.0 },
    );
    variables.set_number("NAV DME:1", "meters", if signal { distance } else { 0.0 });
    variables.set_number("NAV SIGNAL:1", "number", if signal { 1.0 } else { 0.0 });
    variables.set_number("NAV TOFROM:1", "enum", flag);
    variables.set_number("NAV RADIO CDI:1", "number", needle);
    if read(variables, "GPS DRIVES NAV1", "bool")? == 0.0 {
        variables.set_number("NAV CDI:1", "number", needle);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_script::Value;
    use esp_sim::flight::State;
    use glam::DVec3;

    #[test]
    fn ils_provides_localizer_and_glideslope_only_in_coverage() {
        let mut variables = Variables::default();
        variables.set_number("VOR STATION ACTIVE", "bool", 1.0);
        variables.set_number("VOR STATION LOCALIZER", "bool", 1.0);
        variables.set_number("VOR STATION FREQUENCY", "megahertz", 110.3);
        variables.set_number("NAV ACTIVE FREQUENCY:1", "megahertz", 110.3);
        let mut state = State {
            position: DVec3::new(-10_000.0, 100.0, -10_000.0 * 3_f64.to_radians().tan()),
            ..State::default()
        };
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS LOCALIZER:1", "bool"), 1.0);
        assert_eq!(read(&mut variables, "HSI HAS LOCALIZER", "bool"), 1.0);
        assert_eq!(read(&mut variables, "HSI GSI NEEDLE VALID", "bool"), 1.0);
        assert!(read(&mut variables, "NAV CDI:1", "number") < 0.0);
        assert!(read(&mut variables, "NAV GSI:1", "number").abs() < 0.1);
        state.position.z = -10_000.0 * 2_f64.to_radians().tan();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "HSI GSI NEEDLE", "number"), 119.0);
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "HSI GSI NEEDLE VALID", "bool"), 0.0);
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        state.position.y = 10_000.0;
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS LOCALIZER:1", "bool"), 0.0);
        assert_eq!(read(&mut variables, "HSI GSI NEEDLE VALID", "bool"), 0.0);
        assert!(validate_ils(0.0, 3.0, 110.0).is_err());
        assert!(validate_ils(0.0, 0.0, 110.3).is_err());
    }

    #[test]
    fn legacy_gauge_event_ids_control_radio_obs_and_gyro() {
        let mut variables = Variables::default();
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        variables
            .dispatch_key_event(0x100b4, Value::Number(90.0))
            .unwrap();
        variables
            .dispatch_key_event(0x1007f, Value::Number(0.0))
            .unwrap();
        variables
            .dispatch_key_event(0x1033f, Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut variables, "NAV OBS:1", "degrees"), 81.0);
        variables
            .dispatch_key_event(0x1038f, Value::Number(0x1400 as f64))
            .unwrap();
        variables
            .dispatch_key_event(0x10390, Value::Number(0.0))
            .unwrap();
        assert_eq!(
            read(&mut variables, "NAV ACTIVE FREQUENCY:1", "megahertz"),
            114.0
        );
        variables
            .dispatch_key_event(0x10347, Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut variables, "GPS DRIVES NAV1", "bool"), 1.0);
        variables
            .dispatch_key_event(0x101fe, Value::Number(10.0))
            .unwrap();
        variables
            .dispatch_key_event(0x10155, Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut variables, "GYRO DRIFT ERROR", "degrees"), 11.0);
        variables
            .dispatch_key_event(0x10020, Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut variables, "GYRO DRIFT ERROR", "degrees"), 0.0);
    }

    #[test]
    fn nav_radio_tuning_and_swap_drive_receiver() {
        let mut variables = Variables::default();
        variables.set_number("VOR STATION ACTIVE", "bool", 1.0);
        let state = State {
            position: DVec3::new(-10_000.0, 0.0, -1000.0),
            ..State::default()
        };
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        variables
            .dispatch_event("NAV1_STBY_SET", Value::Number(0x1400 as f64))
            .unwrap();
        assert_eq!(
            read(&mut variables, "NAV1 STANDBY FREQUENCY", "megahertz"),
            114.0
        );
        variables
            .dispatch_event("NAV1_RADIO_SWAP", Value::Number(0.0))
            .unwrap();
        assert_eq!(
            read(&mut variables, "NAV ACTIVE FREQUENCY:1", "megahertz"),
            114.0
        );
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 0.0);
        variables
            .dispatch_event("NAV1_RADIO_SWAP", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 1.0);
        variables
            .dispatch_event("NAV1_STBY_SET_HZ", Value::Number(117_975_000.0))
            .unwrap();
        variables
            .dispatch_event("NAV1_RADIO_FRACT_INC", Value::Number(0.0))
            .unwrap();
        assert_eq!(
            read(&mut variables, "NAV STANDBY FREQUENCY:1", "hertz"),
            117_000_000.0
        );
        variables
            .dispatch_event("NAV1_RADIO_WHOLE_INC", Value::Number(0.0))
            .unwrap();
        assert_eq!(
            read(&mut variables, "NAV STANDBY FREQUENCY:1", "hertz"),
            108_000_000.0
        );
        variables
            .dispatch_event("NAV1_RADIO_FRACT_DEC_CARRY", Value::Number(0.0))
            .unwrap();
        assert_eq!(
            read(&mut variables, "NAV STANDBY FREQUENCY:1", "hertz"),
            117_950_000.0
        );
        assert!(
            variables
                .dispatch_event("NAV1_STBY_SET", Value::Number(0x1a00 as f64))
                .is_err()
        );
        assert_eq!(
            read(&mut variables, "NAV STANDBY FREQUENCY:1", "hertz"),
            117_950_000.0
        );
    }

    #[test]
    fn vor_receiver_tracks_obs_tuning_range_and_hsi_source() {
        let mut variables = Variables::default();
        variables.set_number("VOR STATION ACTIVE", "bool", 1.0);
        variables
            .dispatch_event("VOR1_SET", Value::Number(360.0))
            .unwrap();
        let mut state = State {
            position: DVec3::new(-10_000.0, 1000.0, -1000.0),
            ..State::default()
        };
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 1.0);
        assert_eq!(read(&mut variables, "NAV TOFROM:1", "enum"), 1.0);
        let needle = read(&mut variables, "NAV CDI:1", "number");
        assert!(needle < 0.0 && needle > -127.0);
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE", "number"), needle);
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE VALID", "bool"), 1.0);
        variables.set_number("GPS IS DIRECTTO FLIGHTPLAN", "bool", 1.0);
        variables.set_number("GPS WP NEXT LON", "degrees", 1.0);
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 1.0);
        assert_ne!(read(&mut variables, "NAV CDI:1", "number"), needle);
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV CDI:1", "number"), needle);
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE", "number"), needle);
        let dme = read(&mut variables, "NAV DME:1", "meters");
        assert!((dme - 10_099.5).abs() < 1.0);
        variables
            .dispatch_event("VOR1_OBI_DEC", Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut variables, "NAV OBS:1", "degrees"), 359.0);
        variables
            .dispatch_event("NAV1_RADIO_SET_HZ", Value::Number(114_000_000.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 0.0);
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE VALID", "bool"), 0.0);
        variables
            .dispatch_event("NAV1_RADIO_SET_HZ", Value::Number(113_000_000.0))
            .unwrap();
        state.position.x = -100_000.0;
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 0.0);
        state.position = DVec3::new(10_000.0, 0.0, -1000.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV TOFROM:1", "enum"), 2.0);
    }

    #[test]
    fn station_configuration_rejects_invalid_coordinates_and_channels() {
        assert!(validate(90.1, 0.0, 113.0, 40.0).is_err());
        assert!(validate(0.0, 0.0, 113.01, 40.0).is_err());
        assert!(validate(0.0, 0.0, 113.0, 0.0).is_err());
    }
}

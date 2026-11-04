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
    let signal = (tuned - station_frequency).abs() < 0.0005
        && ground_distance <= range * 1852.0
        && ground_distance > 1.0;
    let mut radial = 0.0;
    let mut error = 0.0;
    let mut needle = 0.0;
    let mut flag = 0.0;
    if signal {
        let true_radial = (delta_lon.sin() * latitude.cos()).atan2(
            station_latitude.cos() * latitude.sin()
                - station_latitude.sin() * latitude.cos() * delta_lon.cos(),
        );
        radial = (true_radial + variation).rem_euclid(TAU);
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
    variables.set_number("NAV HAS NAV:1", "bool", f64::from(signal));
    variables.set_number("NAV HAS DME:1", "bool", f64::from(signal));
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

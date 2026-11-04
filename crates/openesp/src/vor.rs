use anyhow::{Result, ensure};
use esp_script::{Host, Reference};
use esp_sim::variables::Variables;
use std::f64::consts::{PI, TAU};

const EARTH_RADIUS: f64 = 6_371_008.8;

#[derive(Clone, Debug)]
pub struct Station {
    pub latitude: f64,
    pub longitude: f64,
    pub frequency: f64,
    pub range: f64,
}

impl std::str::FromStr for Station {
    type Err = anyhow::Error;
    fn from_str(text: &str) -> Result<Self> {
        let fields = text.split(',').collect::<Vec<_>>();
        ensure!(fields.len() == 4, "expected LAT,LON,MHZ,RANGE_NM");
        let station = Self {
            latitude: fields[0].trim().parse()?,
            longitude: fields[1].trim().parse()?,
            frequency: fields[2].trim().parse()?,
            range: fields[3].trim().parse()?,
        };
        validate(
            station.latitude,
            station.longitude,
            station.frequency,
            station.range,
        )?;
        Ok(station)
    }
}

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
    for index in 1..=2 {
        publish_receiver(index, latitude, longitude, altitude, variation, variables)?;
    }
    Ok(())
}

fn publish_receiver(
    index: u32,
    latitude: f64,
    longitude: f64,
    altitude: f64,
    variation: f64,
    variables: &mut Variables,
) -> Result<()> {
    let key = |name: &str| format!("{name}:{index}");
    fn read(v: &mut Variables, name: &str, unit: &str) -> Result<f64> {
        Ok(v.read(&Reference::new('A', name, Some(unit)))?.number()?)
    }
    let count = read(variables, "VOR STATION COUNT", "number")?;
    ensure!(
        count.is_finite() && count.fract() == 0.0 && (1.0..=64.0).contains(&count),
        "invalid VOR station count"
    );
    let frequency_name = key("NAV ACTIVE FREQUENCY");
    let obs_name = key("NAV OBS");
    let tuned = read(variables, &frequency_name, "megahertz")?;
    let obs = read(variables, &obs_name, "radians")?;
    ensure!(
        tuned.is_finite() && obs.is_finite(),
        "invalid NAV radio setting"
    );
    let mut any_active = false;
    let mut best: Option<(u32, Station, f64)> = None;
    for station_index in 1..=count as u32 {
        let station_key = |field: &str| {
            if station_index == 1 {
                format!("VOR STATION {field}")
            } else {
                format!("VOR STATION {field}:{station_index}")
            }
        };
        if read(variables, &station_key("ACTIVE"), "bool")? == 0.0 {
            continue;
        }
        any_active = true;
        let station = Station {
            latitude: read(variables, &station_key("LATITUDE"), "degrees")?,
            longitude: read(variables, &station_key("LONGITUDE"), "degrees")?,
            frequency: read(variables, &station_key("FREQUENCY"), "megahertz")?,
            range: read(variables, &station_key("RANGE"), "nautical miles")?,
        };
        validate(
            station.latitude,
            station.longitude,
            station.frequency,
            station.range,
        )?;
        if (tuned - station.frequency).abs() >= 0.0005 {
            continue;
        }
        let lat1 = latitude.to_radians();
        let lat2 = station.latitude.to_radians();
        let h = (((lat2 - lat1) / 2.0).sin().powi(2)
            + lat1.cos()
                * lat2.cos()
                * ((station.longitude - longitude).to_radians() / 2.0)
                    .sin()
                    .powi(2))
        .clamp(0.0, 1.0);
        let distance = 2.0 * EARTH_RADIUS * h.sqrt().atan2((1.0 - h).sqrt());
        if distance > station.range * 1852.0 {
            continue;
        }
        if best
            .as_ref()
            .is_none_or(|(_, _, current)| distance < *current)
        {
            best = Some((station_index, station, distance));
        }
    }
    variables.set_number(&key("NAV RADIO OWNED"), "bool", f64::from(any_active));
    let Some((station_index, station, _)) = best else {
        variables.set_number(&key("NAV HAS NAV"), "bool", 0.0);
        variables.set_number(&key("NAV HAS DME"), "bool", 0.0);
        variables.set_number(&key("NAV HAS LOCALIZER"), "bool", 0.0);
        variables.set_number(&key("NAV HAS GLIDE SLOPE"), "bool", 0.0);
        for (name, unit) in [
            ("NAV RADIO CDI", "number"),
            ("NAV GSI", "number"),
            ("NAV DME", "meters"),
            ("NAV VOR DISTANCE", "meters"),
            ("NAV SIGNAL", "number"),
            ("NAV TOFROM", "enum"),
        ] {
            variables.set_number(&key(name), unit, 0.0);
        }
        if any_active && (index != 1 || read(variables, "GPS DRIVES NAV1", "bool")? == 0.0) {
            variables.set_number(&key("NAV CDI"), "number", 0.0);
        }
        return Ok(());
    };
    let station_latitude = station.latitude.to_radians();
    let station_longitude = station.longitude;
    let station_frequency = station.frequency;
    let range = station.range;
    let latitude = latitude.to_radians();
    let delta_lon = (longitude - station_longitude).to_radians();
    let haversine = (((latitude - station_latitude) / 2.0).sin().powi(2)
        + station_latitude.cos() * latitude.cos() * (delta_lon / 2.0).sin().powi(2))
    .clamp(0.0, 1.0);
    let ground_distance = 2.0 * EARTH_RADIUS * haversine.sqrt().atan2((1.0 - haversine).sqrt());
    let distance = ground_distance.hypot(altitude);
    // Model every configured station as collocated with a DME transponder.
    // DME range does not depend on directional localizer or VOR reception.
    let dme_signal =
        (tuned - station_frequency).abs() < 0.0005 && ground_distance <= range * 1852.0;
    let mut signal = dme_signal && ground_distance > 1.0;
    let mut radial = 0.0;
    let mut error = 0.0;
    let mut needle = 0.0;
    let mut flag = 0.0;
    let localizer = if station_index == 1 {
        read(variables, "VOR STATION LOCALIZER", "bool")? != 0.0
    } else {
        read(
            variables,
            &format!("VOR STATION LOCALIZER:{station_index}"),
            "bool",
        )? != 0.0
    };
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
    variables.set_number(&key("NAV HAS NAV"), "bool", f64::from(signal));
    variables.set_number(&key("NAV HAS DME"), "bool", f64::from(dme_signal));
    variables.set_number(
        &key("NAV HAS LOCALIZER"),
        "bool",
        f64::from(signal && localizer),
    );
    variables.set_number(
        &key("NAV HAS GLIDE SLOPE"),
        "bool",
        f64::from(glideslope_signal),
    );
    let localizer_course = if localizer {
        read(variables, "ILS LOCALIZER COURSE", "degrees")?
    } else {
        0.0
    };
    variables.set_number(&key("NAV LOCALIZER"), "degrees", localizer_course);
    variables.set_number(&key("NAV GSI"), "number", glideslope_needle);
    variables.set_number(&key("NAV RADIAL"), "radians", radial);
    variables.set_number(&key("NAV RADIAL ERROR"), "radians", error);
    variables.set_number(
        &key("NAV VOR DISTANCE"),
        "meters",
        if signal { ground_distance } else { 0.0 },
    );
    variables.set_number(
        &key("NAV DME"),
        "meters",
        if dme_signal { distance } else { 0.0 },
    );
    variables.set_number(&key("NAV SIGNAL"), "number", if signal { 1.0 } else { 0.0 });
    variables.set_number(&key("NAV TOFROM"), "enum", flag);
    variables.set_number(&key("NAV RADIO CDI"), "number", needle);
    if index != 1 || read(variables, "GPS DRIVES NAV1", "bool")? == 0.0 {
        variables.set_number(&key("NAV CDI"), "number", needle);
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
    fn receivers_pick_different_stations_and_nearest_matching_signal() {
        let mut variables = Variables::default();
        variables.set_number("VOR STATION ACTIVE", "bool", 1.0);
        variables.set_number("VOR STATION COUNT", "number", 2.0);
        for (name, unit, value) in [
            ("ACTIVE", "bool", 1.0),
            ("LATITUDE", "degrees", -0.08),
            ("LONGITUDE", "degrees", 0.0),
            ("FREQUENCY", "megahertz", 114.0),
            ("RANGE", "nautical miles", 40.0),
            ("LOCALIZER", "bool", 0.0),
        ] {
            variables.set_number(&format!("VOR STATION {name}:2"), unit, value);
        }
        variables.set_number("NAV ACTIVE FREQUENCY:2", "megahertz", 114.0);
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
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        let distance1 = read(&mut variables, "NAV DME:1", "meters");
        let distance2 = read(&mut variables, "NAV DME:2", "meters");
        assert!(distance1 > 10_000.0 && distance2 < 2000.0);
        variables.set_number("VOR STATION FREQUENCY:2", "megahertz", 113.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!((read(&mut variables, "NAV DME:1", "meters") - distance2).abs() < 1e-6);
        assert_eq!(read(&mut variables, "NAV HAS NAV:2", "bool"), 0.0);
    }

    #[test]
    fn parses_additional_stations() {
        let station: Station = "-33.9,151.2,113.5,40".parse().unwrap();
        assert_eq!(station.latitude, -33.9);
        assert_eq!(station.frequency, 113.5);
        assert!("0,0,113.01,40".parse::<Station>().is_err());
        assert!("0,0,113".parse::<Station>().is_err());
    }

    #[test]
    fn two_receivers_tune_and_select_courses_independently() {
        let mut variables = Variables::default();
        variables.set_number("VOR STATION ACTIVE", "bool", 1.0);
        let state = State {
            position: DVec3::new(-10_000.0, 1000.0, -1000.0),
            ..State::default()
        };
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        variables
            .dispatch_key_event(0x100ad, Value::Number(0x1400 as f64))
            .unwrap();
        variables
            .dispatch_key_event(0x100b5, Value::Number(90.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 1.0);
        assert_eq!(read(&mut variables, "NAV HAS NAV:2", "bool"), 0.0);
        assert_eq!(read(&mut variables, "NAV OBS:1", "degrees"), 0.0);
        assert_eq!(read(&mut variables, "NAV OBS:2", "degrees"), 90.0);
        variables
            .dispatch_event("NAV2_STBY_SET", Value::Number(0x1300 as f64))
            .unwrap();
        variables
            .dispatch_key_event(0x10394, Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:2", "bool"), 1.0);
        let second = read(&mut variables, "NAV CDI:2", "number");
        assert_ne!(read(&mut variables, "NAV CDI:1", "number"), second);
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV CDI:2", "number"), second);
        assert_eq!(
            read(&mut variables, "NAV2 STANDBY FREQUENCY", "megahertz"),
            114.0
        );
    }

    #[test]
    fn legacy_bcd_frequency_round_trips_through_gauge_and_radio_event() {
        let mut variables = Variables::default();
        for (encoded, expected_hz) in [
            (0x1300, 113_000_000.0),
            (0x1302, 113_025_000.0),
            (0x1305, 113_050_000.0),
            (0x1307, 113_075_000.0),
        ] {
            variables
                .dispatch_event("NAV1_RADIO_SET", Value::Number(encoded as f64))
                .unwrap();
            let read = |v: &mut Variables, unit| {
                v.read(&Reference::new('A', "NAV ACTIVE FREQUENCY:1", Some(unit)))
                    .unwrap()
                    .number()
                    .unwrap()
            };
            assert_eq!(read(&mut variables, "hertz"), expected_hz);
            assert_eq!(read(&mut variables, "frequency bcd16"), encoded as f64);
        }
        assert!(
            variables
                .dispatch_event("NAV1_RADIO_SET", Value::Number(0x1a00 as f64))
                .is_err()
        );
    }

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
    fn dme_remains_available_overhead_and_outside_localizer_sector() {
        let mut variables = Variables::default();
        variables.set_number("VOR STATION ACTIVE", "bool", 1.0);
        variables.set_number("VOR STATION LOCALIZER", "bool", 1.0);
        variables.set_number("VOR STATION FREQUENCY", "megahertz", 110.3);
        variables.set_number("NAV ACTIVE FREQUENCY:1", "megahertz", 110.3);
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        let mut state = State {
            position: DVec3::new(-10_000.0, 10_000.0, -500.0),
            ..State::default()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 0.0);
        assert_eq!(read(&mut variables, "NAV HAS DME:1", "bool"), 1.0);
        let distance = read(&mut variables, "NAV DME:1", "meters");
        assert!((distance - (10_000.0_f64.hypot(10_000.0)).hypot(500.0)).abs() < 1.0);
        assert_eq!(read(&mut variables, "HSI DISTANCE", "meters"), distance);
        state.position = DVec3::new(0.0, 0.0, -1000.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 0.0);
        assert_eq!(read(&mut variables, "NAV HAS DME:1", "bool"), 1.0);
        assert!((read(&mut variables, "NAV DME:1", "meters") - 1000.0).abs() < 1e-6);
        variables
            .dispatch_event("NAV1_RADIO_SET_HZ", Value::Number(114_000_000.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS DME:1", "bool"), 0.0);
        assert_eq!(read(&mut variables, "NAV DME:1", "meters"), 0.0);
        assert_eq!(read(&mut variables, "HSI DISTANCE", "meters"), 0.0);
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

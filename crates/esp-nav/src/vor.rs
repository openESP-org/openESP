use anyhow::{Result, ensure};
use esp_physics::variables::VariableStore;
use esp_script::Reference;
use std::f64::consts::{PI, TAU};

const EARTH_RADIUS: f64 = 6_371_008.8;

#[derive(Clone, Debug)]
pub struct Station {
    pub latitude: f64,
    pub longitude: f64,
    pub frequency: f64,
    pub range: f64,
    pub elevation: f64,
}

impl std::str::FromStr for Station {
    type Err = anyhow::Error;
    fn from_str(text: &str) -> Result<Self> {
        let fields = text.split(',').collect::<Vec<_>>();
        ensure!(
            (4..=5).contains(&fields.len()),
            "expected LAT,LON,MHZ,RANGE_NM[,ELEVATION_M]"
        );
        let station = Self {
            latitude: fields[0].trim().parse()?,
            longitude: fields[1].trim().parse()?,
            frequency: fields[2].trim().parse()?,
            range: fields[3].trim().parse()?,
            elevation: fields
                .get(4)
                .map(|v| v.trim().parse())
                .transpose()?
                .unwrap_or(0.0),
        };
        validate(
            station.latitude,
            station.longitude,
            station.frequency,
            station.range,
        )?;
        ensure!(
            station.elevation.is_finite(),
            "VOR elevation must be finite"
        );
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
    variables: &mut dyn VariableStore,
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
    variables: &mut dyn VariableStore,
) -> Result<()> {
    let key = |name: &str| format!("{name}:{index}");
    fn read(v: &mut dyn VariableStore, name: &str, unit: &str) -> Result<f64> {
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
            elevation: read(variables, &station_key("ELEVATION"), "meters").unwrap_or(0.0),
        };
        validate(
            station.latitude,
            station.longitude,
            station.frequency,
            station.range,
        )?;
        ensure!(
            station.elevation.is_finite(),
            "VOR elevation must be finite"
        );
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
    let relative_altitude = altitude - station.elevation;
    let distance = ground_distance.hypot(relative_altitude);
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
                let elevation = relative_altitude.atan2(ground_distance);
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

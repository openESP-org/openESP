//! esp-weather owns weather observations and external weather data sources.
use anyhow::{Context, Result, ensure};
use std::time::Duration;

#[derive(Clone, Debug, PartialEq)]
pub struct Observation {
    pub station: String,
    pub wind_direction: Option<f64>,
    pub wind_knots: Option<f64>,
    pub gust_knots: Option<f64>,
    pub temperature_c: Option<f64>,
    pub pressure_hpa: Option<f64>,
    pub visibility_m: Option<f64>,
}

/// Parse the primary fields of a raw METAR, leaving missing values unset.
pub fn parse(raw: &str, station: &str) -> Result<Observation> {
    let mut words = raw.split_whitespace().peekable();
    if matches!(words.peek(), Some(&"METAR" | &"SPECI")) {
        words.next();
    }
    let found = words.next().context("METAR missing station")?;
    ensure!(
        found.eq_ignore_ascii_case(station),
        "METAR station mismatch"
    );
    let mut result = Observation {
        station: found.to_ascii_uppercase(),
        wind_direction: None,
        wind_knots: None,
        gust_knots: None,
        temperature_c: None,
        pressure_hpa: None,
        visibility_m: None,
    };
    let words: Vec<_> = words.collect();
    for (index, &word) in words.iter().enumerate() {
        if word == "RMK" {
            break;
        }
        if !word.is_ascii() {
            continue;
        }
        if let Some(wind) = word.strip_suffix("KT").or_else(|| word.strip_suffix("MPS")) {
            if wind.len() >= 5 && wind.is_ascii() {
                let (direction, rest) = wind.split_at(3);
                let (speed, gust) = rest
                    .split_once('G')
                    .map_or((rest, None), |(speed, gust)| (speed, Some(gust)));
                if let Ok(speed) = speed.parse::<f64>() {
                    let factor = if word.ends_with("MPS") {
                        3600.0 / 1852.0
                    } else {
                        1.0
                    };
                    result.wind_direction = direction.parse::<f64>().ok().filter(|v| *v <= 360.0);
                    result.wind_knots = Some(speed * factor);
                    result.gust_knots =
                        gust.and_then(|v| v.parse::<f64>().ok()).map(|v| v * factor);
                }
            }
        } else if word.len() == 5 && word.starts_with('Q') {
            result.pressure_hpa = word[1..]
                .parse::<f64>()
                .ok()
                .filter(|v| (800.0..=1100.0).contains(v));
        } else if word.len() == 5 && word.starts_with('A') {
            result.pressure_hpa = word[1..]
                .parse::<f64>()
                .ok()
                .map(|value| value * 0.3386389)
                .filter(|v| (800.0..=1100.0).contains(v));
        } else if let Some(value) = word.strip_suffix("SM") {
            let value = value.trim_start_matches(['P', 'M']);
            let miles = value
                .split_once('/')
                .and_then(|(a, b)| Some(a.parse::<f64>().ok()? / b.parse::<f64>().ok()?))
                .or_else(|| value.parse::<f64>().ok())
                .map(|miles| {
                    let whole = if value.contains('/') && index > 0 {
                        words[index - 1]
                            .parse::<u8>()
                            .ok()
                            .filter(|value| *value < 10)
                            .unwrap_or(0)
                    } else {
                        0
                    };
                    miles + f64::from(whole)
                });
            result.visibility_m = miles
                .filter(|v| v.is_finite() && *v > 0.0)
                .map(|v| v * 1609.344);
        } else if let Some((temperature, _)) = word.split_once('/') {
            if !temperature.is_empty() && temperature.len() <= 3 {
                let (negative, digits) = temperature
                    .strip_prefix('M')
                    .map_or((false, temperature), |value| (true, value));
                if let Ok(value) = digits.parse::<f64>() {
                    result.temperature_c = Some(if negative { -value } else { value });
                }
            }
        } else if word == "CAVOK" {
            result.visibility_m = Some(10_000.0);
        } else if word.len() == 4
            && word.bytes().all(|v| v.is_ascii_digit())
            && let Ok(value) = word.parse::<f64>()
        {
            result.visibility_m = Some(value.max(1.0));
        }
    }
    ensure!(
        result.wind_knots.is_some()
            || result.pressure_hpa.is_some()
            || result.temperature_c.is_some(),
        "METAR lacks usable weather data"
    );
    Ok(result)
}

/// Fetch the latest report from NOAA's Aviation Weather Center.
pub fn fetch(station: &str) -> Result<Observation> {
    ensure!(
        station.len() == 4 && station.bytes().all(|b| b.is_ascii_alphanumeric()),
        "invalid METAR station"
    );
    let agent: ureq::Agent = ureq::Agent::config_builder()
        .timeout_global(Some(Duration::from_secs(10)))
        .build()
        .into();
    let text = agent
        .get("https://aviationweather.gov/api/data/metar")
        .query("ids", station)
        .query("format", "raw")
        .header("User-Agent", "openESP/0.1 (flight simulator)")
        .call()
        .context("fetching METAR")?
        .body_mut()
        .read_to_string()
        .context("reading METAR")?;
    parse(text.lines().next().context("no METAR available")?, station)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn parses_us_and_international_reports_without_using_remarks() {
        let us = parse(
            "METAR KLAX 080753Z 05006G16KT 10SM CLR 22/06 A3001 RMK Q0999",
            "KLAX",
        )
        .unwrap();
        assert_eq!(
            (us.wind_direction, us.wind_knots, us.gust_knots),
            (Some(50.0), Some(6.0), Some(16.0))
        );
        assert_eq!(us.temperature_c, Some(22.0));
        assert!((us.pressure_hpa.unwrap() - 1016.25).abs() < 0.1);
        assert!((us.visibility_m.unwrap() - 16093.44).abs() < 0.1);
        let european = parse("EGLL 081020Z 27005MPS 9999 BKN020 M02/M05 Q1008", "EGLL").unwrap();
        assert_eq!(european.temperature_c, Some(-2.0));
        assert_eq!(european.pressure_hpa, Some(1008.0));
        assert_eq!(european.visibility_m, Some(9999.0));
        assert!((european.wind_knots.unwrap() - 9.719).abs() < 0.01);
        let mixed = parse("KJFK 081020Z 00000KT 1 1/2SM -RA 05/04 A2992", "KJFK").unwrap();
        assert!((mixed.visibility_m.unwrap() - 1.5 * 1609.344).abs() < 0.1);
        assert_eq!(mixed.temperature_c, Some(5.0));
        assert!(parse("METAR KLAX 080753Z 05006KT", "EGLL").is_err());
    }
    #[test]
    #[ignore = "requires network access to aviationweather.gov"]
    fn fetches_real_metar() {
        assert_eq!(fetch("KLAX").unwrap().station, "KLAX");
    }
}

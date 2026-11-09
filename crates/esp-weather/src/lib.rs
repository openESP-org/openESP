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
    /// Normalized observed rainfall and snowfall intensities.
    pub precipitation: [f64; 2],
    /// None means the report did not specify sky conditions; an empty list is clear sky.
    pub clouds: Option<Vec<CloudLayer>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CloudLayer {
    pub base_agl_m: f64,
    pub coverage: f64,
}

/// Accumulated cloud displacement in local east/north metres.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CloudDrift {
    pub east: f64,
    pub north: f64,
}

impl CloudDrift {
    pub fn advance(&mut self, dt: f64, wind_east: f64, wind_north: f64) -> Result<()> {
        ensure!(
            dt.is_finite() && dt >= 0.0 && wind_east.is_finite() && wind_north.is_finite(),
            "invalid cloud advection"
        );
        let east = self.east + wind_east * dt;
        let north = self.north + wind_north * dt;
        ensure!(
            east.is_finite() && north.is_finite(),
            "invalid cloud displacement"
        );
        *self = Self { east, north };
        Ok(())
    }
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
        precipitation: [0.0; 2],
        clouds: None,
    };
    let words: Vec<_> = words.collect();
    for (index, &word) in words.iter().enumerate() {
        if matches!(word, "RMK" | "NOSIG" | "TEMPO" | "BECMG") {
            break;
        }
        if !word.is_ascii() {
            continue;
        }
        if !word.starts_with("VC")
            && !word.starts_with("RE")
            && !word.contains("BLSN")
            && !word.contains("DRSN")
        {
            let intensity: f64 = if word.starts_with('-') {
                0.3
            } else if word.starts_with('+') {
                1.0
            } else {
                0.6
            };
            if word.contains("RA") || word.contains("DZ") {
                result.precipitation[0] = result.precipitation[0].max(intensity);
            }
            if word.contains("SN") || word.contains("SG") {
                result.precipitation[1] = result.precipitation[1].max(intensity);
            }
        }
        if matches!(word, "CLR" | "SKC" | "NSC" | "NCD" | "CAVOK") {
            result.clouds = Some(Vec::new());
        } else if word.len() >= 6 {
            let coverage = match &word[..3] {
                "FEW" => Some(0.19),
                "SCT" => Some(0.44),
                "BKN" => Some(0.75),
                "OVC" => Some(1.0),
                _ => None,
            };
            if let Some(coverage) = coverage
                && let Ok(height) = word[3..6].parse::<u32>()
            {
                result.clouds.get_or_insert_with(Vec::new).push(CloudLayer {
                    base_agl_m: f64::from(height) * 30.48,
                    coverage,
                });
            }
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
    fn advection_is_continuous_when_wind_changes() {
        let mut drift = CloudDrift::default();
        drift.advance(10.0, 5.0, 0.0).unwrap();
        assert_eq!(
            drift,
            CloudDrift {
                east: 50.0,
                north: 0.0
            }
        );
        drift.advance(5.0, 0.0, -4.0).unwrap();
        assert_eq!(
            drift,
            CloudDrift {
                east: 50.0,
                north: -20.0
            }
        );
        assert!(drift.advance(f64::NAN, 1.0, 1.0).is_err());
        assert_eq!(
            drift,
            CloudDrift {
                east: 50.0,
                north: -20.0
            }
        );
    }
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
        assert_eq!(
            european.clouds,
            Some(vec![CloudLayer {
                base_agl_m: 609.6,
                coverage: 0.75
            }])
        );
        assert_eq!(us.clouds, Some(vec![]));
        assert!((european.wind_knots.unwrap() - 9.719).abs() < 0.01);
        let mixed = parse("KJFK 081020Z 00000KT 1 1/2SM -RA 05/04 A2992", "KJFK").unwrap();
        assert!((mixed.visibility_m.unwrap() - 1.5 * 1609.344).abs() < 0.1);
        assert_eq!(mixed.temperature_c, Some(5.0));
        assert_eq!(mixed.precipitation, [0.3, 0.0]);
        let snow = parse(
            "METAR TEST 081020Z 00000KT 2000 +SHSN BKN010 M02/M04 Q1005",
            "TEST",
        )
        .unwrap();
        assert_eq!(snow.precipitation, [0.0, 1.0]);
        let clear = parse(
            "METAR TEST 081020Z 00000KT CAVOK 05/04 Q1005 NOSIG RMK RAB05",
            "TEST",
        )
        .unwrap();
        assert_eq!(clear.precipitation, [0.0, 0.0]);
        let layers = parse(
            "METAR LQBK 220600Z 33007KT 9999 SCT012 BKN027 12/10 Q1015",
            "LQBK",
        )
        .unwrap()
        .clouds
        .unwrap();
        assert_eq!(layers.len(), 2);
        assert!((layers[0].base_agl_m - 365.76).abs() < 0.01);
        assert!((layers[1].base_agl_m - 822.96).abs() < 0.01);
        assert_eq!((layers[0].coverage, layers[1].coverage), (0.44, 0.75));
        assert!(parse("METAR KLAX 080753Z 05006KT", "EGLL").is_err());
    }
    #[test]
    #[ignore = "requires network access to aviationweather.gov"]
    fn fetches_real_metar() {
        assert_eq!(fetch("KLAX").unwrap().station, "KLAX");
    }
}

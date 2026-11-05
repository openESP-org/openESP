use anyhow::{Context, Result, ensure};
use glam::DVec3;
use std::path::Path;

use crate::navigation::Origin;

pub struct AirportScene {
    pub runways: Vec<esp_render::Runway>,
    pub taxiways: Vec<esp_render::Taxiway>,
}

/// Place BGL runways and taxiways on the current local ground plane.
pub fn airport(
    path: &Path,
    ident: Option<&str>,
    latitude: f64,
    longitude: f64,
) -> Result<AirportScene> {
    let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    let airports = esp_formats::bgl::Bgl::parse(&bytes)?.airports()?;
    let airport = airports
        .iter()
        .find(|airport| {
            ident.map_or(!airport.runways.is_empty(), |ident| {
                airport.ident.eq_ignore_ascii_case(ident)
            })
        })
        .with_context(|| {
            format!(
                "airport {} not found in {}",
                ident.unwrap_or("with runways"),
                path.display()
            )
        })?;
    ensure!(
        !airport.runways.is_empty(),
        "airport {} has no runways",
        airport.ident
    );
    let origin = Origin::new(latitude, longitude)?;
    let runways = airport
        .runways
        .iter()
        .map(|runway| {
            ensure!(
                runway.length_m.is_finite()
                    && (1.0..=20_000.0).contains(&runway.length_m)
                    && runway.width_m.is_finite()
                    && (1.0..=1_000.0).contains(&runway.width_m)
                    && runway.heading_deg.is_finite(),
                "invalid runway geometry at {}",
                airport.ident
            );
            let (north, east) = origin.local_position(runway.latitude, runway.longitude)?;
            let heading = f64::from(runway.heading_deg).to_radians();
            let half_length = f64::from(runway.length_m) * 0.5;
            Ok(esp_render::Runway {
                threshold: DVec3::new(
                    east - heading.sin() * half_length,
                    0.0,
                    north - heading.cos() * half_length,
                ),
                heading,
                length: runway.length_m,
                width: runway.width_m,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let mut taxiways = Vec::new();
    for path in &airport.taxi_paths {
        // Parking links use a different endpoint index space; runway links are
        // already covered by the dedicated runway surface.
        if path.kind != 1
            || !path.draw_surface
            || !path.width_m.is_finite()
            || !(0.1..=1000.0).contains(&path.width_m)
        {
            continue;
        }
        let (Some(start), Some(end)) = (
            airport.taxi_points.get(path.start as usize),
            airport.taxi_points.get(path.end as usize),
        ) else {
            continue;
        };
        let (start_north, start_east) = origin.local_position(start.latitude, start.longitude)?;
        let (end_north, end_east) = origin.local_position(end.latitude, end.longitude)?;
        taxiways.push(esp_render::Taxiway {
            start: DVec3::new(start_east, 0.0, start_north),
            end: DVec3::new(end_east, 0.0, end_north),
            width: path.width_m,
            centerline: path.centerline,
        });
    }
    Ok(AirportScene { runways, taxiways })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore = "requires a P3D airport BGL in OPENESP_AIRPORT_BGL"]
    fn loads_real_airport_runway_positions() {
        let path = std::env::var_os("OPENESP_AIRPORT_BGL").expect("set OPENESP_AIRPORT_BGL");
        let scene = airport(Path::new(&path), Some("KLAX"), 33.9377117, -118.4010096).unwrap();
        let runways = scene.runways;
        assert_eq!(scene.taxiways.len(), 406);
        assert_eq!(runways.len(), 4);
        assert!(runways.iter().any(|r| (r.length - 3938.93).abs() < 0.1));
        let first = runways
            .iter()
            .find(|r| (r.length - 3938.93).abs() < 0.1)
            .unwrap();
        let center = first.threshold
            + DVec3::new(first.heading.sin(), 0.0, first.heading.cos())
                * f64::from(first.length)
                * 0.5;
        assert!(center.length() < 2.0);
        assert!(super::airport(Path::new(&path), Some("ZZZZ"), 33.9, -118.4).is_err());
    }
}

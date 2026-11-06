use anyhow::{Context, Result, ensure};
use esp_formats::bgl::{Bgl, airport::Airport};
use esp_nav::navigation::Origin;

use crate::cli::Arguments;

pub(super) fn configure(arguments: &mut Arguments) -> Result<()> {
    if arguments.runway.is_none() && arguments.parking.is_none() {
        return Ok(());
    }
    let path = arguments
        .scenery_bgl
        .as_ref()
        .context("airport BGL is required for spawning")?;
    let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    let airports = Bgl::parse(&bytes)?.airports()?;
    let ident = arguments
        .airport
        .as_deref()
        .context("airport ICAO is required")?;
    let airport = airports
        .iter()
        .find(|airport| airport.ident.eq_ignore_ascii_case(ident))
        .with_context(|| format!("airport {ident} not found in {}", path.display()))?;
    let (latitude, longitude, heading) =
        location(airport, arguments.runway.as_deref(), arguments.parking)?;
    arguments.latitude = latitude;
    arguments.longitude = longitude;
    arguments.heading = Some(heading);
    arguments.height = 0.0;
    arguments.airspeed = Some(0.0);
    eprintln!(
        "spawn: {} at {latitude:.7}, {longitude:.7}, heading {heading:.1}",
        airport.ident
    );
    Ok(())
}

fn location(
    airport: &Airport,
    runway: Option<&str>,
    parking: Option<usize>,
) -> Result<(f64, f64, f64)> {
    if let Some(index) = parking {
        let stand = airport.taxi_parkings.get(index).with_context(|| {
            format!(
                "parking index {index} not found at {} ({} stands)",
                airport.ident,
                airport.taxi_parkings.len()
            )
        })?;
        ensure!(stand.heading_deg.is_finite(), "invalid parking heading");
        return Ok((
            stand.latitude,
            stand.longitude,
            f64::from(stand.heading_deg).rem_euclid(360.0),
        ));
    }
    let requested = runway
        .context("runway or parking is required")?
        .trim()
        .to_ascii_uppercase();
    let runway_name = |number: u8, designator: u8| {
        let suffix = match designator {
            0 => "",
            1 => "L",
            2 => "R",
            3 => "C",
            4 => "W",
            5 => "A",
            6 => "B",
            _ => "?",
        };
        format!("{number:02}{suffix}")
    };
    for runway in &airport.runways {
        let heading = if runway_name(runway.primary_number, runway.primary_designator) == requested
        {
            f64::from(runway.heading_deg)
        } else if runway_name(runway.secondary_number, runway.secondary_designator) == requested {
            f64::from(runway.heading_deg) + 180.0
        } else {
            continue;
        };
        ensure!(
            heading.is_finite() && runway.length_m.is_finite() && runway.length_m > 60.0,
            "invalid runway geometry"
        );
        let heading = heading.rem_euclid(360.0);
        let distance = 30.0 - f64::from(runway.length_m) * 0.5;
        let (latitude, longitude) = Origin::new(runway.latitude, runway.longitude)?.position(
            distance * heading.to_radians().cos(),
            distance * heading.to_radians().sin(),
        )?;
        return Ok((latitude, longitude, heading));
    }
    anyhow::bail!("runway {requested} not found at {}", airport.ident)
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;

    #[test]
    fn spawn_selection_requires_airport_and_excludes_manual_placement() {
        let base = [
            "openesp",
            "--aircraft",
            "test",
            "--scenery-bgl",
            "airport.bgl",
            "--airport",
            "TEST",
            "--airspeed",
            "0",
            "--runway",
            "06",
        ];
        assert!(Arguments::try_parse_from(base).is_ok());
        for extra in [
            ["--parking", "0"],
            ["--heading", "90"],
            ["--start-north", "100"],
        ] {
            let mut args = base.to_vec();
            args.extend(extra);
            assert!(Arguments::try_parse_from(args).is_err());
        }
        assert!(
            Arguments::try_parse_from(["openesp", "--aircraft", "test", "--runway", "06"]).is_err()
        );
    }

    #[test]
    #[ignore = "requires LTFO BGL in OPENESP_SCENERY_PLACEMENT_BGL"]
    fn spawns_at_both_runway_ends_and_parking_in_real_airport() {
        let bytes =
            std::fs::read(std::env::var_os("OPENESP_SCENERY_PLACEMENT_BGL").unwrap()).unwrap();
        let airports = Bgl::parse(&bytes).unwrap().airports().unwrap();
        let airport = airports
            .iter()
            .find(|airport| airport.ident == "LTFO")
            .unwrap();
        let runway = &airport.runways[0];
        let primary = format!("{:02}", runway.primary_number);
        let secondary = format!("{:02}", runway.secondary_number);
        let a = location(airport, Some(&primary), None).unwrap();
        let b = location(airport, Some(&secondary), None).unwrap();
        let (north, east) = Origin::new(a.0, a.1)
            .unwrap()
            .local_position(b.0, b.1)
            .unwrap();
        assert!((north.hypot(east) - (f64::from(runway.length_m) - 60.0)).abs() < 2.0);
        assert!(((b.2 - a.2).rem_euclid(360.0) - 180.0).abs() < 1e-6);
        assert!(location(airport, Some("99"), None).is_err());
        let stand = &airport.taxi_parkings[0];
        assert_eq!(
            location(airport, None, Some(0)).unwrap(),
            (
                stand.latitude,
                stand.longitude,
                f64::from(stand.heading_deg).rem_euclid(360.0)
            )
        );
        assert!(location(airport, None, Some(usize::MAX)).is_err());
        eprintln!(
            "LTFO runway {primary}: {a:?}; parking: {:?}",
            location(airport, None, Some(0)).unwrap()
        );
    }
}

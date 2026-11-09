use super::*;
use std::{path::Path, sync::Arc};

#[test]
#[ignore = "requires Prepar3D elevation and original aircraft data"]
fn aircraft_contact_tracks_real_elevated_terrain() {
    let root = std::env::var_os("OPENESP_P3D_ROOT").unwrap();
    let terrain =
        esp_terrain::elevation::Terrain::load(Path::new(&root), 41.17574, 40.84646, 35_000.0)
            .unwrap();
    let reference = terrain.height(41.17574, 40.84646).unwrap();
    let mesh = Arc::new(
        esp_terrain::mesh::Mesh::build(&terrain, 41.17574, 40.84646, 30_000.0, 128, reference)
            .unwrap(),
    );
    let vertex = mesh
        .vertices
        .iter()
        .find(|v| {
            v[1] > 100.0
                && v[0].abs() < 25_000.0
                && v[2].abs() < 25_000.0
                && mesh.height(v[0] + 5.0, v[2] + 5.0).is_some()
        })
        .unwrap();
    let (east, north) = (vertex[0] + 5.0, vertex[2] + 5.0);
    let height = mesh.height(east, north).unwrap();
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
        let mut variables = controls(&data);
        for _ in 0..600 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        flight.current.position.x = north;
        flight.current.position.y = east;
        flight.current.position.z -= height;
        flight.set_ground(crate::flight::ground::Ground::new(Arc::clone(&mesh)));
        for _ in 0..600 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        assert!(
            flight.on_ground,
            "{path:?}: aircraft did not settle on elevated terrain"
        );
        let surface = flight.surface_at(&flight.current).elevation;
        assert!(surface > 50.0 && -flight.current.position.z > surface);
        flight.publish_at(&flight.current, &mut variables).unwrap();
        let read = |variables: &mut Variables, name| {
            variables
                .read(&Reference::new('A', name, Some("meters")))
                .unwrap()
                .number()
                .unwrap()
        };
        let msl = reference - flight.current.position.z;
        assert!((read(&mut variables, "PLANE ALTITUDE") - msl).abs() < 1e-6);
        assert!((read(&mut variables, "GPS POSITION ALT") - msl).abs() < 1e-6);
        flight.configure_controls(&mut variables).unwrap();
        assert!((read(&mut variables, "AUTOPILOT ALTITUDE LOCK VAR") - msl).abs() < 1e-6);
        assert!((read(&mut variables, "GROUND ALTITUDE") - (reference + surface)).abs() < 1e-6);
        assert!(
            (read(&mut variables, "PLANE ALT ABOVE GROUND")
                - (-flight.current.position.z - surface))
                .abs()
                < 1e-6
        );
    }
}

#[test]
fn instruments_and_navigation_use_msl_and_terrain_clearance() {
    let state = State {
        position: DVec3::new(0.0, 0.0, -200.0),
        velocity: DVec3::X * 50.0,
        ..State::default()
    };
    let mut variables = Variables::default();
    variables.set_number("WORLD ORIGIN ALTITUDE", "meters", 1000.0);
    variables.set_number("GROUND ALTITUDE", "meters", 1150.0);
    Flight::publish(&state, &mut variables).unwrap();
    for (name, expected) in [
        ("PLANE ALTITUDE", 1200.0),
        ("GPS POSITION ALT", 1200.0),
        ("PLANE ALT ABOVE GROUND", 50.0),
        ("INDICATED ALTITUDE", 1200.0),
    ] {
        let actual = variables
            .read(&Reference::new('A', name, Some("meters")))
            .unwrap()
            .number()
            .unwrap();
        assert!((actual - expected).abs() < 1e-6, "{name}: {actual}");
    }
}

use super::*;
use std::{path::Path, sync::Arc};

#[test]
#[ignore = "requires an original aircraft in OPENESP_AIRCRAFT_PATHS"]
fn pitot_icing_affects_airspeed_until_powered_heat_clears_it() {
    let path = content::aircraft_paths().into_iter().next().unwrap();
    let data = FlightData::load(&path, 0).unwrap();
    let mut flight = Flight::new(&data, 180.0, 1000.0, 0.5, true).unwrap();
    let mut variables = controls(&data);
    variables
        .configure_electrical(
            &esp_formats::cfg::electrical::Electrical::from_config(&data.configuration).unwrap(),
            false,
        )
        .unwrap();
    variables.update_electrical().unwrap();
    variables.set_number("SEA LEVEL TEMPERATURE", "celsius", -5.0);
    variables.set_number("OPENESP CLOUD BASE:1", "meters", 500.0);
    variables.set_number("OPENESP CLOUD COVER:1", "number", 1.0);
    flight.pitot_ice = 0.49999;
    flight.publish_at(&flight.current, &mut variables).unwrap();
    flight.advance(1.0 / 60.0, &mut variables).unwrap();
    assert!(flight.pitot_ice_blocked);
    flight.publish_at(&flight.current, &mut variables).unwrap();
    let read_speed = |vars: &mut Variables| {
        vars.read(&Reference::new(
            'A',
            "AIRSPEED INDICATED",
            Some("meters per second"),
        ))
        .unwrap()
        .number()
        .unwrap()
    };
    let blocked_speed = read_speed(&mut variables);
    let mut faster = flight.current;
    faster.velocity.x += 30.0;
    flight.publish_at(&faster, &mut variables).unwrap();
    assert!((read_speed(&mut variables) - blocked_speed).abs() < 1e-6);

    variables.set_number("OPENESP CLOUD COVER:1", "number", 0.0);
    variables.set_number("PITOT HEAT", "bool", 1.0);
    flight.pitot_ice = 0.11;
    for _ in 0..60 {
        flight.advance(1.0 / 60.0, &mut variables).unwrap();
    }
    assert!(flight.pitot_ice_blocked);
    assert!((flight.pitot_ice - 0.11).abs() < 1e-9);
    variables
        .dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
        .unwrap();
    variables.update_electrical().unwrap();
    for _ in 0..600 {
        flight.advance(1.0 / 60.0, &mut variables).unwrap();
    }
    assert!(!flight.pitot_ice_blocked);
    flight.publish_at(&faster, &mut variables).unwrap();
    assert!(read_speed(&mut variables) > blocked_speed + 10.0);
}

#[test]
#[ignore = "requires the original B737 in OPENESP_AIRCRAFT_PATHS"]
fn real_aircraft_structural_ice_changes_flight_and_deice_removes_it() {
    let path = content::aircraft_paths().into_iter().next().unwrap();
    let data = FlightData::load(&path, 0).unwrap();
    let mut clean = Flight::new(&data, 180.0, 1000.0, 0.5, true).unwrap();
    let mut iced = Flight::new(&data, 180.0, 1000.0, 0.5, true).unwrap();
    assert!(
        iced.structural_deice_available,
        "test aircraft has no configured structural deice"
    );
    iced.structural_ice = 0.8;
    let mut clean_variables = controls(&data);
    let mut iced_variables = controls(&data);
    for variables in [&mut clean_variables, &mut iced_variables] {
        variables.set_number("SEA LEVEL TEMPERATURE", "celsius", -5.0);
    }
    for _ in 0..60 {
        clean.advance(1.0 / 60.0, &mut clean_variables).unwrap();
        iced.advance(1.0 / 60.0, &mut iced_variables).unwrap();
    }
    assert!((clean.current.velocity - iced.current.velocity).length() > 0.1);
    let published = iced_variables
        .read(&Reference::new('A', "STRUCTURAL ICE PCT", Some("percent")))
        .unwrap()
        .number()
        .unwrap();
    assert!(published > 70.0);
    assert!((iced.structural_ice - 0.8).abs() < 1e-9);
    iced_variables
        .dispatch_key_event(0x10321, Value::Number(0.0))
        .unwrap();
    let before = iced.structural_ice;
    for _ in 0..600 {
        iced.advance(1.0 / 60.0, &mut iced_variables).unwrap();
    }
    assert!(iced.structural_ice < before - 0.01);
    let mut accumulating = Flight::new(&data, 180.0, 1000.0, 0.5, true).unwrap();
    let mut observed = controls(&data);
    observed.set_number("SEA LEVEL TEMPERATURE", "celsius", -5.0);
    observed.set_number("OPENESP CLOUD BASE:1", "meters", 500.0);
    observed.set_number("OPENESP CLOUD COVER:1", "number", 1.0);
    for _ in 0..600 {
        accumulating.advance(1.0 / 60.0, &mut observed).unwrap();
    }
    assert!(
        accumulating.structural_ice > 0.001,
        "ice did not accumulate in cold cloud"
    );
}

#[test]
#[ignore = "requires an original wheeled aircraft in OPENESP_AIRCRAFT_PATHS"]
fn contaminated_surface_reduces_real_aircraft_braking() {
    let path = content::aircraft_paths().into_iter().next().unwrap();
    let data = FlightData::load(&path, 0).unwrap();
    let mut resting = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
    let mut rest_variables = controls(&data);
    for _ in 0..600 {
        resting.advance(1.0 / 60.0, &mut rest_variables).unwrap();
    }
    let mut speeds = Vec::new();
    for snow in [0.0, 1.0] {
        let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
        let mut variables = controls(&data);
        flight.current = resting.current;
        flight.current.velocity = DVec3::X * 15.0;
        flight.surface_condition.snow = snow;
        variables
            .dispatch_event("PARKING_BRAKES", Value::Number(0.0))
            .unwrap();
        for _ in 0..60 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        speeds.push(flight.current.velocity.length());
    }
    assert!(
        speeds[1] > speeds[0] + 0.2,
        "snow should reduce braking: {speeds:?}"
    );
}

#[test]
#[ignore = "requires an original aircraft in OPENESP_AIRCRAFT_PATHS"]
fn real_aircraft_responds_to_reported_gusts() {
    let path = content::aircraft_paths().into_iter().next().unwrap();
    let data = FlightData::load(&path, 0).unwrap();
    let mut steady = Flight::new(&data, 180.0, 1000.0, 0.5, true).unwrap();
    let mut gusty = Flight::new(&data, 180.0, 1000.0, 0.5, true).unwrap();
    let mut steady_variables = controls(&data);
    let mut gust_variables = controls(&data);
    for variables in [&mut steady_variables, &mut gust_variables] {
        variables.set_number("AMBIENT WIND DIRECTION", "degrees", 0.0);
        variables.set_number("AMBIENT WIND VELOCITY", "knots", 20.0);
    }
    gust_variables.set_number("OPENESP WIND GUST", "knots", 35.0);
    for step in 0..240 {
        let time = Value::Number(f64::from(step) / 60.0);
        steady_variables.set('E', "SIMULATION TIME", Some("seconds"), time.clone());
        gust_variables.set('E', "SIMULATION TIME", Some("seconds"), time);
        steady.advance(1.0 / 60.0, &mut steady_variables).unwrap();
        gusty.advance(1.0 / 60.0, &mut gust_variables).unwrap();
    }
    let difference = (steady.current.velocity - gusty.current.velocity).length();
    assert!(
        difference > 0.01 && difference.is_finite(),
        "gust did not change aircraft motion: {difference}"
    );
}

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

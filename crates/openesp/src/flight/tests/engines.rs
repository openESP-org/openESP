use super::*;

#[test]
#[ignore = "requires original fixed-pitch piston aircraft"]
fn piston_aircraft_accelerates_and_lifts_off_from_the_ground() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(0.0)
            || data
                .configuration
                .number("propeller", "propeller_type")
                .unwrap()
                != Some(1.0)
        {
            continue;
        }
        let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
        flight.start_running(&data).unwrap();
        let mut vars = controls(&data);
        flight.configure_engines(&mut vars).unwrap();
        vars.dispatch_event("THROTTLE_FULL", Value::Number(0.0))
            .unwrap();
        let mut lifted = false;
        for _ in 0..3600 {
            if flight.current.velocity.x > 28.0 {
                vars.set_number("ELEVATOR DEFLECTION PCT", "percent", 35.0);
            }
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
            if !flight.on_ground
                && flight.current.position.z < -3.0
                && flight.current.velocity.z < 0.0
            {
                lifted = true;
                break;
            }
        }
        assert!(
            lifted,
            "{}: failed to lift off: {:?}",
            path.display(),
            flight.current
        );
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original fixed-pitch piston aircraft"]
fn cold_piston_start_requires_electrical_power_and_continues_after_release() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(0.0)
            || data
                .configuration
                .number("propeller", "propeller_type")
                .unwrap()
                != Some(1.0)
        {
            continue;
        }
        let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
        flight.prepare_engines(&data).unwrap();
        let mut vars = controls(&data);
        flight.configure_engines(&mut vars).unwrap();
        vars.configure_electrical(
            &esp_formats::cfg::electrical::Electrical::from_config(&data.configuration).unwrap(),
            false,
        )
        .unwrap();
        vars.dispatch_event("PARKING_BRAKES", Value::Number(0.0))
            .unwrap();
        vars.dispatch_event("MAGNETO1_BOTH", Value::Number(0.0))
            .unwrap();
        vars.dispatch_key_event(0x1018c, Value::Number(1.0))
            .unwrap();
        for _ in 0..60 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        let read = |vars: &mut Variables, name, unit| {
            vars.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        assert_eq!(read(&mut vars, "GENERAL ENG RPM:1", "rpm"), 0.0);
        assert_eq!(read(&mut vars, "GENERAL ENG STARTER ACTIVE:1", "bool"), 0.0);
        vars.dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
            .unwrap();
        for _ in 0..480 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        let rpm = read(&mut vars, "GENERAL ENG RPM:1", "rpm");
        assert!(rpm > 400.0, "rpm={rpm}");
        assert_eq!(read(&mut vars, "GENERAL ENG STARTER ACTIVE:1", "bool"), 1.0);
        assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION:1", "bool"), 1.0);
        vars.dispatch_key_event(0x1018c, Value::Number(0.0))
            .unwrap();
        for _ in 0..120 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert_eq!(read(&mut vars, "GENERAL ENG STARTER ACTIVE:1", "bool"), 0.0);
        assert!(read(&mut vars, "GENERAL ENG RPM:1", "rpm") > 400.0);
        assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION:1", "bool"), 1.0);
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original piston aircraft with engine-driven vacuum pump"]
fn piston_vacuum_follows_pump_and_failure() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(0.0)
            || data
                .configuration
                .number("propeller", "propeller_type")
                .unwrap()
                != Some(1.0)
            || data
                .configuration
                .number("vacuum_system", "vacuum_type")
                .unwrap()
                != Some(1.0)
            || data
                .configuration
                .value("vacuum_system", "engine_map")
                .unwrap()
                .is_none()
        {
            continue;
        }
        let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
        flight.prepare_engines(&data).unwrap();
        let mut vars = controls(&data);
        flight.configure_engines(&mut vars).unwrap();
        flight.publish_engines(&mut vars).unwrap();
        let read = |v: &mut Variables| {
            v.read(&Reference::new('A', "SUCTION PRESSURE", Some("inhg")))
                .unwrap()
                .number()
                .unwrap()
        };
        assert_eq!(read(&mut vars), 0.0);
        flight.start_running(&data).unwrap();
        flight.publish_engines(&mut vars).unwrap();
        assert!(read(&mut vars) > 0.0);
        vars.dispatch_event("TOGGLE_VACUUM_FAILURE", Value::Number(0.0))
            .unwrap();
        flight.publish_engines(&mut vars).unwrap();
        assert_eq!(read(&mut vars), 0.0);
        vars.dispatch_event("TOGGLE_VACUUM_FAILURE", Value::Number(0.0))
            .unwrap();
        flight.publish_engines(&mut vars).unwrap();
        assert!(read(&mut vars) > 0.0);
        verified += 1;
    }
    assert!(verified > 0);
}
#[test]
#[ignore = "requires original fixed-pitch piston aircraft and AIR flight data"]
fn piston_aircraft_idle_taxi_and_shutdown_follow_live_engine_state() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(0.0)
            || data
                .configuration
                .number("propeller", "propeller_type")
                .unwrap()
                != Some(1.0)
        {
            continue;
        }
        let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
        flight.start_running(&data).unwrap();
        let mut variables = controls(&data);
        flight.configure_engines(&mut variables).unwrap();
        variables
            .dispatch_event("PARKING_BRAKES", Value::Number(0.0))
            .unwrap();
        for _ in 0..600 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        let parked = flight.current.position;
        for _ in 0..600 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        assert!((flight.current.position - parked).truncate().length() < 0.001);
        let read = |variables: &mut Variables, name, unit| {
            variables
                .read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        let idle = read(&mut variables, "GENERAL ENG RPM:1", "rpm");
        assert!(idle > 300.0 && idle < 1500.0);
        assert_eq!(read(&mut variables, "GENERAL ENG1 RPM", "rpm"), idle);
        assert_eq!(read(&mut variables, "PROP RPM:1", "rpm"), idle);
        let angle = read(&mut variables, "PROP ROTATION ANGLE:1", "radians");
        flight.advance(1.0 / 60.0, &mut variables).unwrap();
        assert_ne!(
            read(&mut variables, "PROP ROTATION ANGLE:1", "radians"),
            angle
        );
        assert!(read(&mut variables, "PROP MAX RPM PERCENT:1", "percent") > 0.0);
        let fuel = flight.loading.fuel_gallons;
        variables
            .dispatch_event("PARKING_BRAKES", Value::Number(0.0))
            .unwrap();
        variables
            .dispatch_event("THROTTLE_FULL", Value::Number(0.0))
            .unwrap();
        for _ in 0..300 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        assert!(flight.current.velocity.x > 2.0);
        assert!(read(&mut variables, "CENTER WHEEL RPM", "rpm") > 0.0);
        assert_eq!(
            read(&mut variables, "CENTER WHEEL ROTATION ANGLE", "radians"),
            read(&mut variables, "WHEEL ROTATION ANGLE:0", "radians")
        );
        variables.set_number("RUDDER DEFLECTION PCT", "percent", 50.0);
        flight.advance(1.0 / 60.0, &mut variables).unwrap();
        assert!(read(&mut variables, "GEAR CENTER STEER ANGLE", "degrees") > 0.0);
        variables.set_number("RUDDER DEFLECTION PCT", "percent", 0.0);
        assert!(flight.loading.fuel_gallons < fuel);
        assert!(read(&mut variables, "GENERAL ENG RPM:1", "rpm") > idle);
        assert!(read(&mut variables, "RECIP ENG BRAKE POWER:1", "horsepower") > 50.0);
        assert!(read(&mut variables, "RECIP ENG MANIFOLD PRESSURE:1", "inhg") > 20.0);
        variables
            .dispatch_key_event(0x100f2, Value::Number(0.0))
            .unwrap();
        flight.advance(1.0 / 60.0, &mut variables).unwrap();
        assert_eq!(
            read(&mut variables, "GENERAL ENG COMBUSTION:1", "bool"),
            0.0
        );
        variables
            .dispatch_event("MIXTURE_RICH", Value::Number(0.0))
            .unwrap();
        flight.advance(1.0 / 60.0, &mut variables).unwrap();
        assert_eq!(
            read(&mut variables, "GENERAL ENG COMBUSTION:1", "bool"),
            1.0
        );
        variables
            .dispatch_key_event(0x10187, Value::Number(0.0))
            .unwrap();
        flight.advance(1.0 / 60.0, &mut variables).unwrap();
        assert_eq!(
            read(&mut variables, "GENERAL ENG COMBUSTION:1", "bool"),
            0.0
        );
        variables
            .dispatch_event("MAGNETO1_BOTH", Value::Number(0.0))
            .unwrap();
        flight.advance(1.0 / 60.0, &mut variables).unwrap();
        assert_eq!(
            read(&mut variables, "GENERAL ENG COMBUSTION:1", "bool"),
            1.0
        );
        variables
            .dispatch_event("ENGINE_AUTO_SHUTDOWN", Value::Number(0.0))
            .unwrap();
        flight.advance(1.0 / 60.0, &mut variables).unwrap();
        assert_eq!(
            read(&mut variables, "GENERAL ENG COMBUSTION:1", "bool"),
            0.0
        );
        assert_eq!(
            read(&mut variables, "ENG FUEL FLOW PPH:1", "pounds per hour"),
            0.0
        );
        assert!(read(&mut variables, "PROP RPM:1", "rpm") > idle);
        let fuel = flight.loading.fuel_gallons;
        for _ in 0..120 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        assert_eq!(flight.loading.fuel_gallons, fuel);
        eprintln!(
            "{}: parked idle {:.2} RPM, taxi velocity {:?}",
            path.display(),
            idle,
            flight.current.velocity
        );
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original piston aircraft and AIR flight data"]
fn fuel_valve_event_allows_restart_after_automatic_shutdown() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(0.0)
        {
            continue;
        }
        let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
        flight.start_running(&data).unwrap();
        let mut vars = controls(&data);
        flight.configure_engines(&mut vars).unwrap();
        vars.configure_electrical(
            &esp_formats::cfg::electrical::Electrical::from_config(&data.configuration).unwrap(),
            false,
        )
        .unwrap();
        vars.dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
            .unwrap();
        vars.dispatch_event("PARKING_BRAKES", Value::Number(0.0))
            .unwrap();
        vars.dispatch_event("ENGINE_AUTO_SHUTDOWN", Value::Number(0.0))
            .unwrap();
        for _ in 0..1800 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        let read = |vars: &mut Variables, name, unit| {
            vars.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION:1", "bool"), 0.0);
        assert!(read(&mut vars, "GENERAL ENG RPM:1", "rpm") < 50.0);
        vars.dispatch_key_event(0x103be, Value::Number(0.0))
            .unwrap();
        vars.dispatch_event("STARTER1_SET", Value::Number(1.0))
            .unwrap();
        for _ in 0..480 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        vars.dispatch_event("STARTER1_SET", Value::Number(0.0))
            .unwrap();
        for _ in 0..120 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert_eq!(read(&mut vars, "GENERAL ENG FUEL VALVE:1", "bool"), 1.0);
        assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION:1", "bool"), 1.0);
        assert!(read(&mut vars, "GENERAL ENG RPM:1", "rpm") > 400.0);
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original piston and turbine aircraft"]
fn configured_master_ignition_stops_all_engines_without_closing_fuel_valves() {
    let mut verified = [false; 2];
    for path in content::aircraft_paths() {
        let mut data = FlightData::load(&path, 0).unwrap();
        let kind = data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            .unwrap() as usize;
        if kind > 1 {
            continue;
        }
        let source = std::fs::read_to_string(path.join("aircraft.cfg")).unwrap();
        let source = source
            .lines()
            .flat_map(|line| {
                if line.trim().eq_ignore_ascii_case("[GeneralEngineData]") {
                    vec![line.to_owned(), "master_ignition_switch=1".to_owned()]
                } else {
                    vec![line.to_owned()]
                }
            })
            .collect::<Vec<_>>()
            .join("\n");
        data.configuration = esp_formats::cfg::Config::parse(&source).unwrap();
        let (speed, height) = if kind == 0 {
            (0.0, 0.0)
        } else {
            (160.0, 5000.0)
        };
        let mut flight = Flight::new(&data, speed, height, 0.5, false).unwrap();
        flight.start_running(&data).unwrap();
        let mut vars = controls(&data);
        flight.configure_engines(&mut vars).unwrap();
        vars.dispatch_key_event(0x103e7, Value::Number(0.0))
            .unwrap();
        flight.advance(1.0 / 60.0, &mut vars).unwrap();
        for index in flight.engines.as_ref().unwrap().indices() {
            let read = |vars: &mut Variables, prefix| {
                vars.read(&Reference::new(
                    'A',
                    &format!("{prefix}:{index}"),
                    Some("bool"),
                ))
                .unwrap()
                .number()
                .unwrap()
            };
            assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION"), 0.0);
            assert_eq!(read(&mut vars, "GENERAL ENG FUEL VALVE"), 1.0);
        }
        verified[kind] = true;
    }
    assert_eq!(verified, [true, true]);
}

#[test]
#[ignore = "requires original turbine aircraft and AIR flight data"]
fn turbine_start_requires_starter_power_and_fuel_then_runs_without_starter() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(1.0)
        {
            continue;
        }
        let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
        flight.prepare_engines(&data).unwrap();
        let mut vars = controls(&data);
        flight.configure_engines(&mut vars).unwrap();
        vars.configure_electrical(
            &esp_formats::cfg::electrical::Electrical::from_config(&data.configuration).unwrap(),
            false,
        )
        .unwrap();
        vars.dispatch_event("PARKING_BRAKES", Value::Number(0.0))
            .unwrap();
        vars.dispatch_event("TOGGLE_FUEL_VALVE_ENG1", Value::Number(0.0))
            .unwrap();
        vars.dispatch_event("STARTER1_SET", Value::Number(1.0))
            .unwrap();
        let read = |vars: &mut Variables, name, unit| {
            vars.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        let cold = read(&mut vars, "TURB ENG N2:1", "percent");
        for _ in 0..120 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert_eq!(read(&mut vars, "TURB ENG N2:1", "percent"), cold);
        vars.dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
            .unwrap();
        for _ in 0..1500 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert!(read(&mut vars, "TURB ENG N2:1", "percent") >= 19.4);
        assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION:1", "bool"), 0.0);
        assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION:2", "bool"), 0.0);
        vars.dispatch_key_event(0x1049d, Value::Number(0.0))
            .unwrap();
        vars.dispatch_event("TOGGLE_FUEL_VALVE_ENG1", Value::Number(0.0))
            .unwrap();
        for _ in 0..60 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert_eq!(read(&mut vars, "TURB ENG IGNITION ON", "bool"), 0.0);
        assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION:1", "bool"), 0.0);
        vars.dispatch_event("TURBINE_IGNITION_SWITCH_AUTO", Value::Number(0.0))
            .unwrap();
        for _ in 0..600 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION:1", "bool"), 1.0);
        assert_eq!(read(&mut vars, "TURB ENG IGNITION ON", "bool"), 1.0);
        vars.dispatch_event("STARTER1_SET", Value::Number(0.0))
            .unwrap();
        for _ in 0..600 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION:1", "bool"), 1.0);
        assert_eq!(read(&mut vars, "TURB ENG IGNITION ON", "bool"), 0.0);
        assert!(read(&mut vars, "TURB ENG N2:1", "percent") > 25.0);
        assert_eq!(read(&mut vars, "GENERAL ENG STARTER ACTIVE:1", "bool"), 0.0);
        vars.dispatch_event("TOGGLE_FUEL_VALVE_ENG1", Value::Number(0.0))
            .unwrap();
        flight.advance(1.0 / 60.0, &mut vars).unwrap();
        assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION:1", "bool"), 0.0);
        vars.dispatch_event("TOGGLE_FUEL_VALVE_ENG1", Value::Number(0.0))
            .unwrap();
        flight.advance(1.0 / 60.0, &mut vars).unwrap();
        assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION:1", "bool"), 0.0);
        vars.dispatch_key_event(0x10795, Value::Number(0.0))
            .unwrap();
        flight.advance(1.0 / 60.0, &mut vars).unwrap();
        assert_eq!(read(&mut vars, "TURB ENG IGNITION ON", "bool"), 1.0);
        assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION:1", "bool"), 1.0);
        vars.dispatch_event("TURBINE_IGNITION_SWITCH_OFF", Value::Number(0.0))
            .unwrap();
        flight.advance(1.0 / 60.0, &mut vars).unwrap();
        assert_eq!(read(&mut vars, "TURB ENG IGNITION ON", "bool"), 0.0);
        assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION:1", "bool"), 1.0);
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original wheeled aircraft and AIR flight data"]
fn parked_turbines_hold_the_airframe_stationary_against_idle_thrust() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(1.0)
        {
            continue;
        }
        let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
        flight.start_running(&data).unwrap();
        let mut variables = controls(&data);
        flight.configure_engines(&mut variables).unwrap();
        variables
            .dispatch_event("PARKING_BRAKES", Value::Number(0.0))
            .unwrap();
        for _ in 0..600 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        let origin = flight.current.position - flight.current.orientation * flight.loading.cg;
        for _ in 0..600 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        let after = flight.current.position - flight.current.orientation * flight.loading.cg;
        let drift = (after - origin).truncate().length();
        assert!(
            drift < 0.001,
            "{path:?}: parked airframe crept {drift} m under idle thrust"
        );
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original wheeled aircraft and AIR flight data"]
fn ground_start_braking_and_landing_share_the_flight_model() {
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let mut variables = controls(&data);
        let mut ground = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
        for _ in 0..600 {
            ground.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        assert!(ground.on_ground, "{path:?}");
        assert!(
            ground.current.velocity.length() < 0.03,
            "{path:?}: {:?}",
            ground.current
        );
        assert_eq!(
            variables
                .read(&Reference::new('A', "SIM ON GROUND", Some("bool")))
                .unwrap(),
            Value::Number(1.0)
        );
        ground.current.velocity = DVec3::X * 15.0;
        variables
            .dispatch_event("PARKING_BRAKES", Value::Number(0.0))
            .unwrap();
        for _ in 0..600 {
            ground.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        assert!(
            ground.current.velocity.length() < 0.03,
            "{path:?}: brakes did not stop flight state: {:?}",
            ground.current
        );
        let mut landing =
            Flight::new(&data, 1.0, -ground.current.position.z + 0.5, 0.5, true).unwrap();
        landing.current.velocity = DVec3::ZERO;
        landing.current.orientation = ground.current.orientation;
        for _ in 0..1200 {
            landing.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        assert!(
            landing.on_ground,
            "{path:?}: landing did not produce weight on wheels"
        );
        assert!(
            landing.current.position.z < 0.0 && landing.current.velocity.length() < 0.03,
            "{path:?}: landing did not settle: {:?}",
            landing.current
        );
    }
}

#[test]
#[ignore = "requires original wheeled aircraft and AIR flight data"]
fn brake_axis_events_turn_the_aircraft_toward_the_braked_side() {
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let mut variables = controls(&data);
        let mut resting = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
        for _ in 0..600 {
            resting.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        for (event, sign) in [("AXIS_LEFT_BRAKE_SET", -1.0), ("AXIS_RIGHT_BRAKE_SET", 1.0)] {
            let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
            flight.current = resting.current;
            flight.current.velocity = DVec3::X * 15.0;
            let mut variables = controls(&data);
            variables
                .dispatch_event(event, Value::Number(16383.0))
                .unwrap();
            for _ in 0..120 {
                flight.advance(1.0 / 60.0, &mut variables).unwrap();
            }
            let yaw = flight.current.orientation.to_euler(EulerRot::ZYX).0;
            assert!(yaw * sign > 0.0001, "{path:?}: {event} produced yaw {yaw}");
        }
    }
}

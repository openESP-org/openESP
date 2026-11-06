use super::*;

#[test]
#[ignore = "requires original aircraft with spoilerons"]
fn spoilerons_follow_aileron_direction_and_add_roll_moment() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("airplane_geometry", "spoilerons_available")
            .unwrap()
            .unwrap_or(0.0)
            == 0.0
            || data
                .configuration
                .number("airplane_geometry", "min_flaps_for_spoilerons")
                .unwrap()
                .unwrap_or(0.0)
                > 0.0
        {
            continue;
        }
        let mut flight = Flight::new(&data, 200.0, 5000.0, 0.5, true).unwrap();
        let mut gated_data = FlightData::load(&path, 0).unwrap();
        let source = std::fs::read_to_string(path.join("aircraft.cfg")).unwrap();
        let mut changed = false;
        let source = source
            .lines()
            .map(|line| {
                if line
                    .split('=')
                    .next()
                    .unwrap_or("")
                    .trim()
                    .eq_ignore_ascii_case("min_flaps_for_spoilerons")
                {
                    changed = true;
                    "min_flaps_for_spoilerons=0.5".to_owned()
                } else {
                    line.to_owned()
                }
            })
            .collect::<Vec<_>>()
            .join("\n");
        assert!(changed);
        gated_data.configuration = esp_formats::cfg::Config::parse(&source).unwrap();
        let gated = Aerodynamics::from_data(&gated_data).unwrap();
        assert_eq!(gated.spoiler_targets(0.0, 1.0, 0.0), [0.0; 2]);
        assert!(gated.spoiler_targets(0.0, 1.0, 0.5)[1] > 0.0);
        let mut vars = controls(&data);
        vars.set_number("AILERON LEFT DEFLECTION PCT", "percent", 25.0);
        for _ in 0..60 {
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert_eq!(flight.spoiler_surfaces, [0.0, 0.0]);
        vars.set_number("AILERON LEFT DEFLECTION PCT", "percent", 100.0);
        for _ in 0..60 {
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert!(flight.spoiler_surfaces[1] > 0.0 && flight.spoiler_surfaces[0] == 0.0);
        let reference = |name| Reference::new('A', name, Some("percent over 100"));
        let left = vars
            .read(&reference("SPOILERS LEFT POSITION"))
            .unwrap()
            .number()
            .unwrap();
        let right = vars
            .read(&reference("SPOILERS RIGHT POSITION"))
            .unwrap()
            .number()
            .unwrap();
        assert_eq!([left, right], flight.spoiler_surfaces);
        let mut base = Controls {
            aileron: 1.0,
            spoilers: (left + right) * 0.5,
            ..Controls::default()
        };
        let flow = DVec3::X * 80.0;
        let clean = flight
            .model
            .forces(
                flow,
                DVec3::ZERO,
                1.0,
                base,
                &flight.flaps,
                flight.loading.cg,
            )
            .unwrap();
        base.spoiler_difference = left - right;
        let asymmetric = flight
            .model
            .forces(
                flow,
                DVec3::ZERO,
                1.0,
                base,
                &flight.flaps,
                flight.loading.cg,
            )
            .unwrap();
        assert!(asymmetric.moment.x > clean.moment.x);
        vars.set_number("AILERON LEFT DEFLECTION PCT", "percent", -100.0);
        for _ in 0..180 {
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert!(flight.spoiler_surfaces[0] > 0.0 && flight.spoiler_surfaces[1] < 1e-9);
        vars.dispatch_event("SPOILERS_ON", Value::Number(0.0))
            .unwrap();
        for _ in 0..180 {
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert!(flight.spoiler_surfaces.iter().all(|v| *v > 0.99));
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires an original aircraft with automatic spoilers"]
fn automatic_spoilers_deploy_on_touchdown_and_retract_for_go_around() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("airplane_geometry", "auto_spoiler_available")
            .unwrap()
            .unwrap_or(0.0)
            == 0.0
            || data
                .configuration
                .number("GeneralEngineData", "engine_type")
                .unwrap()
                != Some(1.0)
        {
            continue;
        }
        let mut ground = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
        let mut ground_vars = controls(&data);
        for _ in 0..600 {
            ground.advance(1.0 / 60.0, &mut ground_vars).unwrap();
        }
        for mode in 0..3 {
            let armed = mode != 0;
            let mut flight =
                Flight::new(&data, 1.0, -ground.current.position.z + 0.5, 0.5, true).unwrap();
            flight.current.velocity = DVec3::ZERO;
            flight.current.orientation = ground.current.orientation;
            flight.start_running(&data).unwrap();
            let mut vars = controls(&data);
            flight.configure_engines(&mut vars).unwrap();
            if armed {
                vars.dispatch_key_event(0x1013d, Value::Number(0.0))
                    .unwrap();
            }
            for _ in 0..900 {
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert!(flight.on_ground);
            if armed {
                assert!(
                    flight
                        .spoiler_surfaces
                        .iter()
                        .all(|position| *position > 0.9)
                );
                assert_eq!(
                    vars.read(&Reference::new(
                        'A',
                        "SPOILERS HANDLE POSITION",
                        Some("percent over 100")
                    ))
                    .unwrap(),
                    Value::Number(1.0)
                );
                if mode == 2 {
                    vars.dispatch_event("SPOILERS_SET", Value::Number(8191.5))
                        .unwrap();
                    flight.advance(1.0 / 60.0, &mut vars).unwrap();
                    assert!(!flight.spoiler_auto_deployed);
                    assert_eq!(
                        vars.read(&Reference::new(
                            'A',
                            "SPOILERS HANDLE POSITION",
                            Some("percent over 100")
                        ))
                        .unwrap(),
                        Value::Number(0.5)
                    );
                    continue;
                }
                vars.dispatch_event("THROTTLE_FULL", Value::Number(0.0))
                    .unwrap();
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
                assert!(!flight.spoiler_auto_deployed);
                assert_eq!(
                    vars.read(&Reference::new(
                        'A',
                        "SPOILERS HANDLE POSITION",
                        Some("percent over 100")
                    ))
                    .unwrap(),
                    Value::Number(0.0)
                );
                for _ in 0..180 {
                    flight.advance(1.0 / 60.0, &mut vars).unwrap();
                }
                assert!(
                    flight
                        .spoiler_surfaces
                        .iter()
                        .all(|position| *position < 1e-9)
                );
            } else {
                assert_eq!(flight.spoiler_surfaces, [0.0; 2]);
            }
        }
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original electric and hydraulic flap aircraft"]
fn configured_flaps_stop_without_power_and_resume() {
    let mut verified = [false; 2];
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let sets = esp_formats::cfg::aircraft::flaps(&data.configuration).unwrap();
        let Some(kind @ (0 | 1)) = sets.first().and_then(|set| set.system_type) else {
            continue;
        };
        let mut flight = Flight::new(&data, 80.0, 5000.0, 0.5, true).unwrap();
        let mut vars = controls(&data);
        vars.configure_electrical(
            &esp_formats::cfg::electrical::Electrical::from_config(&data.configuration).unwrap(),
            false,
        )
        .unwrap();
        if kind == 1 {
            flight.start_running(&data).unwrap();
            flight.configure_engines(&mut vars).unwrap();
            vars.dispatch_key_event(0x10305, Value::Number(0.0))
                .unwrap();
        }
        vars.dispatch_event("FLAPS_DOWN", Value::Number(0.0))
            .unwrap();
        for _ in 0..60 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        let position = || {
            Reference::new(
                'A',
                "TRAILING EDGE FLAPS LEFT PERCENT",
                Some("percent over 100"),
            )
        };
        assert_eq!(vars.read(&position()).unwrap().number().unwrap(), 0.0);
        if kind == 0 {
            vars.dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
                .unwrap();
        } else {
            vars.dispatch_key_event(0x10305, Value::Number(0.0))
                .unwrap();
        }
        for _ in 0..60 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert!(vars.read(&position()).unwrap().number().unwrap() > 0.0);
        verified[kind as usize] = true;
    }
    assert_eq!(verified, [true, true]);
}

#[test]
#[ignore = "requires original hydraulic aircraft"]
fn configured_spoilers_stop_without_power_and_resume() {
    let mut verified = false;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if esp_formats::cfg::contact::ContactPoints::from_config(&data.configuration)
            .unwrap()
            .gear_system_type
            != Some(1)
        {
            continue;
        }
        for kind in [0, 1] {
            let mut flight = Flight::new(&data, 160.0, 5000.0, 0.5, true).unwrap();
            flight.spoiler_system_type = Some(kind);
            let mut vars = controls(&data);
            vars.configure_electrical(
                &esp_formats::cfg::electrical::Electrical::from_config(&data.configuration)
                    .unwrap(),
                false,
            )
            .unwrap();
            if kind == 1 {
                flight.start_running(&data).unwrap();
                flight.configure_engines(&mut vars).unwrap();
                vars.dispatch_key_event(0x10305, Value::Number(0.0))
                    .unwrap();
            }
            vars.dispatch_event("SPOILERS_ON", Value::Number(0.0))
                .unwrap();
            for _ in 0..60 {
                vars.update_electrical().unwrap();
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert_eq!(flight.spoiler_surfaces, [0.0; 2]);
            if kind == 0 {
                vars.dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
                    .unwrap();
            } else {
                vars.dispatch_key_event(0x10305, Value::Number(0.0))
                    .unwrap();
            }
            for _ in 0..60 {
                vars.update_electrical().unwrap();
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert!(flight.spoiler_surfaces.iter().all(|surface| *surface > 0.0));
        }
        verified = true;
    }
    assert!(verified);
}

#[test]
#[ignore = "requires original hydraulic aircraft"]
fn hydraulic_boost_limits_surfaces_without_losing_pilot_input() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let mut data = FlightData::load(&path, 0).unwrap();
        if esp_formats::cfg::contact::ContactPoints::from_config(&data.configuration)
            .unwrap()
            .gear_system_type
            != Some(1)
        {
            continue;
        }
        let source = std::fs::read_to_string(path.join("aircraft.cfg")).unwrap();
        let source = source
            .lines()
            .flat_map(|line| {
                if line.trim().eq_ignore_ascii_case("[airplane_geometry]") {
                    vec![
                        line.to_owned(),
                        "elevator_hydraulic_boost_percent=0.8".to_owned(),
                        "aileron_hydraulic_boost_percent=0.5".to_owned(),
                        "rudder_hydraulic_boost_percent=1.0".to_owned(),
                    ]
                } else {
                    vec![line.to_owned()]
                }
            })
            .collect::<Vec<_>>()
            .join("\n");
        data.configuration = esp_formats::cfg::Config::parse(&source).unwrap();
        let mut flight = Flight::new(&data, 160.0, 5000.0, 0.5, false).unwrap();
        flight.start_running(&data).unwrap();
        let mut vars = controls(&data);
        flight.configure_engines(&mut vars).unwrap();
        for (name, value) in [
            ("ELEVATOR DEFLECTION PCT", 50.0),
            ("AILERON LEFT DEFLECTION PCT", 40.0),
            ("AILERON RIGHT DEFLECTION PCT", -40.0),
            ("RUDDER DEFLECTION PCT", 30.0),
        ] {
            vars.set_number(name, "percent", value);
        }
        let mut powered = Flight::new(&data, 160.0, 5000.0, 0.5, false).unwrap();
        powered.start_running(&data).unwrap();
        let mut powered_vars = controls(&data);
        powered.configure_engines(&mut powered_vars).unwrap();
        for (name, value) in [
            ("ELEVATOR DEFLECTION PCT", 50.0),
            ("AILERON LEFT DEFLECTION PCT", 40.0),
            ("AILERON RIGHT DEFLECTION PCT", -40.0),
            ("RUDDER DEFLECTION PCT", 30.0),
        ] {
            powered_vars.set_number(name, "percent", value);
        }
        vars.dispatch_key_event(0x10305, Value::Number(0.0))
            .unwrap();
        powered.advance(1.0 / 60.0, &mut powered_vars).unwrap();
        flight.advance(1.0 / 60.0, &mut vars).unwrap();
        assert!(
            (powered.current.angular_velocity - flight.current.angular_velocity).length() > 1e-5
        );
        for _ in 0..60 {
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        let read = |vars: &mut Variables, name| {
            vars.read(&Reference::new('A', name, Some("percent")))
                .unwrap()
                .number()
                .unwrap()
        };
        for (name, expected) in [
            ("ELEVATOR DEFLECTION PCT", 10.0),
            ("AILERON LEFT DEFLECTION PCT", 20.0),
            ("AILERON RIGHT DEFLECTION PCT", -20.0),
            ("RUDDER DEFLECTION PCT", 0.0),
        ] {
            assert!((read(&mut vars, name) - expected).abs() < 1e-9, "{name}");
        }
        assert_eq!(vars.control_input("ELEVATOR DEFLECTION PCT").unwrap(), 0.5);
        vars.dispatch_key_event(0x10305, Value::Number(0.0))
            .unwrap();
        flight.advance(1.0 / 60.0, &mut vars).unwrap();
        for (name, expected) in [
            ("ELEVATOR DEFLECTION PCT", 50.0),
            ("AILERON LEFT DEFLECTION PCT", 40.0),
            ("AILERON RIGHT DEFLECTION PCT", -40.0),
            ("RUDDER DEFLECTION PCT", 30.0),
        ] {
            assert!((read(&mut vars, name) - expected).abs() < 1e-9, "{name}");
        }
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original hydraulic aircraft"]
fn hydraulic_failure_stops_gear_until_pressure_is_restored() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if esp_formats::cfg::contact::ContactPoints::from_config(&data.configuration)
            .unwrap()
            .gear_system_type
            != Some(1)
            || data
                .configuration
                .number("GeneralEngineData", "engine_type")
                .unwrap()
                != Some(1.0)
        {
            continue;
        }
        let mut flight = Flight::new(&data, 160.0, 5000.0, 0.5, true).unwrap();
        flight.start_running(&data).unwrap();
        let mut vars = controls(&data);
        flight.configure_engines(&mut vars).unwrap();
        vars.dispatch_event("GEAR_UP", Value::Number(0.0)).unwrap();
        vars.dispatch_key_event(0x10305, Value::Number(0.0))
            .unwrap();
        for _ in 0..60 {
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert_eq!(flight.gear.extension(), 1.0);
        assert_eq!(
            vars.read(&Reference::new('A', "HYDRAULIC PRESSURE:1", Some("psi")))
                .unwrap(),
            Value::Number(0.0)
        );
        vars.dispatch_key_event(0x10305, Value::Number(0.0))
            .unwrap();
        for _ in 0..60 {
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert!(flight.gear.extension() < 1.0);
        assert!(
            vars.read(&Reference::new('A', "HYDRAULIC PRESSURE:1", Some("psi")))
                .unwrap()
                .number()
                .unwrap()
                > 0.0
        );
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original retractable aircraft"]
fn electric_gear_stops_without_power_and_resumes_when_power_returns() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let mut data = FlightData::load(&path, 0).unwrap();
        if esp_formats::cfg::contact::ContactPoints::from_config(&data.configuration)
            .unwrap()
            .gear_system_type
            != Some(1)
        {
            continue;
        }
        let source = std::fs::read_to_string(path.join("aircraft.cfg")).unwrap();
        let mut changed = false;
        let source = source
            .lines()
            .map(|line| {
                if line
                    .split('=')
                    .next()
                    .unwrap_or("")
                    .trim()
                    .eq_ignore_ascii_case("gear_system_type")
                {
                    changed = true;
                    "gear_system_type=0".to_owned()
                } else {
                    line.to_owned()
                }
            })
            .collect::<Vec<_>>()
            .join("\n");
        assert!(changed);
        data.configuration = esp_formats::cfg::Config::parse(&source).unwrap();
        let mut flight = Flight::new(&data, 160.0, 5000.0, 0.5, true).unwrap();
        let mut vars = controls(&data);
        vars.configure_electrical(
            &esp_formats::cfg::electrical::Electrical::from_config(&data.configuration).unwrap(),
            false,
        )
        .unwrap();
        vars.dispatch_event("GEAR_UP", Value::Number(0.0)).unwrap();
        for _ in 0..60 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert_eq!(flight.gear.extension(), 1.0);
        vars.dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
            .unwrap();
        for _ in 0..60 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        let partial = flight.gear.extension();
        assert!(partial > 0.0 && partial < 1.0);
        vars.dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
            .unwrap();
        for _ in 0..60 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert_eq!(flight.gear.extension(), partial);
        vars.dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
            .unwrap();
        for _ in 0..480 {
            vars.update_electrical().unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert_eq!(flight.gear.extension(), 0.0);
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original fixed and retractable aircraft"]
fn landing_gear_retracts_and_extends_at_authored_rates() {
    let mut retractable = 0;
    let mut fixed = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let mut flight = Flight::new(&data, 160.0, 5000.0, 0.5, true).unwrap();
        let mut vars = controls(&data);
        vars.set_number("HYDRAULIC SWITCH", "bool", 1.0);
        vars.set_number("CIRCUIT HYDRAULIC PUMP ON", "bool", 1.0);
        vars.dispatch_key_event(0x10022, Value::Number(0.0))
            .unwrap();
        for _ in 0..120 {
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        if flight.gear.retractable() {
            let positions = flight.gear.extensions().collect::<Vec<_>>();
            assert!(
                positions
                    .iter()
                    .all(|(_, position)| *position > 0.0 && *position < 1.0)
            );
            let contacts =
                esp_formats::cfg::contact::ContactPoints::from_config(&data.configuration).unwrap();
            for (index, position) in positions {
                let wheel = contacts
                    .points
                    .iter()
                    .find(|point| point.index == index)
                    .unwrap();
                assert!((position - (1.0 - 2.0 / wheel.retraction_seconds)).abs() < 1e-8);
            }
            for _ in 0..480 {
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert!(flight.gear.extension() < 1e-9);
            vars.dispatch_event("GEAR_DOWN", Value::Number(0.0))
                .unwrap();
            for _ in 0..540 {
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert!((flight.gear.extension() - 1.0).abs() < 1e-9);
            retractable += 1;
        } else {
            assert_eq!(flight.gear.extension(), 1.0);
            fixed += 1;
        }
    }
    assert!(retractable > 0 && fixed > 0);
}

#[test]
#[ignore = "requires original aircraft with wheel contacts"]
fn gear_animation_tracks_actual_suspension_compression() {
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let airborne = Flight::new(&data, 100.0, 1000.0, 0.5, true).unwrap();
        let mut ground = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
        let mut vars = controls(&data);
        airborne.publish_loading(&mut vars);
        let position = |vars: &mut Variables, index| {
            vars.read(&Reference::new(
                'A',
                &format!("GEAR ANIMATION POSITION:{index}"),
                Some("keyframe"),
            ))
            .unwrap()
            .number()
            .unwrap()
        };
        for index in 0..3 {
            assert_eq!(position(&mut vars, index), 100.0);
        }
        for _ in 0..600 {
            ground.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        for index in 0..3 {
            let value = position(&mut vars, index);
            assert!(
                value > 100.0 && value <= 200.0,
                "{} gear {index} frame {value}",
                path.display()
            );
        }
        assert_eq!(
            vars.read(&Reference::new(
                'A',
                "GEAR CENTER POSITION",
                Some("percent over 100")
            ))
            .unwrap(),
            Value::Number(1.0)
        );
    }
}

#[test]
#[ignore = "requires original aircraft with configured spoilers"]
fn spoiler_handle_moves_surfaces_and_changes_aircraft_forces() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let limit = data
            .configuration
            .number("airplane_geometry", "spoiler_limit")
            .unwrap()
            .unwrap_or(0.0);
        let time = data
            .configuration
            .number("airplane_geometry", "spoiler_extension_time")
            .unwrap()
            .unwrap_or(0.0);
        if limit <= 0.0 || time <= 0.5 {
            continue;
        }
        let mut clean = Flight::new(&data, 200.0, 5000.0, 0.5, true).unwrap();
        let mut dirty = Flight::new(&data, 200.0, 5000.0, 0.5, true).unwrap();
        let mut clean_vars = controls(&data);
        let mut dirty_vars = controls(&data);
        dirty_vars
            .dispatch_key_event(0x10210, Value::Number(0.0))
            .unwrap();
        for _ in 0..30 {
            clean.advance(1.0 / 60.0, &mut clean_vars).unwrap();
            dirty.advance(1.0 / 60.0, &mut dirty_vars).unwrap();
        }
        let left = dirty_vars
            .read(&Reference::new(
                'A',
                "SPOILERS LEFT POSITION",
                Some("percent over 100"),
            ))
            .unwrap()
            .number()
            .unwrap();
        assert!((left - 0.5 / time).abs() < 1e-9);
        assert!(dirty.current.velocity.x < clean.current.velocity.x);
        dirty_vars
            .dispatch_key_event(0x10211, Value::Number(0.0))
            .unwrap();
        for _ in 0..30 {
            dirty.advance(1.0 / 60.0, &mut dirty_vars).unwrap();
        }
        assert!(
            dirty
                .spoiler_surfaces
                .iter()
                .all(|position| *position < 1e-9)
        );
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original aircraft and AIR data"]
fn elevator_trim_events_change_flight_pitch() {
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let mut neutral = Flight::new(&data, 120.0, 2000.0, 0.5, true).unwrap();
        let mut trimmed = Flight::new(&data, 120.0, 2000.0, 0.5, true).unwrap();
        let mut base_vars = controls(&data);
        let mut trim_vars = controls(&data);
        trim_vars
            .dispatch_key_event(0x100e6, Value::Number(8191.5))
            .unwrap();
        for _ in 0..60 {
            neutral.advance(1.0 / 60.0, &mut base_vars).unwrap();
            trimmed.advance(1.0 / 60.0, &mut trim_vars).unwrap();
        }
        assert!(
            trimmed.current.angular_velocity.y > neutral.current.angular_velocity.y,
            "{}",
            path.display()
        );
    }
}

#[test]
#[ignore = "requires original aircraft with lateral trim records"]
fn lateral_trim_events_change_aircraft_moments_using_air_data() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let records = esp_formats::air::parse(&data.air).unwrap();
        let Some(record) = records.iter().find(|record| record.id == 0x516) else {
            continue;
        };
        let roll = f64::from_le_bytes(record.data[0..8].try_into().unwrap());
        let yaw = f64::from_le_bytes(record.data[8..16].try_into().unwrap());
        let mut neutral = Flight::new(&data, 120.0, 2000.0, 0.5, true).unwrap();
        let mut trimmed = Flight::new(&data, 120.0, 2000.0, 0.5, true).unwrap();
        let mut base_vars = controls(&data);
        let mut trim_vars = controls(&data);
        trim_vars
            .dispatch_key_event(0x104ab, Value::Number(9010.65))
            .unwrap();
        trim_vars
            .dispatch_key_event(0x104ac, Value::Number(9010.65))
            .unwrap();
        neutral.advance(1.0 / 60.0, &mut base_vars).unwrap();
        trimmed.advance(1.0 / 60.0, &mut trim_vars).unwrap();
        let delta = trimmed.current.angular_velocity - neutral.current.angular_velocity;
        if roll != 0.0 {
            assert!(delta.x * roll < 0.0, "{}: {delta:?}", path.display());
        }
        if yaw != 0.0 {
            assert!(delta.z * yaw > 0.0, "{}: {delta:?}", path.display());
        }
        assert!(
            (trim_vars
                .read(&Reference::new('A', "RUDDER TRIM PCT", Some("percent")))
                .unwrap()
                .number()
                .unwrap()
                - 10.0)
                .abs()
                < 1e-9
        );
        assert!(
            (trim_vars
                .read(&Reference::new('A', "AILERON TRIM", Some("degrees")))
                .unwrap()
                .number()
                .unwrap()
                - 1.0)
                .abs()
                < 1e-9
        );
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original aircraft ground-effect AIR data"]
fn ground_effect_changes_near_surface_flight_response() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let records = esp_formats::air::parse(&data.air).unwrap();
        let Some(record) = records.iter().find(|r| r.id == 0x400) else {
            continue;
        };
        let range = record.offset..record.offset + 8 + record.data.len();
        let mut without = FlightData::load(&path, 0).unwrap();
        without.air.drain(range);
        let mut near = Flight::new(&data, 100.0, 5.0, 0.5, false).unwrap();
        let mut baseline = Flight::new(&without, 100.0, 5.0, 0.5, false).unwrap();
        let mut near_vars = controls(&data);
        let mut base_vars = controls(&without);
        near.advance(1.0 / 60.0, &mut near_vars).unwrap();
        baseline.advance(1.0 / 60.0, &mut base_vars).unwrap();
        assert!(
            near.current.velocity.z < baseline.current.velocity.z,
            "{}",
            path.display()
        );
        verified += 1;
    }
    assert!(verified > 0);
}

use super::*;

#[test]
#[ignore = "requires original aircraft and AIR data"]
fn autopilot_holds_heading_altitude_and_vertical_speed() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let kind = data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            .unwrap();
        if kind > 1.0 {
            continue;
        }
        let speed = if kind == 0.0 { 100.0 } else { 220.0 };
        let mut flight = Flight::new(&data, speed, 2000.0, 0.5, true).unwrap();
        flight.start_running(&data).unwrap();
        let mut variables = controls(&data);
        flight.configure_engines(&mut variables).unwrap();
        variables
            .dispatch_event(
                "THROTTLE_SET",
                Value::Number(if kind == 0.0 { 16383.0 } else { 8191.5 }),
            )
            .unwrap();

        variables
            .dispatch_event("AP_MASTER", Value::Number(0.0))
            .unwrap();
        variables
            .dispatch_event("AP_WING_LEVELER", Value::Number(0.0))
            .unwrap();
        variables
            .dispatch_event("AP_PANEL_ALTITUDE_HOLD", Value::Number(0.0))
            .unwrap();
        for _ in 0..1800 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        let (_, _, bank) = flight.current.orientation.to_euler(EulerRot::ZYX);
        assert!(
            bank.to_degrees().abs() < 3.0,
            "{}: wing leveler bank {}",
            path.display(),
            bank.to_degrees()
        );
        assert!(
            (-flight.current.position.z - 2000.0).abs() < 25.0,
            "{}: altitude hold altitude {}",
            path.display(),
            -flight.current.position.z
        );

        let (heading, _, _) = flight.current.orientation.to_euler(EulerRot::ZYX);
        let selected = (heading.to_degrees() + 40.0).rem_euclid(360.0);
        variables.set_number("AUTOPILOT HEADING LOCK DIR", "degrees", selected);
        variables
            .dispatch_event("AP_PANEL_HEADING_HOLD", Value::Number(0.0))
            .unwrap();
        for _ in 0..3600 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        let (heading, _, bank) = flight.current.orientation.to_euler(EulerRot::ZYX);
        let error = (selected - heading.to_degrees() + 540.0).rem_euclid(360.0) - 180.0;
        assert!(
            error.abs() < 5.0,
            "{}: heading error {error}, bank {}",
            path.display(),
            bank.to_degrees()
        );
        assert!(bank.to_degrees().abs() <= 26.0);

        let altitude = -flight.current.position.z;
        variables
            .dispatch_event("AP_ALT_VAR_SET_METRIC", Value::Number(altitude + 100.0))
            .unwrap();
        for _ in 0..7200 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        let altitude_now = -flight.current.position.z;
        assert!(
            (altitude_now - (altitude + 100.0)).abs() < 15.0,
            "{}: altitude {} vs {}",
            path.display(),
            altitude_now,
            altitude + 100.0
        );

        variables
            .dispatch_event("AP_VS_VAR_SET_METRIC", Value::Number(3.0))
            .unwrap();
        variables
            .dispatch_event("AP_PANEL_VS_HOLD", Value::Number(0.0))
            .unwrap();
        for _ in 0..1800 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        let vertical_speed = -flight.current.velocity.z;
        assert!(
            (vertical_speed - 3.0).abs() < 1.0,
            "{}: vertical speed {vertical_speed}",
            path.display()
        );
        eprintln!(
            "{}: autopilot heading, altitude and vertical speed verified",
            path.display()
        );
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original jet AIR records and OPENESP_AIRCRAFT_PATHS"]
fn powered_flight_accelerates_burns_fuel_and_publishes_engine_instruments() {
    let mut count = 0;
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
        let mut powered = Flight::new(&data, 200.0, 10000.0, 0.5, true).unwrap();
        let mut glide = Flight::new(&data, 200.0, 10000.0, 0.5, true).unwrap();
        powered.start_running(&data).unwrap();
        let mut variables = controls(&data);
        powered.configure_engines(&mut variables).unwrap();
        variables
            .dispatch_event("THROTTLE_FULL", esp_script::Value::Number(0.0))
            .unwrap();
        let initial_fuel = powered.loading.fuel_gallons;
        let initial_mass = powered.loading.mass.kilograms();
        let mut integrated_fuel = 0.0;
        for _ in 0..600 {
            powered.advance(1.0 / 60.0, &mut variables).unwrap();
            glide.advance(1.0 / 60.0, &mut variables).unwrap();
            for index in [1, 2] {
                integrated_fuel += variables
                    .read(&Reference::new(
                        'A',
                        &format!("TURB ENG FUEL FLOW PPH:{index}"),
                        Some("kilograms per second"),
                    ))
                    .unwrap()
                    .number()
                    .unwrap()
                    / 60.0;
            }
        }
        assert!(powered.current.position.x > glide.current.position.x + 5.0);
        let consumed = (initial_fuel - powered.loading.fuel_gallons)
            * powered.loading.fuel_pounds_per_gallon
            * 0.45359237;
        assert!(consumed > 0.0);
        assert!((consumed - (initial_mass - powered.loading.mass.kilograms())).abs() < 1e-8);
        assert!((consumed - integrated_fuel).abs() < 0.1);
        let n1 = variables
            .read(&Reference::new('A', "TURB ENG N1:1", Some("percent")))
            .unwrap()
            .number()
            .unwrap();
        assert!(n1 > 80.0);
        eprintln!(
            "{}: powered x={}m, glide x={}m, fuel used={}kg, N1={}%",
            path.display(),
            powered.current.position.x,
            glide.current.position.x,
            consumed,
            n1
        );
        count += 1;
    }
    assert!(count > 0);
}

#[test]
#[ignore = "requires original piston and jet aircraft"]
fn fuel_selectors_route_and_stop_individual_engines() {
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
        if kind == 1 {
            let source = std::fs::read_to_string(path.join("aircraft.cfg")).unwrap();
            let source = source
                .lines()
                .map(|line| {
                    if line
                        .split('=')
                        .next()
                        .unwrap_or("")
                        .trim()
                        .eq_ignore_ascii_case("number_of_tank_selectors")
                    {
                        "number_of_tank_selectors=2"
                    } else {
                        line
                    }
                })
                .collect::<Vec<_>>()
                .join("\n");
            data.configuration = esp_formats::cfg::Config::parse(&source).unwrap();
        }
        let (speed, height) = if kind == 0 {
            (0.0, 0.0)
        } else {
            (160.0, 5000.0)
        };
        let mut flight = Flight::new(&data, speed, height, 0.5, false).unwrap();
        flight.start_running(&data).unwrap();
        let mut vars = controls(&data);
        flight.configure_engines(&mut vars).unwrap();
        if kind == 0 {
            vars.dispatch_event("PARKING_BRAKES", Value::Number(0.0))
                .unwrap();
        }
        vars.dispatch_event("FUEL_SELECTOR_LEFT", Value::Number(0.0))
            .unwrap();
        if kind == 1 {
            vars.dispatch_event("FUEL_SELECTOR_2_RIGHT", Value::Number(0.0))
                .unwrap();
        }
        let left = flight
            .weights
            .tanks()
            .iter()
            .position(|tank| tank.name == "LeftMain")
            .unwrap();
        let right = flight
            .weights
            .tanks()
            .iter()
            .position(|tank| tank.name == "RightMain")
            .unwrap();
        let before = flight.loading.tank_gallons.clone();
        for _ in 0..120 {
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        assert!(flight.loading.tank_gallons[left] < before[left]);
        if kind == 0 {
            assert_eq!(flight.loading.tank_gallons[right], before[right]);
        } else {
            assert!(flight.loading.tank_gallons[right] < before[right]);
        }
        vars.dispatch_event("FUEL_SELECTOR_OFF", Value::Number(0.0))
            .unwrap();
        for _ in 0..60 {
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
        }
        let combustion = |vars: &mut Variables, index| {
            vars.read(&Reference::new(
                'A',
                &format!("GENERAL ENG COMBUSTION:{index}"),
                Some("bool"),
            ))
            .unwrap()
            .number()
            .unwrap()
        };
        assert_eq!(combustion(&mut vars, 1), 0.0);
        if kind == 1 {
            assert_eq!(combustion(&mut vars, 2), 1.0);
            vars.dispatch_key_event(0x103bf, Value::Number(0.0))
                .unwrap();
            flight.advance(1.0 / 60.0, &mut vars).unwrap();
            assert_eq!(combustion(&mut vars, 2), 0.0);
        }
        verified[kind] = true;
    }
    assert_eq!(verified, [true, true]);
}

#[test]
#[ignore = "requires original turbine aircraft with thrust reversers"]
fn configured_reverse_thrust_changes_motion_and_cockpit_state() {
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
        let mut reverse = Flight::new(&data, 160.0, 5000.0, 0.5, false).unwrap();
        let mut idle = Flight::new(&data, 160.0, 5000.0, 0.5, false).unwrap();
        reverse.start_running(&data).unwrap();
        idle.start_running(&data).unwrap();
        let mut reverse_vars = controls(&data);
        let mut idle_vars = controls(&data);
        reverse.configure_engines(&mut reverse_vars).unwrap();
        idle.configure_engines(&mut idle_vars).unwrap();
        let lower = reverse_vars
            .read(&Reference::new(
                'A',
                "THROTTLE LOWER LIMIT",
                Some("percent"),
            ))
            .unwrap()
            .number()
            .unwrap();
        if lower >= 0.0 {
            continue;
        }
        reverse_vars
            .dispatch_event("AXIS_THROTTLE_REVERSER_SET", Value::Number(16383.0))
            .unwrap();
        for _ in 0..120 {
            reverse.advance(1.0 / 60.0, &mut reverse_vars).unwrap();
            idle.advance(1.0 / 60.0, &mut idle_vars).unwrap();
        }
        assert!(reverse.current.velocity.x < idle.current.velocity.x);
        for index in reverse.engines.as_ref().unwrap().indices() {
            let nozzle = Reference::new(
                'A',
                &format!("TURB ENG REVERSE NOZZLE PERCENT:{index}"),
                Some("percent over 100"),
            );
            assert_eq!(reverse_vars.read(&nozzle).unwrap().number().unwrap(), 1.0);
            assert_eq!(idle_vars.read(&nozzle).unwrap().number().unwrap(), 0.0);
        }
        reverse_vars
            .dispatch_event("THROTTLE_CUT", Value::Number(0.0))
            .unwrap();
        reverse.advance(1.0 / 60.0, &mut reverse_vars).unwrap();
        assert_eq!(
            reverse_vars
                .read(&Reference::new(
                    'A',
                    "TURB ENG REVERSE NOZZLE PERCENT:1",
                    Some("percent over 100")
                ))
                .unwrap()
                .number()
                .unwrap(),
            0.0
        );
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original jet AIR records and OPENESP_AIRCRAFT_PATHS"]
fn starvation_on_one_route_does_not_reduce_other_engine_thrust() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let mut data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(1.0)
        {
            continue;
        }
        let source = std::fs::read_to_string(path.join("aircraft.cfg")).unwrap();
        let source = source
            .lines()
            .map(|line| {
                if line
                    .split('=')
                    .next()
                    .unwrap_or("")
                    .trim()
                    .eq_ignore_ascii_case("number_of_tank_selectors")
                {
                    "number_of_tank_selectors=2"
                } else {
                    line
                }
            })
            .collect::<Vec<_>>()
            .join("\n");
        data.configuration = esp_formats::cfg::Config::parse(&source).unwrap();
        let setup = || {
            let mut flight = Flight::new(&data, 200.0, 10000.0, 0.5, false).unwrap();
            flight.start_running(&data).unwrap();
            let available = flight.weights.selected_fuel(&flight.loading, 2);
            flight.loading = flight
                .weights
                .burn_selected(&flight.loading, 2, available - 1e-10, false)
                .unwrap();
            let mut vars = controls(&data);
            flight.configure_engines(&mut vars).unwrap();
            vars.dispatch_event("FUEL_SELECTOR_LEFT", Value::Number(0.0))
                .unwrap();
            vars.dispatch_event("FUEL_SELECTOR_2_RIGHT", Value::Number(0.0))
                .unwrap();
            (flight, vars)
        };
        let (mut starved, mut starved_vars) = setup();
        let (mut cutoff, mut cutoff_vars) = setup();
        cutoff_vars
            .dispatch_event("TOGGLE_FUEL_VALVE_ENG1", Value::Number(0.0))
            .unwrap();
        starved.advance(1.0 / 60.0, &mut starved_vars).unwrap();
        cutoff.advance(1.0 / 60.0, &mut cutoff_vars).unwrap();
        let difference = (starved.current.velocity - cutoff.current.velocity).length();
        assert!(difference < 5e-5, "velocity difference {difference}");
        for (index, expected) in [(1, 0.0), (2, 1.0)] {
            assert_eq!(
                starved_vars
                    .read(&Reference::new(
                        'A',
                        &format!("GENERAL ENG COMBUSTION:{index}"),
                        Some("bool")
                    ))
                    .unwrap()
                    .number()
                    .unwrap(),
                expected
            );
        }
        assert!(starved.weights.selected_fuel(&starved.loading, 3) > 0.0);
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original jet AIR records and OPENESP_AIRCRAFT_PATHS"]
fn unusable_fuel_remains_after_engine_exhaustion() {
    let mut count = 0;
    for path in content::aircraft_paths() {
        let mut data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(1.0)
        {
            continue;
        }
        let source = std::fs::read_to_string(path.join("aircraft.cfg")).unwrap();
        let mut section = false;
        let source = source
            .lines()
            .map(|line| {
                let trimmed = line.trim();
                if trimmed.starts_with('[') {
                    section = trimmed.eq_ignore_ascii_case("[fuel]");
                }
                if section
                    && let Some((name, value)) = line.split_once('=')
                    && [
                        "center1",
                        "center2",
                        "center3",
                        "leftmain",
                        "leftaux",
                        "lefttip",
                        "rightmain",
                        "rightaux",
                        "righttip",
                        "external1",
                        "external2",
                    ]
                    .iter()
                    .any(|tank| name.trim().eq_ignore_ascii_case(tank))
                {
                    let value = value.split("//").next().unwrap();
                    if let Some((fields, _)) = value.rsplit_once(',') {
                        return format!("{name}={fields},1");
                    }
                }
                line.to_owned()
            })
            .collect::<Vec<_>>()
            .join("\n");
        data.configuration = esp_formats::cfg::Config::parse(&source).unwrap();
        let mut flight = Flight::new(&data, 200.0, 10000.0, 1.0, false).unwrap();
        flight.start_running(&data).unwrap();
        flight.loading = flight
            .weights
            .burn(
                &flight.loading,
                flight.loading.usable_fuel_gallons - 1e-7,
                false,
            )
            .unwrap();
        let mut variables = controls(&data);
        flight.configure_engines(&mut variables).unwrap();
        for _ in 0..120 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        assert_eq!(flight.loading.usable_fuel_gallons, 0.0);
        assert!(flight.loading.fuel_gallons > 0.0);
        assert_eq!(
            variables
                .read(&Reference::new(
                    'A',
                    "GENERAL ENG COMBUSTION:1",
                    Some("bool")
                ))
                .unwrap()
                .number()
                .unwrap(),
            0.0
        );
        assert_eq!(
            variables
                .read(&Reference::new('A', "FUEL TOTAL QUANTITY", Some("gallons")))
                .unwrap()
                .number()
                .unwrap(),
            flight.loading.fuel_gallons
        );
        count += 1;
    }
    assert!(count > 0);
}

#[test]
#[ignore = "requires original jet AIR records and OPENESP_AIRCRAFT_PATHS"]
fn fuel_exhaustion_extinguishes_combustion_without_negative_quantity() {
    let mut count = 0;
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
        let mut flight = Flight::new(&data, 200.0, 10000.0, 1e-8, false).unwrap();
        flight.start_running(&data).unwrap();
        let mut variables = controls(&data);
        flight.configure_engines(&mut variables).unwrap();
        variables
            .dispatch_event("THROTTLE_FULL", esp_script::Value::Number(0.0))
            .unwrap();
        for _ in 0..120 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        assert_eq!(flight.loading.fuel_gallons, 0.0);
        for index in [1, 2] {
            assert_eq!(
                variables
                    .read(&Reference::new(
                        'A',
                        &format!("GENERAL ENG COMBUSTION:{index}"),
                        Some("bool")
                    ))
                    .unwrap()
                    .number()
                    .unwrap(),
                0.0
            );
        }
        count += 1;
    }
    assert!(count > 0);
}

#[test]
#[ignore = "requires original aircraft AIR data and OPENESP_AIRCRAFT_PATHS"]
fn transforms_aircraft_about_its_cg_and_preserves_simulation_during_interpolation() {
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let mut flight = Flight::new(&data, 100.0, 1000.0, 0.0, false).unwrap();
        let mut variables = Variables::default();
        Flight::configure_ground(&mut variables);
        variables
            .configure_flaps(&esp_formats::cfg::aircraft::flaps(&data.configuration).unwrap())
            .unwrap();
        for name in [
            "ELEVATOR DEFLECTION PCT",
            "AILERON LEFT DEFLECTION PCT",
            "RUDDER DEFLECTION PCT",
        ] {
            variables.set_number(name, "percent", 0.0);
        }
        flight.advance(1.0 / 60.0, &mut variables).unwrap();
        let physical = flight.current;
        for alpha in [0.0, 0.25, 0.5, 1.0] {
            let mut state = flight.sample(alpha).unwrap();
            state.orientation = DQuat::from_euler(EulerRot::ZYX, 0.4, 0.2, 0.3);
            let (origin, orientation) = flight.pose(&state);
            let cg = flight.loading.cg;
            let rendered_cg = DVec3::new(cg.y, -cg.z, cg.x);
            let expected = DVec3::new(state.position.y, -state.position.z, state.position.x);
            assert!((origin + orientation.as_dquat() * rendered_cg).abs_diff_eq(expected, 1e-6));
            let forward = state.orientation * DVec3::X;
            assert!(
                (orientation * glam::Vec3::Z)
                    .as_dvec3()
                    .abs_diff_eq(DVec3::new(forward.y, -forward.z, forward.x), 1e-6)
            );
        }
        assert_eq!(flight.current, physical);
    }
}

#[test]
#[ignore = "requires original aircraft AIR data and OPENESP_AIRCRAFT_PATHS"]
fn altitude_changes_forces_and_indicated_airspeed() {
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let mut low = Flight::new(&data, 150.0, 1000.0, 0.0, false).unwrap();
        let mut high = Flight::new(&data, 150.0, 10_000.0, 0.0, false).unwrap();
        let mut variables = Variables::default();
        Flight::configure_ground(&mut variables);
        variables
            .configure_flaps(&esp_formats::cfg::aircraft::flaps(&data.configuration).unwrap())
            .unwrap();
        for name in [
            "ELEVATOR DEFLECTION PCT",
            "AILERON LEFT DEFLECTION PCT",
            "RUDDER DEFLECTION PCT",
        ] {
            variables.set_number(name, "percent", 0.0);
        }
        low.advance(1.0 / 60.0, &mut variables).unwrap();
        high.advance(1.0 / 60.0, &mut variables).unwrap();
        assert!(low.current.velocity.x < high.current.velocity.x);
        assert!(low.current.velocity.z < high.current.velocity.z);
        let reference = Reference::new('A', "AIRSPEED INDICATED", Some("knots"));
        Flight::publish(&low.current, &mut variables).unwrap();
        let low_ias = variables.read(&reference).unwrap().number().unwrap();
        Flight::publish(&high.current, &mut variables).unwrap();
        let high_ias = variables.read(&reference).unwrap().number().unwrap();
        assert!(high_ias < low_ias && low_ias < 150.0);
        let pressure = variables
            .read(&Reference::new('A', "AMBIENT PRESSURE", Some("millibars")))
            .unwrap()
            .number()
            .unwrap();
        assert!((pressure - 264.999).abs() < 0.01);
    }
}

#[test]
#[ignore = "requires original aircraft AIR data and OPENESP_AIRCRAFT_PATHS"]
fn configured_loading_changes_flight_response_and_weight_variables() {
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let mut empty = Flight::new(&data, 150.0, 1000.0, 0.0, false).unwrap();
        let mut full = Flight::new(&data, 150.0, 1000.0, 1.0, true).unwrap();
        assert!(full.loading.mass.kilograms() > empty.loading.mass.kilograms());
        assert!(full.loading.payload_kilograms > 0.0 && full.loading.fuel_gallons > 0.0);
        assert!(!full.loading.cg.abs_diff_eq(empty.loading.cg, 1e-6));
        let mut variables = Variables::default();
        Flight::configure_ground(&mut variables);
        variables
            .configure_flaps(&esp_formats::cfg::aircraft::flaps(&data.configuration).unwrap())
            .unwrap();
        for name in [
            "ELEVATOR DEFLECTION PCT",
            "AILERON LEFT DEFLECTION PCT",
            "RUDDER DEFLECTION PCT",
        ] {
            variables.set_number(name, "percent", 0.0);
        }
        full.publish_loading(&mut variables);
        let total = variables
            .read(&Reference::new('A', "TOTAL WEIGHT", Some("pounds")))
            .unwrap()
            .number()
            .unwrap();
        let fuel = variables
            .read(&Reference::new(
                'A',
                "FUEL TOTAL QUANTITY WEIGHT",
                Some("kilograms"),
            ))
            .unwrap()
            .number()
            .unwrap();
        assert!((total * 0.45359237 - full.loading.mass.kilograms()).abs() < 1e-8);
        assert!(
            (fuel + full.loading.payload_kilograms + full.loading.empty_kilograms
                - full.loading.mass.kilograms())
            .abs()
                < 1e-8
        );
        empty.advance(1.0 / 60.0, &mut variables).unwrap();
        full.advance(1.0 / 60.0, &mut variables).unwrap();
        assert!(full.current.velocity.z > empty.current.velocity.z);
    }
}

#[test]
fn airspeed_selection_switches_between_true_and_indicated() {
    let mut variables = Variables::default();
    let state = State {
        position: DVec3::new(0.0, 0.0, -5000.0),
        velocity: DVec3::X * 100.0,
        ..State::default()
    };
    let read = |variables: &mut Variables, name| {
        variables
            .read(&Reference::new('A', name, Some("meters per second")))
            .unwrap()
            .number()
            .unwrap()
    };
    Flight::publish(&state, &mut variables).unwrap();
    let indicated = read(&mut variables, "AIRSPEED INDICATED");
    assert!(indicated < 100.0);
    assert_eq!(
        read(&mut variables, "AIRSPEED SELECT INDICATED OR TRUE:2"),
        indicated
    );
    variables.set_number("TRUE AIRSPEED SELECTED", "bool", 1.0);
    Flight::publish(&state, &mut variables).unwrap();
    assert_eq!(
        read(&mut variables, "AIRSPEED SELECT INDICATED OR TRUE"),
        100.0
    );
    assert_eq!(
        read(&mut variables, "AIRSPEED SELECT INDICATED OR TRUE:2"),
        100.0
    );
    assert_eq!(read(&mut variables, "AIRSPEED INDICATED"), indicated);
    variables.set_number("TRUE AIRSPEED SELECTED", "bool", 0.0);
    Flight::publish(&state, &mut variables).unwrap();
    assert_eq!(
        read(&mut variables, "AIRSPEED SELECT INDICATED OR TRUE"),
        indicated
    );
}

#[test]
fn altimeter_settings_change_indications_independently_even_with_blocked_static() {
    let state = State {
        position: DVec3::new(0.0, 0.0, -1000.0),
        velocity: DVec3::X * 50.0,
        ..State::default()
    };
    let mut variables = Variables::default();
    let read = |variables: &mut Variables, name, unit| {
        variables
            .read(&Reference::new('A', name, Some(unit)))
            .unwrap()
            .number()
            .unwrap()
    };
    Flight::publish(&state, &mut variables).unwrap();
    assert!((read(&mut variables, "KOHLSMAN SETTING HG", "inhg") - 29.92126).abs() < 0.00001);
    variables
        .dispatch_key_event(0x1015b, Value::Number(2.0))
        .unwrap();
    assert!(
        (read(&mut variables, "KOHLSMAN SETTING MB:2", "hectopascals") - 1013.58863895).abs()
            < 1e-8
    );
    Flight::publish(&state, &mut variables).unwrap();
    assert!((read(&mut variables, "INDICATED ALTITUDE", "meters") - 1000.0).abs() < 1e-8);
    assert!(read(&mut variables, "INDICATED ALTITUDE:2", "meters") > 1000.0);
    variables
        .dispatch_event("TOGGLE_STATIC_PORT_BLOCKAGE", Value::Number(0.0))
        .unwrap();
    variables
        .dispatch_event("KOHLSMAN_INC", Value::Number(2.0))
        .unwrap();
    Flight::publish(&state, &mut variables).unwrap();
    assert!(read(&mut variables, "INDICATED ALTITUDE:2", "meters") > 1005.0);
    variables
        .dispatch_key_event(0x1051e, Value::Number(2.0))
        .unwrap();
    Flight::publish(&state, &mut variables).unwrap();
    assert!((read(&mut variables, "INDICATED ALTITUDE:2", "meters") - 1000.0).abs() < 1e-8);
    variables
        .dispatch_key_event(0x101f8, Value::Number(1000.0 * 16.0))
        .unwrap();
    assert!((read(&mut variables, "KOHLSMAN SETTING HG:1", "millibars") - 1000.0).abs() < 1e-10);
    assert!(
        variables
            .dispatch_event("KOHLSMAN_SET", Value::Number(f64::NAN))
            .is_err()
    );
}

#[test]
fn blocked_pressure_lines_respond_to_changes_in_flight() {
    let mut variables = Variables::default();
    let mut state = State {
        position: DVec3::new(0.0, 0.0, -1000.0),
        velocity: DVec3::X * 100.0,
        ..State::default()
    };
    let speed = |variables: &mut Variables| {
        variables
            .read(&Reference::new(
                'A',
                "AIRSPEED INDICATED",
                Some("meters per second"),
            ))
            .unwrap()
            .number()
            .unwrap()
    };
    Flight::publish(&state, &mut variables).unwrap();
    let initial = speed(&mut variables);
    variables
        .dispatch_event("TOGGLE_PITOT_BLOCKAGE", Value::Number(0.0))
        .unwrap();
    state.velocity.x = 150.0;
    Flight::publish(&state, &mut variables).unwrap();
    assert!((speed(&mut variables) - initial).abs() < 1e-10);
    state.position.z = -2000.0;
    Flight::publish(&state, &mut variables).unwrap();
    assert!(speed(&mut variables) > initial);
    variables
        .dispatch_event("TOGGLE_PITOT_BLOCKAGE", Value::Number(0.0))
        .unwrap();
    Flight::publish(&state, &mut variables).unwrap();
    variables
        .dispatch_event("TOGGLE_STATIC_PORT_BLOCKAGE", Value::Number(0.0))
        .unwrap();
    let static_initial = speed(&mut variables);
    state.velocity.x = 180.0;
    Flight::publish(&state, &mut variables).unwrap();
    assert!(speed(&mut variables) > static_initial);
    state.position.z = -3000.0;
    Flight::publish(&state, &mut variables).unwrap();
    let blocked_indication = speed(&mut variables);
    let accurate = Atmosphere::standard(3000.0)
        .unwrap()
        .calibrated_airspeed(180.0)
        .unwrap();
    assert!(blocked_indication < accurate);
    variables
        .dispatch_event("TOGGLE_PITOT_BLOCKAGE", Value::Number(0.0))
        .unwrap();
    state.velocity.x = 220.0;
    state.position.z = -4000.0;
    Flight::publish(&state, &mut variables).unwrap();
    assert!((speed(&mut variables) - blocked_indication).abs() < 1e-10);
    variables
        .dispatch_event("TOGGLE_PITOT_BLOCKAGE", Value::Number(0.0))
        .unwrap();
    variables
        .dispatch_event("TOGGLE_STATIC_PORT_BLOCKAGE", Value::Number(0.0))
        .unwrap();
    Flight::publish(&state, &mut variables).unwrap();
    let accurate = Atmosphere::standard(4000.0)
        .unwrap()
        .calibrated_airspeed(220.0)
        .unwrap();
    assert!((speed(&mut variables) - accurate).abs() < 1e-10);
}

#[test]
fn pitot_ice_and_manual_blockages_are_independent() {
    let mut variables = Variables::default();
    let mut state = State {
        velocity: DVec3::X * 50.0,
        ..State::default()
    };
    let speed = |vars: &mut Variables| {
        vars.read(&Reference::new(
            'A',
            "AIRSPEED INDICATED",
            Some("meters per second"),
        ))
        .unwrap()
        .number()
        .unwrap()
    };
    Flight::publish(&state, &mut variables).unwrap();
    let initial = speed(&mut variables);
    variables.set_number("OPENESP PITOT ICE BLOCKED", "bool", 1.0);
    variables
        .dispatch_event("TOGGLE_PITOT_BLOCKAGE", Value::Number(0.0))
        .unwrap();
    state.velocity.x = 100.0;
    Flight::publish(&state, &mut variables).unwrap();
    assert_eq!(speed(&mut variables), initial);
    variables.set_number("OPENESP PITOT ICE BLOCKED", "bool", 0.0);
    Flight::publish(&state, &mut variables).unwrap();
    assert_eq!(speed(&mut variables), initial);
    variables
        .dispatch_event("TOGGLE_PITOT_BLOCKAGE", Value::Number(0.0))
        .unwrap();
    Flight::publish(&state, &mut variables).unwrap();
    assert!(speed(&mut variables) > initial + 40.0);
}

#[test]
fn pressure_blockages_hold_indications_until_repaired() {
    let mut variables = Variables::default();
    let mut state = State {
        position: DVec3::new(0.0, 0.0, -1000.0),
        velocity: DVec3::new(50.0, 0.0, -5.0),
        ..State::default()
    };
    let read = |variables: &mut Variables, name, unit| {
        variables
            .read(&Reference::new('A', name, Some(unit)))
            .unwrap()
            .number()
            .unwrap()
    };
    Flight::publish(&state, &mut variables).unwrap();
    let airspeed = read(&mut variables, "AIRSPEED INDICATED", "meters per second");
    variables
        .dispatch_key_event(0x10303, Value::Number(0.0))
        .unwrap();
    state.velocity.x = 100.0;
    Flight::publish(&state, &mut variables).unwrap();
    assert_eq!(
        read(&mut variables, "AIRSPEED INDICATED", "meters per second"),
        airspeed
    );
    assert!(read(&mut variables, "AIRSPEED TRUE", "meters per second") > 100.0);
    variables
        .dispatch_key_event(0x10304, Value::Number(0.0))
        .unwrap();
    state.position.z = -2000.0;
    Flight::publish(&state, &mut variables).unwrap();
    assert!((read(&mut variables, "INDICATED ALTITUDE", "meters") - 1000.0).abs() < 1e-8);
    assert_eq!(read(&mut variables, "PLANE ALTITUDE", "meters"), 2000.0);
    assert_eq!(
        read(&mut variables, "VERTICAL SPEED", "meters per second"),
        0.0
    );
    variables
        .dispatch_event("TOGGLE_PITOT_BLOCKAGE", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "PARTIAL PANEL PITOT", "enum"), 1.0);
    variables
        .dispatch_event("TOGGLE_STATIC_PORT_BLOCKAGE", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "PARTIAL PANEL PITOT", "enum"), 0.0);
    Flight::publish(&state, &mut variables).unwrap();
    assert!(read(&mut variables, "AIRSPEED INDICATED", "meters per second") > airspeed);
    assert!((read(&mut variables, "INDICATED ALTITUDE", "meters") - 2000.0).abs() < 1e-8);
    assert_eq!(
        read(&mut variables, "VERTICAL SPEED", "meters per second"),
        5.0
    );
    variables
        .dispatch_key_event(0x10301, Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "PARTIAL PANEL VACUUM", "enum"), 1.0);
    assert_eq!(read(&mut variables, "PARTIAL PANEL VACUUM", "bool"), 1.0);
    variables
        .dispatch_event("TOGGLE_VACUUM_FAILURE", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "PARTIAL PANEL VACUUM", "enum"), 0.0);
}

#[test]
fn publishes_attitude_and_airspeed_using_simulator_units() {
    let state = State {
        position: DVec3::new(0.0, 0.0, -1000.0),
        velocity: DVec3::X * 50.0,
        orientation: DQuat::from_euler(EulerRot::ZYX, 0.4, 0.1, 0.2),
        ..State::default()
    };
    let mut variables = Variables::default();
    Flight::publish(&state, &mut variables).unwrap();
    for (name, unit, expected) in [
        ("AIRSPEED TRUE", "knots", 50.0 * 3600.0 / 1852.0),
        (
            "AIRSPEED SELECT INDICATED OR TRUE:2",
            "meters per second",
            Atmosphere::standard(1000.0)
                .unwrap()
                .calibrated_airspeed(50.0)
                .unwrap(),
        ),
        ("PLANE ALTITUDE", "feet", 1000.0 / 0.3048),
        ("PLANE PITCH DEGREES", "radians", -0.1),
        ("PLANE BANK DEGREES", "radians", -0.2),
        ("PLANE HEADING DEGREES TRUE", "radians", 0.4),
    ] {
        let actual = variables
            .read(&Reference::new('A', name, Some(unit)))
            .unwrap()
            .number()
            .unwrap();
        assert!((actual - expected).abs() < 1e-9, "{name}: {actual}");
    }
}

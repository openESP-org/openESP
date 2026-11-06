use super::*;

#[test]
fn airspeed_reference_card_events_wrap_and_validate_values() {
    let mut variables = Variables::default();
    variables
        .dispatch_key_event(0x1015e, Value::Number(0.0))
        .unwrap();
    let reference = Reference::new('A', "AIRSPEED TRUE CALIBRATE", Some("degrees"));
    assert_eq!(variables.read(&reference).unwrap(), Value::Number(359.0));
    variables
        .dispatch_key_event(0x1015d, Value::Number(0.0))
        .unwrap();
    assert_eq!(variables.read(&reference).unwrap(), Value::Number(0.0));
    variables
        .dispatch_key_event(0x101fb, Value::Number(45.0))
        .unwrap();
    assert_eq!(variables.read(&reference).unwrap(), Value::Number(45.0));
    assert!(
        variables
            .dispatch_event("TRUE_AIRSPEED_CAL_SET", Value::Number(f64::NAN))
            .is_err()
    );
    assert_eq!(variables.read(&reference).unwrap(), Value::Number(45.0));
}

#[test]
fn autopilot_master_and_mode_events_are_exclusive() {
    let mut variables = Variables::default();
    let read = |variables: &mut Variables, name| {
        variables
            .read(&Reference::new('A', name, Some("bool")))
            .unwrap()
            .number()
            .unwrap()
    };

    variables
        .dispatch_event("AP_PANEL_HEADING_HOLD", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "AUTOPILOT HEADING LOCK"), 1.0);
    variables
        .dispatch_event("AP_WING_LEVELER", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "AUTOPILOT HEADING LOCK"), 0.0);
    assert_eq!(read(&mut variables, "AUTOPILOT WING LEVELER"), 1.0);
    variables
        .dispatch_event("AP_PANEL_HEADING_HOLD", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "AUTOPILOT HEADING LOCK"), 1.0);
    assert_eq!(read(&mut variables, "AUTOPILOT WING LEVELER"), 0.0);

    variables
        .dispatch_event("AP_PANEL_ALTITUDE_HOLD", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "AUTOPILOT ALTITUDE LOCK"), 1.0);
    variables
        .dispatch_event("AP_PANEL_VS_HOLD", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "AUTOPILOT ALTITUDE LOCK"), 0.0);
    assert_eq!(read(&mut variables, "AUTOPILOT VERTICAL HOLD"), 1.0);
    variables
        .dispatch_event("AP_PANEL_VS_HOLD", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "AUTOPILOT VERTICAL HOLD"), 0.0);
    variables
        .dispatch_event("AP_PANEL_ALTITUDE_HOLD", Value::Number(0.0))
        .unwrap();

    variables
        .dispatch_event("AUTOPILOT_ON", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "AUTOPILOT MASTER"), 1.0);
    assert_eq!(read(&mut variables, "AUTOPILOT HEADING LOCK"), 1.0);
    variables
        .dispatch_event("AP_MASTER", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "AUTOPILOT MASTER"), 0.0);
    assert_eq!(read(&mut variables, "AUTOPILOT HEADING LOCK"), 0.0);
    assert_eq!(read(&mut variables, "AUTOPILOT ALTITUDE LOCK"), 0.0);

    variables
        .dispatch_event("AP_MASTER", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "AUTOPILOT MASTER"), 1.0);
    assert_eq!(read(&mut variables, "AUTOPILOT ALTITUDE LOCK"), 0.0);
    variables
        .dispatch_event("AUTOPILOT_DISENGAGE", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "AUTOPILOT MASTER"), 0.0);
}

#[test]
fn autopilot_selectors_increment_convert_and_bound_selected_values() {
    let mut variables = Variables::default();
    let read = |variables: &mut Variables, name, unit| {
        variables
            .read(&Reference::new('A', name, Some(unit)))
            .unwrap()
            .number()
            .unwrap()
    };
    variables.set_number("AUTOPILOT MAX BANK", "degrees", 25.0);
    variables
        .dispatch_event("AP_MAX_BANK_INC", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "AUTOPILOT MAX BANK", "degrees"), 30.0);
    for _ in 0..10 {
        variables
            .dispatch_event("AP_MAX_BANK_DEC", Value::Number(0.0))
            .unwrap();
    }
    assert_eq!(read(&mut variables, "AUTOPILOT MAX BANK", "degrees"), 10.0);
    variables
        .dispatch_event("AP_MAX_BANK_SET", Value::Number(22.0))
        .unwrap();
    assert_eq!(read(&mut variables, "AUTOPILOT MAX BANK", "degrees"), 22.0);

    variables
        .dispatch_event("AP_ALT_VAR_SET_ENGLISH", Value::Number(5000.0))
        .unwrap();
    assert_eq!(
        read(&mut variables, "AUTOPILOT ALTITUDE LOCK VAR", "feet"),
        5000.0
    );
    variables
        .dispatch_event("AP_ALT_VAR_INC", Value::Number(0.0))
        .unwrap();
    assert_eq!(
        read(&mut variables, "AUTOPILOT ALTITUDE LOCK VAR", "feet"),
        5100.0
    );
    variables
        .dispatch_event("AP_ALT_VAR_DEC", Value::Number(0.0))
        .unwrap();
    variables
        .dispatch_event("AP_ALT_VAR_DEC", Value::Number(0.0))
        .unwrap();
    assert_eq!(
        read(&mut variables, "AUTOPILOT ALTITUDE LOCK VAR", "feet"),
        4900.0
    );
    variables
        .dispatch_event("AP_ALT_VAR_SET_METRIC", Value::Number(1524.0))
        .unwrap();
    let altitude = read(&mut variables, "AUTOPILOT ALTITUDE LOCK VAR", "feet");
    assert!((altitude - 5000.0).abs() < 0.01);
    for _ in 0..2000 {
        variables
            .dispatch_event("AP_ALT_VAR_INC", Value::Number(0.0))
            .unwrap();
    }
    assert_eq!(
        read(&mut variables, "AUTOPILOT ALTITUDE LOCK VAR", "feet"),
        100_000.0
    );

    variables
        .dispatch_event("AP_VS_VAR_SET_ENGLISH", Value::Number(1500.0))
        .unwrap();
    assert_eq!(
        read(
            &mut variables,
            "AUTOPILOT VERTICAL HOLD VAR",
            "feet per minute"
        ),
        1500.0
    );
    variables
        .dispatch_event("AP_VS_VAR_DEC", Value::Number(0.0))
        .unwrap();
    assert_eq!(
        read(
            &mut variables,
            "AUTOPILOT VERTICAL HOLD VAR",
            "feet per minute"
        ),
        1400.0
    );
    variables
        .dispatch_event("AP_VS_VAR_SET_METRIC", Value::Number(-3.0))
        .unwrap();
    let vertical_speed = read(
        &mut variables,
        "AUTOPILOT VERTICAL HOLD VAR",
        "feet per minute",
    );
    assert!((vertical_speed + 590.551).abs() < 0.01);
    variables
        .dispatch_event("AP_VS_VAR_INC", Value::Number(0.0))
        .unwrap();
    assert!(
        (read(
            &mut variables,
            "AUTOPILOT VERTICAL HOLD VAR",
            "feet per minute"
        ) + 490.551)
            .abs()
            < 0.01
    );
    for _ in 0..100 {
        variables
            .dispatch_event("AP_VS_VAR_INC", Value::Number(0.0))
            .unwrap();
    }
    assert_eq!(
        read(
            &mut variables,
            "AUTOPILOT VERTICAL HOLD VAR",
            "feet per minute"
        ),
        6000.0
    );

    let altitude = read(&mut variables, "AUTOPILOT ALTITUDE LOCK VAR", "feet");
    let vertical_speed = read(
        &mut variables,
        "AUTOPILOT VERTICAL HOLD VAR",
        "feet per minute",
    );
    assert!(
        variables
            .dispatch_event("AP_ALT_VAR_SET_ENGLISH", Value::Number(f64::NAN))
            .is_err()
    );
    assert!(
        variables
            .dispatch_event("AP_VS_VAR_SET_ENGLISH", Value::Number(f64::INFINITY))
            .is_err()
    );
    assert_eq!(
        read(&mut variables, "AUTOPILOT ALTITUDE LOCK VAR", "feet"),
        altitude
    );
    assert_eq!(
        read(
            &mut variables,
            "AUTOPILOT VERTICAL HOLD VAR",
            "feet per minute"
        ),
        vertical_speed
    );
}

#[test]
fn incremental_controls_use_pilot_input_and_aircraft_travel() {
    let mut vars = Variables::default();
    vars.configure_aileron_travel(25.0).unwrap();
    vars.dispatch_event("AILERON_SET", Value::Number(8191.5))
        .unwrap();
    vars.dispatch_event("RUDDER_SET", Value::Number(-8191.5))
        .unwrap();
    vars.dispatch_event("ELEVATOR_SET", Value::Number(8191.5))
        .unwrap();
    vars.set_control_surfaces([0.1, 0.1, -0.1, -0.1]);
    vars.dispatch_key_event(0x1004b, Value::Number(0.0))
        .unwrap();
    vars.dispatch_key_event(0x10050, Value::Number(0.0))
        .unwrap();
    vars.dispatch_key_event(0x1009a, Value::Number(0.0))
        .unwrap();
    assert!((vars.control_input("AILERON LEFT DEFLECTION PCT").unwrap() - 0.46).abs() < 1e-12);
    assert!((vars.control_input("ELEVATOR DEFLECTION PCT").unwrap() - 0.55).abs() < 1e-12);
    assert!((vars.control_input("RUDDER DEFLECTION PCT").unwrap() + 0.45).abs() < 1e-12);
    vars.dispatch_key_event(0x10099, Value::Number(0.0))
        .unwrap();
    assert_eq!(vars.control_input("RUDDER DEFLECTION PCT").unwrap(), 0.0);
    assert!((vars.control_input("AILERON LEFT DEFLECTION PCT").unwrap() - 0.46).abs() < 1e-12);
    for _ in 0..100 {
        vars.dispatch_event("ELEV_UP", Value::Number(0.0)).unwrap();
    }
    assert_eq!(vars.control_input("ELEVATOR DEFLECTION PCT").unwrap(), 1.0);
    assert!(
        vars.dispatch_event("ELEV_DOWN", Value::Number(f64::NAN))
            .is_err()
    );
    assert_eq!(vars.control_input("ELEVATOR DEFLECTION PCT").unwrap(), 1.0);
    vars.dispatch_event("AILERON_CENTER", Value::Number(0.0))
        .unwrap();
    assert_eq!(
        vars.control_input("AILERON LEFT DEFLECTION PCT").unwrap(),
        0.0
    );
    assert_eq!(vars.control_input("ELEVATOR DEFLECTION PCT").unwrap(), 1.0);
}

#[test]
fn primary_control_events_update_pilot_inputs() {
    let mut vars = Variables::default();
    vars.dispatch_event("AXIS_ELEVATOR_SET", Value::Number(8191.5))
        .unwrap();
    vars.dispatch_key_event(0x100e3, Value::Number(-16383.0))
        .unwrap();
    vars.dispatch_key_event(0x100a0, Value::Number(20000.0))
        .unwrap();
    let read = |vars: &mut Variables, name| {
        vars.read(&Reference::new('A', name, Some("percent over 100")))
            .unwrap()
            .number()
            .unwrap()
    };
    assert_eq!(read(&mut vars, "YOKE Y POSITION"), 0.5);
    assert_eq!(read(&mut vars, "ELEVATOR DEFLECTION PCT"), 0.5);
    assert_eq!(read(&mut vars, "YOKE X POSITION"), -1.0);
    assert_eq!(read(&mut vars, "AILERON LEFT DEFLECTION PCT"), -1.0);
    assert_eq!(read(&mut vars, "AILERON RIGHT DEFLECTION PCT"), 1.0);
    assert_eq!(read(&mut vars, "RUDDER PEDAL POSITION"), 1.0);
    assert!(
        vars.dispatch_event("RUDDER_SET", Value::Number(f64::NAN))
            .is_err()
    );
    assert_eq!(read(&mut vars, "RUDDER DEFLECTION PCT"), 1.0);
    vars.dispatch_key_event(0x1004c, Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut vars, "AILERON LEFT DEFLECTION PCT"), 0.0);
    assert_eq!(read(&mut vars, "RUDDER DEFLECTION PCT"), 0.0);
    assert_eq!(read(&mut vars, "ELEVATOR DEFLECTION PCT"), 0.5);
}

#[test]
fn turbine_ignition_events_keep_the_three_switch_positions() {
    let mut vars = Variables::default();
    vars.set_number("TURB ENG IGNITION SWITCH", "enum", 1.0);
    let switch = Reference::new('A', "TURB ENG IGNITION SWITCH", Some("enum"));
    vars.dispatch_key_event(0x1049d, Value::Number(0.0))
        .unwrap();
    assert_eq!(vars.read(&switch).unwrap(), Value::Number(0.0));
    vars.dispatch_key_event(0x10795, Value::Number(0.0))
        .unwrap();
    assert_eq!(vars.read(&switch).unwrap(), Value::Number(2.0));
    assert!(
        vars.dispatch_event("TURBINE_IGNITION_SWITCH_SET", Value::Number(2.5))
            .is_err()
    );
    assert_eq!(vars.read(&switch).unwrap(), Value::Number(2.0));
    vars.dispatch_key_event(0x10797, Value::Number(0.0))
        .unwrap();
    assert_eq!(vars.read(&switch).unwrap(), Value::Number(1.0));
}

#[test]
fn indexed_magnetos_preserve_other_engines_and_report_the_switch_position() {
    let mut variables = Variables::default();
    variables.configure_engines([1, 2].into_iter()).unwrap();
    for index in [1, 2] {
        for side in ["LEFT", "RIGHT"] {
            variables.set_number(&format!("RECIP ENG {side} MAGNETO:{index}"), "bool", 1.0);
        }
    }
    variables
        .dispatch_key_event(0x1018e, Value::Number(0.0))
        .unwrap();
    let read = |variables: &mut Variables, name, unit| {
        variables
            .read(&Reference::new('A', name, Some(unit)))
            .unwrap()
            .number()
            .unwrap()
    };
    assert_eq!(
        read(&mut variables, "RECIP ENG RIGHT MAGNETO:2", "bool"),
        0.0
    );
    assert_eq!(
        read(&mut variables, "RECIP ENG LEFT MAGNETO:2", "bool"),
        1.0
    );
    assert_eq!(
        read(&mut variables, "RECIP ENG LEFT MAGNETO:1", "bool"),
        1.0
    );
    assert_eq!(read(&mut variables, "RECIP ENG MAGNETO:2", "enum"), 2.0);
    assert!(
        variables
            .dispatch_event("MAGNETO2_SET", Value::Number(4.0))
            .is_err()
    );
    assert_eq!(read(&mut variables, "RECIP ENG MAGNETO:2", "enum"), 2.0);
}

#[test]
fn mixture_events_control_indexed_levers_and_reject_invalid_values() {
    let mut variables = Variables::default();
    variables.configure_engines([1, 2].into_iter()).unwrap();
    for index in [1, 2] {
        variables.set_number(
            &format!("GENERAL ENG MIXTURE LEVER POSITION:{index}"),
            "percent",
            100.0,
        );
    }
    variables
        .dispatch_key_event(0x10180, Value::Number(8191.5))
        .unwrap();
    let read = |variables: &mut Variables, index| {
        variables
            .read(&Reference::new(
                'A',
                &format!("GENERAL ENG MIXTURE LEVER POSITION:{index}"),
                Some("percent"),
            ))
            .unwrap()
            .number()
            .unwrap()
    };
    assert_eq!(read(&mut variables, 1), 100.0);
    assert_eq!(read(&mut variables, 2), 50.0);
    variables
        .dispatch_event("AXIS_MIXTURE1_SET", Value::Number(-16383.0))
        .unwrap();
    assert_eq!(read(&mut variables, 1), 0.0);
    assert!(
        variables
            .dispatch_event("MIXTURE_SET", Value::Number(f64::NAN))
            .is_err()
    );
    assert_eq!(read(&mut variables, 2), 50.0);
    variables
        .dispatch_event("MIXTURE_RICH", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, 1), 100.0);
    assert_eq!(read(&mut variables, 2), 100.0);
}

#[test]
fn legacy_engine_names_share_canonical_indexed_values() {
    let mut variables = Variables::default();
    variables.set_number("TURB ENG N1:2", "percent", 82.5);
    assert_eq!(
        variables
            .read(&Reference::new('A', "Turb eng2 N1", Some("percent")))
            .unwrap()
            .number()
            .unwrap(),
        82.5
    );
    variables.set_number("GENERAL ENG1 THROTTLE LEVER POSITION", "percent", 35.0);
    let throttle = variables
        .read(&Reference::new(
            'A',
            "GENERAL ENG THROTTLE LEVER POSITION:1",
            Some("percent over 100"),
        ))
        .unwrap()
        .number()
        .unwrap();
    assert!((throttle - 0.35).abs() < 1e-12);
    variables.set('L', "TURB ENG1 N1", None, Value::Number(7.0));
    assert_eq!(
        variables
            .read(&Reference::new('L', "TURB ENG1 N1", None))
            .unwrap()
            .number()
            .unwrap(),
        7.0
    );
    assert!(
        variables
            .read(&Reference::new('A', "TURB ENG0 N1", Some("percent")))
            .is_err()
    );
}

#[test]
fn throttle_events_share_indexed_engine_state_and_preserve_other_engines() {
    let mut variables = Variables::default();
    variables.configure_engines([1, 2].into_iter()).unwrap();
    let read = |variables: &mut Variables, index| {
        variables
            .read(&Reference::new(
                'A',
                &format!("GENERAL ENG THROTTLE LEVER POSITION:{index}"),
                Some("percent"),
            ))
            .unwrap()
            .number()
            .unwrap()
    };
    variables
        .dispatch_event("THROTTLE_FULL", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, 1), 100.0);
    assert_eq!(read(&mut variables, 2), 100.0);
    variables
        .dispatch_event("AXIS_THROTTLE1_SET", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, 1), 50.0);
    assert_eq!(read(&mut variables, 2), 100.0);
    variables
        .dispatch_event("THROTTLE2_SET", Value::Number(4095.75))
        .unwrap();
    assert_eq!(read(&mut variables, 2), 25.0);
    assert!(
        variables
            .dispatch_event("THROTTLE_SET", Value::Number(f64::NAN))
            .is_err()
    );
    assert_eq!(read(&mut variables, 2), 25.0);
    variables
        .dispatch_event("ENGINE_AUTO_SHUTDOWN", Value::Number(0.0))
        .unwrap();
    assert_eq!(
        variables
            .read(&Reference::new(
                'A',
                "GENERAL ENG FUEL VALVE:1",
                Some("bool")
            ))
            .unwrap()
            .number()
            .unwrap(),
        0.0
    );
    variables
        .dispatch_key_event(0x103be, Value::Number(0.0))
        .unwrap();
    assert_eq!(
        variables
            .read(&Reference::new(
                'A',
                "GENERAL ENG FUEL VALVE:1",
                Some("bool")
            ))
            .unwrap()
            .number()
            .unwrap(),
        1.0
    );
    assert_eq!(
        variables
            .read(&Reference::new(
                'A',
                "GENERAL ENG FUEL VALVE:2",
                Some("bool")
            ))
            .unwrap()
            .number()
            .unwrap(),
        0.0
    );
    variables
        .dispatch_event("TOGGLE_FUEL_VALVE_ALL", Value::Number(0.0))
        .unwrap();
    assert_eq!(
        variables
            .read(&Reference::new(
                'A',
                "GENERAL ENG FUEL VALVE:1",
                Some("bool")
            ))
            .unwrap()
            .number()
            .unwrap(),
        0.0
    );
    assert_eq!(
        variables
            .read(&Reference::new(
                'A',
                "GENERAL ENG FUEL VALVE:2",
                Some("bool")
            ))
            .unwrap()
            .number()
            .unwrap(),
        1.0
    );
    assert!(
        variables
            .dispatch_event("TOGGLE_FUEL_VALVE_ENG3", Value::Number(0.0))
            .is_err()
    );
    variables.set_number("TURB ENG JET THRUST:1", "newtons", 4.4482216152605 * 1000.0);
    assert!(
        (variables
            .read(&Reference::new(
                'A',
                "TURB ENG JET THRUST:1",
                Some("pounds")
            ))
            .unwrap()
            .number()
            .unwrap()
            - 1000.0)
            .abs()
            < 1e-9
    );
}

#[test]
fn throttle_events_respect_the_configured_reverse_range() {
    let mut variables = Variables::default();
    variables.configure_engines([1, 2].into_iter()).unwrap();
    variables.configure_throttle_limit(-0.25).unwrap();
    for _ in 0..30 {
        variables
            .dispatch_event("THROTTLE_DECR", Value::Number(0.0))
            .unwrap();
    }
    let throttle = |index| {
        Reference::new(
            'A',
            &format!("GENERAL ENG THROTTLE LEVER POSITION:{index}"),
            Some("percent"),
        )
    };
    assert_eq!(
        variables.read(&throttle(1)).unwrap().number().unwrap(),
        -25.0
    );
    variables
        .dispatch_event("THROTTLE2_SET", Value::Number(8191.5))
        .unwrap();
    assert_eq!(
        variables.read(&throttle(1)).unwrap().number().unwrap(),
        -25.0
    );
    assert_eq!(
        variables.read(&throttle(2)).unwrap().number().unwrap(),
        50.0
    );
    variables
        .dispatch_event("THROTTLE_CUT", Value::Number(0.0))
        .unwrap();
    assert_eq!(variables.read(&throttle(1)).unwrap().number().unwrap(), 0.0);
    variables
        .dispatch_event("AXIS_THROTTLE_REVERSER1_SET", Value::Number(0.0))
        .unwrap();
    assert_eq!(
        variables.read(&throttle(1)).unwrap().number().unwrap(),
        -12.5
    );
    assert_eq!(variables.read(&throttle(2)).unwrap().number().unwrap(), 0.0);
    variables
        .dispatch_key_event(0x109d0, Value::Number(16383.0))
        .unwrap();
    assert_eq!(
        variables.read(&throttle(2)).unwrap().number().unwrap(),
        -25.0
    );
    variables
        .dispatch_key_event(0x109ce, Value::Number(-20000.0))
        .unwrap();
    assert_eq!(variables.read(&throttle(1)).unwrap().number().unwrap(), 0.0);
    assert_eq!(variables.read(&throttle(2)).unwrap().number().unwrap(), 0.0);
    assert!(
        variables
            .dispatch_event("AXIS_THROTTLE_REVERSER3_SET", Value::Number(0.0))
            .is_err()
    );
    variables.configure_throttle_limit(0.0).unwrap();
    variables
        .dispatch_event("THROTTLE_FULL", Value::Number(0.0))
        .unwrap();
    assert!(
        variables
            .dispatch_event("AXIS_THROTTLE_REVERSER_SET", Value::Number(16383.0))
            .is_err()
    );
    assert_eq!(
        variables.read(&throttle(1)).unwrap().number().unwrap(),
        100.0
    );
    assert!(variables.configure_throttle_limit(-1.1).is_err());
}

#[test]
fn lateral_trim_events_preserve_axes_and_bound_inputs() {
    let mut variables = Variables::default();
    variables.set_number("AILERON TRIM PCT", "percent", 0.0);
    variables.set_number("RUDDER TRIM PCT", "percent", 0.0);
    variables
        .dispatch_key_event(0x102e4, Value::Number(0.0))
        .unwrap();
    variables
        .dispatch_event("RUDDER_TRIM_RIGHT", Value::Number(0.0))
        .unwrap();
    let aileron = Reference::new('A', "AILERON TRIM PCT", Some("percent"));
    let rudder = Reference::new('A', "RUDDER TRIM PCT", Some("percent"));
    assert_eq!(variables.read(&aileron).unwrap().number().unwrap(), -1.0);
    assert_eq!(variables.read(&rudder).unwrap().number().unwrap(), 1.0);
    variables
        .dispatch_event("AILERON_TRIM_SET", Value::Number(1e9))
        .unwrap();
    assert_eq!(variables.read(&aileron).unwrap().number().unwrap(), 100.0);
    assert!(
        variables
            .dispatch_event("RUDDER_TRIM_SET", Value::Number(f64::NAN))
            .is_err()
    );
    assert_eq!(variables.read(&rudder).unwrap().number().unwrap(), 1.0);
}
use esp_script::Program;

#[test]
fn flap_actuators_use_airspeed_units_and_publish_surface_angles() {
    let config = esp_formats::cfg::Config::parse(
        "[flaps.0]\ntype=1\nextending-time=10\nflaps-position.0=0\nflaps-position.1=30,120",
    )
    .unwrap();
    let mut variables = Variables::default();
    variables
        .configure_flaps(&esp_formats::cfg::aircraft::flaps(&config).unwrap())
        .unwrap();
    variables
        .dispatch_event("FLAPS_DOWN", Value::Number(0.0))
        .unwrap();
    let angle = Reference::new('A', "TRAILING EDGE FLAPS0 LEFT ANGLE", Some("radians"));
    variables.set_number("AIRSPEED INDICATED", "meters per second", 70.0);
    variables.advance(10.0).unwrap();
    assert_eq!(variables.read(&angle).unwrap(), Value::Number(0.0));
    variables.set_number("AIRSPEED INDICATED", "meters per second", 50.0);
    variables.advance(10.0).unwrap();
    assert!(
        (variables.read(&angle).unwrap().number().unwrap() - std::f64::consts::PI / 6.0).abs()
            < 1e-12
    );
    variables.set_number("AIRSPEED INDICATED", "knots", f64::NAN);
    assert!(variables.advance(1.0).is_err());
}

#[test]
fn shares_values_across_compatible_units() {
    let mut variables = Variables::default();
    let read = |variables: &mut Variables, unit| {
        variables
            .read(&Reference::new('A', "BRAKE PARKING POSITION", Some(unit)))
            .unwrap()
            .number()
            .unwrap()
    };
    variables.set_number("BRAKE PARKING POSITION", "percent", 100.0);
    assert_eq!(read(&mut variables, "bool"), 1.0);
    assert_eq!(read(&mut variables, "percent over 100"), 1.0);
    variables
        .dispatch_key_event(0x100d8, Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "percent"), 0.0);
    variables
        .dispatch_event("PARKING_BRAKES", Value::Number(0.0))
        .unwrap();
    assert_eq!(read(&mut variables, "percent"), 100.0);
    assert_eq!(read(&mut variables, "boolean"), 1.0);
    variables.set_number(
        "AUTOPILOT HEADING LOCK DIR",
        "radians",
        std::f64::consts::PI,
    );
    variables
        .dispatch_event("HEADING_BUG_INC", Value::Number(0.0))
        .unwrap();
    assert_eq!(
        variables
            .read(&Reference::new(
                'A',
                "AUTOPILOT HEADING LOCK DIR",
                Some("degrees")
            ))
            .unwrap(),
        Value::Number(181.0)
    );
    assert!(
        variables
            .read(&Reference::new(
                'A',
                "BRAKE PARKING POSITION",
                Some("degrees")
            ))
            .is_err()
    );
    assert!(
        variables
            .read(&Reference::new(
                'A',
                "BRAKE PARKING POSITION",
                Some("unknown")
            ))
            .is_err()
    );
}

#[test]
fn numeric_and_script_events_share_switch_state() {
    for (id, event, name) in [
        (
            0x102c1,
            "TOGGLE_MASTER_BATTERY",
            "ELECTRICAL MASTER BATTERY",
        ),
        (0x102f5, "TOGGLE_AVIONICS_MASTER", "AVIONICS MASTER SWITCH"),
        (0x102bf, "TOGGLE_BEACON_LIGHTS", "LIGHT BEACON"),
        (0x102c0, "TOGGLE_TAXI_LIGHTS", "LIGHT TAXI"),
        (0x1034b, "TOGGLE_NAV_LIGHTS", "LIGHT NAV"),
        (0x10018, "STROBES_TOGGLE", "LIGHT STROBE"),
        (0x100d7, "LANDING_LIGHTS_TOGGLE", "LIGHT LANDING"),
        (0x10142, "PITOT_HEAT_TOGGLE", "PITOT HEAT"),
        (0x100d8, "PARKING_BRAKES", "BRAKE PARKING POSITION"),
        (
            0x102f0,
            "TOGGLE_FLIGHT_DIRECTOR",
            "AUTOPILOT FLIGHT DIRECTOR ACTIVE",
        ),
    ] {
        let mut variables = Variables::default();
        let reference = Reference::new('A', name, Some("bool"));
        variables
            .dispatch_key_event(id, Value::Number(0.0))
            .unwrap();
        assert_eq!(variables.read(&reference).unwrap(), Value::Number(1.0));
        variables.dispatch_event(event, Value::Number(0.0)).unwrap();
        assert_eq!(variables.read(&reference).unwrap(), Value::Number(0.0));
        assert!(
            variables
                .dispatch_key_event(id, Value::Number(f64::NAN))
                .is_err()
        );
        assert_eq!(variables.read(&reference).unwrap(), Value::Number(0.0));
    }
    assert!(
        Variables::default()
            .dispatch_key_event(u32::MAX, Value::Number(0.0))
            .is_err()
    );
}

#[test]
fn separates_local_and_gauge_register_namespaces() {
    let mut variables = Variables::default();
    Program::compile("4 (>G:Var1) 8 (>L:Var1)")
        .unwrap()
        .evaluate(&mut variables)
        .unwrap();
    assert_eq!(
        variables
            .read(&Reference::new('G', "Var1", Some("number")))
            .unwrap(),
        Value::Number(4.0)
    );
    assert_eq!(
        variables.read(&Reference::new('L', "Var1", None)).unwrap(),
        Value::Number(8.0)
    );
    assert_eq!(
        variables.read(&Reference::new('G', "Var2", None)).unwrap(),
        Value::Number(0.0)
    );
    assert!(
        variables
            .read(&Reference::new('A', "MISSING", Some("bool")))
            .is_err()
    );
}

#[test]
fn dispatches_queued_flight_director_events() {
    let mut variables = Variables::default();
    Program::compile("(>K:TOGGLE_FLIGHT_DIRECTOR)")
        .unwrap()
        .evaluate(&mut variables)
        .unwrap();
    let events = variables.drain_events().collect::<Vec<_>>();
    assert_eq!(
        events,
        [("TOGGLE_FLIGHT_DIRECTOR".into(), Value::Number(0.0))]
    );
    for (name, value) in events {
        variables.dispatch_event(&name, value).unwrap();
    }
    let reference = Reference::new('A', "AUTOPILOT FLIGHT DIRECTOR ACTIVE", Some("bool"));
    assert_eq!(variables.read(&reference).unwrap(), Value::Number(1.0));
    variables
        .dispatch_event("TOGGLE_FLIGHT_DIRECTOR", Value::Number(0.0))
        .unwrap();
    assert_eq!(variables.read(&reference).unwrap(), Value::Number(0.0));
    assert_eq!(variables.drain_events().count(), 0);
    assert!(
        variables
            .dispatch_event("UNKNOWN", Value::Number(0.0))
            .is_err()
    );
    assert!(
        variables
            .dispatch_event("TOGGLE_FLIGHT_DIRECTOR", Value::Number(f64::NAN))
            .is_err()
    );
    assert!(
        variables
            .dispatch_event("TOGGLE_FLIGHT_DIRECTOR", Value::Text("1".into()))
            .is_err()
    );
    assert!(variables.write(&reference, Value::Number(1.0)).is_err());
}

#[test]
fn processes_events_even_after_script_failure() {
    let mut variables = Variables::default();
    assert!(
        Program::compile("(>K:TOGGLE_FLIGHT_DIRECTOR) (A:MISSING,bool)")
            .unwrap()
            .evaluate(&mut variables)
            .is_err()
    );
    assert!(variables.dispatch_events().is_empty());
    assert_eq!(variables.drain_events().count(), 0);
    assert_eq!(
        variables
            .read(&Reference::new(
                'A',
                "AUTOPILOT FLIGHT DIRECTOR ACTIVE",
                Some("bool")
            ))
            .unwrap(),
        Value::Number(1.0)
    );
    assert!(variables.dispatch_events().is_empty());
}

#[test]
fn spoiler_handle_events_are_bounded_and_finite() {
    let mut variables = Variables::default();
    let reference = Reference::new('A', "SPOILERS HANDLE POSITION", Some("percent"));
    for (input, expected) in [
        (-1.0, 0.0),
        (0.0, 0.0),
        (8191.5, 50.0),
        (16383.0, 100.0),
        (16384.0, 100.0),
    ] {
        variables
            .dispatch_event("SPOILERS_SET", Value::Number(input))
            .unwrap();
        assert_eq!(variables.read(&reference).unwrap(), Value::Number(expected));
    }
    assert!(
        variables
            .dispatch_event("SPOILERS_SET", Value::Number(f64::NAN))
            .is_err()
    );
    assert_eq!(variables.read(&reference).unwrap(), Value::Number(100.0));
    let armed = Reference::new('A', "SPOILERS ARMED", Some("bool"));
    variables
        .dispatch_key_event(0x10214, Value::Number(1.0))
        .unwrap();
    assert_eq!(variables.read(&armed).unwrap(), Value::Number(1.0));
    variables
        .dispatch_key_event(0x1013d, Value::Number(0.0))
        .unwrap();
    assert_eq!(variables.read(&armed).unwrap(), Value::Number(0.0));
    assert!(
        variables
            .dispatch_event("SPOILERS_ARM_SET", Value::Number(f64::NAN))
            .is_err()
    );
}

#[test]
fn selected_heading_events_wrap_and_preserve_state_on_error() {
    let mut variables = Variables::default();
    let reference = Reference::new('A', "AUTOPILOT HEADING LOCK DIR", Some("degrees"));
    for (event, input, expected) in [
        ("HEADING_BUG_DEC", 0.0, 359.0),
        ("HEADING_BUG_INC", 0.0, 0.0),
        ("HEADING_BUG_SET", 725.0, 5.0),
        ("HEADING_BUG_SET", -10.0, 350.0),
        ("HEADING_BUG_INC", 1.0, 351.0),
    ] {
        variables
            .dispatch_event(event, Value::Number(input))
            .unwrap();
        assert_eq!(variables.read(&reference).unwrap(), Value::Number(expected));
    }
    assert!(
        variables
            .dispatch_event("HEADING_BUG_SET", Value::Number(f64::INFINITY))
            .is_err()
    );
    assert_eq!(variables.read(&reference).unwrap(), Value::Number(351.0));
}
#[test]
fn fuel_selector_events_preserve_index_and_reject_unsupported_routes() {
    let mut variables = Variables::default();
    variables.configure_fuel_selectors(2);
    variables
        .dispatch_key_event(0x101a5, Value::Number(0.0))
        .unwrap();
    variables
        .dispatch_event("FUEL_SELECTOR_2_SET", Value::Number(3.0))
        .unwrap();
    assert_eq!(
        variables
            .read(&Reference::new('A', "FUEL TANK SELECTOR", Some("enum")))
            .unwrap()
            .number()
            .unwrap(),
        2.0
    );
    assert_eq!(
        variables
            .read(&Reference::new('A', "FUEL TANK SELECTOR:2", Some("enum")))
            .unwrap()
            .number()
            .unwrap(),
        3.0
    );
    assert!(
        variables
            .dispatch_event("FUEL_SELECTOR_SET", Value::Number(13.0))
            .is_err()
    );
    assert!(
        variables
            .dispatch_event("FUEL_SELECTOR_3_OFF", Value::Number(0.0))
            .is_err()
    );
}

use crate::autopilot::*;
use crate::variables::Variables;
use esp_script::Host;
use esp_script::Reference;

fn variables() -> Variables {
    let mut variables = Variables::default();
    Autopilot::configure(&mut variables, 1000.0).unwrap();
    variables
}

fn inputs() -> Inputs {
    Inputs {
        heading: 0.0,
        altitude: 1000.0,
        ..Inputs::default()
    }
}

#[test]
fn heading_hold_banks_toward_the_selected_heading_and_respects_the_limit() {
    let mut variables = variables();
    variables.set_number("AUTOPILOT MASTER", "bool", 1.0);
    variables.set_number("AUTOPILOT HEADING LOCK", "bool", 1.0);
    variables.set_number("AUTOPILOT HEADING LOCK DIR", "degrees", 90.0);
    variables.set_number("AUTOPILOT MAX BANK", "degrees", 25.0);
    let mut autopilot = Autopilot::default();
    let output = autopilot.update(0.1, inputs(), &mut variables).unwrap();
    assert!(output.aileron.unwrap() > 0.0);

    let banked = Inputs {
        bank: 25.0,
        roll_rate: 5.0,
        ..inputs()
    };
    let output = autopilot.update(0.1, banked, &mut variables).unwrap();
    assert!(output.aileron.unwrap() <= 0.0);

    variables.set_number("AUTOPILOT HEADING LOCK DIR", "degrees", 350.0);
    let output = autopilot.update(0.1, inputs(), &mut variables).unwrap();
    assert!(output.aileron.unwrap() < 0.0);
}

#[test]
fn heading_hold_uses_magnetic_heading() {
    let mut variables = variables();
    variables.set_number("AUTOPILOT MASTER", "bool", 1.0);
    variables.set_number("AUTOPILOT HEADING LOCK", "bool", 1.0);
    variables.set_number("MAGVAR", "degrees", -10.0);
    variables.set_number("AUTOPILOT HEADING LOCK DIR", "degrees", 350.0);
    let mut autopilot = Autopilot::default();
    let output = autopilot.update(0.1, inputs(), &mut variables).unwrap();
    assert!(output.aileron.unwrap().abs() < 1e-10);
    variables.set_number("AUTOPILOT HEADING LOCK DIR", "degrees", 0.0);
    assert!(
        autopilot
            .update(0.1, inputs(), &mut variables)
            .unwrap()
            .aileron
            .unwrap()
            > 0.0
    );
    variables.set_number("GYRO DRIFT ERROR", "degrees", 10.0);
    assert!(
        autopilot
            .update(0.1, inputs(), &mut variables)
            .unwrap()
            .aileron
            .unwrap()
            .abs()
            < 1e-10
    );
}

#[test]
fn wing_leveler_rolls_back_to_wings_level() {
    let mut variables = variables();
    variables.set_number("AUTOPILOT MASTER", "bool", 1.0);
    variables.set_number("AUTOPILOT WING LEVELER", "bool", 1.0);
    let mut autopilot = Autopilot::default();
    let output = autopilot
        .update(
            0.1,
            Inputs {
                bank: 10.0,
                ..inputs()
            },
            &mut variables,
        )
        .unwrap();
    assert!(output.aileron.unwrap() < 0.0);
    let output = autopilot
        .update(
            0.1,
            Inputs {
                bank: -10.0,
                ..inputs()
            },
            &mut variables,
        )
        .unwrap();
    assert!(output.aileron.unwrap() > 0.0);
}

#[test]
fn altitude_and_vertical_speed_modes_command_the_pitch_axis() {
    let mut variables = variables();
    variables.set_number("AUTOPILOT MASTER", "bool", 1.0);
    variables.set_number("AUTOPILOT ALTITUDE LOCK", "bool", 1.0);
    variables.set_number("AUTOPILOT ALTITUDE LOCK VAR", "meters", 1100.0);
    let mut autopilot = Autopilot::default();
    let output = autopilot.update(0.1, inputs(), &mut variables).unwrap();
    assert!(output.elevator.unwrap() > 0.0);

    variables.set_number("AUTOPILOT ALTITUDE LOCK VAR", "meters", 900.0);
    let output = autopilot.update(0.1, inputs(), &mut variables).unwrap();
    assert!(output.elevator.unwrap() < 0.0);

    variables.set_number("AUTOPILOT ALTITUDE LOCK", "bool", 0.0);
    variables.set_number("AUTOPILOT VERTICAL HOLD", "bool", 1.0);
    variables.set_number("AUTOPILOT VERTICAL HOLD VAR", "meters per second", 3.0);
    let output = autopilot.update(0.1, inputs(), &mut variables).unwrap();
    assert!(output.elevator.unwrap() > 0.0);

    variables.set_number("AUTOPILOT VERTICAL HOLD VAR", "meters per second", -3.0);
    let output = autopilot.update(0.1, inputs(), &mut variables).unwrap();
    assert!(output.elevator.unwrap() < 0.0);
}

#[test]
fn disengaged_or_unpowered_autopilot_does_not_drive_controls() {
    let mut variables = variables();
    variables.set_number("AUTOPILOT HEADING LOCK", "bool", 1.0);
    let mut autopilot = Autopilot::default();
    let output = autopilot.update(0.1, inputs(), &mut variables).unwrap();
    assert_eq!(output.aileron, None);
    assert_eq!(output.elevator, None);

    variables.set_number("AUTOPILOT MASTER", "bool", 1.0);
    variables.set_number("CIRCUIT AUTOPILOT ON", "bool", 0.0);
    let output = autopilot.update(0.1, inputs(), &mut variables).unwrap();
    assert_eq!(output.aileron, None);
    assert_eq!(
        variables
            .read(&Reference::new('A', "AUTOPILOT MASTER", Some("bool")))
            .unwrap()
            .number()
            .unwrap(),
        0.0
    );
}

#[test]
fn selected_values_are_bounded() {
    assert_eq!(selected_altitude_increment(1000.0, 100.0).unwrap(), 1100.0);
    assert_eq!(
        selected_vertical_speed_increment(0.0, -100.0).unwrap(),
        -100.0
    );
    assert!(selected_altitude_meters(f64::NAN).is_err());
    assert!(selected_vertical_speed_meters_per_second(f64::NAN).is_err());
}

use esp_script::{Host, Reference};
use esp_sim::{flight::State, variables::Variables};
use glam::EulerRot;
use std::f64::consts::TAU;

pub fn publish(state: &State, variables: &mut Variables) {
    let (heading, _, _) = state.orientation.to_euler(EulerRot::ZYX);
    let heading = heading.rem_euclid(TAU);
    let ground_speed = state.velocity.truncate().length();
    let track = if ground_speed > 0.5 {
        state.velocity.y.atan2(state.velocity.x).rem_euclid(TAU)
    } else {
        variables
            .read(&Reference::new(
                'A',
                "GPS GROUND TRUE TRACK",
                Some("radians"),
            ))
            .ok()
            .and_then(|value| value.number().ok())
            .unwrap_or(heading)
    };
    let body = state.orientation.conjugate() * state.velocity;
    for (name, value) in [
        ("GPS GROUND SPEED", ground_speed),
        ("HSI SPEED", ground_speed),
        ("VELOCITY WORLD X", state.velocity.y),
        ("VELOCITY WORLD Y", -state.velocity.z),
        ("VELOCITY WORLD Z", state.velocity.x),
        ("VELOCITY BODY X", body.y),
        ("VELOCITY BODY Y", -body.z),
        ("VELOCITY BODY Z", body.x),
    ] {
        variables.set_number(name, "meters per second", value);
    }
    variables.set_number("GPS GROUND TRUE HEADING", "radians", heading);
    variables.set_number("GPS GROUND TRUE TRACK", "radians", track);
    variables.set_number("GPS POSITION ALT", "meters", -state.position.z);
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DQuat, DVec3};

    #[test]
    fn crosswind_separates_heading_from_ground_track() {
        let mut variables = Variables::default();
        variables.set_number("AMBIENT WIND DIRECTION", "degrees", 90.0);
        variables.set_number("AMBIENT WIND VELOCITY", "knots", 30.0);
        let wind = crate::weather::wind(&mut variables).unwrap();
        let mut state = State {
            position: DVec3::new(0.0, 0.0, -2000.0),
            velocity: DVec3::X * 100.0 + wind + DVec3::Z * -5.0,
            orientation: DQuat::IDENTITY,
            ..State::default()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        let read = |vars: &mut Variables, name, unit| {
            vars.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        let track = read(&mut variables, "GPS GROUND TRUE TRACK", "degrees");
        assert!(track > 350.0 && track < 360.0);
        assert_eq!(
            read(&mut variables, "GPS GROUND TRUE HEADING", "degrees"),
            0.0
        );
        assert!(
            (read(&mut variables, "GPS GROUND SPEED", "meters per second")
                - state.velocity.truncate().length())
            .abs()
                < 1e-10
        );
        assert!(
            (read(&mut variables, "VELOCITY WORLD X", "meters per second") - wind.y).abs() < 1e-10
        );
        assert_eq!(
            read(&mut variables, "VELOCITY WORLD Y", "meters per second"),
            5.0
        );
        assert_eq!(read(&mut variables, "GPS POSITION ALT", "meters"), 2000.0);
        assert!(
            (read(&mut variables, "AIRSPEED TRUE", "meters per second") - 100.125).abs() < 0.001
        );
        state.velocity = DVec3::ZERO;
        state.orientation = DQuat::from_rotation_z(1.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!((read(&mut variables, "GPS GROUND TRUE TRACK", "degrees") - track).abs() < 1e-10);
        assert!((read(&mut variables, "GPS GROUND TRUE HEADING", "radians") - 1.0).abs() < 1e-10);
    }
}

#[cfg(test)]
mod navigation {
    use esp_nav::navigation::*;
    use esp_physics::{flight::State, variables::Variables};
    use esp_script::{Host, Reference};
    use glam::EulerRot;
    use std::f64::consts::PI;
    const EARTH_RADIUS_METERS: f64 = 6_371_008.8;
    use esp_script::Value;
    use glam::{DQuat, DVec3};

    #[test]
    fn approach_starts_on_course_and_glideslope_with_crosswind_correction() {
        let wind = DVec3::new(-5.0, 0.0, 0.0);
        let (placement, height) = approach_start(200.0, 300.0, 90.0, 3.0, 5.0, 80.0, wind).unwrap();
        let speed = 80.0 * 1852.0 / 3600.0;
        let mut state = State {
            position: DVec3::new(0.0, 0.0, -height),
            velocity: DVec3::X * speed,
            ..State::default()
        };
        placement.apply(&mut state);
        state.velocity += wind;
        assert!((state.position.x - 200.0).abs() < 1e-9);
        assert!((state.position.y - (300.0 - 5.0 * 1852.0)).abs() < 1e-9);
        assert!((height - 5.0 * 1852.0 * 3_f64.to_radians().tan()).abs() < 1e-9);
        assert!(state.velocity.x.abs() < 1e-9 && state.velocity.y > 0.0);
        assert!((state.velocity.z / state.velocity.y - 3_f64.to_radians().tan()).abs() < 1e-10);
        assert!(((state.velocity - wind).length() - speed).abs() < 1e-9);
        let (heading, _, _) = state.orientation.to_euler(EulerRot::ZYX);
        assert!(heading.to_degrees() < 90.0);
        assert!(approach_start(0.0, 0.0, 90.0, 3.0, 5.0, 80.0, DVec3::X * 100.0).is_err());
        assert!(approach_start(0.0, 0.0, 0.0, 3.0, 0.0, 80.0, DVec3::ZERO).is_err());
    }

    #[test]
    fn initial_placement_rotates_velocity_and_preserves_ground_pitch() {
        let mut state = State {
            position: DVec3::new(0.0, 0.0, -2.0),
            velocity: DVec3::X * 40.0,
            orientation: DQuat::from_rotation_y(0.1),
            ..State::default()
        };
        InitialPlacement::new(-1000.0, 250.0, -90.0)
            .unwrap()
            .apply(&mut state);
        assert_eq!(state.position, DVec3::new(-1000.0, 250.0, -2.0));
        assert!(state.velocity.abs_diff_eq(-DVec3::Y * 40.0, 1e-10));
        let (heading, pitch, _) = state.orientation.to_euler(EulerRot::ZYX);
        assert!((heading + PI / 2.0).abs() < 1e-10);
        assert!((pitch - 0.1).abs() < 1e-10);
        assert!(InitialPlacement::new(f64::NAN, 0.0, 0.0).is_err());
        assert!(InitialPlacement::new(0.0, 0.0, f64::INFINITY).is_err());
    }

    #[test]
    fn slip_ball_follows_specific_force_and_stays_centered_in_coordinated_bank() {
        let mut variables = Variables::default();
        let before = State::default();
        let right_force = State {
            velocity: DVec3::Y * 3.0,
            ..before
        };
        publish_acceleration(&before, &right_force, 1.0, &mut variables).unwrap();
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        let ball = read(&mut variables, "TURN COORDINATOR BALL", "position 128");
        assert!(ball < -100.0 && ball > -127.0);
        assert!(
            (read(&mut variables, "TURN COORDINATOR BALL", "position") - ball / 127.0).abs()
                < 1e-10
        );
        assert_eq!(
            read(&mut variables, "TURN COORDINATOR BALL INV", "position 128"),
            -ball
        );
        variables.set_number("TURN COORDINATOR BALL", "position 128", 0.0);
        let orientation = DQuat::from_rotation_x(30_f64.to_radians());
        let acceleration = orientation * DVec3::new(0.0, 0.0, -9.80665) + DVec3::Z * 9.80665;
        let banked_before = State {
            orientation,
            ..before
        };
        let banked_after = State {
            velocity: acceleration,
            ..banked_before
        };
        publish_acceleration(&banked_before, &banked_after, 1.0, &mut variables).unwrap();
        assert!(read(&mut variables, "TURN COORDINATOR BALL", "position 128").abs() < 1e-9);
        assert!(publish_acceleration(&before, &right_force, f64::NAN, &mut variables).is_err());
    }

    #[test]
    fn publishes_turn_rate_and_roll_response_in_supported_units() {
        let mut variables = Variables::default();
        let state = State {
            angular_velocity: DVec3::new(6_f64.to_radians(), 0.0, 3_f64.to_radians()),
            ..State::default()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        assert!(
            (read(
                &mut variables,
                "TURN INDICATOR RATE:1",
                "degrees per second"
            ) - 3.0)
                .abs()
                < 1e-10
        );
        assert!(
            (read(
                &mut variables,
                "TURN COORDINATOR RATE",
                "degrees per second"
            ) - (3.0 + 6.0 * 30_f64.to_radians().tan()))
            .abs()
                < 1e-10
        );
    }

    #[test]
    fn hsi_follows_selected_gps_source_and_clears_when_switched_off() {
        let mut variables = Variables::default();
        variables.set_number("GPS IS DIRECTTO FLIGHTPLAN", "bool", 1.0);
        variables.set_number("GPS WP NEXT LON", "degrees", 1.0);
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        let mut state = State {
            velocity: DVec3::Y * 100.0,
            ..State::default()
        };
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        state.position.x = 500.0;
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE VALID", "bool"), 1.0);
        assert_eq!(read(&mut variables, "HSI BEARING VALID", "bool"), 1.0);
        assert_eq!(read(&mut variables, "HSI TF FLAGS", "enum"), 1.0);
        let cdi = read(&mut variables, "GPS CDI NEEDLE", "number");
        assert!(cdi > 0.0);
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE", "number"), cdi);
        assert_eq!(read(&mut variables, "NAV CDI:1", "number"), cdi);
        assert_eq!(
            read(&mut variables, "HSI DISTANCE", "meters"),
            read(&mut variables, "GPS WP DISTANCE", "meters")
        );
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE VALID", "bool"), 0.0);
        assert_eq!(read(&mut variables, "NAV CDI:1", "number"), 0.0);
        variables.set_number("NAV CDI:1", "number", 42.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV CDI:1", "number"), 42.0);
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        variables.set_number("GPS IS DIRECTTO FLIGHTPLAN", "bool", 0.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE VALID", "bool"), 0.0);
        assert_eq!(read(&mut variables, "HSI BEARING VALID", "bool"), 0.0);
    }

    #[test]
    fn direct_to_course_tracks_cross_track_and_recaptures_changed_destination() {
        let mut variables = Variables::default();
        variables.set_number("GPS IS DIRECTTO FLIGHTPLAN", "bool", 1.0);
        variables.set_number("GPS WP NEXT LON", "degrees", 1.0);
        variables.set_number("GPS CDI SCALING", "meters", 1000.0);
        let mut state = State {
            velocity: DVec3::Y * 100.0,
            ..State::default()
        };
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!((read(&mut variables, "GPS WP DESIRED TRACK", "degrees") - 90.0).abs() < 1e-8);
        state.position = DVec3::new(500.0, 1000.0, 0.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!((read(&mut variables, "GPS WP CROSS TRK", "meters") + 500.0).abs() < 0.01);
        assert!((read(&mut variables, "GPS CDI NEEDLE", "number") - 63.5).abs() < 0.01);
        state.position.x = -2000.0;
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!((read(&mut variables, "GPS WP CROSS TRK", "meters") - 2000.0).abs() < 0.01);
        assert_eq!(read(&mut variables, "GPS CDI NEEDLE", "number"), -127.0);
        variables.set_number("GPS WP NEXT LAT", "degrees", 1.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!(read(&mut variables, "GPS WP CROSS TRK", "meters").abs() < 1e-6);
        variables.set_number("GPS IS DIRECTTO FLIGHTPLAN", "bool", 0.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "GPS WP PREV VALID", "bool"), 0.0);
        assert_eq!(read(&mut variables, "GPS CDI NEEDLE", "number"), 0.0);
    }

    #[test]
    fn direct_to_updates_distance_bearing_and_time_as_aircraft_moves() {
        let mut variables = Variables::default();
        variables.set_number("WORLD ORIGIN LONGITUDE", "degrees", 179.9);
        variables.set_number("MAGVAR", "degrees", -10.0);
        variables.set_number("GPS IS DIRECTTO FLIGHTPLAN", "bool", 1.0);
        variables.set_number("GPS WP NEXT LAT", "degrees", 0.0);
        variables.set_number("GPS WP NEXT LON", "degrees", -179.9);
        let mut state = State {
            velocity: DVec3::Y * 100.0,
            ..State::default()
        };
        let read = |vars: &mut Variables, name, unit| {
            vars.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        let distance = read(&mut variables, "GPS WP DISTANCE", "meters");
        assert!((distance - EARTH_RADIUS_METERS * 0.2_f64.to_radians()).abs() < 1e-6);
        assert!((read(&mut variables, "GPS WP TRUE BEARING", "degrees") - 90.0).abs() < 1e-9);
        assert!((read(&mut variables, "GPS WP BEARING", "degrees") - 80.0).abs() < 1e-9);
        assert!((read(&mut variables, "GPS WP ETE", "seconds") - distance / 100.0).abs() < 1e-8);
        state.position.y = 1000.0;
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!(
            (read(&mut variables, "GPS WP DISTANCE", "meters") - (distance - 1000.0)).abs() < 1e-6
        );
        variables.set_number("GPS IS DIRECTTO FLIGHTPLAN", "bool", 0.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "GPS IS ACTIVE WAY POINT", "bool"), 0.0);
        assert_eq!(read(&mut variables, "GPS WP DISTANCE", "meters"), 0.0);
    }

    #[test]
    fn gyro_adjustment_moves_indicator_without_moving_compass_and_can_be_synchronized() {
        let mut variables = Variables::default();
        variables.set_number("MAGVAR", "degrees", -10.0);
        let state = State {
            orientation: DQuat::from_rotation_z(5_f64.to_radians()),
            ..State::default()
        };
        let read = |vars: &mut Variables, name| {
            vars.read(&Reference::new('A', name, Some("degrees")))
                .unwrap()
                .number()
                .unwrap()
        };
        variables
            .dispatch_event("GYRO_DRIFT_SET", Value::Number(8.0))
            .unwrap();
        variables
            .dispatch_event("GYRO_DRIFT_DEC", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!((read(&mut variables, "MAGNETIC COMPASS") - 355.0).abs() < 1e-10);
        assert!((read(&mut variables, "HEADING INDICATOR") - 2.0).abs() < 1e-10);
        assert!((read(&mut variables, "GYRO DRIFT ERROR") - 7.0).abs() < 1e-10);
        variables
            .dispatch_event("HEADING_GYRO_SET", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!((read(&mut variables, "PLANE HEADING DEGREES GYRO") - 355.0).abs() < 1e-10);
        assert!(
            variables
                .dispatch_event("GYRO_DRIFT_SET", Value::Number(f64::NAN))
                .is_err()
        );
    }

    #[test]
    fn publishes_magnetic_heading_and_track_with_wraparound() {
        let mut variables = Variables::default();
        variables.set_number("MAGVAR", "degrees", -10.0);
        let state = State {
            orientation: DQuat::from_rotation_z(5_f64.to_radians()),
            velocity: DVec3::new(100.0, 0.0, 0.0),
            ..State::default()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        let read = |vars: &mut Variables, name| {
            vars.read(&Reference::new('A', name, Some("degrees")))
                .unwrap()
                .number()
                .unwrap()
        };
        for name in [
            "PLANE HEADING DEGREES MAGNETIC",
            "MAGNETIC COMPASS",
            "HEADING INDICATOR",
            "PLANE HEADING DEGREES GYRO",
        ] {
            assert!((read(&mut variables, name) - 355.0).abs() < 1e-10);
        }
        assert!((read(&mut variables, "GPS GROUND MAGNETIC TRACK") - 350.0).abs() < 1e-10);
        assert!((read(&mut variables, "GPS MAGVAR") + 10.0).abs() < 1e-10);
        assert!(validate_declination(f64::NAN).is_err());
        assert!(validate_declination(181.0).is_err());
    }

    #[test]
    fn publishes_geographic_position_to_cockpit_variables() {
        let mut variables = Variables::default();
        variables.set_number("WORLD ORIGIN LATITUDE", "degrees", 51.5);
        variables.set_number("WORLD ORIGIN LONGITUDE", "degrees", -0.1);
        let state = State {
            position: DVec3::new(1000.0, 500.0, -2000.0),
            ..State::default()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        let read = |vars: &mut Variables, name| {
            vars.read(&Reference::new('A', name, Some("degrees")))
                .unwrap()
                .number()
                .unwrap()
        };
        let lat = read(&mut variables, "GPS POSITION LAT");
        let lon = read(&mut variables, "GPS POSITION LON");
        assert!(lat > 51.5 && lon > -0.1);
        assert_eq!(lat, read(&mut variables, "PLANE LATITUDE"));
        assert_eq!(lon, read(&mut variables, "PLANE LONGITUDE"));
    }

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

#[cfg(test)]
mod vor {
    use esp_nav::vor::*;
    use esp_physics::flight::State;
    use esp_physics::variables::Variables;
    use esp_script::Value;
    use esp_script::{Host, Reference};
    use glam::DVec3;

    #[test]
    fn receivers_pick_different_stations_and_nearest_matching_signal() {
        let mut variables = Variables::default();
        variables.set_number("VOR STATION ACTIVE", "bool", 1.0);
        variables.set_number("VOR STATION COUNT", "number", 2.0);
        for (name, unit, value) in [
            ("ACTIVE", "bool", 1.0),
            ("LATITUDE", "degrees", -0.08),
            ("LONGITUDE", "degrees", 0.0),
            ("FREQUENCY", "megahertz", 114.0),
            ("RANGE", "nautical miles", 40.0),
            ("LOCALIZER", "bool", 0.0),
        ] {
            variables.set_number(&format!("VOR STATION {name}:2"), unit, value);
        }
        variables.set_number("NAV ACTIVE FREQUENCY:2", "megahertz", 114.0);
        let state = State {
            position: DVec3::new(-10_000.0, 0.0, -1000.0),
            ..State::default()
        };
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        let distance1 = read(&mut variables, "NAV DME:1", "meters");
        let distance2 = read(&mut variables, "NAV DME:2", "meters");
        assert!(distance1 > 10_000.0 && distance2 < 2000.0);
        variables.set_number("VOR STATION FREQUENCY:2", "megahertz", 113.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert!((read(&mut variables, "NAV DME:1", "meters") - distance2).abs() < 1e-6);
        assert_eq!(read(&mut variables, "NAV HAS NAV:2", "bool"), 0.0);
    }

    #[test]
    fn parses_additional_stations() {
        let station: Station = "-33.9,151.2,113.5,40".parse().unwrap();
        assert_eq!(station.latitude, -33.9);
        assert_eq!(station.frequency, 113.5);
        assert!("0,0,113.01,40".parse::<Station>().is_err());
        assert!("0,0,113".parse::<Station>().is_err());
    }

    #[test]
    fn two_receivers_tune_and_select_courses_independently() {
        let mut variables = Variables::default();
        variables.set_number("VOR STATION ACTIVE", "bool", 1.0);
        let state = State {
            position: DVec3::new(-10_000.0, 1000.0, -1000.0),
            ..State::default()
        };
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        variables
            .dispatch_key_event(0x100ad, Value::Number(0x1400 as f64))
            .unwrap();
        variables
            .dispatch_key_event(0x100b5, Value::Number(90.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 1.0);
        assert_eq!(read(&mut variables, "NAV HAS NAV:2", "bool"), 0.0);
        assert_eq!(read(&mut variables, "NAV OBS:1", "degrees"), 0.0);
        assert_eq!(read(&mut variables, "NAV OBS:2", "degrees"), 90.0);
        variables
            .dispatch_event("NAV2_STBY_SET", Value::Number(0x1300 as f64))
            .unwrap();
        variables
            .dispatch_key_event(0x10394, Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:2", "bool"), 1.0);
        let second = read(&mut variables, "NAV CDI:2", "number");
        assert_ne!(read(&mut variables, "NAV CDI:1", "number"), second);
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV CDI:2", "number"), second);
        assert_eq!(
            read(&mut variables, "NAV2 STANDBY FREQUENCY", "megahertz"),
            114.0
        );
    }

    #[test]
    fn legacy_bcd_frequency_round_trips_through_gauge_and_radio_event() {
        let mut variables = Variables::default();
        for (encoded, expected_hz) in [
            (0x1300, 113_000_000.0),
            (0x1302, 113_025_000.0),
            (0x1305, 113_050_000.0),
            (0x1307, 113_075_000.0),
        ] {
            variables
                .dispatch_event("NAV1_RADIO_SET", Value::Number(encoded as f64))
                .unwrap();
            let read = |v: &mut Variables, unit| {
                v.read(&Reference::new('A', "NAV ACTIVE FREQUENCY:1", Some(unit)))
                    .unwrap()
                    .number()
                    .unwrap()
            };
            assert_eq!(read(&mut variables, "hertz"), expected_hz);
            assert_eq!(read(&mut variables, "frequency bcd16"), encoded as f64);
        }
        assert!(
            variables
                .dispatch_event("NAV1_RADIO_SET", Value::Number(0x1a00 as f64))
                .is_err()
        );
    }

    #[test]
    fn ils_provides_localizer_and_glideslope_only_in_coverage() {
        let mut variables = Variables::default();
        variables.set_number("VOR STATION ACTIVE", "bool", 1.0);
        variables.set_number("VOR STATION LOCALIZER", "bool", 1.0);
        variables.set_number("VOR STATION FREQUENCY", "megahertz", 110.3);
        variables.set_number("NAV ACTIVE FREQUENCY:1", "megahertz", 110.3);
        let mut state = State {
            position: DVec3::new(-10_000.0, 100.0, -10_000.0 * 3_f64.to_radians().tan()),
            ..State::default()
        };
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS LOCALIZER:1", "bool"), 1.0);
        assert_eq!(read(&mut variables, "HSI HAS LOCALIZER", "bool"), 1.0);
        assert_eq!(read(&mut variables, "HSI GSI NEEDLE VALID", "bool"), 1.0);
        assert!(read(&mut variables, "NAV CDI:1", "number") < 0.0);
        assert!(read(&mut variables, "NAV GSI:1", "number").abs() < 0.1);
        state.position.z = -10_000.0 * 2_f64.to_radians().tan();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "HSI GSI NEEDLE", "number"), 119.0);
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "HSI GSI NEEDLE VALID", "bool"), 0.0);
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        state.position.y = 10_000.0;
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS LOCALIZER:1", "bool"), 0.0);
        assert_eq!(read(&mut variables, "HSI GSI NEEDLE VALID", "bool"), 0.0);
        assert!(validate_ils(0.0, 3.0, 110.0).is_err());
        assert!(validate_ils(0.0, 0.0, 110.3).is_err());
    }

    #[test]
    fn elevated_station_uses_relative_altitude_for_dme_and_glideslope() {
        let mut variables = Variables::default();
        variables.set_number("VOR STATION ACTIVE", "bool", 1.0);
        variables.set_number("VOR STATION LOCALIZER", "bool", 1.0);
        variables.set_number("VOR STATION FREQUENCY", "megahertz", 110.3);
        variables.set_number("VOR STATION ELEVATION", "meters", 1500.0);
        variables.set_number("NAV ACTIVE FREQUENCY:1", "megahertz", 110.3);
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        let relative_height = 10_000.0 * 3_f64.to_radians().tan();
        let state = State {
            position: DVec3::new(-10_000.0, 0.0, -1500.0 - relative_height),
            ..State::default()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS GLIDE SLOPE:1", "bool"), 1.0);
        assert!(read(&mut variables, "NAV GSI:1", "number").abs() < 1e-5);
        assert!(
            (read(&mut variables, "NAV DME:1", "meters") - 10_000.0_f64.hypot(relative_height))
                .abs()
                < 0.01
        );
        variables.set_number("VOR STATION ELEVATION", "meters", 0.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS GLIDE SLOPE:1", "bool"), 0.0);
        assert!(read(&mut variables, "NAV DME:1", "meters") > 10_000.0_f64.hypot(relative_height));
    }
    #[test]
    fn dme_remains_available_overhead_and_outside_localizer_sector() {
        let mut variables = Variables::default();
        variables.set_number("VOR STATION ACTIVE", "bool", 1.0);
        variables.set_number("VOR STATION LOCALIZER", "bool", 1.0);
        variables.set_number("VOR STATION FREQUENCY", "megahertz", 110.3);
        variables.set_number("NAV ACTIVE FREQUENCY:1", "megahertz", 110.3);
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        let mut state = State {
            position: DVec3::new(-10_000.0, 10_000.0, -500.0),
            ..State::default()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 0.0);
        assert_eq!(read(&mut variables, "NAV HAS DME:1", "bool"), 1.0);
        let distance = read(&mut variables, "NAV DME:1", "meters");
        assert!((distance - (10_000.0_f64.hypot(10_000.0)).hypot(500.0)).abs() < 1.0);
        assert_eq!(read(&mut variables, "HSI DISTANCE", "meters"), distance);
        state.position = DVec3::new(0.0, 0.0, -1000.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 0.0);
        assert_eq!(read(&mut variables, "NAV HAS DME:1", "bool"), 1.0);
        assert!((read(&mut variables, "NAV DME:1", "meters") - 1000.0).abs() < 1e-6);
        variables
            .dispatch_event("NAV1_RADIO_SET_HZ", Value::Number(114_000_000.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS DME:1", "bool"), 0.0);
        assert_eq!(read(&mut variables, "NAV DME:1", "meters"), 0.0);
        assert_eq!(read(&mut variables, "HSI DISTANCE", "meters"), 0.0);
    }

    #[test]
    fn legacy_gauge_event_ids_control_radio_obs_and_gyro() {
        let mut variables = Variables::default();
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        variables
            .dispatch_key_event(0x100b4, Value::Number(90.0))
            .unwrap();
        variables
            .dispatch_key_event(0x1007f, Value::Number(0.0))
            .unwrap();
        variables
            .dispatch_key_event(0x1033f, Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut variables, "NAV OBS:1", "degrees"), 81.0);
        variables
            .dispatch_key_event(0x1038f, Value::Number(0x1400 as f64))
            .unwrap();
        variables
            .dispatch_key_event(0x10390, Value::Number(0.0))
            .unwrap();
        assert_eq!(
            read(&mut variables, "NAV ACTIVE FREQUENCY:1", "megahertz"),
            114.0
        );
        variables
            .dispatch_key_event(0x10347, Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut variables, "GPS DRIVES NAV1", "bool"), 1.0);
        variables
            .dispatch_key_event(0x101fe, Value::Number(10.0))
            .unwrap();
        variables
            .dispatch_key_event(0x10155, Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut variables, "GYRO DRIFT ERROR", "degrees"), 11.0);
        variables
            .dispatch_key_event(0x10020, Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut variables, "GYRO DRIFT ERROR", "degrees"), 0.0);
    }

    #[test]
    fn nav_radio_tuning_and_swap_drive_receiver() {
        let mut variables = Variables::default();
        variables.set_number("VOR STATION ACTIVE", "bool", 1.0);
        let state = State {
            position: DVec3::new(-10_000.0, 0.0, -1000.0),
            ..State::default()
        };
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        variables
            .dispatch_event("NAV1_STBY_SET", Value::Number(0x1400 as f64))
            .unwrap();
        assert_eq!(
            read(&mut variables, "NAV1 STANDBY FREQUENCY", "megahertz"),
            114.0
        );
        variables
            .dispatch_event("NAV1_RADIO_SWAP", Value::Number(0.0))
            .unwrap();
        assert_eq!(
            read(&mut variables, "NAV ACTIVE FREQUENCY:1", "megahertz"),
            114.0
        );
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 0.0);
        variables
            .dispatch_event("NAV1_RADIO_SWAP", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 1.0);
        variables
            .dispatch_event("NAV1_STBY_SET_HZ", Value::Number(117_975_000.0))
            .unwrap();
        variables
            .dispatch_event("NAV1_RADIO_FRACT_INC", Value::Number(0.0))
            .unwrap();
        assert_eq!(
            read(&mut variables, "NAV STANDBY FREQUENCY:1", "hertz"),
            117_000_000.0
        );
        variables
            .dispatch_event("NAV1_RADIO_WHOLE_INC", Value::Number(0.0))
            .unwrap();
        assert_eq!(
            read(&mut variables, "NAV STANDBY FREQUENCY:1", "hertz"),
            108_000_000.0
        );
        variables
            .dispatch_event("NAV1_RADIO_FRACT_DEC_CARRY", Value::Number(0.0))
            .unwrap();
        assert_eq!(
            read(&mut variables, "NAV STANDBY FREQUENCY:1", "hertz"),
            117_950_000.0
        );
        assert!(
            variables
                .dispatch_event("NAV1_STBY_SET", Value::Number(0x1a00 as f64))
                .is_err()
        );
        assert_eq!(
            read(&mut variables, "NAV STANDBY FREQUENCY:1", "hertz"),
            117_950_000.0
        );
    }

    #[test]
    fn vor_receiver_tracks_obs_tuning_range_and_hsi_source() {
        let mut variables = Variables::default();
        variables.set_number("VOR STATION ACTIVE", "bool", 1.0);
        variables
            .dispatch_event("VOR1_SET", Value::Number(360.0))
            .unwrap();
        let mut state = State {
            position: DVec3::new(-10_000.0, 1000.0, -1000.0),
            ..State::default()
        };
        let read = |v: &mut Variables, name, unit| {
            v.read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap()
        };
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 1.0);
        assert_eq!(read(&mut variables, "NAV TOFROM:1", "enum"), 1.0);
        let needle = read(&mut variables, "NAV CDI:1", "number");
        assert!(needle < 0.0 && needle > -127.0);
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE", "number"), needle);
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE VALID", "bool"), 1.0);
        variables.set_number("GPS IS DIRECTTO FLIGHTPLAN", "bool", 1.0);
        variables.set_number("GPS WP NEXT LON", "degrees", 1.0);
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 1.0);
        assert_ne!(read(&mut variables, "NAV CDI:1", "number"), needle);
        variables
            .dispatch_event("TOGGLE_GPS_DRIVES_NAV1", Value::Number(0.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV CDI:1", "number"), needle);
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE", "number"), needle);
        let dme = read(&mut variables, "NAV DME:1", "meters");
        assert!((dme - 10_099.5).abs() < 1.0);
        variables
            .dispatch_event("VOR1_OBI_DEC", Value::Number(0.0))
            .unwrap();
        assert_eq!(read(&mut variables, "NAV OBS:1", "degrees"), 359.0);
        variables
            .dispatch_event("NAV1_RADIO_SET_HZ", Value::Number(114_000_000.0))
            .unwrap();
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 0.0);
        assert_eq!(read(&mut variables, "HSI CDI NEEDLE VALID", "bool"), 0.0);
        variables
            .dispatch_event("NAV1_RADIO_SET_HZ", Value::Number(113_000_000.0))
            .unwrap();
        state.position.x = -100_000.0;
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV HAS NAV:1", "bool"), 0.0);
        state.position = DVec3::new(10_000.0, 0.0, -1000.0);
        crate::flight::Flight::publish(&state, &mut variables).unwrap();
        assert_eq!(read(&mut variables, "NAV TOFROM:1", "enum"), 2.0);
    }

    #[test]
    fn station_configuration_rejects_invalid_coordinates_and_channels() {
        assert!(validate(90.1, 0.0, 113.0, 40.0).is_err());
        assert!(validate(0.0, 0.0, 113.01, 40.0).is_err());
        assert!(validate(0.0, 0.0, 113.0, 0.0).is_err());
        assert!("0,0,113,40,NaN".parse::<Station>().is_err());
        assert_eq!("0,0,113,40".parse::<Station>().unwrap().elevation, 0.0);
        assert_eq!(
            "0,0,113,40,1234".parse::<Station>().unwrap().elevation,
            1234.0
        );
    }
}

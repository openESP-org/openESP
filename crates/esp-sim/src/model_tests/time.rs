use crate::variables::Variables;
use esp_physics::time::*;
use esp_script::{Host, Reference};

#[test]
fn simulation_time_advances_and_utc_wraps_at_midnight() {
    let mut time = SimTime::new(86_399.5).unwrap();
    let mut variables = Variables::default();
    time.publish(&mut variables);
    let read = |variables: &mut Variables, name| {
        variables
            .read(&Reference::new('E', name, Some("seconds")))
            .unwrap()
            .number()
            .unwrap()
    };
    assert_eq!(read(&mut variables, "ZULU TIME"), 86_399.5);
    time.advance(1.0).unwrap();
    assert_eq!(time.unix_seconds(), 86_400.5);
    time.publish(&mut variables);
    assert_eq!(read(&mut variables, "ZULU TIME"), 0.5);
    assert_eq!(read(&mut variables, "SIMULATION TIME"), 1.0);
    time.set_utc(1.0).unwrap();
    assert_eq!(time.unix_seconds(), 86_401.0);
    assert!(time.set_utc(-1.0).is_err());
    assert_eq!(time.unix_seconds(), 86_401.0);
    assert!(time.advance(-1.0).is_err());
    assert!(SimTime::new(86_400.0).is_err());
}

#[test]
fn partitions_render_frames_into_identical_simulation_ticks() {
    for fps in [24, 30, 60, 144, 240] {
        let mut clock = Clock::default();
        let mut total = 0;
        for _ in 0..fps * 10 {
            let frame = clock.advance(1.0 / f64::from(fps)).unwrap();
            assert!((0.0..=1.0).contains(&frame.alpha));
            assert_eq!(frame.discarded, 0.0);
            total += frame.steps;
        }
        assert_eq!(total, 600, "{fps} fps");
        assert!(clock.advance(0.0).unwrap().alpha < 1e-8);
    }
}

#[test]
fn retains_fractional_time_and_bounds_work_after_stalls() {
    let mut clock = Clock::default();
    let frame = clock.advance(STEP * 0.5).unwrap();
    assert_eq!(frame.steps, 0);
    assert!((frame.alpha - 0.5).abs() < 1e-12);
    let frame = clock.advance(1.0).unwrap();
    assert_eq!(frame.steps, 15);
    assert!((frame.alpha - 0.5).abs() < 1e-12);
    assert_eq!(frame.discarded, 0.75);
    assert_eq!(clock.advance(0.0).unwrap().steps, 0);
    for value in [f64::NAN, f64::INFINITY, -1.0] {
        assert!(clock.advance(value).is_err());
    }
    assert!((clock.advance(0.0).unwrap().alpha - 0.5).abs() < 1e-12);
}

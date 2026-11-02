use esp_assets::flight::FlightData;
use esp_sim::{
    aerodynamics::{Aerodynamics, Controls},
    flight::State,
    loading::WeightBalance,
    time::STEP,
};
use glam::{DQuat, DVec3};

#[path = "../../../tests/support/content.rs"]
mod content;

#[test]
#[ignore = "requires original aircraft AIR data and OPENESP_AIRCRAFT_PATHS"]
fn original_aircraft_produce_aerodynamic_forces_and_control_moments() {
    let mut count = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let model = Aerodynamics::from_data(&data).unwrap();
        let loading = WeightBalance::from_config(&data.configuration)
            .unwrap()
            .load(0.0, false)
            .unwrap();
        let flap_count = esp_formats::cfg::aircraft::flaps(&data.configuration)
            .unwrap()
            .len();
        let clean = vec![0.0; flap_count];
        let velocity = DVec3::X * 50.0;
        let forces = |v, omega, controls| {
            model
                .forces(v, omega, 1.225, controls, &clean, loading.cg)
                .unwrap()
        };
        let base = forces(velocity, DVec3::ZERO, Controls::default());
        assert!(base.force.x < 0.0 && base.force.z < 0.0);
        let fast = forces(velocity * 2.0, DVec3::ZERO, Controls::default());
        assert!(fast.force.abs_diff_eq(base.force * 4.0, 1e-8));
        assert!(fast.moment.abs_diff_eq(base.moment * 4.0, 1e-8));
        let thin = model
            .forces(
                velocity,
                DVec3::ZERO,
                0.6125,
                Controls::default(),
                &clean,
                loading.cg,
            )
            .unwrap();
        assert!(thin.force.abs_diff_eq(base.force * 0.5, 1e-8));
        let trim_up = forces(
            velocity,
            DVec3::ZERO,
            Controls {
                elevator_trim: 0.5,
                ..Controls::default()
            },
        );
        let trim_down = forces(
            velocity,
            DVec3::ZERO,
            Controls {
                elevator_trim: -0.5,
                ..Controls::default()
            },
        );
        assert!(trim_up.moment.y > base.moment.y && trim_down.moment.y < base.moment.y);
        assert!(trim_up.force.abs_diff_eq(base.force, 1e-8));
        assert!(
            forces(
                velocity,
                DVec3::ZERO,
                Controls {
                    elevator: 0.5,
                    ..Controls::default()
                }
            )
            .moment
            .y > base.moment.y
        );
        assert!(
            forces(
                velocity,
                DVec3::ZERO,
                Controls {
                    aileron: 0.5,
                    ..Controls::default()
                }
            )
            .moment
            .x > base.moment.x
        );
        assert!(
            forces(
                velocity,
                DVec3::ZERO,
                Controls {
                    rudder: 0.5,
                    ..Controls::default()
                }
            )
            .moment
            .z > base.moment.z
        );
        for axis in [DVec3::X, DVec3::Y, DVec3::Z] {
            let damped = forces(velocity, axis * 0.1, Controls::default());
            assert!(
                (damped.moment - base.moment).dot(axis) < 0.0,
                "{}: damping {axis}",
                path.display()
            );
        }
        let deployed = model
            .forces(
                velocity,
                DVec3::ZERO,
                1.225,
                Controls::default(),
                &vec![20f64.to_radians(); flap_count],
                loading.cg,
            )
            .unwrap();
        assert!(deployed.force.x < base.force.x && deployed.force.z < base.force.z);
        let lift_at = |alpha: f64| {
            let velocity = DVec3::new(alpha.cos(), 0.0, alpha.sin()) * 50.0;
            forces(velocity, DVec3::ZERO, Controls::default())
                .force
                .dot(DVec3::new(alpha.sin(), 0.0, -alpha.cos()))
        };
        assert!(lift_at(16f64.to_radians()) > lift_at(30f64.to_radians()));
        let stopped = forces(DVec3::ZERO, DVec3::splat(0.1), Controls::default());
        assert_eq!(stopped.force, DVec3::ZERO);
        assert_eq!(stopped.moment, DVec3::ZERO);
        eprintln!(
            "{}: mass={}kg force={}N moment={}Nm",
            path.display(),
            loading.mass.kilograms(),
            base.force,
            base.moment
        );
        count += 1;
    }
    assert!(count > 0);
}

#[test]
#[ignore = "requires original aircraft AIR data and OPENESP_AIRCRAFT_PATHS"]
fn unpowered_original_aircraft_integrate_flight_without_creating_energy() {
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let model = Aerodynamics::from_data(&data).unwrap();
        let loading = WeightBalance::from_config(&data.configuration)
            .unwrap()
            .load(0.0, false)
            .unwrap();
        let clean = vec![
            0.0;
            esp_formats::cfg::aircraft::flaps(&data.configuration)
                .unwrap()
                .len()
        ];
        let mut state = State {
            position: DVec3::new(0.0, 0.0, -1000.0),
            velocity: DVec3::X * 75.0,
            orientation: DQuat::from_rotation_y(0.05),
            ..State::default()
        };
        let energy = |state: State| {
            loading.mass.kilograms()
                * (0.5 * state.velocity.length_squared() - 9.80665 * state.position.z)
        };
        let mut previous_energy = energy(state);
        for _ in 0..3600 {
            state
                .advance(&loading.mass, STEP, DVec3::Z * 9.80665, |state| {
                    model.forces(
                        state.air_velocity(DVec3::ZERO),
                        state.angular_velocity,
                        1.225,
                        Controls::default(),
                        &clean,
                        loading.cg,
                    )
                })
                .unwrap();
            let current_energy = energy(state);
            assert!(
                current_energy <= previous_energy + 1e-4,
                "{}: unpowered aircraft gained energy",
                path.display()
            );
            previous_energy = current_energy;
        }
        assert!(state.position.x > 100.0);
        eprintln!(
            "{}: after 60s position={}m velocity={}m/s rates={}rad/s",
            path.display(),
            state.position,
            state.velocity,
            state.angular_velocity
        );
    }
}

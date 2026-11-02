use esp_assets::flight::FlightData;
use esp_sim::{
    flight::State,
    ground::{Controls, LandingGear, Surface},
    loading::WeightBalance,
};
use glam::{DQuat, DVec3};

#[path = "../../../tests/support/content.rs"]
mod content;

#[test]
#[ignore = "requires original wheeled aircraft.cfg and AIR data"]
fn original_aircraft_settle_on_their_configured_landing_gear() {
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let gear = LandingGear::from_data(&data).unwrap();
        assert!(gear.wheel_indices().count() >= 3);
        let weights = WeightBalance::from_config(&data.configuration).unwrap();
        for (fuel, payload) in [(0.0, false), (0.5, true)] {
            let loading = weights.load(fuel, payload).unwrap();
            let pitch = data
                .configuration
                .number("contact_points", "static_pitch")
                .unwrap()
                .unwrap()
                .to_radians();
            let height = data
                .configuration
                .number("contact_points", "static_cg_height")
                .unwrap()
                .unwrap()
                * 0.3048;
            let mut state = State {
                position: DVec3::new(0.0, 0.0, -height),
                orientation: DQuat::from_rotation_y(pitch),
                ..State::default()
            };
            let surface = Surface {
                elevation: 0.0,
                rolling_resistance: 0.02,
                tyre_friction: 0.8,
            };
            for _ in 0..2400 {
                state
                    .advance(&loading.mass, 1.0 / 240.0, DVec3::Z * 9.80665, |s| {
                        Ok(gear
                            .forces(s, loading.cg, surface, Controls::default())?
                            .wrench)
                    })
                    .unwrap();
            }
            let output = gear
                .forces(&state, loading.cg, surface, Controls::default())
                .unwrap();
            assert!(output.on_ground);
            assert!(state.position.z < 0.0, "{path:?}: {state:?}");
            assert!(state.velocity.length() < 0.02, "{path:?}: {state:?}");
            assert!(
                state.angular_velocity.length() < 0.01,
                "{path:?}: {state:?}"
            );
            let weight = loading.mass.kilograms() * 9.80665;
            assert!(
                (-(state.orientation * output.wrench.force).z - weight).abs() / weight < 0.001,
                "{path:?}: support did not balance loaded weight"
            );
            eprintln!(
                "{} fuel={fuel} payload={payload}: height={:.3} m, compression={:.3}, speed={:.6} m/s",
                path.display(),
                -state.position.z,
                output.maximum_compression_ratio,
                state.velocity.length()
            );
            let start = state.position;
            state.velocity = DVec3::X * 15.0;
            let mut coasting = state;
            for _ in 0..2400 {
                coasting
                    .advance(&loading.mass, 1.0 / 240.0, DVec3::Z * 9.80665, |s| {
                        Ok(gear
                            .forces(s, loading.cg, surface, Controls::default())?
                            .wrench)
                    })
                    .unwrap();
                state
                    .advance(&loading.mass, 1.0 / 240.0, DVec3::Z * 9.80665, |s| {
                        Ok(gear
                            .forces(
                                s,
                                loading.cg,
                                surface,
                                Controls {
                                    parking_brake: true,
                                    ..Controls::default()
                                },
                            )?
                            .wrench)
                    })
                    .unwrap();
            }
            assert!(
                state.velocity.length() < 0.02,
                "{path:?}: brakes did not stop the aircraft: {state:?}"
            );
            assert!(
                state.position.x > start.x
                    && state.position.x - start.x < (coasting.position.x - start.x) * 0.5,
                "{path:?}: invalid stopping distance: {state:?}"
            );
            eprintln!(
                "{}: braking stopped 15 m/s motion in {:.3} m",
                path.display(),
                state.position.x - start.x
            );
        }
    }
}

use esp_assets::flight::FlightData;
use esp_physics::{
    aerodynamics::{Aerodynamics, Controls},
    flight::State,
    time::STEP,
};
use esp_sim::loading::WeightBalance;
use glam::{DQuat, DVec3};

#[path = "../../../tests/support/content.rs"]
mod content;

#[test]
#[ignore = "requires original aircraft AIR data and OPENESP_AIRCRAFT_PATHS"]
fn mach_tables_respond_to_local_speed_of_sound() {
    let path = content::aircraft_paths()
        .into_iter()
        .next()
        .expect("aircraft path");
    let mut data = FlightData::load(&path, 0).unwrap();
    let records = esp_formats::air::parse(&data.air).unwrap();
    let mut air = Vec::new();
    for record in &records {
        if ![0x401, 0x154a].contains(&record.id) {
            air.extend(record.id.to_le_bytes());
            air.extend((record.data.len() as u32).to_le_bytes());
            air.extend(record.data);
        }
    }
    for (id, values) in [
        (0x401u32, [0.0f64, 1.0, 1.0, 2.0]),
        (0x154au32, [0.0, 0.0, 1.0, 0.1]),
    ] {
        air.extend(id.to_le_bytes());
        air.extend(36u32.to_le_bytes());
        air.extend(2u32.to_le_bytes());
        for value in values {
            air.extend(value.to_le_bytes());
        }
    }
    data.air = air;
    let model = esp_sim::loading::aerodynamic_model(&data).unwrap();
    let loading = WeightBalance::from_config(&data.configuration)
        .unwrap()
        .load(0.0, false)
        .unwrap();
    let flaps = vec![
        0.0;
        esp_formats::cfg::aircraft::flaps(&data.configuration)
            .unwrap()
            .iter()
            .filter(|f| f.kind != 0)
            .count()
    ];
    let sample = |sound_speed| {
        model
            .forces(
                DVec3::new(100.0, 0.0, 5.0),
                DVec3::ZERO,
                1.225,
                Controls {
                    sound_speed: Some(sound_speed),
                    ..Controls::default()
                },
                &flaps,
                loading.cg,
            )
            .unwrap()
    };
    let warm = sample(400.0);
    let cold = sample(200.0);
    assert!(cold.force.z < warm.force.z);
    assert!(cold.force.x < warm.force.x);
    assert!(
        model
            .forces(
                DVec3::X,
                DVec3::ZERO,
                1.225,
                Controls {
                    sound_speed: Some(0.0),
                    ..Controls::default()
                },
                &flaps,
                loading.cg
            )
            .is_err()
    );
}

#[test]
#[ignore = "requires original aircraft AIR data and OPENESP_AIRCRAFT_PATHS"]
fn angle_of_attack_tables_change_roll_and_yaw_at_the_authored_angle() {
    let path = content::aircraft_paths()
        .into_iter()
        .next()
        .expect("aircraft path");
    let mut data = FlightData::load(&path, 0).unwrap();
    let records = esp_formats::air::parse(&data.air).unwrap();
    let ids = [0x451u32, 0x456, 0x460, 0x464];
    let mut base = Vec::new();
    for record in &records {
        if !ids.contains(&record.id) {
            base.extend(record.id.to_le_bytes());
            base.extend((record.data.len() as u32).to_le_bytes());
            base.extend(record.data);
        }
    }
    data.air = base.clone();
    let reference = esp_sim::loading::aerodynamic_model(&data).unwrap();
    let loading = WeightBalance::from_config(&data.configuration)
        .unwrap()
        .load(0.0, false)
        .unwrap();
    let flaps = vec![
        0.0;
        esp_formats::cfg::aircraft::flaps(&data.configuration)
            .unwrap()
            .iter()
            .filter(|flap| flap.kind != 0)
            .count()
    ];
    let sample = |model: &Aerodynamics, alpha: f64| {
        model
            .forces(
                DVec3::new(50.0, 5.0, 50.0 * alpha.to_radians().tan()),
                DVec3::new(0.2, 0.0, 0.2),
                1.225,
                Controls::default(),
                &flaps,
                loading.cg,
            )
            .unwrap()
    };
    let low = sample(&reference, 0.0);
    let high = sample(&reference, 10.0);
    for (index, id) in ids.into_iter().enumerate() {
        data.air = base.clone();
        data.air.extend(id.to_le_bytes());
        data.air.extend(36u32.to_le_bytes());
        data.air.extend(2u32.to_le_bytes());
        for value in [0.0f64, 1.0, 10.0, 0.0] {
            data.air.extend(value.to_le_bytes());
        }
        let model = esp_sim::loading::aerodynamic_model(&data).unwrap();
        let same = sample(&model, 0.0);
        let changed = sample(&model, 10.0);
        assert!(same.moment.abs_diff_eq(low.moment, 1e-8));
        let delta = changed.moment - high.moment;
        if index < 2 {
            assert!(delta.x.abs() > 1e-3, "record {id:#x} did not affect roll");
        } else {
            assert!(delta.z.abs() > 1e-3, "record {id:#x} did not affect yaw");
        }
    }
}

#[test]
#[ignore = "requires original aircraft AIR data and OPENESP_AIRCRAFT_PATHS"]
fn sdk_aerodynamic_blocks_drive_the_same_forces_as_legacy_coefficients() {
    let path = content::aircraft_paths()
        .into_iter()
        .next()
        .expect("aircraft path");
    let mut data = FlightData::load(&path, 0).unwrap();
    let legacy = esp_sim::loading::aerodynamic_model(&data).unwrap();
    let records = esp_formats::air::parse(&data.air).unwrap();
    let coefficients = records
        .iter()
        .find(|r| r.id == 0x1101)
        .unwrap()
        .legacy_aerodynamics()
        .unwrap()
        .unwrap();
    let mut modern = Vec::new();
    for record in &records {
        if record.id != 0x1101 {
            modern.extend(record.id.to_le_bytes());
            modern.extend((record.data.len() as u32).to_le_bytes());
            modern.extend(record.data);
        }
    }
    let c = coefficients;
    let blocks: &[(u32, &[f64])] = &[
        (
            0x1539,
            &[
                c.lift.cl_spoiler,
                c.lift.cl_flaps,
                c.lift.unused0,
                c.lift.cl_ih,
                c.lift.cl_de,
                c.lift.unused1,
            ],
        ),
        (
            0x1540,
            &[
                c.drag.cd0,
                c.drag.cd_flaps,
                c.drag.cd_gear,
                c.drag.cd_spoiler,
            ],
        ),
        (
            0x1541,
            &[
                c.pitch.cm_ih,
                c.pitch.cm_de,
                c.pitch.cm_de_propwash,
                c.pitch.cl_q,
                c.pitch.cl_adot,
                c.pitch.cm_adot,
                c.pitch.cm_q,
                c.pitch.cm_q_propwash,
                c.pitch.cm0,
                c.pitch.cm_flap,
                c.pitch.cm_delta_trim,
                c.pitch.cm_gear,
                c.pitch.cm_spoiler,
            ],
        ),
        (
            0x1542,
            &[
                c.side.cy_beta,
                c.side.cy_p,
                c.side.cy_r,
                c.side.cy_delta_rudder,
            ],
        ),
        (
            0x1543,
            &[
                c.roll.cl_beta,
                c.roll.cl_p,
                c.roll.cl_r,
                c.roll.cl_delta_spoiler,
                c.roll.cl_delta_aileron,
                c.roll.cl_delta_rudder,
            ],
        ),
        (
            0x1544,
            &[
                c.yaw.cn_beta,
                c.yaw.cn_p,
                c.yaw.cn_r,
                c.yaw.cn_r_propwash,
                c.yaw.unused[0],
                c.yaw.unused[1],
                c.yaw.cn_delta_aileron,
                c.yaw.cn_delta_rudder,
                c.yaw.cn_delta_rudder_propwash,
            ],
        ),
    ];
    for (id, values) in blocks {
        modern.extend(id.to_le_bytes());
        modern.extend(((values.len() * 8) as u32).to_le_bytes());
        for value in *values {
            modern.extend(value.to_le_bytes());
        }
    }
    data.air = modern;
    let model = esp_sim::loading::aerodynamic_model(&data).unwrap();
    let loading = WeightBalance::from_config(&data.configuration)
        .unwrap()
        .load(0.0, false)
        .unwrap();
    let flaps = vec![
        0.0;
        esp_formats::cfg::aircraft::flaps(&data.configuration)
            .unwrap()
            .iter()
            .filter(|flap| flap.kind != 0)
            .count()
    ];
    let sample = |model: &Aerodynamics| {
        model
            .forces(
                DVec3::new(50.0, 3.0, 4.0),
                DVec3::new(0.03, 0.05, -0.02),
                1.225,
                Controls {
                    elevator: 0.2,
                    aileron: -0.3,
                    rudder: 0.1,
                    ..Controls::default()
                },
                &flaps,
                loading.cg,
            )
            .unwrap()
    };
    let expected = sample(&legacy);
    let actual = sample(&model);
    assert!(actual.force.abs_diff_eq(expected.force, 1e-9));
    assert!(actual.moment.abs_diff_eq(expected.moment, 1e-9));
    let drag_offset = esp_formats::air::parse(&data.air)
        .unwrap()
        .iter()
        .find(|record| record.id == 0x1540)
        .unwrap()
        .offset
        + 8;
    data.air[drag_offset..drag_offset + 8].copy_from_slice(&(c.drag.cd0 + 0.1).to_le_bytes());
    let increased_drag = sample(&esp_sim::loading::aerodynamic_model(&data).unwrap());
    assert!(increased_drag.force.x < actual.force.x);
}

#[test]
#[ignore = "requires original aircraft AIR data and OPENESP_AIRCRAFT_PATHS"]
fn authored_control_scaling_and_elasticity_change_control_response() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let mut data = FlightData::load(&path, 0).unwrap();
        let records = esp_formats::air::parse(&data.air).unwrap();
        let wanted = [0x341, 0x342, 0x343, 0x517, 0x518, 0x519];
        let mut ranges = Vec::new();
        for id in wanted {
            let Some(record) = records.iter().find(|record| record.id == id) else {
                continue;
            };
            let Some(esp_formats::air::table::Table::Curve(curve)) = record.table().unwrap() else {
                continue;
            };
            ranges.push((id, record.offset + 8, curve.points));
        }
        let trim_record = records
            .iter()
            .find(|record| record.id == 0x1525)
            .map(|record| (record.offset + 8, record.table().unwrap()))
            .and_then(|(offset, table)| match table {
                Some(esp_formats::air::table::Table::Curve(curve)) => {
                    Some((offset, curve.points.len()))
                }
                _ => None,
            });
        if ranges.len() != wanted.len() {
            continue;
        }
        for (id, start, points) in &ranges {
            for (index, _) in points.iter().enumerate() {
                let scalar: f64 = if *id <= 0x343 { 0.0 } else { 1.0 };
                let offset = start + 4 + index * 16 + 8;
                data.air[offset..offset + 8].copy_from_slice(&scalar.to_le_bytes());
            }
        }
        let model = esp_sim::loading::aerodynamic_model(&data).unwrap();
        let loading = WeightBalance::from_config(&data.configuration)
            .unwrap()
            .load(0.0, false)
            .unwrap();
        let flaps = vec![
            0.0;
            esp_formats::cfg::aircraft::flaps(&data.configuration)
                .unwrap()
                .iter()
                .filter(|flap| flap.kind != 0)
                .count()
        ];
        let sample = |controls| {
            model
                .forces(
                    DVec3::X * 50.0,
                    DVec3::ZERO,
                    1.225,
                    controls,
                    &flaps,
                    loading.cg,
                )
                .unwrap()
        };
        let baseline = sample(Controls::default());
        let deflected = sample(Controls {
            elevator: 0.5,
            aileron: 0.5,
            rudder: 0.5,
            ..Controls::default()
        });
        assert!(baseline.force.abs_diff_eq(deflected.force, 1e-9));
        assert!(baseline.moment.abs_diff_eq(deflected.moment, 1e-9));
        for (id, start, points) in &ranges {
            for (index, _) in points.iter().enumerate() {
                let scalar: f64 = if *id == 0x517 && index > 0 { 0.0 } else { 1.0 };
                let offset = start + 4 + index * 16 + 8;
                data.air[offset..offset + 8].copy_from_slice(&scalar.to_le_bytes());
            }
        }
        let model = esp_sim::loading::aerodynamic_model(&data).unwrap();
        let pressure =
            ranges.iter().find(|(id, _, _)| *id == 0x517).unwrap().2[1].input * 47.88025898033584;
        assert!(pressure > 0.0);
        let speed = (2.0 * pressure / 1.225).sqrt();
        let baseline = model
            .forces(
                DVec3::X * speed,
                DVec3::ZERO,
                1.225,
                Controls::default(),
                &flaps,
                loading.cg,
            )
            .unwrap();
        let deflected = model
            .forces(
                DVec3::X * speed,
                DVec3::ZERO,
                1.225,
                Controls {
                    elevator: 0.5,
                    ..Controls::default()
                },
                &flaps,
                loading.cg,
            )
            .unwrap();
        assert!(baseline.force.abs_diff_eq(deflected.force, 1e-8));
        assert!(baseline.moment.abs_diff_eq(deflected.moment, 1e-8));
        if let Some((start, count)) = trim_record {
            let trim = Controls {
                elevator_trim: 0.5,
                ..Controls::default()
            };
            let before = model
                .forces(
                    DVec3::X * speed,
                    DVec3::ZERO,
                    1.225,
                    trim,
                    &flaps,
                    loading.cg,
                )
                .unwrap();
            assert!((before.moment.y - baseline.moment.y).abs() > 1.0);
            for index in 0..count {
                let offset = start + 4 + index * 16 + 8;
                data.air[offset..offset + 8].copy_from_slice(&0.0f64.to_le_bytes());
            }
            let disabled = esp_sim::loading::aerodynamic_model(&data)
                .unwrap()
                .forces(
                    DVec3::X * speed,
                    DVec3::ZERO,
                    1.225,
                    trim,
                    &flaps,
                    loading.cg,
                )
                .unwrap();
            assert!(disabled.moment.abs_diff_eq(baseline.moment, 1e-8));
        }
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original aircraft AIR data and OPENESP_AIRCRAFT_PATHS"]
fn authored_ground_effect_changes_lift_by_height_above_the_surface() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let records = esp_formats::air::parse(&data.air).unwrap();
        let Some(record) = records.iter().find(|r| r.id == 0x400) else {
            continue;
        };
        let Some(esp_formats::air::table::Table::Curve(curve)) = record.table().unwrap() else {
            panic!("invalid ground effect curve")
        };
        let model = esp_sim::loading::aerodynamic_model(&data).unwrap();
        let loading = WeightBalance::from_config(&data.configuration)
            .unwrap()
            .load(0.0, false)
            .unwrap();
        let flap_count = esp_formats::cfg::aircraft::flaps(&data.configuration)
            .unwrap()
            .iter()
            .filter(|f| f.kind != 0)
            .count();
        let flaps = vec![0.0; flap_count];
        let span = data
            .configuration
            .number("airplane_geometry", "wing_span")
            .unwrap()
            .unwrap()
            * 0.3048;
        let lift = |height| {
            model
                .forces(
                    DVec3::X * 50.0,
                    DVec3::ZERO,
                    1.225,
                    Controls {
                        ground_height: Some(height),
                        ..Controls::default()
                    },
                    &flaps,
                    loading.cg,
                )
                .unwrap()
                .force
                .z
        };
        let low = lift(0.0);
        let high = lift(span * 2.0);
        let first = curve.points.first().unwrap().output;
        let last = curve.points.last().unwrap().output;
        assert!(first > last && high < 0.0 && low < high);
        assert!((low / high - first / last).abs() < 1e-10);
        let unbounded = model
            .forces(
                DVec3::X * 50.0,
                DVec3::ZERO,
                1.225,
                Controls::default(),
                &flaps,
                loading.cg,
            )
            .unwrap();
        assert!((unbounded.force.z - high).abs() < 1e-8);
        let state = State {
            position: DVec3::new(0.0, 0.0, -100.0),
            ..State::default()
        };
        let above = model.wing_height(&state, loading.cg, 0.0);
        assert!((model.wing_height(&state, loading.cg, 5.0) - (above - 5.0)).abs() < 1e-10);
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original aircraft AIR data and OPENESP_AIRCRAFT_PATHS"]
fn original_aircraft_produce_aerodynamic_forces_and_control_moments() {
    let mut count = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let model = esp_sim::loading::aerodynamic_model(&data).unwrap();
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
        let contact_points =
            esp_formats::cfg::contact::ContactPoints::from_config(&data.configuration).unwrap();
        if contact_points
            .points
            .iter()
            .any(|point| point.class == 1 && point.retraction_seconds > 0.0)
        {
            let down = forces(
                velocity,
                DVec3::ZERO,
                Controls {
                    gear: 1.0,
                    ..Controls::default()
                },
            );
            assert!(
                down.force.x < base.force.x,
                "{}: gear did not increase drag",
                path.display()
            );
        }
        // Hold Mach constant while doubling dynamic pressure's velocity term.
        let fast = forces(
            velocity * 2.0,
            DVec3::ZERO,
            Controls {
                sound_speed: Some(2.0 * 340.293_988),
                ..Controls::default()
            },
        );
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
        let model = esp_sim::loading::aerodynamic_model(&data).unwrap();
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

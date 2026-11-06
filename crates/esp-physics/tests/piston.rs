use esp_assets::flight::FlightData;
use esp_formats::air;
use esp_physics::{
    atmosphere::Atmosphere,
    piston::{Charge, Command, PistonCycle, Pistons},
    propeller::Propeller,
};

#[path = "../../../tests/support/content.rs"]
mod content;

fn scale_curve(data: &mut FlightData, id: u32, scale: f64) {
    let offset = air::parse(&data.air)
        .unwrap()
        .iter()
        .find(|r| r.id == id)
        .unwrap()
        .offset;
    let count = u32::from_le_bytes(data.air[offset + 8..offset + 12].try_into().unwrap()) as usize;
    for point in 0..count {
        let at = offset + 20 + point * 16;
        let value = f64::from_le_bytes(data.air[at..at + 8].try_into().unwrap());
        data.air[at..at + 8].copy_from_slice(&(value * scale).to_le_bytes());
    }
}

#[test]
#[ignore = "requires original fixed-pitch piston aircraft and their AIR engine/propeller tables"]
fn original_piston_tables_drive_cycle_torque_and_propeller_equilibrium() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(0.0)
            || data
                .configuration
                .number("propeller", "propeller_type")
                .unwrap()
                != Some(1.0)
        {
            continue;
        }
        let air = Atmosphere::standard(0.0).unwrap();
        let charge = Charge {
            pressure: air.pressure,
            temperature: air.temperature,
            heat_release: 0.083 * 43_500_000.0,
            oil_viscosity: 1.0,
        };
        let cycle = PistonCycle::from_data(&data).unwrap();
        let rated_rpm = data
            .configuration
            .number("piston_engine", "max_rated_rpm")
            .unwrap()
            .unwrap();
        let reference = cycle.sample(rated_rpm, charge).unwrap();
        assert!(
            reference.shaft_power > 0.0 && reference.best_power_fuel_kilograms_per_second > 0.0
        );
        let mut changed = FlightData::load(&path, 0).unwrap();
        scale_curve(&mut changed, 0x508, 0.5);
        let reduced = PistonCycle::from_data(&changed)
            .unwrap()
            .sample(rated_rpm, charge)
            .unwrap();
        assert_eq!(reduced.gross_torque, reference.gross_torque * 0.5);
        assert_eq!(reduced.friction_torque, reference.friction_torque);
        let mut changed = FlightData::load(&path, 0).unwrap();
        scale_curve(&mut changed, 0x509, 2.0);
        let friction = PistonCycle::from_data(&changed)
            .unwrap()
            .sample(rated_rpm, charge)
            .unwrap();
        assert_eq!(friction.friction_torque, reference.friction_torque * 2.0);
        assert_eq!(friction.gross_torque, reference.gross_torque);
        let propeller = Propeller::from_data(&data).unwrap();
        let mut lo = rated_rpm * 0.1;
        let mut hi = rated_rpm * 1.5;
        for _ in 0..50 {
            let rpm = (lo + hi) * 0.5;
            let net = cycle.sample(rpm, charge).unwrap().shaft_torque
                - propeller.sample(&air, rpm, 0.0).unwrap().absorbed_torque;
            if net > 0.0 {
                lo = rpm;
            } else {
                hi = rpm;
            }
        }
        let rpm = (lo + hi) * 0.5;
        let engine = cycle.sample(rpm, charge).unwrap();
        let prop = propeller.sample(&air, rpm, 0.0).unwrap();
        assert!((engine.shaft_torque - prop.absorbed_torque).abs() < 1e-6);
        assert!((engine.shaft_power - prop.absorbed_power).abs() < 1e-4);
        assert!(rpm > rated_rpm * 0.5 && rpm < rated_rpm);
        eprintln!(
            "{}: rated-RPM cycle output {:.3} W; static propeller equilibrium {:.3} RPM, {:.3} W, {:.3} N thrust",
            path.display(),
            reference.shaft_power,
            rpm,
            engine.shaft_power,
            prop.thrust
        );
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original fixed-pitch piston aircraft and AIR engine/propeller tables"]
fn original_piston_shaft_responds_to_controls_load_inertia_and_altitude() {
    use glam::DVec3;
    let sea = Atmosphere::standard(0.0).unwrap();
    let high = Atmosphere::standard(4000.0).unwrap();
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(0.0)
            || data
                .configuration
                .number("propeller", "propeller_type")
                .unwrap()
                != Some(1.0)
        {
            continue;
        }
        let mut engine = Pistons::from_data(&data).unwrap();
        engine.start_running(&sea, DVec3::ZERO).unwrap();
        let idle = engine.output(0, &sea, DVec3::ZERO).unwrap();
        assert!(idle.combustion && idle.rpm > 300.0 && idle.rpm < 1500.0);
        assert!((idle.cycle.shaft_torque - idle.propeller.absorbed_torque).abs() < 1e-6);
        let mut commands = engine
            .indices()
            .map(|_| Command {
                throttle: 1.0,
                ..Command::default()
            })
            .collect::<Vec<_>>();
        let mut heavy_data = FlightData::load(&path, 0).unwrap();
        let config = String::from_utf8(std::fs::read(path.join("aircraft.cfg")).unwrap()).unwrap();
        // Change only the in-memory inertia definition, retaining all authored tables.
        let old = data
            .configuration
            .number("propeller", "propeller_moi")
            .unwrap()
            .unwrap();
        let config = config
            .lines()
            .map(|line| {
                if line
                    .split('=')
                    .next()
                    .unwrap_or("")
                    .trim()
                    .eq_ignore_ascii_case("propeller_moi")
                {
                    format!("propeller_moi={}", old * 4.0)
                } else {
                    line.into()
                }
            })
            .collect::<Vec<_>>()
            .join("\n");
        heavy_data.configuration = esp_formats::cfg::Config::parse(&config).unwrap();
        let mut heavy = Pistons::from_data(&heavy_data).unwrap();
        heavy.start_running(&sea, DVec3::ZERO).unwrap();
        engine
            .advance(0.1, &sea, DVec3::ZERO, &commands, true)
            .unwrap();
        heavy
            .advance(0.1, &sea, DVec3::ZERO, &commands, true)
            .unwrap();
        let response = engine.output(0, &sea, DVec3::ZERO).unwrap();
        let slow = heavy.output(0, &sea, DVec3::ZERO).unwrap();
        assert!(response.rpm > slow.rpm && slow.rpm > idle.rpm);
        for _ in 0..240 {
            engine
                .advance(0.25, &sea, DVec3::ZERO, &commands, true)
                .unwrap();
            heavy
                .advance(0.25, &sea, DVec3::ZERO, &commands, true)
                .unwrap();
        }
        let full = engine.output(0, &sea, DVec3::ZERO).unwrap();
        let steady_heavy = heavy.output(0, &sea, DVec3::ZERO).unwrap();
        assert!(
            // Different spool-up histories leave slightly different oil temperatures.
            (full.rpm - steady_heavy.rpm).abs() < 0.2,
            "normal={}, heavy={}",
            full.rpm,
            steady_heavy.rpm
        );
        assert!(full.rpm > response.rpm && full.propeller.thrust > idle.propeller.thrust);
        assert!((full.cycle.shaft_torque - full.propeller.absorbed_torque).abs() < 0.05);
        let windmilling = engine.output(0, &sea, DVec3::X * 120.0).unwrap();
        assert!(windmilling.propeller.thrust < 0.0 && windmilling.propeller.absorbed_torque < 0.0);
        // Full rich becomes excessively rich at altitude; leaning recovers power.
        let rich = engine.output(0, &high, DVec3::ZERO).unwrap();
        for command in &mut commands {
            command.mixture = 0.45;
        }
        engine
            .advance(0.0, &high, DVec3::ZERO, &commands, true)
            .unwrap();
        let lean = engine.output(0, &high, DVec3::ZERO).unwrap();
        assert!(lean.cycle.gross_torque > rich.cycle.gross_torque);
        assert!(lean.cycle.gross_torque < full.cycle.gross_torque);
        for command in &mut commands {
            command.mixture = 0.0;
        }
        engine
            .advance(0.0, &sea, DVec3::ZERO, &commands, true)
            .unwrap();
        let cutoff = engine.output(0, &sea, DVec3::ZERO).unwrap();
        assert_eq!(cutoff.rpm, full.rpm);
        assert!(!cutoff.combustion);
        assert_eq!(cutoff.fuel_kilograms_per_second, 0.0);
        for _ in 0..120 {
            engine
                .advance(0.25, &sea, DVec3::ZERO, &commands, true)
                .unwrap();
        }
        assert!(engine.output(0, &sea, DVec3::ZERO).unwrap().rpm < 1.0);
        eprintln!(
            "{}: idle {:.3} RPM; full {:.3} RPM, {:.3} N, {:.6} kg/s; 0.1 s normal/heavy {:.3}/{:.3} RPM",
            path.display(),
            idle.rpm,
            full.rpm,
            full.propeller.thrust,
            full.fuel_kilograms_per_second,
            response.rpm,
            slow.rpm
        );
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original fixed-pitch piston aircraft and AIR engine/propeller tables"]
fn original_piston_oil_indications_follow_engine_state() {
    use glam::DVec3;
    let air = Atmosphere::standard(0.0).unwrap();
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(0.0)
            || data
                .configuration
                .number("propeller", "propeller_type")
                .unwrap()
                != Some(1.0)
        {
            continue;
        }
        let mut engine = Pistons::from_data(&data).unwrap();
        let cold = engine.output(0, &air, DVec3::ZERO).unwrap();
        assert_eq!(cold.oil_pressure_psi, 0.0);
        assert!((cold.exhaust_temperature_c - (air.temperature - 273.15)).abs() < 1e-9);
        assert!((cold.oil_temperature_c - (air.temperature - 273.15)).abs() < 1e-9);
        let starter = engine
            .indices()
            .map(|_| Command {
                starter: true,
                ..Command::default()
            })
            .collect::<Vec<_>>();
        engine
            .advance(0.1, &air, DVec3::ZERO, &starter, false)
            .unwrap();
        let cranking = engine.output(0, &air, DVec3::ZERO).unwrap();
        assert!(cranking.rpm > 0.0);
        let reference_cycle = PistonCycle::from_data(&data).unwrap();
        let reference = reference_cycle
            .sample(
                cranking.rpm,
                Charge {
                    pressure: air.pressure,
                    temperature: air.temperature,
                    heat_release: 0.0,
                    oil_viscosity: 1.0,
                },
            )
            .unwrap();
        assert!(cranking.cycle.friction_torque > reference.friction_torque);
        engine.start_running(&air, DVec3::ZERO).unwrap();
        let running = engine.output(0, &air, DVec3::ZERO).unwrap();
        assert!(running.oil_pressure_psi > 0.0);
        let warm_reference = reference_cycle
            .sample(
                running.rpm,
                Charge {
                    pressure: air.pressure,
                    temperature: air.temperature,
                    heat_release: 0.0,
                    oil_viscosity: 1.0,
                },
            )
            .unwrap();
        let cold_factor = cranking.cycle.friction_torque / reference.friction_torque;
        let warm_factor = running.cycle.friction_torque / warm_reference.friction_torque;
        assert!(cold_factor > warm_factor);
        assert!(running.exhaust_temperature_c > cold.exhaust_temperature_c);
        assert!(running.oil_temperature_c > cold.oil_temperature_c);
        let lean_commands = engine
            .indices()
            .map(|_| Command {
                mixture: 0.45,
                ..Command::default()
            })
            .collect::<Vec<_>>();
        engine
            .advance(0.0, &air, DVec3::ZERO, &lean_commands, true)
            .unwrap();
        let immediate = engine.output(0, &air, DVec3::ZERO).unwrap();
        assert_eq!(
            immediate.exhaust_temperature_c,
            running.exhaust_temperature_c
        );
        for _ in 0..50 {
            engine
                .advance(0.1, &air, DVec3::ZERO, &lean_commands, true)
                .unwrap();
        }
        let leaned = engine.output(0, &air, DVec3::ZERO).unwrap();
        assert!(leaned.exhaust_temperature_c > running.exhaust_temperature_c);
        let commands = engine
            .indices()
            .map(|_| Command::default())
            .collect::<Vec<_>>();
        for _ in 0..100 {
            engine
                .advance(0.1, &air, DVec3::ZERO, &commands, false)
                .unwrap();
        }
        let stopped = engine.output(0, &air, DVec3::ZERO).unwrap();
        assert!(stopped.oil_pressure_psi < running.oil_pressure_psi);
        assert!(stopped.exhaust_temperature_c > cold.exhaust_temperature_c);
        assert!(stopped.exhaust_temperature_c < leaned.exhaust_temperature_c);
        assert!(stopped.oil_temperature_c < running.oil_temperature_c);
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original fixed-pitch piston aircraft and AIR engine/propeller tables"]
fn original_piston_reaction_torque_follows_rotation_and_tuning() {
    use glam::DVec3;
    let air = Atmosphere::standard(0.0).unwrap();
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let mut data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(0.0)
            || data
                .configuration
                .number("propeller", "propeller_type")
                .unwrap()
                != Some(1.0)
        {
            continue;
        }
        let config = String::from_utf8(std::fs::read(path.join("aircraft.cfg")).unwrap()).unwrap();
        if !config.contains("[propeller]")
            || !config.contains("[flight_tuning]")
            || data
                .configuration
                .value("propeller", "rotation")
                .unwrap()
                .is_some()
            || data
                .configuration
                .value("flight_tuning", "torque_on_roll")
                .unwrap()
                .is_some()
        {
            continue;
        }
        let mut moments = Vec::new();
        let mut gyros = Vec::new();
        for (rotation, scale) in [(1, 1), (-1, 1), (1, 0)] {
            let config = config
                .replace("[propeller]", &format!("[propeller]\nrotation={rotation}"))
                .replace(
                    "[flight_tuning]",
                    &format!("[flight_tuning]\ntorque_on_roll={scale}"),
                );
            data.configuration = esp_formats::cfg::Config::parse(&config).unwrap();
            let mut engine = Pistons::from_data(&data).unwrap();
            engine.start_running(&air, DVec3::ZERO).unwrap();
            let power = engine.indices().map(|_| 1.0).collect::<Vec<_>>();
            let wrench = engine
                .wrench(&air, DVec3::ZERO, DVec3::ZERO, DVec3::ZERO, &power)
                .unwrap();
            let pitch = engine
                .wrench(&air, DVec3::ZERO, DVec3::Y, DVec3::ZERO, &power)
                .unwrap();
            let yaw = engine
                .wrench(&air, DVec3::ZERO, DVec3::Z, DVec3::ZERO, &power)
                .unwrap();
            assert_eq!(pitch.force, wrench.force);
            gyros.push((pitch.moment - wrench.moment, yaw.moment - wrench.moment));
            assert!(
                engine
                    .wrench(
                        &air,
                        DVec3::ZERO,
                        DVec3::splat(f64::NAN),
                        DVec3::ZERO,
                        &power
                    )
                    .is_err()
            );
            moments.push(wrench.moment);
        }
        let clockwise = moments[0] - moments[2];
        let counterclockwise = moments[1] - moments[2];
        assert!(clockwise.x < 0.0);
        assert!(counterclockwise.x > 0.0);
        assert!((clockwise + counterclockwise).length() < 1e-8);
        assert!(gyros[0].0.z > 0.0 && gyros[0].1.y < 0.0);
        assert!((gyros[0].0 + gyros[1].0).length() < 1e-8);
        assert!((gyros[0].1 + gyros[1].1).length() < 1e-8);
        assert!((gyros[0].0 - gyros[2].0).length() < 1e-8);
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original fixed-pitch piston aircraft and AIR engine/propeller tables"]
fn original_piston_crossflow_changes_yaw_and_honors_p_factor_scalar() {
    use glam::DVec3;
    let air = Atmosphere::standard(0.0).unwrap();
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let mut data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(0.0)
            || data
                .configuration
                .number("propeller", "propeller_type")
                .unwrap()
                != Some(1.0)
        {
            continue;
        }
        let config = String::from_utf8(std::fs::read(path.join("aircraft.cfg")).unwrap()).unwrap();
        if !config.contains("[flight_tuning]")
            || data
                .configuration
                .value("flight_tuning", "p_factor_on_yaw")
                .unwrap()
                .is_some()
        {
            continue;
        }
        let mut effects = Vec::new();
        for scale in [0, 1, 2] {
            let config = config.replace(
                "[flight_tuning]",
                &format!("[flight_tuning]\np_factor_on_yaw={scale}"),
            );
            data.configuration = esp_formats::cfg::Config::parse(&config).unwrap();
            let mut engine = Pistons::from_data(&data).unwrap();
            engine.start_running(&air, DVec3::ZERO).unwrap();
            let commands = engine
                .indices()
                .map(|_| Command {
                    throttle: 1.0,
                    ..Command::default()
                })
                .collect::<Vec<_>>();
            for _ in 0..100 {
                engine
                    .advance(0.1, &air, DVec3::ZERO, &commands, true)
                    .unwrap();
            }
            let power = engine.indices().map(|_| 1.0).collect::<Vec<_>>();
            let baseline = engine
                .wrench(
                    &air,
                    DVec3::new(15.0, 0.0, 0.0),
                    DVec3::ZERO,
                    DVec3::ZERO,
                    &power,
                )
                .unwrap();
            let pitched = engine
                .wrench(
                    &air,
                    DVec3::new(15.0, 0.0, 6.0),
                    DVec3::ZERO,
                    DVec3::ZERO,
                    &power,
                )
                .unwrap();
            effects.push(pitched.moment.z - baseline.moment.z);
            assert_eq!(pitched.force, baseline.force);
        }
        assert!(effects[0].abs() < 1e-9);
        assert!(effects[1] < 0.0);
        assert!((effects[2] - 2.0 * effects[1]).abs() < 1e-9);
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original fixed-pitch piston aircraft and AIR engine/propeller tables"]
fn original_piston_wake_improves_low_speed_tail_control() {
    use esp_physics::aerodynamics::Controls;
    use glam::DVec3;
    let air = Atmosphere::standard(0.0).unwrap();
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(0.0)
            || data
                .configuration
                .number("propeller", "propeller_type")
                .unwrap()
                != Some(1.0)
        {
            continue;
        }
        let aero = esp_sim::loading::aerodynamic_model(&data).unwrap();
        let flaps = vec![
            0.0;
            esp_formats::cfg::aircraft::flaps(&data.configuration)
                .unwrap()
                .iter()
                .filter(|f| f.kind != 0)
                .count()
        ];
        let mut engine = Pistons::from_data(&data).unwrap();
        let power = engine.indices().map(|_| 1.0).collect::<Vec<_>>();
        let stopped = engine.tail_wash(&air, DVec3::ZERO, &power).unwrap();
        assert_eq!(stopped, esp_physics::propeller::TailWash::default());
        engine.start_running(&air, DVec3::ZERO).unwrap();
        let wake = engine.tail_wash(&air, DVec3::ZERO, &power).unwrap();
        assert!(wake.horizontal.x > 0.0 && wake.swirl.y < 0.0);
        let no_fuel = vec![0.0; power.len()];
        assert_eq!(
            engine.tail_wash(&air, DVec3::ZERO, &no_fuel).unwrap(),
            esp_physics::propeller::TailWash::default()
        );
        let centered = aero
            .tail_wash_forces(
                (DVec3::ZERO, wake),
                DVec3::ZERO,
                air.density,
                Controls::default(),
                &flaps,
                DVec3::ZERO,
            )
            .unwrap();
        assert!(centered.moment.z < 0.0);
        let config = String::from_utf8(std::fs::read(path.join("aircraft.cfg")).unwrap()).unwrap();
        if data
            .configuration
            .value("airplane_geometry", "htail_pos_lon")
            .unwrap()
            .is_some()
            && data
                .configuration
                .value("airplane_geometry", "vtail_pos_lon")
                .unwrap()
                .is_some()
        {
            let modified = config
                .lines()
                .map(|line| {
                    let key = line.split('=').next().unwrap_or("").trim();
                    if key.eq_ignore_ascii_case("htail_pos_lon")
                        || key.eq_ignore_ascii_case("vtail_pos_lon")
                    {
                        format!("{key}=100")
                    } else {
                        line.to_owned()
                    }
                })
                .collect::<Vec<_>>()
                .join("\n");
            let mut forward_data = FlightData::load(&path, 0).unwrap();
            forward_data.configuration = esp_formats::cfg::Config::parse(&modified).unwrap();
            let mut forward = Pistons::from_data(&forward_data).unwrap();
            forward.start_running(&air, DVec3::ZERO).unwrap();
            assert_eq!(
                forward.tail_wash(&air, DVec3::ZERO, &power).unwrap(),
                esp_physics::propeller::TailWash::default()
            );
        }
        if config.contains("[propeller]")
            && data
                .configuration
                .value("propeller", "rotation")
                .unwrap()
                .is_none()
        {
            let mut reversed_data = FlightData::load(&path, 0).unwrap();
            reversed_data.configuration = esp_formats::cfg::Config::parse(
                &config.replace("[propeller]", "[propeller]\nrotation=-1"),
            )
            .unwrap();
            let mut reversed = Pistons::from_data(&reversed_data).unwrap();
            reversed.start_running(&air, DVec3::ZERO).unwrap();
            let reverse_wake = reversed.tail_wash(&air, DVec3::ZERO, &power).unwrap();
            assert!((wake.horizontal - reverse_wake.horizontal).length() < 1e-10);
            assert!((wake.swirl + reverse_wake.swirl).length() < 1e-10);
            let reversed_load = aero
                .tail_wash_forces(
                    (DVec3::ZERO, reverse_wake),
                    DVec3::ZERO,
                    air.density,
                    Controls::default(),
                    &flaps,
                    DVec3::ZERO,
                )
                .unwrap();
            assert!(reversed_load.moment.z > 0.0);
            assert!((centered.moment.z + reversed_load.moment.z).abs() < 1e-8);
        }
        let horizontal_only = esp_physics::propeller::TailWash {
            horizontal: wake.horizontal,
            ..Default::default()
        };
        let vertical_only = esp_physics::propeller::TailWash {
            vertical: wake.vertical,
            ..Default::default()
        };
        let rudder_only = Controls {
            rudder: 1.0,
            ..Default::default()
        };
        let elevator_only = Controls {
            elevator: 1.0,
            ..Default::default()
        };
        let no_rudder = aero
            .tail_wash_forces(
                (DVec3::ZERO, horizontal_only),
                DVec3::ZERO,
                air.density,
                rudder_only,
                &flaps,
                DVec3::ZERO,
            )
            .unwrap();
        let no_elevator = aero
            .tail_wash_forces(
                (DVec3::ZERO, vertical_only),
                DVec3::ZERO,
                air.density,
                elevator_only,
                &flaps,
                DVec3::ZERO,
            )
            .unwrap();
        assert_eq!(no_rudder.force, DVec3::ZERO);
        assert_eq!(no_rudder.moment, DVec3::ZERO);
        assert_eq!(no_elevator.force, DVec3::ZERO);
        assert_eq!(no_elevator.moment, DVec3::ZERO);
        let rudder_wash = aero
            .tail_wash_forces(
                (DVec3::ZERO, vertical_only),
                DVec3::ZERO,
                air.density,
                rudder_only,
                &flaps,
                DVec3::ZERO,
            )
            .unwrap();
        let elevator_wash = aero
            .tail_wash_forces(
                (DVec3::ZERO, horizontal_only),
                DVec3::ZERO,
                air.density,
                elevator_only,
                &flaps,
                DVec3::ZERO,
            )
            .unwrap();
        assert!(rudder_wash.moment.z.abs() > 0.0);
        assert!(elevator_wash.moment.y.abs() > 0.0);
        let controls = Controls {
            rudder: 1.0,
            elevator: 0.5,
            ..Controls::default()
        };
        let boosted = aero
            .tail_wash_forces(
                (DVec3::ZERO, wake),
                DVec3::ZERO,
                air.density,
                controls,
                &flaps,
                DVec3::ZERO,
            )
            .unwrap();
        assert!(boosted.moment.y.abs() > 0.0 && boosted.moment.z.abs() > 0.0);
        let unpowered = aero
            .tail_wash_forces(
                (DVec3::ZERO, stopped),
                DVec3::ZERO,
                air.density,
                controls,
                &flaps,
                DVec3::ZERO,
            )
            .unwrap();
        assert_eq!(unpowered.moment, DVec3::ZERO);
        verified += 1;
    }
    assert!(verified > 0);
}

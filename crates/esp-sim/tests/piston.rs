use esp_assets::flight::FlightData;
use esp_formats::air;
use esp_sim::{
    atmosphere::Atmosphere,
    piston::{Charge, PistonCycle},
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

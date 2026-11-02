use esp_assets::flight::FlightData;
use esp_sim::{
    atmosphere::Atmosphere,
    time::STEP,
    turbine::{Command, Turbines},
};
use glam::DVec3;

#[path = "../../../tests/support/content.rs"]
mod content;

fn full(model: &mut Turbines, air: &Atmosphere, speed: f64) {
    let commands = model
        .indices()
        .map(|_| Command {
            throttle: 1.0,
            fuel_valve: true,
            starter: false,
        })
        .collect::<Vec<_>>();
    model.start_running(air, speed).unwrap();
    for _ in 0..1800 {
        model.advance(STEP, air, speed, &commands, true).unwrap();
    }
}

#[test]
#[ignore = "requires original jet AIR records and OPENESP_AIRCRAFT_PATHS"]
fn original_turbine_tables_control_thrust_spooling_and_shutdown() {
    let sea = Atmosphere::standard(0.0).unwrap();
    let high = Atmosphere::standard(10000.0).unwrap();
    let mut count = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(1.0)
        {
            continue;
        }
        let mut model = Turbines::from_data(&data).unwrap();
        model.start_running(&sea, 0.0).unwrap();
        let idle = model.output(0, &sea, 0.0).unwrap();
        assert!(idle.n2 > 0.0 && idle.n1 > 0.0 && idle.gross_thrust > 0.0);
        let mut commands = model
            .indices()
            .map(|_| Command {
                throttle: 1.0,
                fuel_valve: true,
                starter: false,
            })
            .collect::<Vec<_>>();
        model.advance(STEP, &sea, 0.0, &commands, true).unwrap();
        let initial = model.output(0, &sea, 0.0).unwrap();
        full(&mut model, &sea, 0.0);
        let maximum = model.output(0, &sea, 0.0).unwrap();
        assert!(
            initial.gross_thrust > idle.gross_thrust && initial.gross_thrust < maximum.gross_thrust
        );
        assert!(maximum.n1 > 80.0 && maximum.n1 < 115.0 && maximum.n2 > 85.0 && maximum.n2 < 115.0);
        let reference = data
            .configuration
            .number("TurbineEngineData", "static_thrust")
            .unwrap()
            .unwrap()
            * 4.4482216152605;
        assert!(
            (maximum.net_thrust / reference - 1.0).abs() < 0.25,
            "thrust={} reference={reference}",
            maximum.net_thrust
        );
        assert_eq!(maximum.ram_drag, 0.0);
        assert!(maximum.fuel_kilograms_per_second > idle.fuel_kilograms_per_second);
        let cg = DVec3::ZERO;
        let mut fractions = vec![1.0; commands.len()];
        let symmetric = model.wrench(&sea, DVec3::ZERO, cg, &fractions).unwrap();
        assert!(symmetric.moment.z.abs() < 1e-8 && symmetric.moment.y > 0.0);
        fractions[0] = 0.0;
        let right_only = model.wrench(&sea, DVec3::ZERO, cg, &fractions).unwrap();
        assert!((right_only.force.x * 2.0 - symmetric.force.x).abs() < 1e-8);
        assert!(right_only.moment.z < 0.0);
        fractions.fill(1.0);
        commands[1].fuel_valve = false;
        model.advance(STEP, &sea, 0.0, &commands, true).unwrap();
        let asymmetric = model.wrench(&sea, DVec3::ZERO, cg, &fractions).unwrap();
        assert!(asymmetric.moment.z > 0.0);
        assert!(!model.output(1, &sea, 0.0).unwrap().combustion);
        full(&mut model, &high, 200.0);
        let altitude = model.output(0, &high, 200.0).unwrap();
        assert!(altitude.gross_thrust < maximum.gross_thrust && altitude.ram_drag > 0.0);
        model.advance(STEP, &high, 200.0, &commands, false).unwrap();
        let starved = model.output(0, &high, 200.0).unwrap();
        assert_eq!(starved.gross_thrust, 0.0);
        assert_eq!(starved.fuel_kilograms_per_second, 0.0);
        assert!(starved.n2 > 0.0 && starved.net_thrust < 0.0);
        eprintln!(
            "{}: idle={}N maximum={}N fuel={}kg/s altitude={}N",
            path.display(),
            idle.net_thrust,
            maximum.net_thrust,
            maximum.fuel_kilograms_per_second,
            altitude.net_thrust
        );
        count += 1;
    }
    assert!(count > 0);
}

#[test]
#[ignore = "requires original jet AIR records and OPENESP_AIRCRAFT_PATHS"]
fn modifying_original_thrust_table_changes_thrust_without_changing_speed_map() {
    let sea = Atmosphere::standard(0.0).unwrap();
    let mut count = 0;
    for path in content::aircraft_paths() {
        let mut data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(1.0)
        {
            continue;
        }
        let mut original = Turbines::from_data(&data).unwrap();
        full(&mut original, &sea, 100.0);
        let original = original.output(0, &sea, 100.0).unwrap();
        let offset = esp_formats::air::parse(&data.air)
            .unwrap()
            .iter()
            .find(|r| r.id == 0x1506)
            .unwrap()
            .offset
            + 8;
        let rows = u32::from_le_bytes(data.air[offset..offset + 4].try_into().unwrap()) as usize;
        let columns =
            u32::from_le_bytes(data.air[offset + 4..offset + 8].try_into().unwrap()) as usize;
        for row in 1..rows {
            for column in 1..columns {
                let offset = offset + 8 + (row * columns + column) * 8;
                let value = f64::from_le_bytes(data.air[offset..offset + 8].try_into().unwrap());
                data.air[offset..offset + 8].copy_from_slice(&(value * 0.5).to_le_bytes());
            }
        }
        let mut changed = Turbines::from_data(&data).unwrap();
        full(&mut changed, &sea, 100.0);
        let changed = changed.output(0, &sea, 100.0).unwrap();
        assert_eq!(changed.n1, original.n1);
        assert_eq!(changed.n2, original.n2);
        assert_eq!(changed.ram_drag, original.ram_drag);
        assert!((changed.gross_thrust - original.gross_thrust * 0.5).abs() < 1e-8);
        assert!(
            (changed.fuel_kilograms_per_second - original.fuel_kilograms_per_second * 0.5).abs()
                < 1e-12
        );
        count += 1;
    }
    assert!(count > 0);
}

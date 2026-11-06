use esp_assets::flight::FlightData;
use esp_formats::air::{self, table::Table};
use esp_physics::{atmosphere::Atmosphere, propeller::Propeller};

#[path = "../../../tests/support/content.rs"]
mod content;

fn scale_table(data: &mut FlightData, id: u32, scale: f64) {
    let (rows, columns, values, offset) = {
        let records = air::parse(&data.air).unwrap();
        let record = records.iter().find(|r| r.id == id).unwrap();
        let Some(Table::Surface(table)) = record.table().unwrap() else {
            panic!("missing propeller surface");
        };
        (
            table.rows as usize,
            table.columns as usize,
            table.values,
            record.offset + 16,
        )
    };
    for row in 1..rows {
        for col in 1..columns {
            let cell = offset + (row * columns + col) * 8;
            data.air[cell..cell + 8]
                .copy_from_slice(&(values[row * columns + col] * scale).to_le_bytes());
        }
    }
}

#[test]
#[ignore = "requires original fixed-pitch propeller AIR tables and aircraft.cfg"]
fn original_propeller_power_and_efficiency_tables_drive_thrust_and_shaft_load() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("propeller", "propeller_type")
            .unwrap()
            != Some(1.0)
        {
            continue;
        }
        let propeller = Propeller::from_data(&data).unwrap();
        let atmosphere = Atmosphere::standard(0.0).unwrap();
        let original = propeller.sample(&atmosphere, 2700.0, 50.0).unwrap();
        assert!(original.thrust > 0.0 && original.absorbed_power > 0.0);
        let mut power = FlightData::load(&path, 0).unwrap();
        scale_table(&mut power, 0x512, 0.5);
        let reduced = Propeller::from_data(&power)
            .unwrap()
            .sample(&atmosphere, 2700.0, 50.0)
            .unwrap();
        assert!((reduced.absorbed_power / original.absorbed_power - 0.5).abs() < 1e-12);
        assert!((reduced.absorbed_torque / original.absorbed_torque - 0.5).abs() < 1e-12);
        assert!((reduced.thrust / original.thrust - 0.5).abs() < 1e-12);
        let mut efficiency = FlightData::load(&path, 0).unwrap();
        scale_table(&mut efficiency, 0x511, 0.5);
        let reduced = Propeller::from_data(&efficiency)
            .unwrap()
            .sample(&atmosphere, 2700.0, 50.0)
            .unwrap();
        assert_eq!(reduced.absorbed_power, original.absorbed_power);
        assert!((reduced.thrust / original.thrust - 0.5).abs() < 1e-12);
        let static_output = propeller.sample(&atmosphere, 2300.0, 0.0).unwrap();
        assert!(static_output.thrust.is_finite() && static_output.thrust > 0.0);
        eprintln!(
            "{}: cruise absorbed power {:.3} W, thrust {:.3} N; static thrust {:.3} N",
            path.display(),
            original.absorbed_power,
            original.thrust,
            static_output.thrust
        );
        verified += 1;
    }
    assert!(verified > 0);
}

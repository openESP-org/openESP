use anyhow::{Context, Result, ensure};
use esp_assets::flight::FlightData;
use esp_formats::air;
use esp_physics::{
    lookup::Grid,
    propeller::{Propeller, PropellerParameters},
};

pub fn propeller(data: &FlightData) -> Result<Propeller> {
    let config = &data.configuration;
    ensure!(
        config.number("propeller", "propeller_type")? == Some(1.0),
        "only fixed-pitch propellers are supported yet"
    );
    let records = air::parse(&data.air)?;
    let grid = |id| -> Result<Grid> {
        let mut matches = records.iter().filter(|r| r.id == id);
        let record = matches
            .next()
            .with_context(|| format!("missing propeller AIR record {id:#x}"))?;
        ensure!(
            matches.next().is_none(),
            "duplicate propeller AIR record {id:#x}"
        );
        let Some(air::table::Table::Surface(table)) = record.table()? else {
            anyhow::bail!("AIR record {id:#x} is not a surface")
        };
        Grid::from_values(table.rows as usize, table.columns as usize, &table.values)
    };
    let required = |key| -> Result<f64> {
        config
            .number("propeller", key)?
            .with_context(|| format!("missing [propeller] {key}"))
    };
    let diameter = required("propeller_diameter")? * 0.3048;
    let inertia = required("propeller_moi")? * 1.3558179483314;
    let reduction = config
        .number("propeller", "gear_reduction_ratio")?
        .unwrap_or(1.0);
    let beta = required("fixed_pitch_beta")?;
    let low_speed_limit = config
        .number("propeller", "low_speed_theory_limit")?
        .unwrap_or(80.0)
        * 0.3048;
    let thrust_scalar = config.number("propeller", "thrust_scalar")?.unwrap_or(1.0);
    ensure!(
        diameter > 0.0
            && inertia > 0.0
            && reduction > 0.0
            && low_speed_limit > 0.0
            && thrust_scalar >= 0.0,
        "invalid propeller dimensions, inertia or tuning"
    );
    Propeller::new(PropellerParameters {
        efficiency: grid(0x511)?,
        power: grid(0x512)?,
        diameter,
        inertia,
        reduction,
        beta,
        low_speed_limit,
        thrust_scalar,
    })
}

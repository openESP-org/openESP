use anyhow::{Context, Result, ensure};
use esp_assets::flight::FlightData;
use esp_formats::air;
use esp_physics::{
    lookup::Curve,
    piston::{PistonCycle, PistonParameters},
};

pub fn piston_cycle(data: &FlightData) -> Result<PistonCycle> {
    let config = &data.configuration;
    ensure!(
        config.number("GeneralEngineData", "engine_type")? == Some(0.0),
        "piston cycle requires a reciprocating engine"
    );
    let records = air::parse(&data.air)?;
    let curve = |id| -> Result<Curve> {
        let mut records = records.iter().filter(|r| r.id == id);
        let record = records
            .next()
            .with_context(|| format!("missing piston AIR record {id:#x}"))?;
        ensure!(
            records.next().is_none(),
            "duplicate piston AIR record {id:#x}"
        );
        let Some(air::table::Table::Curve(table)) = record.table()? else {
            anyhow::bail!("AIR record {id:#x} is not a curve")
        };
        Curve::from_values(
            &table
                .points
                .iter()
                .map(|point| (point.input, point.output))
                .collect::<Vec<_>>(),
        )
    };
    let required = |key| -> Result<f64> {
        config
            .number("piston_engine", key)?
            .with_context(|| format!("missing [piston_engine] {key}"))
    };
    let displacement = required("cylinder_displacement")? * 0.000016387064;
    let cylinders = required("number_of_cylinders")?;
    let compression = required("compression_ratio")?;
    let maximum_rpm = required("max_rated_rpm")?;
    let power_scalar = config
        .number("piston_engine", "power_scalar")?
        .unwrap_or(1.0);
    let scalar = |key| -> Result<f64> { Ok(config.number("piston_engine", key)?.unwrap_or(1.0)) };
    let mechanical_scalars = [
        scalar("idle_rpm_mechanical_efficiency_scalar")?,
        scalar("max_rpm_mechanical_efficiency_scalar")?,
    ];
    let friction_scalars = [
        scalar("idle_rpm_friction_scalar")?,
        scalar("max_rpm_friction_scalar")?,
    ];
    let specific_consumption = config
        .number("piston_engine", "BestPowerSpecificFuelConsumption")?
        .unwrap_or(0.49);
    let fuel_scalar = config
        .number("GeneralEngineData", "fuel_flow_scalar")?
        .unwrap_or(1.0);
    ensure!(
        displacement > 0.0
            && cylinders > 0.0
            && cylinders.fract() == 0.0
            && compression > 1.0
            && maximum_rpm > 0.0,
        "invalid piston geometry or rated RPM"
    );
    ensure!(
        [power_scalar, fuel_scalar, specific_consumption]
            .iter()
            .chain(&mechanical_scalars)
            .chain(&friction_scalars)
            .all(|v| *v >= 0.0),
        "invalid piston tuning scalar"
    );
    let efficiency = curve(0x508)?;
    ensure!(
        efficiency.outputs_nonnegative(),
        "negative piston mechanical efficiency"
    );
    PistonCycle::new(PistonParameters {
        efficiency,
        friction: curve(0x509)?,
        displacement,
        cylinders,
        compression,
        maximum_rpm,
        power_scalar,
        mechanical_scalars,
        friction_scalars,
        fuel_scalar,
        specific_consumption,
    })
}

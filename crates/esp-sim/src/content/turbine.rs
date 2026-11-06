use anyhow::{Context, Result, ensure};
use esp_assets::flight::FlightData;
use esp_formats::{air, cfg};
use esp_physics::{
    lookup::{Curve, Grid},
    turbine::{TurbineEngine, TurbineParameters, Turbines},
};
use glam::DVec3;

const POUND_FORCE: f64 = 4.4482216152605;

fn curve_from_record(record: &air::Record<'_>) -> Result<Curve> {
    let Some(air::table::Table::Curve(table)) = record.table()? else {
        anyhow::bail!("AIR record {:#x} is not a curve", record.id)
    };
    Curve::from_values(
        &table
            .points
            .iter()
            .map(|point| (point.input, point.output))
            .collect::<Vec<_>>(),
    )
}
fn grid_from_record(record: &air::Record<'_>) -> Result<Grid> {
    let Some(air::table::Table::Surface(table)) = record.table()? else {
        anyhow::bail!("AIR record {:#x} is not a surface", record.id)
    };
    Grid::from_values(table.rows as usize, table.columns as usize, &table.values)
}
pub fn turbines(data: &FlightData) -> Result<Turbines> {
    let config = &data.configuration;
    ensure!(
        config.number("GeneralEngineData", "engine_type")? == Some(1.0),
        "only turbojet/turbofan propulsion is implemented yet"
    );
    let records = air::parse(&data.air)?;
    let record = |id| -> Result<&air::Record<'_>> {
        let mut found = records.iter().filter(|r| r.id == id);
        let record = found
            .next()
            .with_context(|| format!("missing engine AIR record {id:#x}"))?;
        ensure!(
            found.next().is_none(),
            "duplicate engine AIR record {id:#x}"
        );
        Ok(record)
    };
    let grid =
        |id| grid_from_record(record(id)?).with_context(|| format!("engine AIR record {id:#x}"));
    let spool = curve_from_record(record(0x1505)?)?;
    ensure!(
        spool.outputs_nondecreasing(),
        "non-monotonic turbine spool table"
    );
    let number = |section, key| -> Result<f64> {
        config
            .number(section, key)?
            .with_context(|| format!("missing [{section}] {key}"))
    };
    let datum = cfg::aircraft::ReferencePoints::from_config(config)?
        .datum_feet
        .context("missing engine reference datum")?;
    let engines = cfg::engine::engines(config)?
        .into_iter()
        .map(|definition| -> Result<TurbineEngine> {
            let index = definition
                .index
                .checked_add(1)
                .context("engine index is too large")?;
            let [x, y, z] = definition.position_feet;
            let pitch = definition.pitch_degrees.to_radians();
            let heading = definition.heading_degrees.to_radians();
            Ok(TurbineEngine {
                index,
                position: DVec3::new(x + datum[0], y + datum[1], -z - datum[2]) * 0.3048,
                direction: DVec3::new(
                    pitch.cos() * heading.cos(),
                    pitch.cos() * heading.sin(),
                    pitch.sin(),
                ),
            })
        })
        .collect::<Result<Vec<_>>>()?;
    ensure!(!engines.is_empty(), "aircraft has no configured engines");
    Turbines::new(TurbineParameters {
        n1: grid(0x1502)?,
        low_n2: grid(0x1503)?,
        high_n2: grid(0x1504)?,
        spool,
        thrust: grid(0x1506)?,
        airflow: grid(0x1507)?,
        static_thrust: number("TurbineEngineData", "static_thrust")? * POUND_FORCE,
        inlet_area: number("TurbineEngineData", "inlet_area")?,
        thrust_scalar: config.number("jet_engine", "thrust_scalar")?.unwrap_or(1.0),
        fuel_scalar: config
            .number("GeneralEngineData", "fuel_flow_scalar")?
            .unwrap_or(1.0),
        consumption: number("TurbineEngineData", "ThrustSpecificFuelConsumption")?,
        gain: number("TurbineEngineData", "fuel_flow_gain")?,
        throttle_lower_limit: config
            .number("GeneralEngineData", "min_throttle_limit")?
            .unwrap_or(0.0),
        reverse_scalar: config
            .number("TurbineEngineData", "reverser_available")?
            .unwrap_or(0.0),
        engines,
    })
}

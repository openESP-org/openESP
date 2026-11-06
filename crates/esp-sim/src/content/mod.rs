mod piston;
mod propeller;
mod turbine;
pub use piston::piston_cycle;
pub use propeller::propeller;
pub use turbine::turbines;

pub fn pistons(
    data: &esp_assets::flight::FlightData,
) -> anyhow::Result<esp_physics::piston::Pistons> {
    esp_physics::piston::Pistons::from_data(data, || propeller(data), || piston_cycle(data))
}

pub fn propulsion(
    data: &esp_assets::flight::FlightData,
) -> anyhow::Result<esp_physics::propulsion::Propulsion> {
    use esp_physics::propulsion::Propulsion;
    match data
        .configuration
        .number("GeneralEngineData", "engine_type")?
    {
        Some(0.0) => Ok(Propulsion::Piston {
            model: Box::new(pistons(data)?),
            commands: Vec::new(),
        }),
        Some(1.0) => Ok(Propulsion::Turbine {
            model: Box::new(turbines(data)?),
            commands: Vec::new(),
        }),
        kind => anyhow::bail!("unsupported running engine type: {kind:?}"),
    }
}

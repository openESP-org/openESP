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
    esp_physics::propulsion::Propulsion::from_data(
        data,
        || propeller(data),
        || piston_cycle(data),
        || turbines(data),
    )
}

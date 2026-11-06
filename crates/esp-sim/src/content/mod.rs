mod propeller;
pub use propeller::propeller;

pub fn pistons(
    data: &esp_assets::flight::FlightData,
) -> anyhow::Result<esp_physics::piston::Pistons> {
    esp_physics::piston::Pistons::from_data(data, || propeller(data))
}

pub fn propulsion(
    data: &esp_assets::flight::FlightData,
) -> anyhow::Result<esp_physics::propulsion::Propulsion> {
    esp_physics::propulsion::Propulsion::from_data(data, || propeller(data))
}

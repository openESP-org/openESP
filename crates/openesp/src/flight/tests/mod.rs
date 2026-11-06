use super::*;
use esp_script::Value;

fn controls(data: &FlightData) -> Variables {
    let mut variables = Variables::default();
    Flight::configure_ground(&mut variables);
    variables
        .configure_flaps(&esp_formats::cfg::aircraft::flaps(&data.configuration).unwrap())
        .unwrap();
    for name in [
        "ELEVATOR DEFLECTION PCT",
        "AILERON LEFT DEFLECTION PCT",
        "RUDDER DEFLECTION PCT",
    ] {
        variables.set_number(name, "percent", 0.0);
    }
    variables
}

mod controls;
mod engines;
mod gauges;
mod systems;

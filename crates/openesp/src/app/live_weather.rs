//! Apply observed weather to simulation variables.
use esp_sim::{variables::Variables, weather};

pub(super) fn apply(variables: &mut Variables, report: &esp_weather::Observation) {
    let altitude = weather::origin_elevation(variables);
    if let Some(temperature) = report.temperature_c {
        variables.set_number(
            "SEA LEVEL TEMPERATURE",
            "celsius",
            (temperature + 0.0065 * altitude).clamp(-60.0, 60.0),
        );
    }
    if let Some(pressure) = report.pressure_hpa {
        variables.set_number("SEA LEVEL PRESSURE", "millibars", pressure);
    }
    if let Some(direction) = report.wind_direction {
        variables.set_number("AMBIENT WIND DIRECTION", "degrees", direction);
    }
    if let Some(speed) = report.wind_knots {
        variables.set_number("AMBIENT WIND VELOCITY", "knots", speed);
    }
    if let Some(visibility) = report.visibility_m {
        variables.set_number("AMBIENT VISIBILITY", "meters", visibility);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_script::{Host, Reference};
    #[test]
    fn observation_drives_flight_wind_atmosphere_and_visibility() {
        let mut variables = Variables::default();
        variables.set_number("WORLD ORIGIN ALTITUDE", "meters", 1000.0);
        let report =
            esp_weather::parse("METAR TEST 081020Z 27020KT 4000 BKN020 10/05 Q1000", "TEST")
                .unwrap();
        apply(&mut variables, &report);
        let wind = weather::wind(&mut variables).unwrap();
        assert!((wind.y - 20.0 * 1852.0 / 3600.0).abs() < 1e-6);
        let atmosphere = weather::conditions(&mut variables)
            .unwrap()
            .at(1000.0)
            .unwrap();
        assert!((atmosphere.temperature - 283.15).abs() < 0.1);
        let visibility = variables
            .read(&Reference::new('A', "AMBIENT VISIBILITY", Some("meters")))
            .unwrap()
            .number()
            .unwrap();
        assert_eq!(visibility, 4000.0);
    }
}

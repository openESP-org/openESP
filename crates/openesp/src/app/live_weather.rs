//! Apply observed weather to simulation variables.
use esp_script::{Host, Reference};
use esp_sim::{variables::Variables, weather};

pub(super) fn render_state(variables: &mut Variables) -> esp_render::Weather {
    let read = |variables: &mut Variables, name: &str, unit: &str| {
        variables
            .read(&Reference::new('A', name, Some(unit)))
            .ok()
            .and_then(|value| value.number().ok())
            .filter(|value| value.is_finite())
    };
    let clouds = std::array::from_fn(|index| {
        let base = read(
            variables,
            &format!("OPENESP CLOUD BASE:{}", index + 1),
            "meters",
        )?;
        let coverage = read(
            variables,
            &format!("OPENESP CLOUD COVER:{}", index + 1),
            "number",
        )?;
        (coverage > 0.0).then_some(esp_render::CloudLayer {
            base_msl: base,
            coverage,
        })
    });
    esp_render::Weather {
        visibility_m: read(variables, "AMBIENT VISIBILITY", "meters").unwrap_or(100_000.0),
        origin_msl: weather::origin_elevation(variables),
        clouds,
    }
}

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
    if let Some(clouds) = &report.clouds {
        for index in 0..3 {
            let layer = clouds.get(index);
            variables.set_number(
                &format!("OPENESP CLOUD BASE:{}", index + 1),
                "meters",
                layer.map_or(0.0, |layer| altitude + layer.base_agl_m),
            );
            variables.set_number(
                &format!("OPENESP CLOUD COVER:{}", index + 1),
                "number",
                layer.map_or(0.0, |layer| layer.coverage),
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
        assert_eq!(
            variables
                .read(&Reference::new('A', "OPENESP CLOUD BASE:1", Some("meters")))
                .unwrap()
                .number()
                .unwrap(),
            1609.6
        );
        let clear = esp_weather::parse("TEST 081030Z 00000KT CAVOK 12/05 Q1000", "TEST").unwrap();
        apply(&mut variables, &clear);
        assert!(
            render_state(&mut variables)
                .clouds
                .iter()
                .all(Option::is_none)
        );
    }
}

//! Apply observed weather to simulation variables.
use esp_script::{Host, Reference};
use esp_sim::{variables::Variables, weather};

pub(super) fn render_state(
    variables: &mut Variables,
    drift: &esp_weather::CloudDrift,
) -> esp_render::Weather {
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
        cloud_offset: [drift.east, drift.north],
        precipitation: weather::precipitation(variables),
        simulation_seconds: variables
            .read(&Reference::new('E', "SIMULATION TIME", Some("seconds")))
            .ok()
            .and_then(|value| value.number().ok())
            .filter(|value| value.is_finite())
            .unwrap_or(0.0),
    }
}

pub(super) fn advance_clouds(
    variables: &mut Variables,
    drift: &mut esp_weather::CloudDrift,
    dt: f64,
) -> anyhow::Result<()> {
    let direction = variables
        .read(&Reference::new(
            'A',
            "AMBIENT WIND DIRECTION",
            Some("degrees"),
        ))?
        .number()?;
    let speed = variables
        .read(&Reference::new('A', "AMBIENT WIND VELOCITY", Some("knots")))?
        .number()?;
    let wind = weather::vector(direction, speed)?;
    drift.advance(dt, wind.y, wind.x)
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
        variables.set_number(
            "OPENESP WIND GUST",
            "knots",
            report.gust_knots.unwrap_or(speed).max(speed),
        );
    }
    if let Some(visibility) = report.visibility_m {
        variables.set_number("AMBIENT VISIBILITY", "meters", visibility);
    }
    variables.set_number(
        "OPENESP PRECIPITATION RAIN",
        "number",
        report.precipitation[0],
    );
    variables.set_number(
        "OPENESP PRECIPITATION SNOW",
        "number",
        report.precipitation[1],
    );
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
    fn cloud_motion_uses_reported_wind_direction_and_remains_continuous() {
        let mut variables = Variables::default();
        let mut drift = esp_weather::CloudDrift::default();
        variables.set_number("AMBIENT WIND DIRECTION", "degrees", 270.0);
        variables.set_number("AMBIENT WIND VELOCITY", "knots", 20.0);
        advance_clouds(&mut variables, &mut drift, 10.0).unwrap();
        assert!((drift.east - 20.0 * 1852.0 / 3600.0 * 10.0).abs() < 1e-8);
        let east = drift.east;
        variables.set_number("AMBIENT WIND DIRECTION", "degrees", 0.0);
        advance_clouds(&mut variables, &mut drift, 5.0).unwrap();
        assert!((drift.east - east).abs() < 1e-8);
        assert!(drift.north < -50.0);
        assert_eq!(
            render_state(&mut variables, &drift).cloud_offset,
            [drift.east, drift.north]
        );
    }
    #[test]
    fn observation_drives_flight_wind_atmosphere_and_visibility() {
        let mut variables = Variables::default();
        variables.set_number("WORLD ORIGIN ALTITUDE", "meters", 1000.0);
        let report = esp_weather::parse(
            "METAR TEST 081020Z 27020G30KT 4000 -RA BKN020 10/05 Q1000",
            "TEST",
        )
        .unwrap();
        apply(&mut variables, &report);
        assert_eq!(weather::precipitation(&mut variables), [0.3, 0.0]);
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
                .read(&Reference::new('A', "OPENESP WIND GUST", Some("knots")))
                .unwrap()
                .number()
                .unwrap(),
            30.0
        );
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
        assert_eq!(weather::precipitation(&mut variables), [0.0, 0.0]);
        assert_eq!(
            variables
                .read(&Reference::new('A', "OPENESP WIND GUST", Some("knots")))
                .unwrap()
                .number()
                .unwrap(),
            0.0
        );
        assert!(
            render_state(&mut variables, &esp_weather::CloudDrift::default())
                .clouds
                .iter()
                .all(Option::is_none)
        );
    }
}

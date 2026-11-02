use super::{
    Config, ValueError,
    camera::{parse_number, vector},
    indexed_name,
};

#[derive(Debug, Clone, PartialEq)]
pub struct Engine {
    pub index: u32,
    pub position_feet: [f64; 3],
    pub pitch_degrees: f64,
    pub heading_degrees: f64,
}

pub fn engines(config: &Config) -> Result<Vec<Engine>, ValueError> {
    let mut engines = Vec::new();
    for (section, properties) in &config.sections {
        let Some(section) = section.filter(|s| s.eq_ignore_ascii_case("GeneralEngineData")) else {
            continue;
        };
        for (key, _) in properties {
            let Some(index) = indexed_name(key, "Engine.")? else {
                continue;
            };
            let position_feet = vector(config, section, key)?.unwrap();
            let angle_key = format!("ThrustAnglesPitchHeading.{index}");
            let (pitch_degrees, heading_degrees) =
                if let Some(value) = config.value(section, &angle_key)? {
                    let (pitch, heading) =
                        value.split_once(',').ok_or_else(|| ValueError::Invalid {
                            section: section.into(),
                            key: angle_key.clone(),
                            value: value.into(),
                        })?;
                    (
                        parse_number(section, &angle_key, pitch)?,
                        parse_number(section, &angle_key, heading)?,
                    )
                } else {
                    (0.0, 0.0)
                };
            engines.push(Engine {
                index,
                position_feet,
                pitch_degrees,
                heading_degrees,
            });
        }
    }
    engines.sort_by_key(|e| e.index);
    if engines.windows(2).any(|e| e[0].index == e[1].index) {
        return Err(ValueError::Duplicate {
            section: "GeneralEngineData".into(),
            key: "Engine".into(),
        });
    }
    Ok(engines)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn retains_sparse_engine_positions_and_thrust_angles() {
        let config = Config::parse(
            "[GENERALENGINEDATA]\nengine.2=1,2,-3\nengine.0=4,5,6\nThrustAnglesPitchHeading.2=5,-2",
        )
        .unwrap();
        let engines = engines(&config).unwrap();
        assert_eq!(engines[0].index, 0);
        assert_eq!(engines[1].position_feet, [1.0, 2.0, -3.0]);
        assert_eq!(
            (engines[1].pitch_degrees, engines[1].heading_degrees),
            (5.0, -2.0)
        );
        assert_eq!(engines[0].pitch_degrees, 0.0);
        for text in [
            "[GeneralEngineData]\nengine.0=1,2,3\nEngine.00=1,2,3",
            "[GeneralEngineData]\nengine.0=1,2,NaN",
            "[GeneralEngineData]\nengine.0=1,2,3\nThrustAnglesPitchHeading.0=1,2,3",
        ] {
            assert!(super::engines(&Config::parse(text).unwrap()).is_err());
        }
    }
}

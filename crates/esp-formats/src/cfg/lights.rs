//! Aircraft light switches, locations and referenced effects.
use super::{Config, ValueError, indexed_name};

#[derive(Debug, Clone, PartialEq)]
pub struct Light {
    pub index: u32,
    pub kind: u32,
    /// Longitudinal, lateral and vertical feet relative to the datum.
    pub position_feet: [f64; 3],
    pub effect: String,
}

pub fn from_config(config: &Config) -> Result<Vec<Light>, ValueError> {
    let mut lights = Vec::new();
    for (section, properties) in &config.sections {
        if !section.is_some_and(|name| name.eq_ignore_ascii_case("lights")) {
            continue;
        }
        for (key, value) in properties {
            let Some(index) = indexed_name(key, "light.")? else {
                continue;
            };
            config.value("lights", key)?;
            let invalid = || ValueError::Invalid {
                section: "lights".into(),
                key: key.into(),
                value: value.into(),
            };
            let fields: Vec<_> = value.split(',').map(str::trim).collect();
            if fields.len() < 5 || fields[4].is_empty() {
                return Err(invalid());
            }
            let kind = fields[0].parse::<u32>().map_err(|_| invalid())?;
            let parse = |s: &str| {
                s.parse::<f64>()
                    .ok()
                    .filter(|n| n.is_finite())
                    .ok_or_else(invalid)
            };
            let position_feet = [parse(fields[1])?, parse(fields[2])?, parse(fields[3])?];
            lights.push(Light {
                index,
                kind,
                position_feet,
                effect: fields[4].into(),
            });
        }
    }
    lights.sort_by_key(|light| light.index);
    Ok(lights)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn reads_sparse_lights_and_rejects_invalid_coordinates() {
        let config = Config::parse(
            "[lights]\nlight.3=2,-25,-57,4,fx_strobe,\nlight.0=3,-24,56,3.9,fx_navgre\n",
        )
        .unwrap();
        let lights = from_config(&config).unwrap();
        assert_eq!(lights.len(), 2);
        assert_eq!(lights[0].position_feet, [-24.0, 56.0, 3.9]);
        assert_eq!(lights[1].effect, "fx_strobe");
        assert!(from_config(&Config::parse("[lights]\nlight.0=3,nan,0,0,fx\n").unwrap()).is_err());
    }
}

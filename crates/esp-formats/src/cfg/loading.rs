use super::{Config, ValueError, camera::parse_number, indexed_name};

#[derive(Debug, Clone, PartialEq)]
pub struct Station {
    pub index: u32,
    pub pounds: f64,
    pub position_feet: [f64; 3],
}

#[derive(Debug, Clone, PartialEq)]
pub struct Tank {
    pub name: &'static str,
    pub position_feet: [f64; 3],
    pub capacity_gallons: f64,
    pub unusable_gallons: f64,
}

pub fn stations(config: &Config) -> Result<Vec<Station>, ValueError> {
    let mut stations = Vec::new();
    for (section, properties) in &config.sections {
        let Some(section) = section.filter(|s| s.eq_ignore_ascii_case("weight_and_balance")) else {
            continue;
        };
        for (key, value) in properties {
            let Some(index) = indexed_name(key, "station_load.")? else {
                continue;
            };
            config.value(section, key)?;
            let [pounds, x, y, z] = numbers::<4>(section, key, value, true)?;
            if pounds < 0.0 {
                return Err(invalid(section, key, value));
            }
            stations.push(Station {
                index,
                pounds,
                position_feet: [x, y, z],
            });
        }
    }
    stations.sort_by_key(|s| s.index);
    if stations.windows(2).any(|p| p[0].index == p[1].index) {
        return Err(ValueError::Duplicate {
            section: "weight_and_balance".into(),
            key: "station_load".into(),
        });
    }
    Ok(stations)
}

pub fn tanks(config: &Config) -> Result<Vec<Tank>, ValueError> {
    let mut tanks = Vec::new();
    for name in [
        "Center1",
        "Center2",
        "Center3",
        "LeftMain",
        "LeftAux",
        "LeftTip",
        "RightMain",
        "RightAux",
        "RightTip",
        "External1",
        "External2",
    ] {
        let Some(value) = config.value("fuel", name)? else {
            continue;
        };
        let [x, y, z, usable, unusable] = numbers::<5>("fuel", name, value, false)?;
        if usable < 0.0 || unusable < 0.0 {
            return Err(invalid("fuel", name, value));
        }
        tanks.push(Tank {
            name,
            position_feet: [x, y, z],
            capacity_gallons: usable,
            unusable_gallons: unusable,
        });
    }
    Ok(tanks)
}

fn numbers<const N: usize>(
    section: &str,
    key: &str,
    value: &str,
    description: bool,
) -> Result<[f64; N], ValueError> {
    let mut fields = value.split(',');
    let mut result = [0.0; N];
    for number in &mut result {
        let field = fields.next().ok_or_else(|| invalid(section, key, value))?;
        *number = parse_number(section, key, field)?;
    }
    if !description && fields.next().is_some() {
        return Err(invalid(section, key, value));
    }
    Ok(result)
}

fn invalid(section: &str, key: &str, value: &str) -> ValueError {
    ValueError::Invalid {
        section: section.into(),
        key: key.into(),
        value: value.into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_sparse_stations_and_tanks_case_insensitively() {
        let config = Config::parse("[WEIGHT_AND_BALANCE]\nStation_Load.3=170,-3,1.5,0,\"Pilot, right\"\nstation_load.0=100,2,0,-1\n[fuel]\nleftMAIN=-3,-2,1,26.5,1\nrightmain=-3,2,1,26.5,1").unwrap();
        let stations = stations(&config).unwrap();
        assert_eq!(stations.iter().map(|s| s.index).collect::<Vec<_>>(), [0, 3]);
        assert_eq!(stations[1].position_feet, [-3.0, 1.5, 0.0]);
        let tanks = tanks(&config).unwrap();
        assert_eq!(tanks[0].name, "LeftMain");
        assert_eq!(tanks[0].capacity_gallons, 26.5);
        assert_eq!(tanks[0].unusable_gallons, 1.0);
        assert_eq!(tanks[1].name, "RightMain");
    }

    #[test]
    fn rejects_ambiguous_and_invalid_loading_data() {
        for text in [
            "[weight_and_balance]\nstation_load.1=1,0,0,0\nstation_load.01=1,0,0,0",
            "[weight_and_balance]\nstation_load.0=-1,0,0,0",
            "[weight_and_balance]\nstation_load.0=1,NaN,0,0",
        ] {
            assert!(stations(&Config::parse(text).unwrap()).is_err());
        }
        for value in ["1,2,3,4", "1,2,3,-4,0", "1,2,3,4,NaN", "1,2,3,4,0,1"] {
            assert!(tanks(&Config::parse(&format!("[fuel]\nLeftMain={value}")).unwrap()).is_err());
        }
    }
}

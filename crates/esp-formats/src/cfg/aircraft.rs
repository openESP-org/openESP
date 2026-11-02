use super::{Config, ValueError, indexed_name};

#[derive(Debug, PartialEq, Eq)]
pub struct Configuration<'a> {
    pub index: u32,
    pub section: &'a str,
    pub title: Option<&'a str>,
    pub sim: Option<&'a str>,
    pub model: Option<&'a str>,
    pub panel: Option<&'a str>,
    pub sound: Option<&'a str>,
    pub texture: Option<&'a str>,
}

#[derive(Debug, PartialEq)]
pub struct ReferencePoints {
    pub datum_feet: Option<[f64; 3]>,
    pub empty_cg_feet: Option<[f64; 3]>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FlapSet {
    pub index: u32,
    pub kind: u32,
    pub extending_time: Option<f64>,
    pub system_type: Option<u32>,
    pub positions: Vec<FlapPosition>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct FlapPosition {
    pub index: u32,
    pub angle: f64,
    pub speed_limit: Option<f64>,
}

pub fn flaps(config: &Config) -> Result<Vec<FlapSet>, ValueError> {
    let mut result = Vec::new();
    for (section, properties) in &config.sections {
        let Some(section) = section else { continue };
        let Some(index) = indexed_name(section, "flaps.")? else {
            continue;
        };
        let kind = config
            .value(section, "type")?
            .map(|value| {
                value
                    .trim()
                    .parse::<u32>()
                    .map_err(|_| flap_error(section, "type", value))
            })
            .transpose()?
            .unwrap_or(0);
        if kind > 2 {
            return Err(flap_error(section, "type", &kind.to_string()));
        }
        let extending_time = super::camera::number(config, section, "extending-time")?;
        let system_type = config
            .value(section, "system_type")?
            .map(|value| {
                value
                    .trim()
                    .parse::<u32>()
                    .map_err(|_| flap_error(section, "system_type", value))
            })
            .transpose()?;
        if system_type.is_some_and(|kind| kind > 4) {
            return Err(flap_error(
                section,
                "system_type",
                "unsupported system type",
            ));
        }
        if extending_time.is_some_and(|time| time <= 0.0) {
            return Err(flap_error(section, "extending-time", "must be positive"));
        }
        let mut positions = Vec::new();
        for (key, value) in properties {
            let Some(position) = indexed_name(key, "flaps-position.")? else {
                continue;
            };
            config.value(section, key)?;
            let mut fields = value.split(',');
            let angle = fields
                .next()
                .unwrap()
                .trim()
                .parse::<f64>()
                .map_err(|_| flap_error(section, key, value))?;
            if !angle.is_finite() {
                return Err(flap_error(section, key, value));
            }
            let speed_limit = fields
                .next()
                .map(|field| {
                    field
                        .trim()
                        .parse::<f64>()
                        .map_err(|_| flap_error(section, key, value))
                })
                .transpose()?;
            if speed_limit.is_some_and(|speed| !speed.is_finite() || speed < 0.0)
                || fields.next().is_some()
            {
                return Err(flap_error(section, key, value));
            }
            positions.push(FlapPosition {
                index: position,
                angle,
                speed_limit,
            });
        }
        positions.sort_by_key(|p| p.index);
        if positions
            .windows(2)
            .any(|pair| pair[0].index == pair[1].index)
        {
            return Err(ValueError::Duplicate {
                section: section.into(),
                key: "flaps-position".into(),
            });
        }
        result.push(FlapSet {
            index,
            kind,
            extending_time,
            system_type,
            positions,
        });
    }
    result.sort_by_key(|set| set.index);
    if result.windows(2).any(|pair| pair[0].index == pair[1].index) {
        return Err(ValueError::Duplicate {
            section: "flaps".into(),
            key: "section".into(),
        });
    }
    Ok(result)
}

fn flap_error(section: &str, key: &str, value: &str) -> ValueError {
    ValueError::Invalid {
        section: section.into(),
        key: key.into(),
        value: value.into(),
    }
}

impl ReferencePoints {
    pub fn from_config(config: &Config) -> Result<Self, ValueError> {
        Ok(Self {
            datum_feet: super::camera::vector(
                config,
                "weight_and_balance",
                "reference_datum_position",
            )?,
            empty_cg_feet: super::camera::vector(
                config,
                "weight_and_balance",
                "empty_weight_cg_position",
            )?,
        })
    }
}

pub fn configurations(config: &Config) -> Result<Vec<Configuration<'_>>, ValueError> {
    let mut result = Vec::new();
    for (section, _) in &config.sections {
        let Some(section) = section else { continue };
        let Some(index) = indexed_name(section, "fltsim.")? else {
            continue;
        };
        result.push(Configuration {
            index,
            section,
            title: config.value(section, "title")?,
            sim: config.value(section, "sim")?,
            model: config.value(section, "model")?,
            panel: config.value(section, "panel")?,
            sound: config.value(section, "sound")?,
            texture: config.value(section, "texture")?,
        });
    }
    result.sort_by_key(|variant| variant.index);
    if let Some(pair) = result
        .windows(2)
        .find(|pair| pair[0].index == pair[1].index)
    {
        return Err(ValueError::Duplicate {
            section: pair[0].section.into(),
            key: "section".into(),
        });
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_flap_sets_without_filling_missing_detents() {
        let config = Config::parse("[flaps.2]\ntype=2\nextending-time=3\nflaps-position.4=1\nflaps-position.0=0\nflaps-position.1=0.5\n[Flaps.0]\ntype=1\nextending-time=7\nflaps-position.0=-7\nflaps-position.1=0\nflaps-position.2=40").unwrap();
        let sets = flaps(&config).unwrap();
        assert_eq!(
            sets[0],
            FlapSet {
                index: 0,
                kind: 1,
                extending_time: Some(7.0),
                system_type: None,
                positions: [(0, -7.0), (1, 0.0), (2, 40.0)]
                    .into_iter()
                    .map(|(index, angle)| FlapPosition {
                        index,
                        angle,
                        speed_limit: None
                    })
                    .collect()
            }
        );
        assert_eq!(
            sets[1]
                .positions
                .iter()
                .map(|p| (p.index, p.angle))
                .collect::<Vec<_>>(),
            [(0, 0.0), (1, 0.5), (4, 1.0)]
        );
        assert!(
            flaps(&Config::parse("[general]\ncategory=airplane").unwrap())
                .unwrap()
                .is_empty()
        );
        let limited = flaps(
            &Config::parse(
                "[flaps.0]\ntype=1\nextending-time=10\nflaps-position.0=0\nflaps-position.1=10,120",
            )
            .unwrap(),
        )
        .unwrap();
        assert_eq!(limited[0].positions[1].speed_limit, Some(120.0));
        let powered = flaps(&Config::parse("[flaps.0]\ntype=1\nsystem_type=1").unwrap()).unwrap();
        assert_eq!(powered[0].system_type, Some(1));
    }

    #[test]
    fn rejects_invalid_or_ambiguous_flap_definitions() {
        for text in [
            "[flaps.0]\ntype=3",
            "[flaps.0]\ntype=1\nTYPE=2",
            "[flaps.0]\nextending-time=0",
            "[flaps.0]\nextending-time=NaN",
            "[flaps.0]\nsystem_type=5",
            "[flaps.0]\nsystem_type=-1",
            "[flaps.0]\nflaps-position.0=NaN",
            "[flaps.0]\nflaps-position.0=10,20,30",
            "[flaps.0]\nflaps-position.0=10,NaN",
            "[flaps.0]\nflaps-position.0=0\nflaps-position.00=1",
            "[flaps.0]\nflaps-position.0=0\nFLAPS-POSITION.0=1",
            "[flaps.0]\n[flaps.00]",
        ] {
            assert!(flaps(&Config::parse(text).unwrap()).is_err(), "{text}");
        }
    }

    #[test]
    fn reads_reference_points_and_rejects_invalid_triplets() {
        let config = Config::parse("[weight_and_balance]\nreference_datum_position=3.6,0,0\nempty_weight_cg_position=-3,0,0").unwrap();
        let points = ReferencePoints::from_config(&config).unwrap();
        assert_eq!(points.datum_feet, Some([3.6, 0.0, 0.0]));
        assert_eq!(points.empty_cg_feet, Some([-3.0, 0.0, 0.0]));
        for value in ["1,2", "1,2,NaN", "1,2,3,4"] {
            assert!(
                ReferencePoints::from_config(
                    &Config::parse(&format!(
                        "[weight_and_balance]\nreference_datum_position={value}"
                    ))
                    .unwrap()
                )
                .is_err()
            );
        }
    }

    #[test]
    fn reads_configuration_references_without_resolving_paths() {
        let config = Config::parse("[Fltsim.3]\nTITLE=Example IFR\nsim=Example\nmodel=interior\npanel=ifr\nsound=\\Shared\\sound\ntexture=blue\ncustom=preserved\n[fltsim.0]\ntitle=Example\nsim=Example\nmodel=\n[General]\ncategory=airplane").unwrap();
        let variants = configurations(&config).unwrap();
        assert_eq!(variants.len(), 2);
        assert_eq!(variants[0].index, 0);
        assert_eq!(variants[0].model, Some(""));
        assert_eq!(variants[0].texture, None);
        assert_eq!(variants[1].title, Some("Example IFR"));
        assert_eq!(variants[1].sim, Some("Example"));
        assert_eq!(variants[1].panel, Some("ifr"));
        assert_eq!(variants[1].sound, Some("\\Shared\\sound"));
        assert_eq!(variants[1].texture, Some("blue"));
        assert_eq!(
            config.value("Fltsim.3", "custom").unwrap(),
            Some("preserved")
        );
    }

    #[test]
    fn rejects_ambiguous_variants_and_keys() {
        for text in [
            "[fltsim.0]\n[FLTSIM.00]\n",
            "[fltsim.4294967296]\n",
            "[fltsim.0]\nmodel=one\nMODEL=two",
            "[fltsim.0]\ntitle=one\ntitle=two",
        ] {
            assert!(
                configurations(&Config::parse(text).unwrap()).is_err(),
                "{text}"
            );
        }
        assert!(
            configurations(&Config::parse("[general]\ncategory=airplane").unwrap())
                .unwrap()
                .is_empty()
        );
    }
}

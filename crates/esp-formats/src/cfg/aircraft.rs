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

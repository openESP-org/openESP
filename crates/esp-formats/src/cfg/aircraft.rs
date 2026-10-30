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

use super::{Config, ValueError};

#[derive(Debug, PartialEq, Eq)]
pub struct Models<'a> {
    pub normal: Option<&'a str>,
    pub interior: Option<&'a str>,
    pub crash: Option<&'a str>,
    pub alias: Option<&'a str>,
}

impl<'a> Models<'a> {
    pub fn from_config(config: &'a Config) -> Result<Self, ValueError> {
        Ok(Self {
            normal: config.value("models", "normal")?,
            interior: config.value("models", "interior")?,
            crash: config.value("models", "crash")?,
            alias: config.value("fltsim", "alias")?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_model_names_and_aliases() {
        let config = Config::parse("[Models]\nNormal=Example\nInterior=Example_Interior\nCrash=damaged\n[custom]\nversion=unknown\n").unwrap();
        assert_eq!(
            Models::from_config(&config).unwrap(),
            Models {
                normal: Some("Example"),
                interior: Some("Example_Interior"),
                crash: Some("damaged"),
                alias: None,
            }
        );
        assert_eq!(config.value("custom", "version").unwrap(), Some("unknown"));
        let config = Config::parse("[fltsim]\nalias=OtherAircraft\\model\n").unwrap();
        let models = Models::from_config(&config).unwrap();
        assert_eq!(models.alias, Some("OtherAircraft\\model"));
        assert_eq!(models.normal, None);
    }

    #[test]
    fn rejects_ambiguous_model_references() {
        let config = Config::parse("[models]\nnormal=one\nNORMAL=two\n").unwrap();
        assert!(matches!(
            Models::from_config(&config),
            Err(ValueError::Duplicate { .. })
        ));
    }
}

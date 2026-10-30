use super::{Config, ValueError};
use std::collections::HashSet;

#[derive(Debug, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    #[error("missing sound section [{0}]")]
    MissingSection(String),
    #[error("sound list contains a cycle at [{0}]")]
    Cycle(String),
}

#[derive(Debug, PartialEq, Eq)]
pub struct Sound<'a> {
    pub section: &'a str,
    pub filenames: Vec<&'a str>,
    pub link: Option<&'a str>,
}

#[derive(Debug, PartialEq, Eq)]
pub struct Sounds<'a> {
    pub alias: Option<&'a str>,
    pub sounds: Vec<Sound<'a>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EngineEvent {
    Combustion,
    Starter,
    CombustionStart,
    Shutdown,
    JetWhine,
    NonCombustion,
    Propeller,
    PropellerStrike,
    Rotor,
}

#[derive(Debug, PartialEq, Eq)]
pub struct EngineList<'a> {
    pub engine: u32,
    pub event: EngineEvent,
    pub first: &'a str,
}

#[derive(Debug, PartialEq, Eq)]
pub struct Engines<'a> {
    pub count: Option<u32>,
    pub lists: Vec<EngineList<'a>>,
}

impl<'a> Engines<'a> {
    pub fn from_config(config: &'a Config) -> Result<Self, ValueError> {
        let count = config
            .value("sound_engine", "number_of_engines")?
            .map(|value| {
                value
                    .parse()
                    .map_err(|_| invalid("sound_engine", "number_of_engines", value))
            })
            .transpose()?;
        let mut lists = Vec::new();
        for engine in 1..=4 {
            for (suffix, event) in [
                ("combustion", EngineEvent::Combustion),
                ("starter", EngineEvent::Starter),
                ("combustion_start", EngineEvent::CombustionStart),
                ("shutdown", EngineEvent::Shutdown),
                ("jet_whine", EngineEvent::JetWhine),
                ("non_combustion", EngineEvent::NonCombustion),
                ("prop", EngineEvent::Propeller),
                ("prop_strike", EngineEvent::PropellerStrike),
                ("rotor", EngineEvent::Rotor),
            ] {
                let key = format!("eng{engine}_{suffix}");
                if let Some(first) = config.value("sound_engine", &key)? {
                    lists.push(EngineList {
                        engine,
                        event,
                        first,
                    });
                }
            }
        }
        Ok(Self { count, lists })
    }
}

#[derive(Debug, Default, PartialEq)]
pub struct Parameters {
    pub flags: Option<u32>,
    pub viewpoint: Option<u32>,
    pub pitch_envelope: Option<Vec<[f64; 2]>>,
    pub volume_envelope: Option<Vec<[f64; 2]>>,
    pub minimum_volume: Option<f64>,
    pub maximum_volume: Option<f64>,
    pub initial_volume: Option<f64>,
    pub minimum_rate: Option<f64>,
    pub maximum_rate: Option<f64>,
    pub minimum_speed: Option<f64>,
    pub maximum_speed: Option<f64>,
    pub inside_cone_angle: Option<f64>,
    pub outside_cone_angle: Option<f64>,
    pub cone_pitch: Option<f64>,
    pub cone_heading: Option<f64>,
    pub outside_cone_volume: Option<f64>,
    pub full_scale_distance: Option<f64>,
    pub panning: Option<f64>,
}

impl Parameters {
    pub fn from_config(config: &Config, section: &str) -> Result<Self, ValueError> {
        let number = |key| -> Result<Option<f64>, ValueError> {
            config
                .value(section, key)?
                .map(|value| {
                    value
                        .parse::<f64>()
                        .ok()
                        .filter(|value| value.is_finite())
                        .ok_or_else(|| invalid(section, key, value))
                })
                .transpose()
        };
        let integer = |key| -> Result<Option<u32>, ValueError> {
            config
                .value(section, key)?
                .map(|value| value.parse().map_err(|_| invalid(section, key, value)))
                .transpose()
        };
        Ok(Self {
            flags: integer("flags")?,
            viewpoint: integer("viewpoint")?,
            pitch_envelope: envelope(config, section, "rparams", 2)?,
            volume_envelope: envelope(config, section, "vparams", 8)?,
            minimum_volume: number("minimum_volume")?,
            maximum_volume: number("maximum_volume")?,
            initial_volume: number("initial_volume")?,
            minimum_rate: number("minimum_rate")?,
            maximum_rate: number("maximum_rate")?,
            minimum_speed: number("minimum_speed")?,
            maximum_speed: number("maximum_speed")?,
            inside_cone_angle: number("insideconeangle")?,
            outside_cone_angle: number("outsideconeangle")?,
            cone_pitch: number("conepitch")?,
            cone_heading: number("coneheading")?,
            outside_cone_volume: number("outsideconevolume")?,
            full_scale_distance: number("full_scale_distance")?,
            panning: number("panning")?,
        })
    }
}

fn invalid(section: &str, key: &str, value: &str) -> ValueError {
    ValueError::Invalid {
        section: section.into(),
        key: key.into(),
        value: value.into(),
    }
}

fn envelope(
    config: &Config,
    section: &str,
    key: &str,
    maximum: usize,
) -> Result<Option<Vec<[f64; 2]>>, ValueError> {
    let Some(value) = config.value(section, key)? else {
        return Ok(None);
    };
    let mut fields = value.split(',');
    let mut points = Vec::new();
    while let Some(input) = fields.next() {
        let output = fields.next().ok_or_else(|| invalid(section, key, value))?;
        let parse = |field: &str| {
            field
                .trim()
                .parse::<f64>()
                .ok()
                .filter(|value| value.is_finite())
                .ok_or_else(|| invalid(section, key, value))
        };
        if points.len() == maximum {
            return Err(invalid(section, key, value));
        }
        points.push([parse(input)?, parse(output)?]);
    }
    Ok(Some(points))
}

impl<'a> Sounds<'a> {
    pub fn from_config(config: &'a Config) -> Result<Self, ValueError> {
        let mut sounds = Vec::new();
        let mut seen = HashSet::new();
        for (name, _) in &config.sections {
            let Some(section) = name else { continue };
            if section.eq_ignore_ascii_case("fltsim")
                || section.eq_ignore_ascii_case("sound_engine")
            {
                continue;
            }
            if !seen.insert(section.to_ascii_lowercase()) {
                return Err(ValueError::Duplicate {
                    section: section.into(),
                    key: "section".into(),
                });
            }
            let filenames = config
                .value(section, "filename")?
                .map(|value| value.split(',').map(str::trim).collect())
                .unwrap_or_default();
            sounds.push(Sound {
                section,
                filenames,
                link: config.value(section, "link")?,
            });
        }
        Ok(Self {
            alias: config.value("fltsim", "alias")?,
            sounds,
        })
    }

    pub fn chain(&self, first: &str) -> Result<Vec<&Sound<'a>>, Error> {
        let mut chain = Vec::new();
        let mut visited = HashSet::new();
        let mut next = Some(first);
        while let Some(section) = next {
            let index = self
                .sounds
                .iter()
                .position(|sound| sound.section.eq_ignore_ascii_case(section))
                .ok_or_else(|| Error::MissingSection(section.into()))?;
            if !visited.insert(index) {
                return Err(Error::Cycle(self.sounds[index].section.into()));
            }
            let sound = &self.sounds[index];
            chain.push(sound);
            next = sound.link.filter(|link| !link.is_empty());
        }
        Ok(chain)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_sound_lists_and_alternative_filenames() {
        let config = Config::parse("[sound_engine]\neng1_starter=starter\n[Starter]\nfilename=xstart\nlink=STARTER.1\n[starter.1]\nfilename=shared\\next\nlink=\n[CRASH_SOUND]\nfilename=crash1, crash2\n[custom]\nunknown=retained").unwrap();
        let sounds = Sounds::from_config(&config).unwrap();
        let chain = sounds.chain("STARTER").unwrap();
        assert_eq!(chain.len(), 2);
        assert_eq!(chain[0].filenames, ["xstart"]);
        assert_eq!(chain[1].filenames, [r"shared\next"]);
        assert_eq!(
            sounds.chain("crash_sound").unwrap()[0].filenames,
            ["crash1", "crash2"]
        );
        assert_eq!(config.value("custom", "unknown").unwrap(), Some("retained"));
        assert_eq!(
            sounds.chain("custom").unwrap()[0].filenames,
            Vec::<&str>::new()
        );
    }

    #[test]
    fn preserves_aliases_and_empty_filenames() {
        let config = Config::parse("[fltsim]\nalias=OtherAircraft\\sound").unwrap();
        let sounds = Sounds::from_config(&config).unwrap();
        assert_eq!(sounds.alias, Some(r"OtherAircraft\sound"));
        assert!(sounds.sounds.is_empty());
        let config = Config::parse("[gear_up]\nfilename=").unwrap();
        assert_eq!(
            Sounds::from_config(&config).unwrap().sounds[0].filenames,
            [""]
        );
    }

    #[test]
    fn rejects_ambiguous_sections_keys_and_broken_chains() {
        for text in [
            "[sound]\nfilename=a\nFILENAME=b",
            "[sound]\nlink=a\nLINK=b",
            "[sound]\nfilename=a\n[SOUND]\nlink=b",
        ] {
            assert!(matches!(
                Sounds::from_config(&Config::parse(text).unwrap()),
                Err(ValueError::Duplicate { .. })
            ));
        }
        let config = Config::parse("[a]\nlink=b\n[b]\nlink=A\n[c]\nlink=missing").unwrap();
        let sounds = Sounds::from_config(&config).unwrap();
        assert!(matches!(sounds.chain("a"), Err(Error::Cycle(_))));
        assert_eq!(
            sounds.chain("c"),
            Err(Error::MissingSection("missing".into()))
        );
        assert_eq!(
            sounds.chain("absent"),
            Err(Error::MissingSection("absent".into()))
        );
    }

    #[test]
    fn reads_envelopes_and_optional_playback_parameters() {
        let config = Config::parse("[wind_sound]\nrparams=16, 0.66, 100,1.464\nvparams=0,0,30.5,11.2,66.3,32.8\nminimum_volume=10000.000\nconepitch=-40\npanning=-10000\nfull_scale_distance=7e1\nflags=2147483648\nviewpoint=7").unwrap();
        let parameters = Parameters::from_config(&config, "wind_sound").unwrap();
        assert_eq!(
            parameters.pitch_envelope,
            Some(vec![[16.0, 0.66], [100.0, 1.464]])
        );
        assert_eq!(
            parameters.volume_envelope,
            Some(vec![[0.0, 0.0], [30.5, 11.2], [66.3, 32.8]])
        );
        assert_eq!(parameters.minimum_volume, Some(10000.0));
        assert_eq!(parameters.cone_pitch, Some(-40.0));
        assert_eq!(parameters.panning, Some(-10000.0));
        assert_eq!(parameters.full_scale_distance, Some(70.0));
        assert_eq!(parameters.flags, Some(0x80000000));
        assert_eq!(parameters.viewpoint, Some(7));
        assert_eq!(parameters.maximum_volume, None);
    }

    #[test]
    fn rejects_malformed_numbers_and_envelopes() {
        for value in [
            "rparams=",
            "rparams=0",
            "rparams=0,1,2",
            "rparams=0,1,2,3,4,5",
            "vparams=0,NaN",
            "vparams=0,inf",
            "rparams=0,1,",
            "flags=-1",
            "minimum_rate=NaN",
            "viewpoint=1.5",
            "panning=bad",
            "panning=10,000",
        ] {
            let config = Config::parse(&format!("[a]\n{value}")).unwrap();
            assert!(
                matches!(
                    Parameters::from_config(&config, "a"),
                    Err(ValueError::Invalid { .. })
                ),
                "{value}"
            );
        }
        for count in [8, 9] {
            let config =
                Config::parse(&format!("[a]\nvparams={}", vec!["0,1"; count].join(","))).unwrap();
            assert_eq!(Parameters::from_config(&config, "a").is_ok(), count == 8);
        }
        let config = Config::parse("[a]\nmaximum_rate=1\nMAXIMUM_RATE=2").unwrap();
        assert!(matches!(
            Parameters::from_config(&config, "a"),
            Err(ValueError::Duplicate { .. })
        ));
    }

    #[test]
    fn reads_engine_events_without_renumbering() {
        let config = Config::parse("[SOUND_ENGINE]\nnumber_of_engines=4\nENG1_COMBUSTION=power\neng2_jet_whine=whine\neng3_prop=propeller\neng4_rotor=\n[power]\nfilename=engine").unwrap();
        let engines = Engines::from_config(&config).unwrap();
        assert_eq!(engines.count, Some(4));
        assert_eq!(
            engines.lists,
            [
                EngineList {
                    engine: 1,
                    event: EngineEvent::Combustion,
                    first: "power"
                },
                EngineList {
                    engine: 2,
                    event: EngineEvent::JetWhine,
                    first: "whine"
                },
                EngineList {
                    engine: 3,
                    event: EngineEvent::Propeller,
                    first: "propeller"
                },
                EngineList {
                    engine: 4,
                    event: EngineEvent::Rotor,
                    first: ""
                },
            ]
        );
        assert_eq!(
            Sounds::from_config(&config)
                .unwrap()
                .chain(engines.lists[0].first)
                .unwrap()[0]
                .filenames,
            ["engine"]
        );
        assert_eq!(
            Engines::from_config(&Config::parse("").unwrap()).unwrap(),
            Engines {
                count: None,
                lists: vec![]
            }
        );
    }

    #[test]
    fn rejects_ambiguous_engine_lists_and_invalid_counts() {
        for text in [
            "number_of_engines=-1",
            "number_of_engines=4294967296",
            "number_of_engines=2\nNUMBER_OF_ENGINES=3",
            "eng1_prop=a\nENG1_PROP=b",
        ] {
            let config = Config::parse(&format!("[sound_engine]\n{text}")).unwrap();
            assert!(Engines::from_config(&config).is_err());
        }
    }
}

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
}

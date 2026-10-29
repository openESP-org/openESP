use ini::{Ini, ParseError, ParseOption};

#[derive(Debug)]
pub struct Config {
    pub sections: Ini,
}

impl Config {
    pub fn parse(text: &str) -> Result<Self, ParseError> {
        let mut normalized = String::with_capacity(text.len());
        for line in text.trim_start_matches('\u{feff}').lines() {
            let mut quoted = false;
            let mut end = line.len();
            for (index, character) in line.char_indices() {
                if character == '"' {
                    quoted = !quoted;
                }
                if !quoted && (character == ';' || line[index..].starts_with("//")) {
                    end = index;
                    break;
                }
            }
            normalized.push_str(&line[..end]);
            normalized.push('\n');
        }
        let sections = Ini::load_from_str_opt(
            &normalized,
            ParseOption {
                enabled_escape: false,
                ..ParseOption::default()
            },
        )?;
        Ok(Self { sections })
    }

    pub fn values<'a>(&'a self, section: &'a str, key: &'a str) -> impl Iterator<Item = &'a str> {
        self.sections
            .iter()
            .filter_map(move |(name, properties)| {
                name.filter(|name| name.eq_ignore_ascii_case(section))
                    .map(|_| properties)
            })
            .flat_map(move |properties| {
                properties.iter().filter_map(move |(name, value)| {
                    name.eq_ignore_ascii_case(key).then_some(value)
                })
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_fsx_paths_comments_and_quoted_values() {
        let config = Config::parse("\u{feff}[fltsim.0]\r\ntitle=\"Example; aircraft\" // comment\r\ntexture=..\\Shared\\texture;comment\r\nmodel=\r\n").unwrap();
        assert_eq!(
            config.values("FLTSIM.0", "TITLE").collect::<Vec<_>>(),
            ["Example; aircraft"]
        );
        assert_eq!(
            config.values("fltsim.0", "texture").collect::<Vec<_>>(),
            ["..\\Shared\\texture"]
        );
        assert_eq!(config.values("fltsim.0", "model").collect::<Vec<_>>(), [""]);
    }

    #[test]
    fn retains_unknown_and_duplicate_entries() {
        let config = Config::parse("[custom]\nitem=first\nitem=second\n").unwrap();
        assert_eq!(
            config.values("CUSTOM", "ITEM").collect::<Vec<_>>(),
            ["first", "second"]
        );
        assert!(Config::parse("[unfinished\nkey=value").is_err());
        assert!(
            Config::parse("")
                .unwrap()
                .values("missing", "key")
                .next()
                .is_none()
        );
    }
}

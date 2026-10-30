use encoding_rs::Encoding;
use ini::{Ini, ParseError, ParseOption};

pub mod aircraft;
pub mod camera;
pub mod model;
pub mod panel;
pub mod sound;
pub mod texture;

#[derive(Debug, PartialEq, Eq, thiserror::Error)]
pub enum ValueError {
    #[error("missing [{section}] {key}")]
    Missing { section: String, key: String },
    #[error("duplicate [{section}] {key}")]
    Duplicate { section: String, key: String },
    #[error("invalid texture fallback index {0}")]
    FallbackIndex(String),
    #[error("invalid numeric index {0}")]
    Index(String),
    #[error("invalid [{section}] {key}: {value}")]
    Invalid {
        section: String,
        key: String,
        value: String,
    },
}

pub(crate) fn indexed_name(name: &str, prefix: &str) -> Result<Option<u32>, ValueError> {
    let Some(start) = name.get(..prefix.len()) else {
        return Ok(None);
    };
    if !start.eq_ignore_ascii_case(prefix) {
        return Ok(None);
    }
    let suffix = &name[prefix.len()..];
    if suffix.is_empty() || !suffix.bytes().all(|byte| byte.is_ascii_digit()) {
        return Ok(None);
    }
    suffix
        .parse()
        .map(Some)
        .map_err(|_| ValueError::Index(name.into()))
}

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("invalid {0} text")]
    Encoding(&'static str),
    #[error(transparent)]
    Syntax(#[from] ParseError),
}

#[derive(Debug)]
pub struct Config {
    pub sections: Ini,
}

impl Config {
    pub fn value(&self, section: &str, key: &str) -> Result<Option<&str>, ValueError> {
        let mut matches = self
            .sections
            .iter()
            .filter(|(name, _)| name.is_some_and(|name| name.eq_ignore_ascii_case(section)))
            .flat_map(|(_, properties)| properties.iter())
            .filter_map(|(name, value)| name.eq_ignore_ascii_case(key).then_some(value));
        let value = matches.next();
        if matches.next().is_some() {
            return Err(ValueError::Duplicate {
                section: section.into(),
                key: key.into(),
            });
        }
        Ok(value)
    }

    pub fn from_bytes(data: &[u8], fallback: &'static Encoding) -> Result<Self, Error> {
        let (encoding, skip) = Encoding::for_bom(data).unwrap_or((fallback, 0));
        let text = encoding
            .decode_without_bom_handling_and_without_replacement(&data[skip..])
            .ok_or(Error::Encoding(encoding.name()))?;
        Ok(Self::parse(&text)?)
    }

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
    fn decodes_explicit_code_pages_and_bom_text() {
        use encoding_rs::{UTF_8, WINDOWS_1252};
        let cfg = Config::from_bytes(b"[fltsim.0]\ntitle=Caf\xe9", WINDOWS_1252).unwrap();
        assert_eq!(cfg.values("fltsim.0", "title").next(), Some("Café"));
        assert!(Config::from_bytes(b"[fltsim.0]\ntitle=Caf\xe9", UTF_8).is_err());
        let mut utf16 = vec![0xff, 0xfe];
        utf16.extend(
            "[models]\nnormal=模型"
                .encode_utf16()
                .flat_map(u16::to_le_bytes),
        );
        let cfg = Config::from_bytes(&utf16, WINDOWS_1252).unwrap();
        assert_eq!(cfg.values("models", "normal").next(), Some("模型"));
        utf16.pop();
        assert!(matches!(
            Config::from_bytes(&utf16, WINDOWS_1252),
            Err(Error::Encoding(_))
        ));
    }

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

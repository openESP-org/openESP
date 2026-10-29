use super::{Config, ValueError};

#[derive(Debug, PartialEq, Eq)]
pub struct Fallback<'a> {
    pub index: u32,
    pub path: &'a str,
}

pub fn fallbacks(config: &Config) -> Result<Vec<Fallback<'_>>, ValueError> {
    let mut result = Vec::new();
    for (section, properties) in &config.sections {
        if !section.is_some_and(|name| name.eq_ignore_ascii_case("fltsim")) {
            continue;
        }
        for (key, path) in properties {
            let Some((prefix, suffix)) = key.split_once('.') else {
                continue;
            };
            if !prefix.eq_ignore_ascii_case("fallback") {
                continue;
            }
            if suffix.is_empty() || !suffix.bytes().all(|byte| byte.is_ascii_digit()) {
                return Err(ValueError::FallbackIndex(key.into()));
            }
            let index = suffix
                .parse()
                .map_err(|_| ValueError::FallbackIndex(key.into()))?;
            result.push(Fallback { index, path });
        }
    }
    result.sort_by_key(|entry| entry.index);
    if let Some(pair) = result
        .windows(2)
        .find(|pair| pair[0].index == pair[1].index)
    {
        return Err(ValueError::Duplicate {
            section: "fltsim".into(),
            key: format!("fallback.{}", pair[0].index),
        });
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orders_fallback_paths_by_index() {
        let config = Config::parse("[fltsim]\nfallback.10=..\\..\\Texture\nFallback.1=..\\Texture\nfallback.3=Z:\\Shared\\Texture\nother=value\n").unwrap();
        assert_eq!(
            fallbacks(&config).unwrap(),
            [
                Fallback {
                    index: 1,
                    path: "..\\Texture"
                },
                Fallback {
                    index: 3,
                    path: "Z:\\Shared\\Texture"
                },
                Fallback {
                    index: 10,
                    path: "..\\..\\Texture"
                },
            ]
        );
    }

    #[test]
    fn rejects_invalid_and_ambiguous_indices() {
        for key in [
            "fallback.x",
            "fallback.",
            "fallback.4294967296",
            "fallback.-1",
        ] {
            let config = Config::parse(&format!("[fltsim]\n{key}=path")).unwrap();
            assert!(matches!(
                fallbacks(&config),
                Err(ValueError::FallbackIndex(_))
            ));
        }
        let config = Config::parse("[fltsim]\nfallback.1=one\nfallback.01=two").unwrap();
        assert!(matches!(
            fallbacks(&config),
            Err(ValueError::Duplicate { .. })
        ));
    }
}

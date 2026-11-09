//! Read stock airport records from an installed simulator's regional BGLs.
use anyhow::{Context, Result};
use esp_formats::bgl::{Bgl, airport::Airport};
use std::path::{Path, PathBuf};

pub struct FoundAirport {
    pub path: PathBuf,
    pub airport: Airport,
}

/// Search stock regional APX files for an ICAO identifier, without writing to the installation.
pub fn find(root: &Path, ident: &str) -> Result<Option<FoundAirport>> {
    let scenery = root.join("Scenery");
    let mut regions = std::fs::read_dir(&scenery)
        .with_context(|| format!("reading {}", scenery.display()))?
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.is_dir()
                && path.file_name().is_some_and(|name| {
                    let name = name.to_string_lossy();
                    name.len() == 4 && name.bytes().all(|byte| byte.is_ascii_digit())
                })
        })
        .collect::<Vec<_>>();
    regions.sort();
    for region in regions {
        let directory = region.join("scenery");
        let Ok(entries) = std::fs::read_dir(&directory) else {
            continue;
        };
        let mut files = entries
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .filter(|path| {
                path.is_file()
                    && path
                        .extension()
                        .is_some_and(|ext| ext.eq_ignore_ascii_case("bgl"))
                    && path.file_name().is_some_and(|name| {
                        name.to_string_lossy()
                            .to_ascii_uppercase()
                            .starts_with("APX")
                    })
            })
            .collect::<Vec<_>>();
        files.sort();
        for path in files {
            let Ok(bytes) = std::fs::read(&path) else {
                continue;
            };
            let Ok(bgl) = Bgl::parse(&bytes) else {
                continue;
            };
            let Ok(airports) = bgl.airports() else {
                continue;
            };
            if let Some(airport) = airports.into_iter().find(|airport| {
                airport.ident.eq_ignore_ascii_case(ident)
                    && (!airport.runways.is_empty() || !airport.helipads.is_empty())
            }) {
                return Ok(Some(FoundAirport { path, airport }));
            }
        }
    }
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    #[ignore = "requires installed Prepar3D in OPENESP_P3D_ROOT"]
    fn finds_real_stock_airports_by_icao() {
        let root = PathBuf::from(std::env::var_os("OPENESP_P3D_ROOT").unwrap());
        for (ident, latitude, longitude) in [("KLAX", 33.94, -118.40), ("EGLL", 51.47, -0.46)] {
            let found = find(&root, ident).unwrap().unwrap();
            assert_eq!(found.airport.ident, ident);
            assert!((found.airport.latitude - latitude).abs() < 0.1);
            assert!((found.airport.longitude - longitude).abs() < 0.1);
            assert!(!found.airport.runways.is_empty());
        }
    }
}

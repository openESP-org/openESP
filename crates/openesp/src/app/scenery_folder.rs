use anyhow::{Context, Result, ensure};
use esp_formats::bgl::Bgl;
use std::path::{Path, PathBuf};

use crate::cli::Arguments;

struct Discovery {
    airport: PathBuf,
    ident: String,
    latitude: f64,
    longitude: f64,
    objects: Vec<PathBuf>,
}

pub(super) fn configure(arguments: &mut Arguments) -> Result<()> {
    let Some(folder) = arguments.scenery.as_deref() else {
        return Ok(());
    };
    let discovery = discover(folder, arguments.airport.as_deref())?;
    arguments.scenery_bgl = Some(discovery.airport);
    arguments.airport = Some(discovery.ident);
    arguments.latitude = discovery.latitude;
    arguments.longitude = discovery.longitude;
    if let Some(first) = discovery.objects.first() {
        arguments.scenery_objects = Some(first.clone());
        arguments.scenery_library = discovery.objects.into_iter().skip(1).collect();
    }
    Ok(())
}

fn discover(folder: &Path, ident: Option<&str>) -> Result<Discovery> {
    ensure!(
        folder.is_dir(),
        "scenery folder {} does not exist",
        folder.display()
    );
    let directory = esp_assets::resolve_path(folder, "scenery")
        .ok()
        .filter(|path| path.is_dir())
        .unwrap_or_else(|| folder.to_path_buf());
    let mut files = Vec::new();
    for entry in
        std::fs::read_dir(&directory).with_context(|| format!("reading {}", directory.display()))?
    {
        let path = entry?.path();
        if path.is_file()
            && path
                .extension()
                .is_some_and(|ext| ext.eq_ignore_ascii_case("bgl"))
        {
            files.push(path);
        }
    }
    files.sort();
    ensure!(
        !files.is_empty(),
        "no BGL files found in {}",
        directory.display()
    );
    let mut selected = None;
    let mut objects = Vec::new();
    for path in files {
        let bytes = std::fs::read(&path).with_context(|| format!("reading {}", path.display()))?;
        let bgl = Bgl::parse(&bytes).with_context(|| format!("parsing {}", path.display()))?;
        if bgl.sections.iter().any(|section| section.kind == 3) {
            for airport in bgl
                .airports()
                .with_context(|| format!("decoding airports in {}", path.display()))?
            {
                if selected.is_none()
                    && (!airport.runways.is_empty() || !airport.helipads.is_empty())
                    && ident.is_none_or(|ident| airport.ident.eq_ignore_ascii_case(ident))
                {
                    selected = Some((path.clone(), airport));
                }
            }
        }
        if bgl
            .sections
            .iter()
            .any(|section| matches!(section.kind, 0x25 | 0x2b))
        {
            objects.push(path);
        }
    }
    let (airport_path, airport) = selected.with_context(|| {
        format!(
            "airport {} not found in {}",
            ident.unwrap_or("with runways or helipads"),
            directory.display()
        )
    })?;
    eprintln!(
        "scenery: discovered {} at {}, {} object/library BGLs",
        airport.ident,
        airport_path.display(),
        objects.len()
    );
    Ok(Discovery {
        airport: airport_path,
        ident: airport.ident,
        latitude: airport.latitude,
        longitude: airport.longitude,
        objects,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;

    #[test]
    fn folder_source_accepts_airport_and_excludes_explicit_files() {
        let base = [
            "openesp",
            "--aircraft",
            "test",
            "--scenery",
            "addon",
            "--airport",
            "LTFO",
        ];
        assert!(Arguments::try_parse_from(base).is_ok());
        let mut both = base.to_vec();
        both.extend(["--scenery-bgl", "airport.bgl"]);
        assert!(Arguments::try_parse_from(both).is_err());
        assert!(
            Arguments::try_parse_from(["openesp", "--aircraft", "test", "--airport", "LTFO"])
                .is_err()
        );
        let empty = tempfile::tempdir().unwrap();
        assert!(discover(empty.path(), None).is_err());
    }

    #[test]
    #[ignore = "requires LTFO add-on folder in OPENESP_SCENERY_FOLDER"]
    fn discovers_real_addon_and_supports_direct_scenery_directory() {
        let folder = PathBuf::from(std::env::var_os("OPENESP_SCENERY_FOLDER").unwrap());
        let package = discover(&folder, Some("ltfo")).unwrap();
        let direct = discover(&folder.join("Scenery"), Some("LTFO")).unwrap();
        assert_eq!(package.airport, direct.airport);
        assert_eq!(package.objects, direct.objects);
        assert!(package.objects.len() > 10);
        assert!(
            package.objects.iter().any(|path| path
                .file_name()
                .unwrap()
                .to_string_lossy()
                .contains("GP"))
        );
        assert!(discover(&folder, Some("ZZZZ")).is_err());
        let mut args = Arguments::parse_from([
            "openesp",
            "--aircraft",
            "test",
            "--scenery",
            folder.to_str().unwrap(),
            "--airport",
            "LTFO",
        ]);
        configure(&mut args).unwrap();
        assert_eq!(args.airport.as_deref(), Some("LTFO"));
        assert!(args.scenery_objects.is_some());
        assert!(args.latitude.abs() > 1.0 && args.longitude.abs() > 1.0);
        let models = crate::scenery::objects(
            args.scenery_objects.as_deref().unwrap(),
            &args.scenery_library,
            None,
            args.latitude,
            args.longitude,
        )
        .unwrap();
        assert!(models.len() >= 448);
    }
}

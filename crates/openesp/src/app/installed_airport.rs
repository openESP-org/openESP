//! Select a stock airport from the configured or discovered simulator install.
use crate::cli::Arguments;
use anyhow::{Context, Result};

pub(super) fn configure(arguments: &mut Arguments) -> Result<()> {
    let Some(ident) = arguments.airport.as_deref() else {
        return Ok(());
    };
    if arguments.scenery_bgl.is_some() {
        return Ok(());
    }
    let root = arguments
        .fsx_root
        .clone()
        .or_else(|| {
            esp_installs::discover()
                .into_iter()
                .next()
                .map(|install| install.root)
        })
        .context("no simulator installation found; set --fsx-root or supply scenery")?;
    let found = esp_nav::airports::find(&root, ident)?
        .with_context(|| format!("stock airport {ident} not found in {}", root.display()))?;
    eprintln!(
        "scenery: found {} in {}",
        found.airport.ident,
        found.path.display()
    );
    arguments.latitude = found.airport.latitude;
    arguments.longitude = found.airport.longitude;
    if arguments.scenery_objects.is_none() {
        arguments.scenery_objects = Some(found.path.clone());
    }
    arguments.scenery_bgl = Some(found.path);
    arguments.fsx_root = Some(root);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::Parser;

    #[test]
    #[ignore = "requires installed Prepar3D in OPENESP_P3D_ROOT"]
    fn configures_stock_airport_and_runway_start() {
        let root = std::env::var("OPENESP_P3D_ROOT").unwrap();
        let mut args = Arguments::parse_from([
            "openesp",
            "--aircraft",
            "test",
            "--fsx-root",
            &root,
            "--airport",
            "KLAX",
            "--runway",
            "25R",
            "--engines-running",
        ]);
        configure(&mut args).unwrap();
        super::super::spawn::configure(&mut args).unwrap();
        assert!(args.scenery_bgl.as_ref().unwrap().is_file());
        assert!(args.scenery_objects.as_ref().unwrap().is_file());
        assert!((args.latitude - 33.94).abs() < 0.1);
        assert!((args.longitude + 118.4).abs() < 0.1);
        assert_eq!(args.airspeed, Some(0.0));
        assert!(args.engines_running);
        assert!(args.heading.unwrap().is_finite());
        let objects = crate::scenery::objects(
            args.scenery_objects.as_deref().unwrap(),
            &args.scenery_library,
            None,
            args.latitude,
            args.longitude,
            None,
            None,
            args.fsx_root.as_deref(),
        )
        .unwrap();
        assert!(
            !objects.is_empty(),
            "stock airport objects should resolve from installed global libraries"
        );
        eprintln!("stock KLAX: {} placed objects", objects.len());
    }
}

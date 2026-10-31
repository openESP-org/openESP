use std::path::PathBuf;

pub fn aircraft_paths() -> Vec<PathBuf> {
    let paths = if let Some(paths) = std::env::var_os("OPENESP_AIRCRAFT_PATHS") {
        std::env::split_paths(&paths).collect::<Vec<_>>()
    } else {
        let root = std::env::var_os("OPENESP_FSX_ROOT")
            .expect("set OPENESP_FSX_ROOT to an FSX installation or OPENESP_AIRCRAFT_PATHS to aircraft folders");
        let directory = PathBuf::from(root).join("SimObjects/Airplanes");
        std::fs::read_dir(&directory)
            .unwrap_or_else(|error| panic!("{}: {error}", directory.display()))
            .map(|entry| entry.expect("read aircraft directory").path())
            .filter(|path| path.is_dir() && path.join("aircraft.cfg").is_file())
            .collect()
    };
    assert!(
        !paths.is_empty(),
        "no external aircraft folders were configured"
    );
    for path in &paths {
        assert!(path.is_dir(), "missing aircraft folder {}", path.display());
    }
    let mut paths = paths;
    paths.sort();
    paths
}

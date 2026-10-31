use std::path::PathBuf;

#[path = "content.rs"]
mod content;

pub fn model_paths() -> Vec<PathBuf> {
    let mut paths = Vec::new();
    if let Some(files) = std::env::var_os("ESP_FORMATS_MDL_FILES") {
        paths.extend(std::env::split_paths(&files));
    } else {
        for aircraft in content::aircraft_paths() {
            for entry in std::fs::read_dir(&aircraft).expect("read aircraft folder") {
                let entry = entry.expect("read aircraft entry");
                let name = entry.file_name().to_string_lossy().to_ascii_lowercase();
                if entry.path().is_dir() && (name == "model" || name.starts_with("model.")) {
                    for file in std::fs::read_dir(entry.path()).expect("read model folder") {
                        let path = file.expect("read model entry").path();
                        if path
                            .extension()
                            .is_some_and(|e| e.eq_ignore_ascii_case("mdl"))
                        {
                            paths.push(path);
                        }
                    }
                }
            }
        }
    }
    assert!(!paths.is_empty(), "no external MDL files were configured");
    paths.sort();
    paths
}

//! Resolve requested scenery models from the simulator's global library.
use esp_assets::Model;
use std::{
    collections::{HashMap, HashSet},
    path::{Path, PathBuf},
    sync::Arc,
};

type Models = HashMap<[u8; 16], (Arc<Model>, PathBuf)>;

pub(super) fn load(root: &Path, missing: &mut HashSet<[u8; 16]>, models: &mut Models) {
    if missing.is_empty() {
        return;
    }
    let Ok(global) = esp_assets::resolve_path(root, "Scenery/Global") else {
        return;
    };
    let Ok(directory) = esp_assets::resolve_path(&global, "scenery") else {
        return;
    };
    let Ok(textures) = esp_assets::resolve_path(&global, "texture") else {
        return;
    };
    let Ok(entries) = std::fs::read_dir(&directory) else {
        return;
    };
    let mut paths: Vec<_> = entries
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.is_file()
                && path
                    .extension()
                    .is_some_and(|ext| ext.eq_ignore_ascii_case("bgl"))
        })
        .collect();
    paths.sort();
    let before = missing.len();
    for path in paths {
        if missing.is_empty() {
            break;
        }
        let Ok(bytes) = std::fs::read(&path) else {
            continue;
        };
        let Ok(bgl) = esp_formats::bgl::Bgl::parse(&bytes) else {
            continue;
        };
        for section in bgl
            .sections
            .iter()
            .filter(|section| section.kind == 0x2b && section.class == 5)
        {
            let Ok(records) = section.guid_records() else {
                continue;
            };
            for record in records {
                let guid = *record.guid.as_bytes();
                if !missing.contains(&guid) {
                    continue;
                }
                match Model::from_bytes(record.data) {
                    Ok(model) => {
                        models.insert(guid, (Arc::new(model), textures.clone()));
                        missing.remove(&guid);
                    }
                    Err(error) => eprintln!("global scenery model {}: {error:#}", record.guid),
                }
            }
        }
    }
    eprintln!(
        "scenery: resolved {} global models ({} references unavailable)",
        before - missing.len(),
        missing.len()
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore = "requires installed Prepar3D in OPENESP_P3D_ROOT"]
    fn resolves_requested_model_from_real_global_library() {
        let root = PathBuf::from(std::env::var_os("OPENESP_P3D_ROOT").unwrap());
        let file =
            esp_assets::resolve_path(&root, "Scenery/Global/scenery/airport_objects.bgl").unwrap();
        let bytes = std::fs::read(file).unwrap();
        let bgl = esp_formats::bgl::Bgl::parse(&bytes).unwrap();
        let guid = bgl
            .sections
            .iter()
            .filter(|section| section.kind == 0x2b && section.class == 5)
            .flat_map(|section| section.guid_records().unwrap())
            .find(|record| Model::from_bytes(record.data).is_ok())
            .map(|record| *record.guid.as_bytes())
            .unwrap();
        let mut missing = HashSet::from([guid]);
        let mut models = HashMap::new();
        load(&root, &mut missing, &mut models);
        assert!(missing.is_empty());
        let (model, textures) = &models[&guid];
        assert!(textures.is_dir());
        assert!(model.lods.iter().any(|lod| !lod.meshes.is_empty()));
    }
}

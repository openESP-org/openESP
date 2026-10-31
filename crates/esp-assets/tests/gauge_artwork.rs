use esp_assets::texture::gauge_bitmap;
use esp_formats::cabinet::Cabinet;

#[path = "../../../tests/support/content.rs"]
mod content;

#[test]
#[ignore = "requires OPENESP_FSX_ROOT or OPENESP_AIRCRAFT_PATHS with a gauge CAB"]
fn decodes_external_gauge_artwork() {
    let mut decoded = 0;
    for aircraft in content::aircraft_paths() {
        let Ok(panel) = esp_assets::resolve_path(&aircraft, "panel") else {
            continue;
        };
        for entry in std::fs::read_dir(panel).unwrap() {
            let path = entry.unwrap().path();
            if !path
                .extension()
                .is_some_and(|ext| ext.eq_ignore_ascii_case("cab"))
            {
                continue;
            }
            let mut cabinet = Cabinet::new(std::fs::File::open(&path).unwrap()).unwrap();
            let names: Vec<_> = cabinet
                .entries()
                .iter()
                .filter(|e| e.name.to_ascii_lowercase().ends_with(".bmp"))
                .map(|e| e.name.clone())
                .collect();
            for name in names.iter().take(5) {
                let bytes = cabinet.read_file_bytes(name, 64 * 1024 * 1024).unwrap();
                let image = gauge_bitmap(&bytes, false, true)
                    .unwrap_or_else(|e| panic!("{}: {name}: {e}", path.display()));
                assert!(image.width() > 0 && image.height() > 0);
                decoded += 1;
            }
        }
    }
    assert!(decoded > 0, "no gauge bitmap artwork found");
    eprintln!("decoded {decoded} gauge bitmaps");
}

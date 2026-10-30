use esp_formats::cfg::{Config, sound::Sounds};

#[test]
#[ignore = "requires ESP_FORMATS_SOUND_FILES containing paths to external sound.cfg files"]
fn parses_external_sound_lists() {
    let paths = std::env::var_os("ESP_FORMATS_SOUND_FILES").expect("set ESP_FORMATS_SOUND_FILES");
    let mut count = 0;
    for path in std::env::split_paths(&paths) {
        let data = std::fs::read(&path).unwrap();
        let config = Config::from_bytes(&data, encoding_rs::WINDOWS_1252).unwrap();
        let sounds =
            Sounds::from_config(&config).unwrap_or_else(|error| panic!("{path:?}: {error}"));
        for sound in &sounds.sounds {
            sounds
                .chain(sound.section)
                .unwrap_or_else(|error| panic!("{path:?}: {error}"));
        }
        count += 1;
    }
    assert!(count > 0);
}

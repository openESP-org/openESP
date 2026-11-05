use esp_assets::sound::SoundSet;
use esp_formats::cfg::sound::EngineEvent;

#[path = "../../../tests/support/content.rs"]
mod content;

#[test]
#[ignore = "requires aircraft with sound.cfg and OPENESP_AIRCRAFT_PATHS"]
fn loads_real_engine_layers_and_samples() {
    for path in content::aircraft_paths() {
        let sounds = SoundSet::load(&path, 0, None).unwrap();
        let layers = sounds.engine_layers(1, EngineEvent::Combustion).unwrap();
        assert!(!layers.is_empty());
        let sample = sounds.sample(&layers[0].filenames[0]).unwrap();
        assert!(sample.channels > 0 && sample.sample_rate > 0 && !sample.samples.is_empty());
        assert!(sample.samples.iter().all(|sample| sample.is_finite()));
    }
}

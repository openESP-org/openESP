use esp_formats::wave::{Wave, core::formats::FormatReader, get_codecs};

#[test]
#[ignore = "requires ESP_FORMATS_WAVE_FILES containing paths to external WAV files"]
fn decodes_external_sound_resources() {
    let paths = std::env::var_os("ESP_FORMATS_WAVE_FILES").expect("set ESP_FORMATS_WAVE_FILES");
    let mut count = 0;
    for path in std::env::split_paths(&paths) {
        let data = std::fs::read(&path).unwrap();
        let wave = Wave::parse(&data).unwrap_or_else(|error| panic!("{path:?}: {error}"));
        let mut reader = wave
            .reader()
            .unwrap_or_else(|error| panic!("{path:?}: {error}"));
        let params = reader.tracks()[0]
            .codec_params
            .as_ref()
            .unwrap()
            .audio()
            .unwrap();
        let mut decoder = get_codecs()
            .make_audio_decoder(params, &Default::default())
            .unwrap();
        let mut frames = 0;
        while let Some(packet) = reader.next_packet().unwrap() {
            frames += decoder.decode(&packet).unwrap().frames();
        }
        assert!(frames > 0, "{path:?}");
        count += 1;
    }
    assert!(count > 0);
}

use esp_assets::sound::decode;

#[test]
#[ignore = "requires OPENESP_OGG_FILE containing an Ogg Vorbis sample"]
fn decodes_external_ogg_vorbis() {
    let path = std::env::var_os("OPENESP_OGG_FILE").expect("set OPENESP_OGG_FILE");
    let sample = decode(&std::fs::read(path).unwrap()).unwrap();
    assert!((1..=2).contains(&sample.channels));
    assert!(sample.sample_rate > 0);
    assert!(!sample.samples.is_empty());
    assert!(sample.samples.iter().all(|value| value.is_finite()));
    assert!(sample.samples.iter().any(|value| value.abs() > 0.001));
}

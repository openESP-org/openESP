use esp_assets::sound::SoundSet;
use esp_audio::engine::{EngineBank, EngineState};
use esp_audio::{Command, Mixer, Spatial, VoiceId};
use esp_formats::cfg::sound::EngineEvent;
use std::sync::Arc;

#[path = "../../../tests/support/content.rs"]
mod content;

#[test]
#[ignore = "requires aircraft sound files and OPENESP_AIRCRAFT_PATHS"]
fn real_jet_cone_changes_volume_between_front_and_rear() {
    for path in content::aircraft_paths() {
        let set = SoundSet::load(&path, 0, None).unwrap();
        let find_layer = || {
            set.engine_layers(1, EngineEvent::JetWhine)
                .unwrap()
                .into_iter()
                .find(|layer| {
                    layer.parameters.viewpoint == Some(2)
                        && layer
                            .parameters
                            .outside_cone_angle
                            .is_some_and(|angle| angle < 360.0)
                        && layer
                            .parameters
                            .outside_cone_volume
                            .is_some_and(|volume| volume < 10_000.0)
                })
                .expect("missing exterior jet cone")
        };
        let layer = find_layer();
        let sample = Arc::new(set.sample(&layer.filenames[0]).unwrap());
        let heading = layer.parameters.cone_heading.unwrap_or(0.0).to_radians();
        let pitch = layer.parameters.cone_pitch.unwrap_or(0.0).to_radians();
        let axis = [
            heading.sin() * pitch.cos(),
            pitch.sin(),
            heading.cos() * pitch.cos(),
        ];
        let expected =
            10.0_f64.powf((layer.parameters.outside_cone_volume.unwrap() - 10_000.0) / 2000.0);
        let render = |direction| {
            let mut mixer = Mixer::new(48_000, 2).unwrap();
            mixer
                .command(Command::Add {
                    id: VoiceId(0),
                    sample: Arc::clone(&sample),
                    parameters: Box::new(find_layer().parameters),
                    looping: true,
                })
                .unwrap();
            mixer
                .command(Command::Set {
                    id: VoiceId(0),
                    level: 0.25,
                    inside: false,
                    active: true,
                })
                .unwrap();
            mixer
                .command(Command::Spatial {
                    id: VoiceId(0),
                    position: Some(Spatial {
                        distance: 0.0,
                        pan: 0.0,
                        source_to_listener: direction,
                    }),
                })
                .unwrap();
            let mut output = vec![0.0; 48_000];
            mixer.render(&mut output);
            output
        };
        let front_samples = render(axis);
        let rear_samples = render(axis.map(|value| -value));
        let rms = |samples: &[f32]| {
            (samples
                .iter()
                .map(|value| f64::from(*value).powi(2))
                .sum::<f64>()
                / samples.len() as f64)
                .sqrt()
        };
        let front = rms(&front_samples);
        let rear = rms(&rear_samples);
        assert!(front > 0.0001, "{path:?}: silent cone sample");
        assert!(
            (rear / front - expected).abs() < 1e-5,
            "{path:?}: front={front}, rear={rear}, expected={expected}"
        );
        let size = ((front_samples.len() + rear_samples.len()) * 2) as u32;
        let mut wav = Vec::with_capacity(size as usize + 44);
        wav.extend(b"RIFF");
        wav.extend((size + 36).to_le_bytes());
        wav.extend(b"WAVEfmt ");
        wav.extend(16u32.to_le_bytes());
        wav.extend(1u16.to_le_bytes());
        wav.extend(2u16.to_le_bytes());
        wav.extend(48_000u32.to_le_bytes());
        wav.extend(192_000u32.to_le_bytes());
        wav.extend(4u16.to_le_bytes());
        wav.extend(16u16.to_le_bytes());
        wav.extend(b"data");
        wav.extend(size.to_le_bytes());
        for sample in front_samples.into_iter().chain(rear_samples) {
            wav.extend(((sample.clamp(-1.0, 1.0) * i16::MAX as f32) as i16).to_le_bytes());
        }
        let preview = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/engine-cone-preview.wav");
        std::fs::write(preview, wav).unwrap();
    }
}

#[test]
#[ignore = "requires aircraft sound files and OPENESP_AIRCRAFT_PATHS"]
fn real_engine_layer_changes_pitch_with_doppler() {
    let path = content::aircraft_paths().into_iter().next().unwrap();
    let set = SoundSet::load(&path, 0, None).unwrap();
    let layer = set
        .engine_layers(1, EngineEvent::Combustion)
        .unwrap()
        .into_iter()
        .find(|layer| layer.parameters.viewpoint == Some(2) && !layer.filenames.is_empty())
        .unwrap();
    let sample = Arc::new(set.sample(&layer.filenames[0]).unwrap());
    let mut mixer = Mixer::new(48_000, 1).unwrap();
    mixer
        .command(Command::Add {
            id: VoiceId(0),
            sample,
            parameters: Box::new(layer.parameters),
            looping: true,
        })
        .unwrap();
    mixer
        .command(Command::Set {
            id: VoiceId(0),
            level: 0.25,
            inside: false,
            active: true,
        })
        .unwrap();
    let mut normal = vec![0.0; 8_000];
    mixer.render(&mut normal);
    mixer
        .command(Command::Set {
            id: VoiceId(0),
            level: 0.25,
            inside: false,
            active: false,
        })
        .unwrap();
    mixer
        .command(Command::Set {
            id: VoiceId(0),
            level: 0.25,
            inside: false,
            active: true,
        })
        .unwrap();
    mixer
        .command(Command::Doppler {
            id: VoiceId(0),
            factor: 1.5,
        })
        .unwrap();
    let mut shifted = vec![0.0; 8_000];
    mixer.render(&mut shifted);
    assert!(normal.iter().any(|sample| sample.abs() > 0.0001));
    assert!(
        normal
            .iter()
            .zip(&shifted)
            .any(|(a, b)| (a - b).abs() > 0.001)
    );
}

#[test]
#[ignore = "requires aircraft sound files and OPENESP_AIRCRAFT_PATHS"]
fn prepares_real_aircraft_sound_banks() {
    for path in content::aircraft_paths() {
        let set = SoundSet::load(&path, 0, None).unwrap();
        let (bank, commands) = EngineBank::prepare(&set, 1, 0).unwrap();
        assert!(
            !commands.is_empty(),
            "{} has no engine sounds",
            path.display()
        );
        let mut mixer = Mixer::new(48000, 2).unwrap();
        for command in commands {
            mixer.command(command).unwrap();
        }
        let state = EngineState {
            combustion: true,
            engine_level: 0.5,
            propeller_level: 0.5,
            inside: true,
            ..Default::default()
        };
        for command in bank.update(None, state).unwrap() {
            mixer.command(command).unwrap();
        }
        let mut audio = vec![0.0; 48000];
        mixer.render(&mut audio);
        assert!(audio.iter().all(|value| value.is_finite()));
        assert!(
            audio.iter().any(|value| value.abs() > 0.0001),
            "{} produced silence",
            path.display()
        );
    }
}

#[test]
#[ignore = "requires aircraft sound files and OPENESP_AIRCRAFT_PATHS"]
fn mixes_real_aircraft_engine_sound_layers() {
    for path in content::aircraft_paths() {
        let set = SoundSet::load(&path, 0, None).unwrap();
        let layers = set.engine_layers(1, EngineEvent::Combustion).unwrap();
        let mut mixer = Mixer::new(48000, 2).unwrap();
        for (index, layer) in layers.into_iter().enumerate() {
            let Some(name) = layer.filenames.first() else {
                continue;
            };
            let sample = set.sample(name).unwrap();
            let id = VoiceId(index as u64);
            mixer
                .command(Command::Add {
                    id,
                    sample: Arc::new(sample),
                    parameters: Box::new(layer.parameters),
                    looping: true,
                })
                .unwrap();
            mixer
                .command(Command::Set {
                    id,
                    level: 0.5,
                    inside: true,
                    active: true,
                })
                .unwrap();
        }
        let mut output = vec![0.0; 48000];
        mixer.render(&mut output);
        assert!(
            output
                .iter()
                .all(|sample| sample.is_finite() && sample.abs() <= 1.0)
        );
        assert!(
            output.iter().any(|sample| sample.abs() > 0.0001),
            "{} was silent",
            path.display()
        );
    }
}

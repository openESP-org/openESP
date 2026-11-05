use esp_assets::sound::SoundSet;
use esp_audio::engine::{EngineBank, EngineState};
use esp_audio::{Command, Mixer, VoiceId};
use esp_formats::cfg::sound::EngineEvent;
use std::sync::Arc;

#[path = "../../../tests/support/content.rs"]
mod content;

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

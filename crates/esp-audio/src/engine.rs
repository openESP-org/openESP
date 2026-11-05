//! Prepared engine and rotor sound layers for aircraft sound sets.
//!
//! Loading and decoding happen before commands are sent to the audio device.
//! The simulation chooses which events play and supplies the corresponding
//! normalized engine, propeller, or rotor speed.
use std::sync::Arc;

use anyhow::{Context, Result, ensure};
use esp_assets::sound::SoundSet;
use esp_formats::cfg::sound::EngineEvent;

use crate::{Command, VoiceId};

const EVENTS: [EngineEvent; 9] = [
    EngineEvent::Combustion,
    EngineEvent::Starter,
    EngineEvent::CombustionStart,
    EngineEvent::Shutdown,
    EngineEvent::JetWhine,
    EngineEvent::NonCombustion,
    EngineEvent::Propeller,
    EngineEvent::PropellerStrike,
    EngineEvent::Rotor,
];

/// Voice identifiers must be reserved exclusively for this bank by the caller.
pub struct EngineBank {
    voices: Vec<(EngineEvent, VoiceId)>,
}

impl EngineBank {
    /// Prepare every configured sound list for a one-based engine number.
    /// Returns commands separately so they can be sent to either an offline
    /// mixer or a device, with queue errors handled by the caller.
    pub fn prepare(set: &SoundSet, engine: u32, first_id: u64) -> Result<(Self, Vec<Command>)> {
        ensure!(
            (1..=4).contains(&engine),
            "sound engine must be between 1 and 4"
        );
        let mut voices = Vec::new();
        let mut commands = Vec::new();
        for event in EVENTS {
            for layer in set.engine_layers(engine, event)? {
                if layer.parameters.flags.unwrap_or(0) & 1 != 0 {
                    continue;
                }
                let Some(filename) = layer.filenames.first() else {
                    continue;
                };
                let sample = set
                    .sample(filename)
                    .with_context(|| format!("loading sound [{0}] ({filename})", layer.section))?;
                let id = VoiceId(
                    first_id
                        .checked_add(voices.len() as u64)
                        .context("sound voice ID overflow")?,
                );
                let looping = !matches!(
                    event,
                    EngineEvent::CombustionStart
                        | EngineEvent::Shutdown
                        | EngineEvent::PropellerStrike
                );
                commands.push(Command::Add {
                    id,
                    sample: Arc::new(sample),
                    parameters: Box::new(layer.parameters),
                    looping,
                });
                voices.push((event, id));
            }
        }
        Ok((Self { voices }, commands))
    }

    /// Commands for all layers in an event. Level is normalized to 0..1.
    /// For one-shot events, issue this only when triggering playback; repeatedly
    /// setting an already finished voice active will restart it.
    pub fn set(
        &self,
        event: EngineEvent,
        level: f64,
        inside: bool,
        active: bool,
    ) -> Result<Vec<Command>> {
        ensure!(level.is_finite(), "invalid sound control");
        Ok(self
            .voices
            .iter()
            .filter(|(kind, _)| *kind == event)
            .map(|(_, id)| Command::Set {
                id: *id,
                level,
                inside,
                active,
            })
            .collect())
    }

    /// Stop and restart a one-shot even if it is already playing.
    pub fn trigger(&self, event: EngineEvent, level: f64, inside: bool) -> Result<Vec<Command>> {
        let mut commands = self.set(event, level, inside, false)?;
        commands.extend(self.set(event, level, inside, true)?);
        Ok(commands)
    }

    pub fn remove(self) -> Vec<Command> {
        self.voices
            .into_iter()
            .map(|(_, id)| Command::Remove(id))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Mixer;

    #[test]
    fn prepares_rotor_layers_and_restarts_one_shots() {
        let root = tempfile::tempdir().unwrap();
        let sound = root.path().join("sound");
        std::fs::create_dir(&sound).unwrap();
        std::fs::write(root.path().join("aircraft.cfg"), "[fltsim.0]\ntitle=test\n").unwrap();
        std::fs::write(sound.join("sound.cfg"), "[sound_engine]\neng1_rotor=rotor\neng1_shutdown=stop\n[rotor]\nfilename=tone\nlink=disabled\n[disabled]\nfilename=missing\nflags=1\n[stop]\nfilename=tone\n").unwrap();
        let mut wave = b"RIFF".to_vec();
        wave.extend(40u32.to_le_bytes());
        wave.extend(b"WAVEfmt ");
        wave.extend(16u32.to_le_bytes());
        wave.extend(1u16.to_le_bytes());
        wave.extend(1u16.to_le_bytes());
        wave.extend(8000u32.to_le_bytes());
        wave.extend(16000u32.to_le_bytes());
        wave.extend(2u16.to_le_bytes());
        wave.extend(16u16.to_le_bytes());
        wave.extend(b"data");
        wave.extend(4u32.to_le_bytes());
        wave.extend(16384i16.to_le_bytes());
        wave.extend(16384i16.to_le_bytes());
        std::fs::write(sound.join("tone.wav"), wave).unwrap();
        let set = SoundSet::load(root.path(), 0, None).unwrap();
        let (bank, commands) = EngineBank::prepare(&set, 1, 12).unwrap();
        assert_eq!(commands.len(), 2);
        let mut mixer = Mixer::new(8000, 1).unwrap();
        for command in commands {
            mixer.command(command).unwrap();
        }
        for command in bank.set(EngineEvent::Rotor, 0.5, true, true).unwrap() {
            mixer.command(command).unwrap();
        }
        let mut output = [0.0; 4];
        mixer.render(&mut output);
        assert!(output.iter().all(|value| (*value - 0.5).abs() < 1e-5));
        for command in bank.set(EngineEvent::Rotor, 0.5, true, false).unwrap() {
            mixer.command(command).unwrap();
        }
        for command in bank.trigger(EngineEvent::Shutdown, 0.0, true).unwrap() {
            mixer.command(command).unwrap();
        }
        mixer.render(&mut output);
        assert!(output[0] > 0.49 && output[2] == 0.0);
        for command in bank.trigger(EngineEvent::Shutdown, 0.0, true).unwrap() {
            mixer.command(command).unwrap();
        }
        mixer.render(&mut output);
        assert!(output[0] > 0.49 && output[2] == 0.0);
        assert!(bank.set(EngineEvent::Rotor, f64::NAN, true, true).is_err());
        assert_eq!(bank.remove().len(), 2);
    }
}

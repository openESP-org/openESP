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
    voices: Vec<(EngineEvent, VoiceId, u32)>,
}

/// Conditions selecting special sound layers authored in sound.cfg.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LayerConditions {
    pub engine_damaged: bool,
    pub propeller_pitch: PropellerPitch,
}

/// Simulation values used to drive one engine's authored sound lists.
/// Levels are normalized to 0..1 and are clamped by the mixer.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct EngineState {
    pub combustion: bool,
    pub starter: bool,
    pub engine_level: f64,
    pub propeller_level: f64,
    pub rotor_level: f64,
    pub inside: bool,
    pub conditions: LayerConditions,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum PropellerPitch {
    #[default]
    Normal,
    Maximum,
    Minimum,
    MinimumReverse,
}

fn layer_enabled(event: EngineEvent, flags: u32, conditions: LayerConditions) -> bool {
    match event {
        EngineEvent::Combustion => (flags & 2 == 0 || conditions.engine_damaged) && flags & 4 == 0,
        EngineEvent::Propeller => {
            let selected = match conditions.propeller_pitch {
                PropellerPitch::Normal => 0,
                PropellerPitch::Maximum => 2,
                PropellerPitch::Minimum => 4,
                PropellerPitch::MinimumReverse => 8,
            };
            let special = flags & 14;
            special == 0 || special & selected != 0
        }
        _ => true,
    }
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
                let flags = layer.parameters.flags.unwrap_or(0);
                if flags & 1 != 0 {
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
                voices.push((event, id, flags));
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
        self.set_with_conditions(event, level, inside, active, LayerConditions::default())
    }

    /// Set an event while selecting its damaged-engine and propeller-pitch layers.
    pub fn set_with_conditions(
        &self,
        event: EngineEvent,
        level: f64,
        inside: bool,
        active: bool,
        conditions: LayerConditions,
    ) -> Result<Vec<Command>> {
        ensure!(level.is_finite(), "invalid sound control");
        Ok(self
            .voices
            .iter()
            .filter(|(kind, _, _)| *kind == event)
            .map(|(_, id, flags)| Command::Set {
                id: *id,
                level,
                inside,
                active: active && layer_enabled(event, *flags, conditions),
            })
            .collect())
    }

    /// Stop and restart a one-shot even if it is already playing.
    pub fn trigger(&self, event: EngineEvent, level: f64, inside: bool) -> Result<Vec<Command>> {
        let mut commands = self.set(event, level, inside, false)?;
        commands.extend(self.set(event, level, inside, true)?);
        Ok(commands)
    }

    /// Build a frame's commands. The caller supplies the last successfully
    /// submitted state so a full device queue cannot silently lose transitions.
    /// An absent previous state initializes loops without playing start or
    /// shutdown samples for an aircraft that is already running or stopped.
    pub fn update(
        &self,
        previous: Option<EngineState>,
        state: EngineState,
    ) -> Result<Vec<Command>> {
        ensure!(
            [state.engine_level, state.propeller_level, state.rotor_level]
                .iter()
                .all(|level| level.is_finite()),
            "invalid engine sound state"
        );
        let mut commands = Vec::new();
        for (event, level, active) in [
            (
                EngineEvent::Combustion,
                state.engine_level,
                state.combustion,
            ),
            (EngineEvent::Starter, state.engine_level, state.starter),
            (
                EngineEvent::JetWhine,
                state.engine_level,
                state.engine_level > 0.0,
            ),
            (
                EngineEvent::NonCombustion,
                state.engine_level,
                state.engine_level > 0.0,
            ),
            (
                EngineEvent::Propeller,
                state.propeller_level,
                state.propeller_level > 0.0,
            ),
            (
                EngineEvent::Rotor,
                state.rotor_level,
                state.rotor_level > 0.0,
            ),
        ] {
            commands.extend(self.set_with_conditions(
                event,
                level,
                state.inside,
                active,
                state.conditions,
            )?);
        }
        if let Some(previous) = previous
            && previous.combustion != state.combustion
        {
            let event = if state.combustion {
                EngineEvent::CombustionStart
            } else {
                EngineEvent::Shutdown
            };
            commands.extend(self.trigger(event, state.engine_level, state.inside)?);
        }
        Ok(commands)
    }

    pub fn remove(self) -> Vec<Command> {
        self.voices
            .into_iter()
            .map(|(_, id, _)| Command::Remove(id))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Mixer;

    #[test]
    fn state_drives_loops_and_only_triggers_transitions_once() {
        let bank = EngineBank {
            voices: vec![
                (EngineEvent::Combustion, VoiceId(0), 0),
                (EngineEvent::Rotor, VoiceId(1), 0),
                (EngineEvent::CombustionStart, VoiceId(2), 0),
                (EngineEvent::Shutdown, VoiceId(3), 0),
            ],
        };
        let mut mixer = Mixer::new(8000, 1).unwrap();
        for id in 0..4 {
            mixer
                .command(Command::Add {
                    id: VoiceId(id),
                    sample: Arc::new(esp_assets::sound::Sample {
                        channels: 1,
                        sample_rate: 8000,
                        samples: vec![0.1],
                    }),
                    parameters: Box::default(),
                    looping: id < 2,
                })
                .unwrap();
        }
        let mut state = EngineState {
            inside: true,
            ..Default::default()
        };
        let apply = |mixer: &mut Mixer, previous, state| {
            for command in bank.update(previous, state).unwrap() {
                mixer.command(command).unwrap();
            }
            let mut output = [0.0; 2];
            mixer.render(&mut output);
            output
        };
        assert_eq!(apply(&mut mixer, None, state), [0.0, 0.0]);
        let previous = state;
        state.combustion = true;
        state.engine_level = 0.5;
        state.rotor_level = 0.5;
        let output = apply(&mut mixer, Some(previous), state);
        assert!((output[0] - 0.3).abs() < 1e-6 && (output[1] - 0.2).abs() < 1e-6);
        let output = apply(&mut mixer, Some(state), state);
        assert!(output.iter().all(|value| (*value - 0.2).abs() < 1e-6));
        let previous = state;
        state.combustion = false;
        let output = apply(&mut mixer, Some(previous), state);
        assert!((output[0] - 0.2).abs() < 1e-6 && (output[1] - 0.1).abs() < 1e-6);
        state.rotor_level = f64::NAN;
        assert!(bank.update(Some(previous), state).is_err());
    }

    #[test]
    fn selects_damage_and_propeller_pitch_layers() {
        let bank = EngineBank {
            voices: vec![
                (EngineEvent::Combustion, VoiceId(0), 0),
                (EngineEvent::Combustion, VoiceId(1), 2),
                (EngineEvent::Combustion, VoiceId(2), 4),
                (EngineEvent::Propeller, VoiceId(3), 0),
                (EngineEvent::Propeller, VoiceId(4), 2),
                (EngineEvent::Propeller, VoiceId(5), 4),
                (EngineEvent::Propeller, VoiceId(6), 8),
            ],
        };
        let mut mixer = Mixer::new(8000, 1).unwrap();
        for id in 0..7 {
            mixer
                .command(Command::Add {
                    id: VoiceId(id),
                    sample: Arc::new(esp_assets::sound::Sample {
                        channels: 1,
                        sample_rate: 8000,
                        samples: vec![0.1],
                    }),
                    parameters: Box::default(),
                    looping: true,
                })
                .unwrap();
        }
        let apply = |mixer: &mut Mixer, conditions| {
            for event in [EngineEvent::Combustion, EngineEvent::Propeller] {
                for command in bank
                    .set_with_conditions(event, 0.5, true, true, conditions)
                    .unwrap()
                {
                    mixer.command(command).unwrap();
                }
            }
            let mut output = [0.0];
            mixer.render(&mut output);
            output[0]
        };
        assert!((apply(&mut mixer, LayerConditions::default()) - 0.2).abs() < 1e-6);
        assert!(
            (apply(
                &mut mixer,
                LayerConditions {
                    engine_damaged: true,
                    propeller_pitch: PropellerPitch::Maximum,
                }
            ) - 0.4)
                .abs()
                < 1e-6
        );
        assert!(
            (apply(
                &mut mixer,
                LayerConditions {
                    engine_damaged: false,
                    propeller_pitch: PropellerPitch::MinimumReverse,
                }
            ) - 0.3)
                .abs()
                < 1e-6
        );
    }

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

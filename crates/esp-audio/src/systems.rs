//! Aircraft mechanism sounds driven by actual gear and flap movement.
use crate::{Command, Spatial, VoiceId};
use anyhow::{Context, Result, ensure};
use esp_assets::sound::SoundSet;
use std::sync::Arc;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Event {
    GearUp,
    GearDown,
    Flaps,
}

pub struct SystemsBank {
    voices: Vec<(Event, VoiceId)>,
}

#[derive(Clone, Copy, Default)]
struct Motion {
    previous: Option<f64>,
    direction: i8,
    idle: f64,
}

impl Motion {
    fn advance(&mut self, position: Option<f64>, elapsed: f64) -> i8 {
        let Some(position) = position else {
            *self = Self::default();
            return 0;
        };
        let delta = self
            .previous
            .replace(position)
            .map_or(0.0, |previous| position - previous);
        let direction = if delta > 1e-6 {
            1
        } else if delta < -1e-6 {
            -1
        } else {
            0
        };
        if direction == 0 {
            self.idle += elapsed;
            if self.idle >= 0.25 {
                self.direction = 0;
            }
            return 0;
        }
        self.idle = 0.0;
        if self.direction == direction {
            return 0;
        }
        self.direction = direction;
        direction
    }
}

/// Last successfully submitted motion state, retained across audio queue retries.
#[derive(Clone, Copy, Default)]
pub struct Tracker {
    gear: Motion,
    flaps: Motion,
    inside: Option<bool>,
}

#[derive(Clone, Copy, Default)]
pub struct State {
    pub gear: Option<f64>,
    pub flaps: Option<f64>,
    pub inside: bool,
}

impl SystemsBank {
    pub fn prepare(set: &SoundSet, first_id: u64) -> Result<(Self, Vec<Command>)> {
        let mut voices = Vec::new();
        let mut commands = Vec::new();
        for (name, event) in [
            ("GEAR_UP", Event::GearUp),
            ("GEAR_DOWN", Event::GearDown),
            ("FLAPS", Event::Flaps),
        ] {
            for layer in set.optional_layers(name)? {
                if layer.parameters.flags.unwrap_or(0) & 1 != 0 {
                    continue;
                }
                let Some(filename) = layer.filenames.first() else {
                    continue;
                };
                let sample = match set.sample(filename) {
                    Ok(sample) => sample,
                    Err(error) => {
                        eprintln!("sound [{name}]: {error:#}");
                        continue;
                    }
                };
                let id = VoiceId(
                    first_id
                        .checked_add(voices.len() as u64)
                        .context("sound voice ID overflow")?,
                );
                commands.push(Command::Add {
                    id,
                    sample: Arc::new(sample),
                    parameters: Box::new(layer.parameters),
                    looping: false,
                });
                voices.push((event, id));
            }
        }
        Ok((Self { voices }, commands))
    }

    pub fn update(
        &self,
        mut tracker: Tracker,
        state: State,
        elapsed: f64,
        spatial: Option<Spatial>,
    ) -> Result<(Tracker, Vec<Command>)> {
        ensure!(
            elapsed.is_finite()
                && (0.0..=1.0).contains(&elapsed)
                && [state.gear, state.flaps]
                    .into_iter()
                    .flatten()
                    .all(|value| value.is_finite() && (0.0..=1.0).contains(&value)),
            "invalid mechanism sound state"
        );
        let gear = tracker.gear.advance(state.gear, elapsed);
        let flaps = tracker.flaps.advance(state.flaps, elapsed);
        let view_changed = tracker.inside != Some(state.inside);
        tracker.inside = Some(state.inside);
        let mut commands = Vec::new();
        for &(event, id) in &self.voices {
            if view_changed {
                commands.push(Command::View {
                    id,
                    inside: state.inside,
                });
            }
            commands.push(Command::Spatial {
                id,
                position: spatial,
            });
            let trigger = matches!(
                (event, gear, flaps),
                (Event::GearUp, -1, _) | (Event::GearDown, 1, _) | (Event::Flaps, _, -1 | 1)
            );
            if trigger {
                commands.push(Command::Set {
                    id,
                    level: 0.0,
                    inside: state.inside,
                    active: false,
                });
                commands.push(Command::Set {
                    id,
                    level: 0.0,
                    inside: state.inside,
                    active: true,
                });
            }
        }
        Ok((tracker, commands))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn triggers_once_per_motion_and_handles_idle_frames_and_reversal() {
        let bank = SystemsBank {
            voices: vec![
                (Event::GearUp, VoiceId(0)),
                (Event::GearDown, VoiceId(1)),
                (Event::Flaps, VoiceId(2)),
            ],
        };
        let count = |commands: &[Command]| {
            commands
                .iter()
                .filter(|command| matches!(command, Command::Set { active: true, .. }))
                .count()
        };
        let (tracker, commands) = bank
            .update(
                Tracker::default(),
                State {
                    gear: Some(1.0),
                    flaps: Some(0.0),
                    inside: true,
                },
                0.016,
                None,
            )
            .unwrap();
        assert_eq!(count(&commands), 0);
        let (tracker, commands) = bank
            .update(
                tracker,
                State {
                    gear: Some(0.9),
                    flaps: Some(0.1),
                    inside: true,
                },
                0.016,
                None,
            )
            .unwrap();
        assert_eq!(count(&commands), 2);
        let (tracker, _) = bank
            .update(
                tracker,
                State {
                    gear: Some(0.9),
                    flaps: Some(0.1),
                    inside: true,
                },
                0.016,
                None,
            )
            .unwrap();
        let (tracker, commands) = bank
            .update(
                tracker,
                State {
                    gear: Some(0.8),
                    flaps: Some(0.2),
                    inside: false,
                },
                0.016,
                None,
            )
            .unwrap();
        assert_eq!(count(&commands), 0);
        assert!(
            commands
                .iter()
                .any(|command| matches!(command, Command::View { inside: false, .. }))
        );
        let (_, commands) = bank
            .update(
                tracker,
                State {
                    gear: Some(0.9),
                    flaps: Some(0.1),
                    inside: false,
                },
                0.016,
                None,
            )
            .unwrap();
        assert_eq!(count(&commands), 2);
    }
}

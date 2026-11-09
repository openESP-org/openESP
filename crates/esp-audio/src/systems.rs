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
    Touchdown(usize),
}

pub struct SystemsBank {
    voices: Vec<(Event, VoiceId)>,
    rolls: Vec<Roll>,
}

struct Roll {
    id: VoiceId,
    flags: u32,
    minimum: f64,
    maximum: f64,
}

fn surface_flag(surface: u16) -> Option<u32> {
    match surface {
        0..=16 => Some(1 << (surface + 1)),
        17 | 19 | 23 => Some(32),
        26..=29 => Some(8),
        _ => None,
    }
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
    contacts: [Contact; 4],
}

#[derive(Clone, Copy, Default)]
struct Contact {
    previous: Option<bool>,
    airborne: f64,
    armed: bool,
}

impl Contact {
    fn advance(&mut self, contact: Option<bool>, elapsed: f64) -> bool {
        let Some(contact) = contact else {
            *self = Self::default();
            return false;
        };
        let previous = self.previous.replace(contact);
        if !contact {
            self.airborne += elapsed;
            if previous.is_none() || self.airborne >= 0.2 {
                self.armed = true;
            }
            return false;
        }
        self.airborne = 0.0;
        let trigger = previous == Some(false) && self.armed;
        self.armed = false;
        trigger
    }
}

#[derive(Clone, Copy, Default)]
pub struct State {
    pub gear: Option<f64>,
    pub flaps: Option<f64>,
    pub inside: bool,
    pub contacts: [Option<bool>; 4],
    /// True airspeed in knots, as used by the authored ground sound parameters.
    pub airspeed_knots: Option<f64>,
    /// FSX/P3D airport surface code, when available.
    pub surface: Option<u16>,
}

impl SystemsBank {
    pub fn prepare(set: &SoundSet, first_id: u64) -> Result<(Self, Vec<Command>)> {
        let mut voices = Vec::new();
        let mut commands = Vec::new();
        let mut rolls = Vec::new();
        for (name, event) in [
            ("GEAR_UP", Event::GearUp),
            ("GEAR_DOWN", Event::GearDown),
            ("FLAPS", Event::Flaps),
            ("CENTER_TOUCHDOWN", Event::Touchdown(0)),
            ("AUX_TOUCHDOWN", Event::Touchdown(1)),
            ("LEFT_TOUCHDOWN", Event::Touchdown(2)),
            ("RIGHT_TOUCHDOWN", Event::Touchdown(3)),
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
        for layer in set.optional_layers("GROUND_ROLL")? {
            let mut parameters = layer.parameters;
            let flags = parameters.flags.unwrap_or(0);
            if flags & 1 != 0 {
                continue;
            }
            let Some(filename) = layer.filenames.first() else {
                continue;
            };
            let sample = match set.sample(filename) {
                Ok(sample) => sample,
                Err(error) => {
                    eprintln!("sound [GROUND_ROLL]: {error:#}");
                    continue;
                }
            };
            let minimum = parameters.minimum_speed.unwrap_or(0.0).max(0.0);
            let maximum = parameters
                .maximum_speed
                .unwrap_or(55.0)
                .max(minimum + 0.001);
            if parameters.volume_envelope.is_none() {
                let volume = |value: f64| {
                    100.0 * 10.0_f64.powf((value.clamp(0.0, 10000.0) - 10000.0) / 1000.0)
                };
                parameters.volume_envelope = Some(vec![
                    [0.0, volume(parameters.minimum_volume.unwrap_or(10000.0))],
                    [1.0, volume(parameters.maximum_volume.unwrap_or(10000.0))],
                ]);
            }
            if parameters.pitch_envelope.is_none() {
                parameters.pitch_envelope = Some(vec![
                    [0.0, parameters.minimum_rate.unwrap_or(1.0)],
                    [1.0, parameters.maximum_rate.unwrap_or(1.0)],
                ]);
            }
            let id = VoiceId(
                first_id
                    .checked_add(commands.len() as u64)
                    .context("sound voice ID overflow")?,
            );
            commands.push(Command::Add {
                id,
                sample: Arc::new(sample),
                parameters: Box::new(parameters),
                looping: true,
            });
            rolls.push(Roll {
                id,
                flags,
                minimum,
                maximum,
            });
        }
        Ok((Self { voices, rolls }, commands))
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
                    .all(|value| value.is_finite() && (0.0..=1.0).contains(&value))
                && state
                    .airspeed_knots
                    .is_none_or(|speed| speed.is_finite() && speed >= 0.0),
            "invalid mechanism sound state"
        );
        let gear = tracker.gear.advance(state.gear, elapsed);
        let flaps = tracker.flaps.advance(state.flaps, elapsed);
        let touchdowns: [bool; 4] = std::array::from_fn(|index| {
            tracker.contacts[index].advance(state.contacts[index], elapsed)
        });
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
            ) || matches!(event, Event::Touchdown(index) if touchdowns[index]);
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
        for (index, roll) in self.rolls.iter().enumerate() {
            let speed = state.airspeed_knots.unwrap_or(0.0);
            let surface = state.surface.and_then(surface_flag);
            let matches_surface = if let Some(surface) = surface {
                roll.flags & !1 == 0 || roll.flags & surface != 0
            } else {
                index == 0
            };
            commands.push(Command::Spatial {
                id: roll.id,
                position: spatial,
            });
            commands.push(Command::Set {
                id: roll.id,
                level: ((speed - roll.minimum) / (roll.maximum - roll.minimum)).clamp(0.0, 1.0),
                inside: state.inside,
                active: matches_surface
                    && state.contacts.contains(&Some(true))
                    && speed >= roll.minimum
                    && speed > 0.0,
            });
        }
        Ok((tracker, commands))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn roll_selects_surface_and_stops_when_airborne() {
        let bank = SystemsBank {
            voices: Vec::new(),
            rolls: vec![
                Roll {
                    id: VoiceId(0),
                    flags: 2 | 32,
                    minimum: 3.0,
                    maximum: 55.0,
                },
                Roll {
                    id: VoiceId(1),
                    flags: 64,
                    minimum: 3.0,
                    maximum: 55.0,
                },
            ],
        };
        let mut state = State {
            airspeed_knots: Some(29.0),
            contacts: [Some(true); 4],
            surface: Some(4),
            ..Default::default()
        };
        let controls = |state| {
            let (_, commands) = bank.update(Tracker::default(), state, 0.016, None).unwrap();
            commands
                .into_iter()
                .filter_map(|command| match command {
                    Command::Set {
                        id, active, level, ..
                    } => Some((id.0, active, level)),
                    _ => None,
                })
                .collect::<Vec<_>>()
        };
        assert_eq!(controls(state), vec![(0, true, 0.5), (1, false, 0.5)]);
        state.surface = Some(5);
        assert_eq!(controls(state), vec![(0, false, 0.5), (1, true, 0.5)]);
        state.contacts = [Some(false); 4];
        assert!(controls(state).iter().all(|(_, active, _)| !active));
        state.contacts[0] = Some(true);
        state.airspeed_knots = Some(2.0);
        assert!(controls(state).iter().all(|(_, active, _)| !active));
    }
    #[test]
    fn triggers_once_per_motion_and_handles_idle_frames_and_reversal() {
        let bank = SystemsBank {
            rolls: Vec::new(),
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
                    ..Default::default()
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
                    ..Default::default()
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
                    ..Default::default()
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
                    ..Default::default()
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
                    ..Default::default()
                },
                0.016,
                None,
            )
            .unwrap();
        assert_eq!(count(&commands), 2);
    }

    #[test]
    fn touchdown_contacts_trigger_independently_and_filter_jitter() {
        let bank = SystemsBank {
            rolls: Vec::new(),
            voices: (0..4)
                .map(|index| (Event::Touchdown(index), VoiceId(index as u64)))
                .collect(),
        };
        let run = |tracker, contacts, elapsed| {
            let (tracker, commands) = bank
                .update(
                    tracker,
                    State {
                        contacts,
                        ..Default::default()
                    },
                    elapsed,
                    None,
                )
                .unwrap();
            let ids: Vec<_> = commands
                .iter()
                .filter_map(|command| match command {
                    Command::Set {
                        id, active: true, ..
                    } => Some(id.0),
                    _ => None,
                })
                .collect();
            (tracker, ids)
        };
        let (tracker, ids) = run(
            Tracker::default(),
            [Some(true), Some(false), Some(false), Some(false)],
            0.01,
        );
        assert!(ids.is_empty());
        let (tracker, ids) = run(
            tracker,
            [Some(true), Some(false), Some(true), Some(false)],
            0.01,
        );
        assert_eq!(ids, [2]);
        let (tracker, _) = run(
            tracker,
            [Some(true), Some(false), Some(false), Some(false)],
            0.01,
        );
        let (tracker, ids) = run(
            tracker,
            [Some(true), Some(false), Some(true), Some(true)],
            0.01,
        );
        assert_eq!(ids, [3]);
        let (tracker, _) = run(
            tracker,
            [Some(true), Some(false), Some(false), Some(true)],
            0.21,
        );
        let (_, ids) = run(
            tracker,
            [Some(true), Some(false), Some(true), Some(true)],
            0.01,
        );
        assert_eq!(ids, [2]);
    }
}

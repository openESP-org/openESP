//! Connect authored aircraft engine sounds to the running flight session.
use anyhow::Result;
use esp_assets::sound::SoundSet;
use esp_audio::{
    Command, Output, Spatial,
    engine::{EngineBank, EngineState},
    systems::{State as SystemsState, SystemsBank, Tracker},
};
use esp_script::{Host, Reference};
use esp_sim::variables::Variables;
use glam::{DVec3, Mat4};
use std::path::Path;

pub(super) struct Prepared {
    banks: Vec<(u32, EngineBank, DVec3)>,
    commands: Vec<Command>,
    systems: Option<SystemsBank>,
}

impl Prepared {
    pub fn load(path: &Path, variant: u32, root: Option<&Path>) -> Result<Self> {
        let sound = SoundSet::load(path, variant, root)?;
        let data = esp_assets::flight::FlightData::load(path, variant)?;
        let datum = esp_formats::cfg::aircraft::ReferencePoints::from_config(&data.configuration)?
            .datum_feet
            .unwrap_or([0.0; 3]);
        let mut positions = [DVec3::ZERO; 4];
        for engine in esp_formats::cfg::engine::engines(&data.configuration)? {
            if let Some(position) = positions.get_mut(engine.index as usize) {
                let [x, y, z] = engine.position_feet;
                *position = DVec3::new(y + datum[1], z + datum[2], x + datum[0]) * 0.3048;
            }
        }
        let mut commands = Vec::new();
        let mut banks = Vec::new();
        for engine in 1..=4 {
            match EngineBank::prepare(&sound, engine, commands.len() as u64) {
                Ok((bank, loaded)) if !loaded.is_empty() => {
                    commands.extend(loaded);
                    banks.push((engine, bank, positions[(engine - 1) as usize]));
                }
                Ok(_) => {}
                Err(error) => eprintln!("sound engine {engine}: {error:#}"),
            }
        }
        let systems = match SystemsBank::prepare(&sound, commands.len() as u64) {
            Ok((systems, loaded)) if !loaded.is_empty() => {
                commands.extend(loaded);
                Some(systems)
            }
            Ok(_) => None,
            Err(error) => {
                eprintln!("mechanism sounds: {error:#}");
                None
            }
        };
        Ok(Self {
            banks,
            commands,
            systems,
        })
    }

    pub fn open(self) -> Result<Option<Runtime>> {
        if self.banks.is_empty() && self.systems.is_none() {
            return Ok(None);
        }
        let output = Output::open_default()?;
        for command in self.commands {
            output.try_command(command)?;
        }
        Ok(Some(Runtime {
            output,
            banks: self.banks,
            previous: [None; 4],
            systems: self.systems,
            systems_tracker: Tracker::default(),
        }))
    }
}

pub(super) struct Runtime {
    output: Output,
    banks: Vec<(u32, EngineBank, DVec3)>,
    previous: [Option<EngineState>; 4],
    systems: Option<SystemsBank>,
    systems_tracker: Tracker,
}

pub(super) struct Listener {
    pub position: DVec3,
    pub right: DVec3,
    pub aircraft_origin: DVec3,
    pub aircraft_rotation: Mat4,
}

impl Listener {
    fn relative(&self, engine: DVec3) -> Spatial {
        let source = self.aircraft_origin
            + self
                .aircraft_rotation
                .transform_vector3(engine.as_vec3())
                .as_dvec3();
        let offset = source - self.position;
        let distance = offset.length();
        let pan = offset
            .normalize_or_zero()
            .dot(self.right.normalize_or_zero())
            .clamp(-1.0, 1.0);
        let local_direction = self
            .aircraft_rotation
            .transpose()
            .transform_vector3((-offset).as_vec3())
            .normalize_or_zero();
        Spatial {
            distance,
            pan,
            source_to_listener: local_direction.as_dvec3().to_array(),
        }
    }
}

impl Runtime {
    pub fn update(
        &mut self,
        variables: &mut Variables,
        inside: bool,
        listener: &Listener,
        elapsed: f64,
    ) -> Result<()> {
        while let Some(error) = self.output.try_error() {
            eprintln!("audio output: {error}");
        }
        let mut commands = Vec::new();
        let mut states = self.previous;
        for (engine, bank, position) in &self.banks {
            let index = (*engine - 1) as usize;
            let state = engine_state(variables, *engine, inside);
            commands.extend(bank.update(self.previous[index], state)?);
            commands.extend(bank.spatial((!inside).then(|| listener.relative(*position))));
            states[index] = Some(state);
        }
        let mut systems_tracker = self.systems_tracker;
        if let Some(systems) = &self.systems {
            let (tracker, changes) = systems.update(
                self.systems_tracker,
                systems_state(variables, inside),
                elapsed,
                (!inside).then(|| listener.relative(DVec3::ZERO)),
            )?;
            systems_tracker = tracker;
            commands.extend(changes);
        }
        self.output.try_controls(commands)?;
        self.previous = states;
        self.systems_tracker = systems_tracker;
        Ok(())
    }
}

fn systems_state(variables: &mut Variables, inside: bool) -> SystemsState {
    let read = |variables: &mut Variables, name| {
        variables
            .read(&Reference::new('A', name, Some("percent over 100")))
            .ok()
            .and_then(|value| value.number().ok())
            .filter(|value| value.is_finite())
            .map(|value| value.clamp(0.0, 1.0))
    };
    SystemsState {
        gear: read(variables, "GEAR TOTAL PCT EXTENDED"),
        flaps: read(variables, "TRAILING EDGE FLAPS LEFT PERCENT"),
        inside,
    }
}

fn engine_state(variables: &mut Variables, engine: u32, inside: bool) -> EngineState {
    let read = |variables: &mut Variables, name: &str, unit: &str| -> Option<f64> {
        variables
            .read(&Reference::new(
                'A',
                &format!("{name}:{engine}"),
                Some(unit),
            ))
            .ok()
            .and_then(|value| value.number().ok())
            .filter(|value| value.is_finite())
    };
    let percent = |variables: &mut Variables, name| {
        read(variables, name, "percent").map(|value| (value / 100.0).clamp(0.0, 1.0))
    };
    let engine_level = percent(variables, "TURB ENG N1")
        .or_else(|| percent(variables, "GENERAL ENG PCT MAX RPM"))
        .unwrap_or(0.0);
    let propeller_level = percent(variables, "PROP MAX RPM PERCENT").unwrap_or(0.0);
    EngineState {
        combustion: read(variables, "GENERAL ENG COMBUSTION", "bool").unwrap_or(0.0) != 0.0,
        starter: read(variables, "GENERAL ENG STARTER ACTIVE", "bool").unwrap_or(0.0) != 0.0,
        engine_level,
        propeller_level,
        rotor_level: percent(variables, "ROTOR RPM PCT").unwrap_or(propeller_level),
        inside,
        ..Default::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore = "requires aircraft in OPENESP_AIRCRAFT_PATHS"]
    fn real_aircraft_mechanisms_follow_movement() {
        let paths = std::env::var_os("OPENESP_AIRCRAFT_PATHS").unwrap();
        let path = std::env::split_paths(&paths).next().unwrap();
        let prepared = Prepared::load(&path, 0, None).unwrap();
        let systems = prepared
            .systems
            .expect("aircraft has no gear or flap sounds");
        let mut mixer = esp_audio::Mixer::new(48_000, 2).unwrap();
        for command in prepared.commands {
            mixer.command(command).unwrap();
        }
        let initial = SystemsState {
            gear: Some(1.0),
            flaps: Some(0.0),
            inside: true,
        };
        let (tracker, commands) = systems
            .update(Tracker::default(), initial, 0.016, None)
            .unwrap();
        for command in commands {
            mixer.command(command).unwrap();
        }
        let mut before = [0.0; 200];
        mixer.render(&mut before);
        assert!(before.iter().all(|sample| *sample == 0.0));
        let moving = SystemsState {
            gear: Some(0.9),
            flaps: Some(0.1),
            inside: true,
        };
        let (tracker, commands) = systems.update(tracker, moving, 0.016, None).unwrap();
        for command in commands {
            mixer.command(command).unwrap();
        }
        let mut samples = vec![0.0; 48_000];
        mixer.render(&mut samples);
        assert!(samples.iter().any(|sample| sample.abs() > 0.0001));
        write_preview("b737-mechanisms.wav", &samples);
        let (_, commands) = systems
            .update(
                tracker,
                SystemsState {
                    gear: Some(0.8),
                    flaps: Some(0.2),
                    inside: true,
                },
                0.016,
                None,
            )
            .unwrap();
        assert!(
            !commands
                .iter()
                .any(|command| matches!(command, Command::Set { active: true, .. }))
        );
    }

    #[test]
    fn listener_tracks_rotated_engine_positions() {
        let mut listener = Listener {
            position: DVec3::ZERO,
            right: DVec3::X,
            aircraft_origin: DVec3::new(0.0, 0.0, 10.0),
            aircraft_rotation: Mat4::IDENTITY,
        };
        let right = listener.relative(DVec3::new(5.0, 0.0, 0.0));
        assert!((right.distance - 125.0_f64.sqrt()).abs() < 1e-6);
        assert!(right.pan > 0.4);
        assert!(right.source_to_listener[0] < -0.4 && right.source_to_listener[2] < -0.8);
        listener.aircraft_rotation = Mat4::from_rotation_y(std::f32::consts::PI);
        let left = listener.relative(DVec3::new(5.0, 0.0, 0.0));
        assert!(left.pan < -0.4);
        assert!(left.source_to_listener[0] < -0.4 && left.source_to_listener[2] > 0.8);
    }
    #[test]
    fn engine_state_tracks_spool_combustion_and_view() {
        let mut variables = Variables::default();
        variables.set_number("TURB ENG N1:2", "percent", 74.0);
        variables.set_number("GENERAL ENG COMBUSTION:2", "bool", 1.0);
        variables.set_number("GENERAL ENG STARTER ACTIVE:2", "bool", 1.0);
        let state = engine_state(&mut variables, 2, false);
        assert_eq!(state.engine_level, 0.74);
        assert!(state.combustion && state.starter && !state.inside);
        variables.set_number("GENERAL ENG PCT MAX RPM:1", "percent", 85.0);
        variables.set_number("PROP MAX RPM PERCENT:1", "percent", 75.0);
        let state = engine_state(&mut variables, 1, true);
        assert_eq!(state.engine_level, 0.85);
        assert_eq!(state.propeller_level, 0.75);
        assert!(state.inside);
    }

    #[test]
    fn mechanism_state_uses_actual_positions_and_units() {
        let mut variables = Variables::default();
        variables.set_number("GEAR TOTAL PCT EXTENDED", "percent", 70.0);
        variables.set_number("TRAILING EDGE FLAPS LEFT PERCENT", "percent", 25.0);
        let state = systems_state(&mut variables, true);
        assert!((state.gear.unwrap() - 0.7).abs() < 1e-9);
        assert!((state.flaps.unwrap() - 0.25).abs() < 1e-9);
        assert!(state.inside);
    }

    #[test]
    #[ignore = "requires aircraft in OPENESP_AIRCRAFT_PATHS"]
    fn prepares_real_engine_sounds_for_live_state() {
        let paths = std::env::var_os("OPENESP_AIRCRAFT_PATHS").unwrap();
        let path = std::env::split_paths(&paths).next().unwrap();
        let prepared = Prepared::load(&path, 0, None).unwrap();
        assert!(!prepared.banks.is_empty());
        let mut mixer = esp_audio::Mixer::new(48_000, 2).unwrap();
        for command in prepared.commands {
            mixer.command(command).unwrap();
        }
        let mut variables = Variables::default();
        variables.set_number("TURB ENG N1:1", "percent", 50.0);
        variables.set_number("GENERAL ENG COMBUSTION:1", "bool", 1.0);
        let (engine, bank, _) = &prepared.banks[0];
        let inside = engine_state(&mut variables, *engine, true);
        for command in bank.update(None, inside).unwrap() {
            mixer.command(command).unwrap();
        }
        let mut samples = vec![0.0; 144_000];
        mixer.render(&mut samples[..48_000]);
        let outside = engine_state(&mut variables, *engine, false);
        for command in bank.update(Some(inside), outside).unwrap() {
            mixer.command(command).unwrap();
        }
        for command in bank.spatial(Some(Spatial {
            distance: 50.0,
            pan: 1.0,
            source_to_listener: [0.0, 0.0, 1.0],
        })) {
            mixer.command(command).unwrap();
        }
        mixer.render(&mut samples[48_000..96_000]);
        for command in bank.spatial(Some(Spatial {
            distance: 5_000.0,
            pan: 1.0,
            source_to_listener: [0.0, 0.0, 1.0],
        })) {
            mixer.command(command).unwrap();
        }
        mixer.render(&mut samples[96_000..]);
        assert!(samples[..48_000].iter().any(|sample| sample.abs() > 0.0001));
        assert!(
            samples[48_000..96_000]
                .iter()
                .any(|sample| sample.abs() > 0.0001)
        );
        assert_ne!(&samples[..48_000], &samples[48_000..96_000]);
        let rms = |values: &[f32]| {
            (values.iter().map(|sample| sample * sample).sum::<f32>() / values.len() as f32).sqrt()
        };
        assert!(rms(&samples[96_000..]) < rms(&samples[48_000..96_000]) * 0.1);
        write_preview("b737-engine-views.wav", &samples);
    }

    fn write_preview(name: &str, samples: &[f32]) {
        let mut wav = Vec::with_capacity(44 + samples.len() * 2);
        let byte_length = (samples.len() * 2) as u32;
        wav.extend(b"RIFF");
        wav.extend((36 + byte_length).to_le_bytes());
        wav.extend(b"WAVEfmt ");
        wav.extend(16u32.to_le_bytes());
        wav.extend(1u16.to_le_bytes());
        wav.extend(2u16.to_le_bytes());
        wav.extend(48_000u32.to_le_bytes());
        wav.extend(192_000u32.to_le_bytes());
        wav.extend(4u16.to_le_bytes());
        wav.extend(16u16.to_le_bytes());
        wav.extend(b"data");
        wav.extend(byte_length.to_le_bytes());
        for &sample in samples {
            wav.extend(((sample.clamp(-1.0, 1.0) * i16::MAX as f32) as i16).to_le_bytes());
        }
        let output = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../target")
            .join(name);
        std::fs::write(output, wav).unwrap();
    }
}

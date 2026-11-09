//! Connect authored aircraft engine sounds to the running flight session.
use anyhow::Result;
use esp_assets::sound::SoundSet;
use esp_audio::{
    Command, Output,
    engine::{EngineBank, EngineState},
};
use esp_script::{Host, Reference};
use esp_sim::variables::Variables;
use std::path::Path;

pub(super) struct Prepared {
    banks: Vec<(u32, EngineBank)>,
    commands: Vec<Command>,
}

impl Prepared {
    pub fn load(path: &Path, variant: u32, root: Option<&Path>) -> Result<Self> {
        let sound = SoundSet::load(path, variant, root)?;
        let mut commands = Vec::new();
        let mut banks = Vec::new();
        for engine in 1..=4 {
            match EngineBank::prepare(&sound, engine, commands.len() as u64) {
                Ok((bank, loaded)) if !loaded.is_empty() => {
                    commands.extend(loaded);
                    banks.push((engine, bank));
                }
                Ok(_) => {}
                Err(error) => eprintln!("sound engine {engine}: {error:#}"),
            }
        }
        Ok(Self { banks, commands })
    }

    pub fn open(self) -> Result<Option<Runtime>> {
        if self.banks.is_empty() {
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
        }))
    }
}

pub(super) struct Runtime {
    output: Output,
    banks: Vec<(u32, EngineBank)>,
    previous: [Option<EngineState>; 4],
}

impl Runtime {
    pub fn update(&mut self, variables: &mut Variables, inside: bool) -> Result<()> {
        while let Some(error) = self.output.try_error() {
            eprintln!("audio output: {error}");
        }
        let mut commands = Vec::new();
        let mut states = self.previous;
        for (engine, bank) in &self.banks {
            let index = (*engine - 1) as usize;
            let state = engine_state(variables, *engine, inside);
            commands.extend(bank.update(self.previous[index], state)?);
            states[index] = Some(state);
        }
        self.output.try_controls(commands)?;
        self.previous = states;
        Ok(())
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
        let (engine, bank) = &prepared.banks[0];
        let inside = engine_state(&mut variables, *engine, true);
        for command in bank.update(None, inside).unwrap() {
            mixer.command(command).unwrap();
        }
        let mut samples = vec![0.0; 96_000];
        mixer.render(&mut samples[..48_000]);
        let outside = engine_state(&mut variables, *engine, false);
        for command in bank.update(Some(inside), outside).unwrap() {
            mixer.command(command).unwrap();
        }
        mixer.render(&mut samples[48_000..]);
        assert!(samples[..48_000].iter().any(|sample| sample.abs() > 0.0001));
        assert!(samples[48_000..].iter().any(|sample| sample.abs() > 0.0001));
        assert_ne!(&samples[..48_000], &samples[48_000..]);
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
        for sample in samples {
            wav.extend(((sample.clamp(-1.0, 1.0) * i16::MAX as f32) as i16).to_le_bytes());
        }
        let output =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/b737-engine-views.wav");
        std::fs::write(output, wav).unwrap();
    }
}

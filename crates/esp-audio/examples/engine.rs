use anyhow::{Context, Result};
use esp_assets::sound::SoundSet;
use esp_audio::{Command, Output, VoiceId};
use esp_formats::cfg::sound::EngineEvent;
use std::{path::Path, sync::Arc, time::Duration};

fn main() -> Result<()> {
    let path = std::env::args_os()
        .nth(1)
        .context("usage: engine <aircraft folder>")?;
    let set = SoundSet::load(Path::new(&path), 0, None)?;
    let layers = set.engine_layers(1, EngineEvent::Combustion)?;
    let output = Output::open_default()?;
    for (index, layer) in layers.into_iter().enumerate() {
        let Some(filename) = layer.filenames.first() else {
            continue;
        };
        let sample = set.sample(filename)?;
        let id = VoiceId(index as u64);
        output.try_command(Command::Add {
            id,
            sample: Arc::new(sample),
            parameters: Box::new(layer.parameters),
            looping: true,
        })?;
        output.try_command(Command::Set {
            id,
            level: 0.5,
            inside: true,
            active: true,
        })?;
    }
    std::thread::sleep(Duration::from_secs(2));
    if let Some(error) = output.try_error() {
        anyhow::bail!("audio output: {error}");
    }
    Ok(())
}

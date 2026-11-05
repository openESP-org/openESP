use anyhow::{Context, Result};
use esp_assets::sound::SoundSet;
use esp_audio::{Output, engine::EngineBank};
use esp_formats::cfg::sound::EngineEvent;
use std::{path::Path, time::Duration};

fn main() -> Result<()> {
    let path = std::env::args_os()
        .nth(1)
        .context("usage: engine <aircraft folder>")?;
    let set = SoundSet::load(Path::new(&path), 0, None)?;
    let (bank, commands) = EngineBank::prepare(&set, 1, 0)?;
    let output = Output::open_default()?;
    for command in commands {
        output.try_command(command)?;
    }
    for command in bank.set(EngineEvent::Combustion, 0.5, true, true)? {
        output.try_command(command)?;
    }
    std::thread::sleep(Duration::from_secs(2));
    if let Some(error) = output.try_error() {
        anyhow::bail!("audio output: {error}");
    }
    Ok(())
}

use std::path::Path;

use anyhow::{Context, Result};
use esp_formats::cfg;

pub struct FlightData {
    pub configuration: cfg::Config,
    pub air: Vec<u8>,
}

impl FlightData {
    pub fn load(root: &Path, index: u32) -> Result<Self> {
        let configuration = super::config(&super::resolve_path(root, "aircraft.cfg")?)?;
        let variants = cfg::aircraft::configurations(&configuration)?;
        let variant = variants
            .iter()
            .find(|v| v.index == index)
            .with_context(|| format!("aircraft variant {index} is absent"))?;
        let name = variant
            .sim
            .filter(|name| !name.trim().is_empty())
            .context("aircraft variant has no AIR file reference")?;
        let path = super::resolve_path(root, &format!("{name}.air"))?;
        let air = std::fs::read(&path).with_context(|| format!("reading {}", path.display()))?;
        Ok(Self { configuration, air })
    }
}

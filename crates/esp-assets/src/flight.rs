use std::path::Path;

use anyhow::{Context, Result};
use esp_formats::cfg;

pub struct FlightData {
    pub configuration: cfg::Config,
    pub air: Vec<u8>,
}

impl FlightData {
    pub fn load(root: &Path, index: u32) -> Result<Self> {
        let configuration = super::config(&super::aircraft_config_path(root)?)?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sound::SoundSet;

    #[test]
    fn loads_rotorcraft_flight_and_sounds_from_sim_cfg() {
        let root = tempfile::tempdir().unwrap();
        std::fs::write(
            root.path().join("SIM.CFG"),
            "[fltsim.0]\ntitle=Rotorcraft\nsim=Rotor\n",
        )
        .unwrap();
        std::fs::write(root.path().join("Rotor.air"), [1, 2, 3]).unwrap();
        std::fs::create_dir(root.path().join("sound")).unwrap();
        std::fs::write(root.path().join("sound/sound.cfg"), "[sound_engine]\n").unwrap();
        let flight = FlightData::load(root.path(), 0).unwrap();
        assert_eq!(flight.air, [1, 2, 3]);
        assert!(SoundSet::load(root.path(), 0, None).is_ok());
        std::fs::write(
            root.path().join("aircraft.cfg"),
            "[fltsim.0]\ntitle=Aircraft\nsim=Other\n",
        )
        .unwrap();
        assert_eq!(
            super::super::aircraft_config_path(root.path()).unwrap(),
            root.path().join("aircraft.cfg")
        );
        assert!(FlightData::load(root.path(), 0).is_err());
    }
}

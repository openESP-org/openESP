use anyhow::{Context, Result};
use esp_formats::{cabinet::Cabinet, cfg};
use std::{
    io::Read,
    path::{Path, PathBuf},
};

pub struct Panel {
    pub directory: PathBuf,
    config: cfg::Config,
}

impl Panel {
    pub fn load(aircraft: &Path, suffix: Option<&str>) -> Result<Option<Self>> {
        let name = suffix
            .filter(|suffix| !suffix.is_empty())
            .map_or_else(|| "panel".to_owned(), |suffix| format!("panel.{suffix}"));
        let Some(directory) = child(aircraft, &name)? else {
            return Ok(None);
        };
        let path = child(&directory, "panel.cfg")?.context("panel folder has no panel.cfg")?;
        let config = super::config(&path)?;
        cfg::panel::Panel::from_config(&config)?;
        Ok(Some(Self { directory, config }))
    }

    pub fn layout(&self) -> Result<cfg::panel::Panel<'_>> {
        Ok(cfg::panel::Panel::from_config(&self.config)?)
    }

    pub fn background(&self, section: &str) -> Result<[u8; 4]> {
        let Some(value) = self.config.value(section, "background_color")? else {
            return Ok([0, 0, 0, 255]);
        };
        let components = value
            .split(',')
            .map(|part| part.trim().parse::<u8>())
            .collect::<std::result::Result<Vec<_>, _>>()?;
        anyhow::ensure!(components.len() == 3, "invalid panel background color");
        Ok([components[0], components[1], components[2], 255])
    }

    pub fn read_xml_gauge(&self, reference: &str, limit: usize) -> Result<Option<Vec<u8>>> {
        let (module, gauge) = reference
            .split_once('!')
            .context("invalid gauge reference")?;
        anyhow::ensure!(
            !module.is_empty() && !gauge.is_empty(),
            "invalid gauge reference"
        );
        anyhow::ensure!(
            !module.contains(['/', '\\']) && !gauge.contains(['/', '\\']),
            "invalid gauge reference"
        );
        let name = format!("{gauge}.xml");
        self.read_resource(module, &name, limit)
    }

    pub fn read_resource(&self, module: &str, name: &str, limit: usize) -> Result<Option<Vec<u8>>> {
        anyhow::ensure!(
            !module.is_empty()
                && !name.is_empty()
                && !module.contains(['/', '\\'])
                && !name.contains(['/', '\\']),
            "invalid gauge resource name"
        );
        if let Some(directory) = child(&self.directory, module)?
            && directory.is_dir()
            && let Some(path) = child(&directory, name)?
        {
            let size = std::fs::metadata(&path)?.len();
            anyhow::ensure!(size <= limit as u64, "gauge XML exceeds size limit");
            let mut bytes = Vec::new();
            std::fs::File::open(path)?
                .take((limit as u64).saturating_add(1))
                .read_to_end(&mut bytes)?;
            anyhow::ensure!(bytes.len() <= limit, "gauge XML exceeds size limit");
            return Ok(Some(bytes));
        }
        if let Some(path) = child(&self.directory, &format!("{module}.cab"))? {
            let mut cabinet = Cabinet::new(std::fs::File::open(&path)?)
                .with_context(|| format!("opening {}", path.display()))?;
            if cabinet.entry(name).is_some() {
                return Ok(Some(cabinet.read_file_bytes(name, limit)?));
            }
        }
        Ok(None)
    }
}

fn child(directory: &Path, name: &str) -> Result<Option<PathBuf>> {
    let mut found = None;
    for entry in
        std::fs::read_dir(directory).with_context(|| format!("reading {}", directory.display()))?
    {
        let entry = entry?;
        if entry
            .file_name()
            .to_string_lossy()
            .eq_ignore_ascii_case(name)
        {
            anyhow::ensure!(
                found.is_none(),
                "ambiguous {name} in {}",
                directory.display()
            );
            found = Some(entry.path());
        }
    }
    Ok(found)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn loads_selected_panel_and_unpacked_gauge() {
        let root = tempfile::tempdir().unwrap();
        let directory = root.path().join("Panel.IFR");
        std::fs::create_dir(&directory).unwrap();
        std::fs::write(
            directory.join("PANEL.CFG"),
            "[VCockpit01]\nsize_mm=128,128\ntexture=$panel\ngauge00=Example!Dial,1,2,30,30",
        )
        .unwrap();
        std::fs::create_dir(directory.join("example")).unwrap();
        std::fs::write(directory.join("example/DIAL.XML"), b"<Gauge />").unwrap();
        let panel = Panel::load(root.path(), Some("ifr")).unwrap().unwrap();
        assert_eq!(panel.layout().unwrap().windows[0].texture, Some("$panel"));
        assert_eq!(
            panel.read_xml_gauge("Example!Dial", 100).unwrap().unwrap(),
            b"<Gauge />"
        );
        assert!(
            panel
                .read_xml_gauge("Example!Missing", 100)
                .unwrap()
                .is_none()
        );
        assert!(panel.read_xml_gauge("Example!Dial", 1).is_err());
        assert!(panel.read_xml_gauge("../Example!Dial", 100).is_err());
        assert!(Panel::load(root.path(), Some("missing")).unwrap().is_none());
    }
}

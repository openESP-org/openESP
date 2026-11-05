use anyhow::{Context, Result};
use esp_formats::{cabinet::Cabinet, cfg};
use std::{
    io::Read,
    path::{Path, PathBuf},
};

pub struct Panel {
    pub directory: PathBuf,
    gauges: Option<PathBuf>,
    fonts: Option<PathBuf>,
    config: cfg::Config,
}

impl Panel {
    pub fn load(aircraft: &Path, suffix: Option<&str>) -> Result<Option<Self>> {
        Self::load_with_root(aircraft, suffix, None)
    }

    pub fn load_with_root(
        aircraft: &Path,
        suffix: Option<&str>,
        fsx_root: Option<&Path>,
    ) -> Result<Option<Self>> {
        let name = suffix
            .filter(|suffix| !suffix.is_empty())
            .map_or_else(|| "panel".to_owned(), |suffix| format!("panel.{suffix}"));
        let Some(directory) = child(aircraft, &name)? else {
            return Ok(None);
        };
        let (directory, config) =
            super::resolve_container_alias(&directory, "panel.cfg", fsx_root)?;
        cfg::panel::Panel::from_config(&config)?;
        let gauges = fsx_root
            .map(|root| child(root, "Gauges"))
            .transpose()?
            .flatten();
        let fonts = fsx_root
            .map(|root| child(root, "Fonts"))
            .transpose()?
            .flatten();
        Ok(Some(Self {
            directory,
            gauges,
            fonts,
            config,
        }))
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

    /// Load outline fonts shipped with the selected simulator or gauge module.
    /// Keep these scoped to this panel rather than installing them globally.
    pub(crate) fn font_resources(&self, module: &str) -> Result<Vec<Vec<u8>>> {
        anyhow::ensure!(
            !module.is_empty() && !module.contains(['/', '\\']),
            "invalid gauge module"
        );
        const FONT_LIMIT: usize = 16 * 1024 * 1024;
        const TOTAL_LIMIT: usize = 96 * 1024 * 1024;
        fn is_font(name: &str) -> bool {
            let name = name.to_ascii_lowercase();
            name.ends_with(".ttf") || name.ends_with(".otf") || name.ends_with(".ttc")
        }
        fn directory_fonts(
            path: &Path,
            output: &mut Vec<Vec<u8>>,
            total: &mut usize,
        ) -> Result<()> {
            if !path.is_dir() {
                return Ok(());
            }
            let mut paths = std::fs::read_dir(path)?
                .filter_map(|entry| entry.ok().map(|e| e.path()))
                .filter(|path| {
                    path.file_name()
                        .is_some_and(|name| is_font(&name.to_string_lossy()))
                })
                .collect::<Vec<_>>();
            paths.sort();
            for path in paths {
                let size = std::fs::metadata(&path)?.len();
                if size == 0 || size > FONT_LIMIT as u64 || *total + size as usize > TOTAL_LIMIT {
                    continue;
                }
                let bytes = std::fs::read(path)?;
                *total += bytes.len();
                output.push(bytes);
            }
            Ok(())
        }
        let mut output = Vec::new();
        let mut total = 0;
        directory_fonts(&self.directory, &mut output, &mut total)?;
        if let Some(fonts) = &self.fonts {
            directory_fonts(fonts, &mut output, &mut total)?;
        }
        for root in std::iter::once(&self.directory).chain(self.gauges.iter()) {
            if let Some(directory) = child(root, module)? {
                directory_fonts(&directory, &mut output, &mut total)?;
            }
            if let Some(path) = child(root, &format!("{module}.cab"))? {
                let mut cabinet = Cabinet::new(std::fs::File::open(path)?)?;
                let names: Vec<_> = cabinet
                    .entries()
                    .iter()
                    .filter(|entry| is_font(&entry.name))
                    .map(|entry| entry.name.clone())
                    .collect();
                for name in names {
                    if let Ok(bytes) = cabinet.read_file_bytes(&name, FONT_LIMIT)
                        && total + bytes.len() <= TOTAL_LIMIT
                    {
                        total += bytes.len();
                        output.push(bytes);
                    }
                }
            }
        }
        Ok(output)
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
                && !module.contains(':')
                && module != "."
                && module != "..",
            "invalid gauge resource name"
        );
        let components: Vec<_> = name.split(['/', '\\']).collect();
        anyhow::ensure!(
            components.iter().all(|part| !part.is_empty()
                && *part != "."
                && *part != ".."
                && !part.contains(':')),
            "invalid gauge resource name"
        );
        let archive_name = components.join("\\");
        for root in std::iter::once(&self.directory).chain(self.gauges.iter()) {
            if let Some(directory) = child(root, module)?
                && directory.is_dir()
                && let Some(path) = nested_child(&directory, &components)?
            {
                let size = std::fs::metadata(&path)?.len();
                anyhow::ensure!(size <= limit as u64, "gauge resource exceeds size limit");
                let mut bytes = Vec::new();
                std::fs::File::open(path)?
                    .take((limit as u64).saturating_add(1))
                    .read_to_end(&mut bytes)?;
                anyhow::ensure!(bytes.len() <= limit, "gauge resource exceeds size limit");
                return Ok(Some(bytes));
            }
            if let Some(path) = child(root, &format!("{module}.cab"))? {
                let mut cabinet = Cabinet::new(std::fs::File::open(&path)?)
                    .with_context(|| format!("opening {}", path.display()))?;
                if cabinet.entry(&archive_name).is_some() {
                    return Ok(Some(cabinet.read_file_bytes(&archive_name, limit)?));
                }
            }
        }
        Ok(None)
    }
}

fn nested_child(directory: &Path, components: &[&str]) -> Result<Option<PathBuf>> {
    let mut current = directory.to_path_buf();
    for name in components {
        if !current.is_dir() {
            return Ok(None);
        }
        let Some(next) = child(&current, name)? else {
            return Ok(None);
        };
        current = next;
    }
    Ok(Some(current))
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
    fn loads_nested_gauge_art_from_directories_and_cabinets() {
        use cabinet::{CabinetBuilder, WriteCompression};
        use std::io::Cursor;
        let root = tempfile::tempdir().unwrap();
        let panel_dir = root.path().join("panel");
        let image_dir = panel_dir.join("Pack/Art/Day");
        std::fs::create_dir_all(&image_dir).unwrap();
        std::fs::write(panel_dir.join("panel.cfg"), "[VCockpit01]\ntexture=$panel").unwrap();
        std::fs::write(image_dir.join("Needle.BMP"), b"directory").unwrap();
        let mut builder = CabinetBuilder::new(WriteCompression::MsZip);
        builder
            .add_file("Art\\Night\\Needle.BMP", b"cabinet")
            .unwrap();
        let mut archive = Cursor::new(Vec::new());
        builder.write(&mut archive).unwrap();
        std::fs::write(panel_dir.join("Pack.cab"), archive.into_inner()).unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        assert_eq!(
            panel
                .read_resource("pack", "art\\day/needle.bmp", 20)
                .unwrap()
                .unwrap(),
            b"directory"
        );
        assert_eq!(
            panel
                .read_resource("pack", "ART/night/NEEDLE.bmp", 20)
                .unwrap()
                .unwrap(),
            b"cabinet"
        );
        assert!(
            panel
                .read_resource("pack", "art/night/needle.bmp", 1)
                .is_err()
        );
        for path in [
            "../panel.cfg",
            "art/../panel.cfg",
            "/absolute",
            "C:\\absolute",
            "art//x",
        ] {
            assert!(panel.read_resource("pack", path, 100).is_err(), "{path}");
        }
    }

    #[test]
    fn renders_gauge_with_nested_bitmap() {
        let root = tempfile::tempdir().unwrap();
        let panel_dir = root.path().join("panel");
        let module = panel_dir.join("pack");
        std::fs::create_dir_all(module.join("Art")).unwrap();
        std::fs::write(
            panel_dir.join("panel.cfg"),
            "[VCockpit01]\nsize_mm=2,2\npixel_size=2,2\ntexture=$panel\ngauge00=pack!dial,0,0,2,2",
        )
        .unwrap();
        std::fs::write(
            module.join("dial.xml"),
            r#"<Gauge><Size X="2" Y="2"/><Image Name="art\lamp.bmp"/></Gauge>"#,
        )
        .unwrap();
        let mut bitmap = Vec::new();
        image::codecs::bmp::BmpEncoder::new(&mut bitmap)
            .encode(&[255, 0, 0], 1, 1, image::ExtendedColorType::Rgb8)
            .unwrap();
        std::fs::write(module.join("Art/LAMP.BMP"), bitmap).unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let image = crate::gauge::load_panel(&panel, "$panel").unwrap().unwrap();
        assert_eq!(image.omitted_elements, 0);
        assert_eq!(image.image.get_pixel(0, 0).0, [255, 0, 0, 255]);
    }
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

    #[test]
    fn resolves_shared_gauges_after_local_resources() {
        let root = tempfile::tempdir().unwrap();
        let aircraft = root.path().join("aircraft");
        let panel_dir = aircraft.join("panel");
        let shared = root.path().join("gAuGeS").join("Example");
        std::fs::create_dir_all(&panel_dir).unwrap();
        std::fs::create_dir_all(&shared).unwrap();
        std::fs::write(panel_dir.join("panel.cfg"), "[VCockpit01]\ntexture=$panel").unwrap();
        std::fs::write(shared.join("DIAL.XML"), b"shared").unwrap();
        std::fs::write(shared.join("image.bmp"), b"art").unwrap();
        let panel = Panel::load_with_root(&aircraft, None, Some(root.path()))
            .unwrap()
            .unwrap();
        assert_eq!(
            panel.read_xml_gauge("example!dial", 20).unwrap().unwrap(),
            b"shared"
        );
        assert_eq!(
            panel
                .read_resource("example", "IMAGE.BMP", 20)
                .unwrap()
                .unwrap(),
            b"art"
        );
        std::fs::create_dir(panel_dir.join("example")).unwrap();
        std::fs::write(panel_dir.join("example/dial.xml"), b"local").unwrap();
        assert_eq!(
            panel.read_xml_gauge("EXAMPLE!DIAL", 20).unwrap().unwrap(),
            b"local"
        );
        assert!(panel.read_resource("example", "image.bmp", 1).is_err());
    }

    #[test]
    fn follows_panel_alias_and_uses_target_gauge_resources() {
        let root = tempfile::tempdir().unwrap();
        let source = root.path().join("SimObjects/Airplanes/Source/panel");
        let target = root.path().join("SimObjects/Airplanes/Shared/panel.IFR");
        std::fs::create_dir_all(&source).unwrap();
        std::fs::create_dir_all(target.join("Example")).unwrap();
        std::fs::write(
            source.join("panel.cfg"),
            "[fltsim]\nalias=shared\\panel.ifr",
        )
        .unwrap();
        std::fs::write(target.join("panel.cfg"), "[VCockpit01]\ntexture=$shared").unwrap();
        std::fs::write(target.join("Example/Dial.xml"), b"<Gauge />").unwrap();
        let aircraft = source.parent().unwrap();
        let panel = Panel::load_with_root(aircraft, None, Some(root.path()))
            .unwrap()
            .unwrap();
        assert_eq!(panel.directory, target);
        assert_eq!(panel.layout().unwrap().windows[0].texture, Some("$shared"));
        assert_eq!(
            panel.read_xml_gauge("Example!Dial", 100).unwrap().unwrap(),
            b"<Gauge />"
        );
    }
}

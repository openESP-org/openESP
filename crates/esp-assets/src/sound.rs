//! Resolves legacy aircraft sound sets and decodes their WAVE resources.
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, ensure};
use esp_formats::{cfg, wave};
use wave::core::formats::FormatReader;

pub struct SoundSet {
    pub directory: PathBuf,
    shared_directory: Option<PathBuf>,
    config: cfg::Config,
}

#[derive(Debug)]
pub struct Layer {
    pub section: String,
    pub filenames: Vec<String>,
    pub parameters: cfg::sound::Parameters,
}

#[derive(Debug)]
pub struct Sample {
    pub channels: u16,
    pub sample_rate: u32,
    /// Interleaved normalized samples in file channel order.
    pub samples: Vec<f32>,
}

impl SoundSet {
    pub fn load(aircraft: &Path, variant: u32, simulator_root: Option<&Path>) -> Result<Self> {
        let aircraft_config = super::config(&super::resolve_path(aircraft, "aircraft.cfg")?)?;
        let variants = cfg::aircraft::configurations(&aircraft_config)?;
        let selected = variants
            .iter()
            .find(|item| item.index == variant)
            .with_context(|| format!("aircraft variant {variant} is absent"))?;
        let directory = super::variant_directory(aircraft, "sound", selected.sound)?;
        let (directory, config) =
            super::resolve_container_alias(&directory, "sound.cfg", simulator_root)?;
        let shared_directory =
            simulator_root.and_then(|root| super::resolve_path(root, "Sound").ok());
        Ok(Self {
            directory,
            shared_directory,
            config,
        })
    }

    pub fn engine_layers(&self, engine: u32, event: cfg::sound::EngineEvent) -> Result<Vec<Layer>> {
        let engines = cfg::sound::Engines::from_config(&self.config)?;
        let Some(list) = engines
            .lists
            .iter()
            .find(|list| list.engine == engine && list.event == event)
        else {
            return Ok(Vec::new());
        };
        self.layers(list.first)
    }

    pub fn layers(&self, first: &str) -> Result<Vec<Layer>> {
        let sounds = cfg::sound::Sounds::from_config(&self.config)?;
        sounds
            .chain(first)?
            .into_iter()
            .map(|sound| {
                Ok(Layer {
                    section: sound.section.to_owned(),
                    filenames: sound
                        .filenames
                        .iter()
                        .filter(|name| !name.is_empty())
                        .map(|name| (*name).to_owned())
                        .collect(),
                    parameters: cfg::sound::Parameters::from_config(&self.config, sound.section)?,
                })
            })
            .collect()
    }

    /// Search the aircraft sound folder first, then the simulator's shared Sound folder.
    pub fn sample_path(&self, name: &str) -> Result<PathBuf> {
        ensure!(!name.trim().is_empty(), "empty sound filename");
        let name = name.trim();
        let normalized = name.replace('\\', "/");
        let path = Path::new(&normalized);
        let filename = if path.extension().is_some() {
            normalized.clone()
        } else {
            format!("{normalized}.wav")
        };
        for directory in std::iter::once(&self.directory).chain(self.shared_directory.iter()) {
            if let Ok(path) = super::resolve_path(directory, &filename)
                && path.is_file()
            {
                return Ok(path);
            }
        }
        anyhow::bail!(
            "sound {filename:?} not found in {} or the shared Sound folder",
            self.directory.display()
        )
    }

    pub fn sample(&self, name: &str) -> Result<Sample> {
        let path = self.sample_path(name)?;
        let bytes = std::fs::read(&path).with_context(|| format!("reading {}", path.display()))?;
        decode(&bytes).with_context(|| format!("decoding {}", path.display()))
    }
}

pub fn decode(bytes: &[u8]) -> Result<Sample> {
    let wave = wave::Wave::parse(bytes)?;
    let format = wave.format()?;
    let mut reader = wave.reader()?;
    let params = reader
        .tracks()
        .first()
        .context("WAVE has no audio track")?
        .codec_params
        .as_ref()
        .context("WAVE has no codec parameters")?
        .audio()
        .context("WAVE track is not audio")?;
    let mut decoder = wave::get_codecs().make_audio_decoder(params, &Default::default())?;
    let mut samples = Vec::new();
    let mut interleaved = Vec::new();
    while let Some(packet) = reader.next_packet()? {
        let audio = decoder.decode(&packet)?;
        audio.copy_to_vec_interleaved(&mut interleaved);
        samples.extend_from_slice(&interleaved);
    }
    Ok(Sample {
        channels: format.channels,
        sample_rate: format.sample_rate,
        samples,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pcm(sample_count: usize) -> Vec<u8> {
        let mut result = b"RIFF".to_vec();
        result.extend((36u32 + 2 * sample_count as u32).to_le_bytes());
        result.extend(b"WAVEfmt ");
        result.extend(16u32.to_le_bytes());
        result.extend(1u16.to_le_bytes());
        result.extend(1u16.to_le_bytes());
        result.extend(8000u32.to_le_bytes());
        result.extend(16000u32.to_le_bytes());
        result.extend(2u16.to_le_bytes());
        result.extend(16u16.to_le_bytes());
        result.extend(b"data");
        result.extend((2 * sample_count as u32).to_le_bytes());
        for _ in 0..sample_count {
            result.extend(16384i16.to_le_bytes());
        }
        result
    }

    #[test]
    fn resolves_aliases_layers_and_shared_samples() {
        let root = tempfile::tempdir().unwrap();
        let category = root.path().join("SimObjects/Airplanes");
        let aircraft = category.join("Original");
        let source = aircraft.join("sound.ifr");
        let target = category.join("Shared/sound");
        let global = root.path().join("Sound");
        for path in [&source, &target, &global] {
            std::fs::create_dir_all(path).unwrap();
        }
        std::fs::write(
            aircraft.join("aircraft.cfg"),
            "[fltsim.0]\ntitle=test\nsound=ifr\n",
        )
        .unwrap();
        std::fs::write(source.join("sound.cfg"), "[fltsim]\nalias=Shared\\sound").unwrap();
        std::fs::write(target.join("sound.cfg"), "[sound_engine]\neng1_combustion=idle\n[idle]\nfilename=LOCAL\nviewpoint=1\nlink=outside\n[outside]\nfilename=shared\nviewpoint=2\n").unwrap();
        std::fs::write(target.join("LOCAL.WAV"), pcm(20000)).unwrap();
        std::fs::write(global.join("shared.wav"), pcm(4)).unwrap();
        let set = SoundSet::load(&aircraft, 0, Some(root.path())).unwrap();
        assert_eq!(set.directory, target);
        let layers = set
            .engine_layers(1, cfg::sound::EngineEvent::Combustion)
            .unwrap();
        assert_eq!(layers.len(), 2);
        assert_eq!(layers[0].parameters.viewpoint, Some(1));
        assert_eq!(layers[1].parameters.viewpoint, Some(2));
        let sample = set.sample(&layers[0].filenames[0]).unwrap();
        assert_eq!(
            (sample.channels, sample.sample_rate, sample.samples.len()),
            (1, 8000, 20000)
        );
        assert!(
            sample
                .samples
                .iter()
                .all(|sample| (*sample - 0.5).abs() < 1e-5)
        );
        assert_eq!(set.sample("sHaReD.wav").unwrap().samples.len(), 4);
        assert!(set.sample("absent").is_err());
        assert!(
            set.engine_layers(2, cfg::sound::EngineEvent::Combustion)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn rejects_broken_wave_data() {
        let bytes = pcm(4);
        for end in 0..bytes.len() {
            assert!(decode(&bytes[..end]).is_err());
        }
    }
}

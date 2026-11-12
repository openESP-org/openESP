//! Resolve aircraft light effects and their shared textures without altering content.
use std::{collections::HashMap, path::Path, sync::Arc};

use anyhow::{Context, Result};
use esp_formats::cfg::{self, effect::Particle, lights::Light};

/// Decoded light placements and effect definitions with original texture bytes.
pub struct LightEffects {
    pub lights: Vec<Light>,
    pub effects: HashMap<String, Vec<Particle>>,
    pub textures: HashMap<String, Arc<[u8]>>,
}

impl LightEffects {
    /// Load what is available, preserving other lights if one add-on resource is missing.
    pub fn load(aircraft: &Path, simulator_root: &Path) -> Result<Self> {
        let config = super::config(&super::aircraft_config_path(aircraft)?)?;
        let lights = cfg::lights::from_config(&config)?;
        let effect_dir = super::resolve_path(simulator_root, "Effects")?;
        let texture_dir = super::resolve_path(&effect_dir, "texture").ok();
        let mut effects = HashMap::new();
        let mut textures = HashMap::new();
        for light in &lights {
            let key = light.effect.to_ascii_lowercase();
            if effects.contains_key(&key) {
                continue;
            }
            let file = if key.ends_with(".fx") {
                light.effect.clone()
            } else {
                format!("{}.fx", light.effect)
            };
            let loaded = (|| -> Result<Vec<Particle>> {
                let path = super::resolve_path(&effect_dir, &file)?;
                let config = super::config(&path)?;
                Ok(cfg::effect::particles(&config)?)
            })();
            let particles = match loaded {
                Ok(particles) => particles,
                Err(error) => {
                    eprintln!("effect {file}: {error:#}");
                    continue;
                }
            };
            if let Some(directory) = &texture_dir {
                for particle in &particles {
                    let Some(name) = &particle.texture else {
                        continue;
                    };
                    let texture_key = name.to_ascii_lowercase();
                    if textures.contains_key(&texture_key) {
                        continue;
                    }
                    let loaded = super::resolve_texture(directory, name).and_then(|path| {
                        std::fs::read(&path).with_context(|| format!("reading {}", path.display()))
                    });
                    match loaded {
                        Ok(bytes) => {
                            textures.insert(texture_key, Arc::from(bytes));
                        }
                        Err(error) => eprintln!("effect texture {name}: {error:#}"),
                    }
                }
            }
            effects.insert(key, particles);
        }
        Ok(Self {
            lights,
            effects,
            textures,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn loads_available_effects_and_reuses_case_insensitive_textures() {
        let root = tempfile::tempdir().unwrap();
        let aircraft = root.path().join("aircraft");
        let effects = root.path().join("Effects");
        let textures = effects.join("texture");
        std::fs::create_dir_all(&aircraft).unwrap();
        std::fs::create_dir_all(&textures).unwrap();
        std::fs::write(aircraft.join("aircraft.cfg"), "[lights]\nlight.0=3,0,-10,0,FX_NAV\nlight.1=3,0,10,0,fx_nav\nlight.2=2,0,0,0,missing\n").unwrap();
        std::fs::write(effects.join("fx_nav.fx"), "[Emitter.0]\nRate=1,1\n[Particle.0]\nStatic=1\n[ParticleAttributes.0]\nTexture=FX_2.BMP\n").unwrap();
        std::fs::write(textures.join("fx_2.bmp"), b"texture").unwrap();
        let loaded = LightEffects::load(&aircraft, root.path()).unwrap();
        assert_eq!(loaded.lights.len(), 3);
        assert_eq!(loaded.effects.len(), 1);
        assert_eq!(loaded.textures.len(), 1);
        assert_eq!(&*loaded.textures["fx_2.bmp"], b"texture");
    }
}

//! Decode the emitter and particle fields in FSX and Prepar3D effect files.
use super::{Config, ValueError, indexed_name};

#[derive(Debug, Clone, PartialEq)]
pub struct Particle {
    pub index: u32,
    pub emitter_lifetime: [f64; 2],
    pub delay: [f64; 2],
    pub rate: [f64; 2],
    pub emitter_offset: [[f64; 2]; 3],
    pub emitter_velocity: [[f64; 2]; 3],
    pub particle_velocity: [[f64; 2]; 3],
    pub lifetime: [f64; 2],
    pub kind: u32,
    pub scale: [[f64; 2]; 2],
    pub is_static: bool,
    pub blend_mode: u32,
    pub texture: Option<String>,
    pub color_start: [f64; 4],
    pub color_end: [f64; 4],
    pub uv1: [f64; 2],
    pub uv2: [f64; 2],
    pub falloff_exponent: f64,
    pub inner_cone_angle: f64,
    pub outer_cone_angle: f64,
    pub intensity_day: f64,
    pub intensity_night: f64,
}

fn invalid(section: &str, key: &str, value: &str) -> ValueError {
    ValueError::Invalid {
        section: section.into(),
        key: key.into(),
        value: value.into(),
    }
}
fn values<const N: usize>(
    config: &Config,
    section: &str,
    key: &str,
    default: [f64; N],
) -> Result<[f64; N], ValueError> {
    let Some(value) = config.value(section, key)? else {
        return Ok(default);
    };
    let fields = value.split(',').map(str::trim).collect::<Vec<_>>();
    if fields.len() != N {
        return Err(invalid(section, key, value));
    }
    let mut result = default;
    for (out, field) in result.iter_mut().zip(fields) {
        *out = field
            .parse::<f64>()
            .ok()
            .filter(|v| v.is_finite())
            .ok_or_else(|| invalid(section, key, value))?;
    }
    Ok(result)
}
fn integer(config: &Config, section: &str, key: &str, default: u32) -> Result<u32, ValueError> {
    config.value(section, key)?.map_or(Ok(default), |v| {
        v.parse().map_err(|_| invalid(section, key, v))
    })
}

pub fn particles(config: &Config) -> Result<Vec<Particle>, ValueError> {
    let mut result = Vec::new();
    for (section, _) in &config.sections {
        let Some(section) = section else {
            continue;
        };
        let Some(index) = indexed_name(section, "Emitter.")? else {
            continue;
        };
        let particle = format!("Particle.{index}");
        let attributes = format!("ParticleAttributes.{index}");
        result.push(Particle {
            index,
            emitter_lifetime: values(config, section, "Lifetime", [0.0; 2])?,
            delay: values(config, section, "Delay", [0.0; 2])?,
            rate: values(config, section, "Rate", [0.0; 2])?,
            emitter_offset: [
                values(config, section, "X Offset", [0.0; 2])?,
                values(config, section, "Y Offset", [0.0; 2])?,
                values(config, section, "Z Offset", [0.0; 2])?,
            ],
            emitter_velocity: [
                values(config, section, "X Emitter Velocity", [0.0; 2])?,
                values(config, section, "Y Emitter Velocity", [0.0; 2])?,
                values(config, section, "Z Emitter Velocity", [0.0; 2])?,
            ],
            particle_velocity: [
                values(config, section, "X Particle Velocity", [0.0; 2])?,
                values(config, section, "Y Particle Velocity", [0.0; 2])?,
                values(config, section, "Z Particle Velocity", [0.0; 2])?,
            ],
            lifetime: values(config, &particle, "Lifetime", [0.0; 2])?,
            kind: integer(config, &particle, "Type", 19)?,
            scale: [
                values(config, &particle, "X Scale", [1.0; 2])?,
                values(config, &particle, "Y Scale", [1.0; 2])?,
            ],
            is_static: integer(config, &particle, "Static", 0)? != 0,
            blend_mode: integer(config, &attributes, "Blend Mode", 1)?,
            texture: config
                .value(&attributes, "Texture")?
                .filter(|s| !s.is_empty())
                .map(str::to_owned),
            color_start: values(config, &attributes, "Color Start", [255.0; 4])?,
            color_end: values(config, &attributes, "Color End", [255.0; 4])?,
            uv1: values(config, &attributes, "uv1", [0.0; 2])?,
            uv2: values(config, &attributes, "uv2", [1.0; 2])?,
            falloff_exponent: config
                .number(&attributes, "Falloff Exponent")?
                .unwrap_or(2.0),
            inner_cone_angle: config
                .number(&attributes, "Inner Cone Angle")?
                .unwrap_or(0.0),
            outer_cone_angle: config
                .number(&attributes, "Outer Cone Angle")?
                .unwrap_or(0.0),
            intensity_day: config.number(&attributes, "Intensity Day")?.unwrap_or(0.0),
            intensity_night: config
                .number(&attributes, "Intensity Night")?
                .unwrap_or(0.0),
        });
    }
    result.sort_by_key(|particle| particle.index);
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn decodes_static_additive_particle_and_delayed_emitter() {
        let config = Config::parse("[Emitter.2]\nDelay=0.2,0.2\nRate=0.5,0.5\n[Particle.2]\nLifetime=0.01,0.01\nType=19\nX Scale=0.5,0.5\nY Scale=0.4,0.4\nStatic=1\n[ParticleAttributes.2]\nBlend Mode=2\nTexture=fx_2.bmp\nColor Start=100,100,150,200\nuv2=0.5,0.5\n").unwrap();
        let result = particles(&config).unwrap();
        assert_eq!(result[0].delay, [0.2, 0.2]);
        assert_eq!(result[0].color_start, [100.0, 100.0, 150.0, 200.0]);
        assert_eq!(result[0].texture.as_deref(), Some("fx_2.bmp"));
        assert!(result[0].is_static);
        let bad = Config::parse("[Emitter.0]\nRate=NaN,1\n").unwrap();
        assert!(particles(&bad).is_err());
    }
}

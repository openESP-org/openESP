use anyhow::{Result, bail};
use esp_formats::mdl::{self, animation::KeyValue, visibility::Condition};
use glam::Vec4;

pub enum Source {
    Sim(String),
    Code {
        expression: String,
        lag: Option<f64>,
    },
}

pub struct Animation {
    pub length: f32,
    pub kind: String,
    pub parameters: String,
    pub source: Option<Source>,
    pub tracks: Vec<Track>,
}

pub struct Track {
    pub id: u32,
    pub kind: u32,
    pub keys: Vec<Key>,
}

pub struct Key {
    pub time: f32,
    pub value: Vec4,
}

pub struct Visibility {
    pub default_visible: bool,
    pub source: Option<Source>,
}

fn text(bytes: &[u8]) -> Result<String> {
    Ok(std::str::from_utf8(bytes.strip_suffix(&[0]).unwrap_or(bytes))?.to_owned())
}

fn code(bytes: &[u8]) -> Result<Source> {
    let code = mdl::parameter::Code::parse(bytes)?;
    Ok(Source::Code {
        expression: code.expression.into(),
        lag: code.lag,
    })
}

pub(super) fn animations(data: &mdl::data::ModelData<'_>) -> Result<Vec<Animation>> {
    let mut animations = Vec::new();
    for animation in data.animations.iter().flat_map(|b| &b.animations) {
        anyhow::ensure!(
            animation.length.is_finite() && animation.length >= 0.0,
            "invalid animation length"
        );
        let mut source = None;
        for chunk in &animation.unknown {
            let parameter = match &chunk.id {
                b"XAPS" => Some(Source::Sim(text(chunk.data)?)),
                b"XAPC" => Some(code(chunk.data)?),
                _ => None,
            };
            if let Some(parameter) = parameter {
                anyhow::ensure!(source.is_none(), "duplicate animation parameter");
                source = Some(parameter);
            }
        }
        let mut tracks = Vec::new();
        for track in &animation.tracks {
            if !matches!(track.kind, 0 | 2) {
                bail!("unsupported animation track {}", track.kind);
            }
            let mut keys: Vec<Key> = Vec::new();
            for key in &track.keys {
                let value = match key.value {
                    KeyValue::Translation(v) if track.kind == 2 => Vec4::new(v[0], v[1], v[2], 0.0),
                    KeyValue::Rotation(v) if track.kind == 0 => Vec4::from_array(v),
                    _ => bail!("unsupported animation key in track {}", track.id),
                };
                anyhow::ensure!(
                    key.time.is_finite() && value.is_finite(),
                    "non-finite animation key"
                );
                if track.kind == 0 {
                    anyhow::ensure!(value.length_squared() > 1e-12, "zero animation quaternion");
                }
                if let Some(previous) = keys.last() {
                    anyhow::ensure!(key.time >= previous.time, "unordered animation keys");
                }
                keys.push(Key {
                    time: key.time,
                    value,
                });
            }
            tracks.push(Track {
                id: track.id,
                kind: track.kind,
                keys,
            });
        }
        animations.push(Animation {
            length: animation.length,
            kind: text(animation.type_name)?,
            parameters: text(animation.type_parameter)?,
            source,
            tracks,
        });
    }
    Ok(animations)
}

pub(super) fn visibility(data: &mdl::data::ModelData<'_>) -> Result<Vec<Visibility>> {
    data.visibility
        .iter()
        .map(|condition| {
            let (default_visible, source) = match condition {
                Condition::Sim {
                    default_visible,
                    definition,
                } => (*default_visible, Some(Source::Sim(text(definition)?))),
                Condition::Code {
                    default_visible,
                    definition,
                } => (*default_visible, Some(code(definition)?)),
                Condition::Unknown(_) => (true, None),
            };
            Ok(Visibility {
                default_visible,
                source,
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    fn data(keys: Vec<mdl::animation::Key<'static>>, kind: u32) -> mdl::data::ModelData<'static> {
        mdl::data::ModelData {
            animations: Some(mdl::animation::AnimationBlock {
                version: Some(9.2),
                unknown: vec![],
                unknown_list_sections: vec![],
                trailing_data: &[],
                animations: vec![mdl::animation::Animation {
                    guid: Default::default(),
                    length: 100.0,
                    type_name: b"Sim",
                    type_parameter: b"AutoPlay",
                    unknown: vec![],
                    tracks: vec![mdl::animation::Track {
                        kind,
                        id: 42,
                        length: 100.0,
                        keys,
                        unknown: vec![],
                    }],
                }],
            }),
            ..Default::default()
        }
    }
    #[test]
    fn rejects_nonfinite_unordered_and_zero_rotation_keys() {
        let key = |time, value| mdl::animation::Key {
            time,
            value: KeyValue::Translation(value),
        };
        assert!(animations(&data(vec![key(f32::NAN, [0.0; 3])], 2)).is_err());
        assert!(animations(&data(vec![key(0.0, [f32::INFINITY; 3])], 2)).is_err());
        assert!(animations(&data(vec![key(2.0, [0.0; 3]), key(1.0, [0.0; 3])], 2)).is_err());
        assert!(
            animations(&data(
                vec![mdl::animation::Key {
                    time: 0.0,
                    value: KeyValue::Rotation([0.0; 4])
                }],
                0
            ))
            .is_err()
        );
        assert!(animations(&data(vec![key(0.0, [0.0; 3]), key(1.0, [1.0; 3])], 2)).is_ok());
    }
}

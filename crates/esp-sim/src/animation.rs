use anyhow::{Context, Result, bail};
use esp_assets::{
    Model,
    animation::{Source, Track},
};
use esp_script::{Host, Program, Reference};
use glam::{Mat4, Quat, Vec3, Vec4};
use petgraph::{algo::toposort, graphmap::DiGraphMap};
use std::collections::HashMap;

enum Driver {
    Sim {
        reference: Reference,
        scale: f64,
        bias: f64,
    },
    Code(Program),
}

struct Parameter {
    driver: Option<Driver>,
    lag: Option<f64>,
    value: Option<f64>,
    reported: bool,
}

impl Parameter {
    fn new(source: Option<&Source>) -> Result<Self> {
        let (driver, lag) = match source {
            Some(Source::Code { expression, lag }) => {
                (Some(Driver::Code(Program::compile(expression)?)), *lag)
            }
            Some(Source::Sim(definition)) => {
                let fields: Vec<_> = definition.split('#').collect();
                anyhow::ensure!(
                    (2..=5).contains(&fields.len()),
                    "unsupported simulation parameter {definition}"
                );
                let number = |index: usize, default: f64| -> Result<f64> {
                    let value = fields
                        .get(index)
                        .filter(|s| !s.is_empty())
                        .map(|s| s.parse::<f64>())
                        .transpose()?
                        .unwrap_or(default);
                    anyhow::ensure!(value.is_finite(), "non-finite simulation parameter");
                    Ok(value)
                };
                let lag = fields.get(4).map(|_| number(4, 0.0)).transpose()?;
                (
                    Some(Driver::Sim {
                        reference: Reference::new('A', fields[0], Some(fields[1])),
                        scale: number(2, 1.0)?,
                        bias: number(3, 0.0)?,
                    }),
                    lag,
                )
            }
            None => (None, None),
        };
        if let Some(lag) = lag {
            anyhow::ensure!(lag >= 0.0, "negative animation lag");
        }
        Ok(Self {
            driver,
            lag,
            value: None,
            reported: false,
        })
    }

    fn evaluate(&mut self, host: &mut impl Host, dt: f64) -> Result<Option<f64>> {
        let target = match &self.driver {
            Some(Driver::Sim {
                reference,
                scale,
                bias,
            }) => host.read(reference)?.number()? * scale + bias,
            Some(Driver::Code(program)) => program
                .evaluate(host)?
                .context("animation expression has no result")?
                .number()?,
            None => return Ok(None),
        };
        anyhow::ensure!(target.is_finite(), "non-finite animation target");
        let value = match (self.value, self.lag) {
            (Some(previous), Some(lag)) if lag > 0.0 => {
                previous + (target - previous).clamp(-lag * dt, lag * dt)
            }
            _ => target,
        };
        self.value = Some(value);
        Ok(Some(value))
    }

    fn evaluate_or_report(
        &mut self,
        host: &mut impl Host,
        dt: f64,
        diagnostics: &mut Vec<String>,
    ) -> Option<f64> {
        match self.evaluate(host, dt) {
            Ok(value) => value,
            Err(error) => {
                if !self.reported {
                    diagnostics.push(error.to_string());
                    self.reported = true;
                }
                self.value
            }
        }
    }
}

pub struct ModelState {
    pub transforms: Vec<Mat4>,
    pub visible: Vec<bool>,
    pub diagnostics: Vec<String>,
    parameters: Vec<Parameter>,
    conditions: Vec<Parameter>,
    order: Vec<usize>,
    track_animations: HashMap<usize, Vec<usize>>,
    elapsed: f64,
}

impl ModelState {
    pub fn new(model: &Model) -> Result<Self> {
        let mut graph = DiGraphMap::<usize, ()>::new();
        for (index, node) in model.nodes.iter().enumerate() {
            graph.add_node(index);
            if let Some(parent) = node.parent {
                anyhow::ensure!(parent < model.nodes.len(), "invalid parent");
                graph.add_edge(parent, index, ());
            }
        }
        let order =
            toposort(&graph, None).map_err(|_| anyhow::anyhow!("cyclic model hierarchy"))?;
        let mut track_animations: HashMap<usize, Vec<usize>> = HashMap::new();
        for (index, animation) in model.animations.iter().enumerate() {
            for track in &animation.tracks {
                let animations = track_animations.entry(track.id as usize).or_default();
                if animations.last() != Some(&index) {
                    animations.push(index);
                }
            }
        }
        let parameters = model
            .animations
            .iter()
            .map(|a| Parameter::new(a.source.as_ref()))
            .collect::<Result<Vec<_>>>()?;
        let conditions = model
            .visibility
            .iter()
            .map(|v| Parameter::new(v.source.as_ref()))
            .collect::<Result<Vec<_>>>()?;
        for node in &model.nodes {
            anyhow::ensure!(
                node.animation
                    .is_none_or(|index| track_animations.contains_key(&index)),
                "invalid animation link"
            );
            anyhow::ensure!(
                node.visibility.is_none_or(|index| index < conditions.len()),
                "invalid visibility link"
            );
        }
        let layered = model
            .nodes
            .iter()
            .filter(|n| {
                n.animation
                    .is_some_and(|id| track_animations[&id].len() > 1)
            })
            .count();
        let diagnostics = if layered > 0 {
            vec![format!(
                "{layered} model nodes use unsupported layered animations; retaining their bind poses"
            )]
        } else {
            Vec::new()
        };
        Ok(Self {
            transforms: model.transforms()?,
            visible: vec![true; model.nodes.len()],
            diagnostics,
            parameters,
            conditions,
            order,
            track_animations,
            elapsed: 0.0,
        })
    }

    pub fn update(&mut self, model: &Model, host: &mut impl Host, dt: f64) -> Result<()> {
        anyhow::ensure!(dt.is_finite() && dt >= 0.0, "invalid animation timestep");
        self.elapsed += dt;
        let values = self
            .parameters
            .iter_mut()
            .enumerate()
            .map(|(index, parameter)| {
                let animation = &model.animations[index];
                parameter
                    .evaluate_or_report(host, dt, &mut self.diagnostics)
                    .or_else(|| {
                        (animation.kind.eq_ignore_ascii_case("Standard")
                            && animation
                                .parameters
                                .split(',')
                                .any(|p| p.eq_ignore_ascii_case("AutoPlay"))
                            && animation.length > 0.0)
                            .then(|| (self.elapsed * 18.0).rem_euclid(f64::from(animation.length)))
                    })
                    .map(|value| (value as f32).clamp(0.0, animation.length))
            })
            .collect::<Vec<_>>();
        let visibility = self
            .conditions
            .iter_mut()
            .enumerate()
            .map(|(index, parameter)| {
                parameter
                    .evaluate_or_report(host, dt, &mut self.diagnostics)
                    .map_or(model.visibility[index].default_visible, |value| {
                        value != 0.0
                    })
            })
            .collect::<Vec<_>>();
        for &index in &self.order {
            let node = &model.nodes[index];
            let mut translation = Vec3::ZERO;
            let mut rotation = Quat::IDENTITY;
            if let Some(link) = node.animation
                && let [animation_index] = self.track_animations[&link].as_slice()
            {
                let animation_index = *animation_index;
                let animation = &model.animations[animation_index];
                if let Some(frame) = values[animation_index] {
                    for track in animation.tracks.iter().filter(|t| t.id as usize == link) {
                        if let Some(value) = sample(track, frame) {
                            match track.kind {
                                0 => rotation = Quat::from_array(value.to_array()).normalize(),
                                2 => translation = value.truncate(),
                                kind => bail!("unsupported animation kind {kind}"),
                            }
                        }
                    }
                }
            }
            let local = node.transform * Mat4::from_rotation_translation(rotation, translation);
            self.transforms[index] = node
                .parent
                .map_or(Mat4::IDENTITY, |parent| self.transforms[parent])
                * local;
            self.visible[index] = node.visibility.is_none_or(|link| visibility[link])
                && node.parent.is_none_or(|parent| self.visible[parent]);
        }
        Ok(())
    }
}

fn sample(track: &Track, time: f32) -> Option<Vec4> {
    let first = track.keys.first()?;
    let after = track.keys.partition_point(|key| key.time <= time);
    if after == 0 {
        return Some(first.value);
    }
    let a = &track.keys[after - 1];
    let Some(b) = track.keys.get(after) else {
        return Some(a.value);
    };
    let weight = (time - a.time) / (b.time - a.time);
    Some(if track.kind == 0 {
        Vec4::from_array(
            Quat::from_array(a.value.to_array())
                .normalize()
                .slerp(Quat::from_array(b.value.to_array()).normalize(), weight)
                .to_array(),
        )
    } else {
        a.value.lerp(b.value, weight)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_assets::animation::Key;
    #[test]
    fn maps_stream_ids_and_propagates_parent_visibility() {
        let model = Model {
            nodes: vec![
                esp_assets::Node {
                    parent: Some(1),
                    transform: Mat4::from_translation(Vec3::X),
                    animation: Some(42),
                    visibility: None,
                },
                esp_assets::Node {
                    parent: None,
                    transform: Mat4::from_rotation_y(std::f32::consts::FRAC_PI_2),
                    animation: None,
                    visibility: Some(0),
                },
            ],
            materials: vec![],
            lods: vec![],
            animations: vec![esp_assets::animation::Animation {
                length: 10.0,
                kind: "Sim".into(),
                parameters: "AutoPlay".into(),
                source: Some(Source::Code {
                    expression: "5".into(),
                    lag: None,
                }),
                tracks: vec![Track {
                    id: 42,
                    kind: 2,
                    keys: vec![
                        Key {
                            time: 0.0,
                            value: Vec4::ZERO,
                        },
                        Key {
                            time: 10.0,
                            value: Vec4::new(10.0, 0.0, 0.0, 0.0),
                        },
                    ],
                }],
            }],
            visibility: vec![esp_assets::animation::Visibility {
                default_visible: true,
                source: Some(Source::Code {
                    expression: "0".into(),
                    lag: None,
                }),
            }],
        };
        let mut state = ModelState::new(&model).unwrap();
        state
            .update(&model, &mut crate::variables::Variables::default(), 0.0)
            .unwrap();
        assert!(
            state.transforms[0]
                .transform_point3(Vec3::ZERO)
                .abs_diff_eq(Vec3::new(0.0, 0.0, -6.0), 1e-5)
        );
        assert_eq!(state.visible, [false, false]);
    }
    #[test]
    fn interpolates_rotations_and_clamps_translation_endpoints() {
        let mut track = Track {
            id: 0,
            kind: 2,
            keys: vec![
                Key {
                    time: 0.0,
                    value: Vec4::ZERO,
                },
                Key {
                    time: 10.0,
                    value: Vec4::new(10.0, 0.0, 0.0, 0.0),
                },
            ],
        };
        assert_eq!(sample(&track, -1.0), Some(Vec4::ZERO));
        assert_eq!(sample(&track, 20.0), Some(track.keys[1].value));
        assert_eq!(sample(&track, 5.0), Some(Vec4::new(5.0, 0.0, 0.0, 0.0)));
        track.kind = 0;
        track.keys[0].value = Vec4::W;
        track.keys[1].value = Vec4::from_array(Quat::from_rotation_y(1.0).to_array());
        assert!(
            Quat::from_array(sample(&track, 5.0).unwrap().to_array())
                .abs_diff_eq(Quat::from_rotation_y(0.5), 1e-6)
        );
    }
    #[test]
    fn lag_limits_keyframes_per_second() {
        let mut parameter = Parameter::new(Some(&Source::Code {
            expression: "100".into(),
            lag: Some(50.0),
        }))
        .unwrap();
        parameter.value = Some(0.0);
        assert_eq!(
            parameter
                .evaluate(&mut crate::variables::Variables::default(), 0.1)
                .unwrap(),
            Some(5.0)
        );
    }
}

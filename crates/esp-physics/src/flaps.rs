use anyhow::{Result, ensure};
use esp_formats::cfg::aircraft::FlapSet;
use esp_script::Error;

pub struct Flaps {
    sets: Vec<Actuator>,
    handle: u32,
    max_handle: u32,
    alpha: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Power {
    pub electric: f64,
    pub hydraulic: f64,
}

impl Default for Power {
    fn default() -> Self {
        Self {
            electric: 1.0,
            hydraulic: 1.0,
        }
    }
}

struct Actuator {
    definition: FlapSet,
    angle: f64,
    previous: f64,
    rate: f64,
}

impl Flaps {
    pub fn angles(&self) -> impl Iterator<Item = f64> + '_ {
        self.sets.iter().map(|set| set.angle.to_radians())
    }

    pub fn new(definitions: &[FlapSet]) -> Result<Self> {
        let mut sets = Vec::new();
        let mut max_handle = 0;
        for definition in definitions.iter().filter(|set| set.kind != 0) {
            ensure!(
                matches!(definition.kind, 1 | 2),
                "unsupported flap type {}",
                definition.kind
            );
            ensure!(
                definition
                    .positions
                    .first()
                    .is_some_and(|position| position.index == 0 && position.angle.is_finite()),
                "flaps.{} has no initial position",
                definition.index
            );
            ensure!(
                definition
                    .positions
                    .windows(2)
                    .all(|pair| pair[0].index < pair[1].index
                        && pair[0].angle <= pair[1].angle
                        && pair[1].angle.is_finite()),
                "invalid flap detents in flaps.{}",
                definition.index
            );
            let last = definition.positions.last().unwrap();
            ensure!(last.index < u32::MAX, "flap handle range is too large");
            max_handle = max_handle.max(last.index);
            let angle = definition.positions[0].angle;
            let time = definition.extending_time.ok_or_else(|| {
                anyhow::anyhow!("flaps.{} has no extending-time", definition.index)
            })?;
            ensure!(
                time.is_finite() && time > 0.0,
                "invalid flap extending-time"
            );
            let rate = (last.angle - angle) / time;
            ensure!(rate.is_finite(), "invalid flap travel rate");
            sets.push(Actuator {
                definition: definition.clone(),
                angle,
                previous: angle,
                rate,
            });
        }
        Ok(Self {
            sets,
            handle: 0,
            max_handle,
            alpha: 1.0,
        })
    }

    pub fn event(&mut self, name: &str, value: f64) -> Result<(), Error> {
        if !value.is_finite() {
            return Err(Error::Number);
        }
        if self.sets.is_empty() {
            return Err(Error::Unsupported(
                "aircraft has no configured flaps".into(),
            ));
        }
        self.handle = match name {
            "FLAPS_UP" => 0,
            "FLAPS_DOWN" | "FLAPS_4" => self.max_handle,
            "FLAPS_1" => 1.min(self.max_handle),
            "FLAPS_2" => 2.min(self.max_handle),
            "FLAPS_3" => 3.min(self.max_handle),
            "FLAPS_INCR" => self.handle.saturating_add(1).min(self.max_handle),
            "FLAPS_DECR" => self.handle.saturating_sub(1),
            "FLAPS_SET" => {
                (value.clamp(0.0, 16383.0) / 16383.0 * f64::from(self.max_handle)).round() as u32
            }
            "AXIS_FLAPS_SET" => ((value.clamp(-16383.0, 16383.0) + 16383.0) / 32766.0
                * f64::from(self.max_handle))
            .round() as u32,
            _ => return Err(Error::Unsupported(format!("flap event {name}"))),
        };
        Ok(())
    }

    pub fn advance(&mut self, dt: f64, airspeed: f64) -> Result<()> {
        self.advance_with_power(dt, airspeed, Power::default())
    }

    pub fn advance_with_power(&mut self, dt: f64, airspeed: f64, power: Power) -> Result<()> {
        ensure!(dt.is_finite() && dt >= 0.0, "invalid flap timestep");
        ensure!(
            [power.electric, power.hydraulic]
                .iter()
                .all(|v| v.is_finite() && (0.0..=1.0).contains(v)),
            "invalid flap power"
        );
        ensure!(
            airspeed.is_finite() && airspeed >= 0.0,
            "invalid flap airspeed"
        );
        self.alpha = 1.0;
        for set in &mut self.sets {
            set.previous = set.angle;
            let positions = &set.definition.positions;
            let index = positions.partition_point(|position| position.index <= self.handle) - 1;
            let position = &positions[index];
            if position.speed_limit.is_some_and(|limit| airspeed > limit) {
                continue;
            }
            let target = position.angle;
            let available = match set.definition.system_type {
                Some(0) => power.electric,
                Some(1) => power.hydraulic,
                Some(4) => 0.0,
                _ => 1.0,
            };
            let travel = set.rate * dt * available;
            set.angle += (target - set.angle).clamp(-travel, travel);
        }
        Ok(())
    }

    pub fn interpolate(&mut self, alpha: f64) -> Result<()> {
        ensure!(
            alpha.is_finite() && (0.0..=1.0).contains(&alpha),
            "invalid simulation interpolation"
        );
        self.alpha = alpha;
        Ok(())
    }

    pub fn read(&self, name: &str) -> Option<(f64, &'static str)> {
        match name {
            "FLAPS AVAILABLE" => return Some((f64::from(!self.sets.is_empty()), "bool")),
            "FLAPS HANDLE INDEX" => return Some((f64::from(self.handle), "number")),
            "FLAPS NUM HANDLE POSITIONS" => {
                return Some((
                    if self.sets.is_empty() {
                        0.0
                    } else {
                        f64::from(self.max_handle) + 1.0
                    },
                    "number",
                ));
            }
            "FLAPS HANDLE PERCENT" => {
                return Some((
                    if self.max_handle == 0 {
                        0.0
                    } else {
                        f64::from(self.handle) / f64::from(self.max_handle)
                    },
                    "percent over 100",
                ));
            }
            _ => {}
        }
        let (edge, rest) = name.split_once(" FLAPS")?;
        let kind = match edge {
            "TRAILING EDGE" => 1,
            "LEADING EDGE" => 2,
            _ => return None,
        };
        let (index, rest) = rest.split_once(' ')?;
        let (side, measurement) = rest.split_once(' ')?;
        if !matches!(side, "LEFT" | "RIGHT") {
            return None;
        }
        let percent = match measurement {
            "ANGLE" => false,
            "PERCENT" => true,
            _ => return None,
        };
        let value = |set: &Actuator| {
            let angle = set.previous + (set.angle - set.previous) * self.alpha;
            let first = set.definition.positions[0].angle;
            let range = set.definition.positions.last().unwrap().angle - first;
            if percent {
                if range == 0.0 {
                    0.0
                } else {
                    (angle - first) / range
                }
            } else {
                angle
            }
        };
        let mut sets = self.sets.iter().filter(|s| s.definition.kind == kind);
        let value = if index.is_empty() {
            sets.map(value).reduce(f64::max).unwrap_or(0.0)
        } else {
            sets.nth(index.parse::<usize>().ok()?)
                .map(value)
                .unwrap_or(0.0)
        };
        Some((
            value,
            if percent {
                "percent over 100"
            } else {
                "degrees"
            },
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_formats::cfg::aircraft::FlapPosition;

    fn positions(points: &[(u32, f64)]) -> Vec<FlapPosition> {
        points
            .iter()
            .map(|&(index, angle)| FlapPosition {
                index,
                angle,
                speed_limit: None,
            })
            .collect()
    }

    fn definition() -> FlapSet {
        FlapSet {
            index: 0,
            kind: 1,
            extending_time: Some(10.0),
            system_type: None,
            positions: positions(&[(0, -5.0), (1, 0.0), (2, 25.0)]),
        }
    }

    #[test]
    fn numbered_and_axis_commands_select_and_bound_detents() {
        let mut primary = definition();
        primary.positions = positions(&[
            (0, 0.0),
            (1, 10.0),
            (2, 20.0),
            (3, 30.0),
            (4, 40.0),
            (5, 50.0),
        ]);
        let mut flaps = Flaps::new(&[primary]).unwrap();
        for (event, input, expected) in [
            ("FLAPS_1", 0.0, 1.0),
            ("FLAPS_2", 0.0, 2.0),
            ("FLAPS_3", 0.0, 3.0),
            ("FLAPS_4", 0.0, 5.0),
            ("AXIS_FLAPS_SET", -20000.0, 0.0),
            ("AXIS_FLAPS_SET", 0.0, 3.0),
            ("AXIS_FLAPS_SET", 20000.0, 5.0),
        ] {
            flaps.event(event, input).unwrap();
            assert_eq!(flaps.read("FLAPS HANDLE INDEX").unwrap().0, expected);
        }
        assert!(flaps.event("AXIS_FLAPS_SET", f64::NAN).is_err());
        assert_eq!(flaps.read("FLAPS HANDLE INDEX").unwrap().0, 5.0);
        let mut short = Flaps::new(&[definition()]).unwrap();
        short.event("FLAPS_3", 0.0).unwrap();
        assert_eq!(short.read("FLAPS HANDLE INDEX").unwrap().0, 2.0);
    }

    #[test]
    fn actuators_follow_their_configured_power_sources() {
        let mut electric = definition();
        electric.system_type = Some(0);
        let mut hydraulic = definition();
        hydraulic.index = 1;
        hydraulic.kind = 2;
        hydraulic.system_type = Some(1);
        let mut flaps = Flaps::new(&[electric, hydraulic]).unwrap();
        flaps.event("FLAPS_DOWN", 0.0).unwrap();
        flaps
            .advance_with_power(
                2.0,
                0.0,
                Power {
                    electric: 1.0,
                    hydraulic: 0.0,
                },
            )
            .unwrap();
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS0 LEFT ANGLE").unwrap().0,
            1.0
        );
        assert_eq!(
            flaps.read("LEADING EDGE FLAPS0 LEFT ANGLE").unwrap().0,
            -5.0
        );
        flaps
            .advance_with_power(
                2.0,
                0.0,
                Power {
                    electric: 0.0,
                    hydraulic: 0.5,
                },
            )
            .unwrap();
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS0 LEFT ANGLE").unwrap().0,
            1.0
        );
        assert_eq!(
            flaps.read("LEADING EDGE FLAPS0 LEFT ANGLE").unwrap().0,
            -2.0
        );
        assert!(
            flaps
                .advance_with_power(
                    1.0,
                    0.0,
                    Power {
                        electric: 1.1,
                        hydraulic: 1.0
                    }
                )
                .is_err()
        );
    }

    #[test]
    fn follows_detents_at_configured_speed_and_reverses_in_transit() {
        let mut flaps = Flaps::new(&[definition()]).unwrap();
        flaps.event("FLAPS_DOWN", 0.0).unwrap();
        assert_eq!(flaps.read("FLAPS HANDLE INDEX"), Some((2.0, "number")));
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS0 LEFT ANGLE"),
            Some((-5.0, "degrees"))
        );
        flaps.advance(5.0, 0.0).unwrap();
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS0 RIGHT ANGLE"),
            Some((10.0, "degrees"))
        );
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS LEFT PERCENT"),
            Some((0.5, "percent over 100"))
        );
        flaps.event("FLAPS_UP", 0.0).unwrap();
        flaps.advance(2.0, 0.0).unwrap();
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS0 LEFT ANGLE"),
            Some((4.0, "degrees"))
        );
        assert!(flaps.advance(-1.0, 0.0).is_err());
        assert!(flaps.advance(f64::NAN, 0.0).is_err());
        assert!(flaps.event("FLAPS_DOWN", f64::INFINITY).is_err());
        flaps.advance(100.0, 0.0).unwrap();
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS0 LEFT ANGLE"),
            Some((-5.0, "degrees"))
        );
    }

    #[test]
    fn interpolation_samples_presentation_without_changing_actuator_state() {
        let mut flaps = Flaps::new(&[definition()]).unwrap();
        flaps.event("FLAPS_DOWN", 0.0).unwrap();
        flaps.advance(1.0, 0.0).unwrap();
        flaps.interpolate(0.5).unwrap();
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS0 LEFT ANGLE").unwrap().0,
            -3.5
        );
        assert_eq!(flaps.read("FLAPS HANDLE INDEX").unwrap().0, 2.0);
        flaps.advance(1.0, 0.0).unwrap();
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS0 LEFT ANGLE").unwrap().0,
            1.0
        );
        flaps.interpolate(0.0).unwrap();
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS0 LEFT ANGLE").unwrap().0,
            -2.0
        );
        flaps.interpolate(0.5).unwrap();
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS0 LEFT ANGLE").unwrap().0,
            -0.5
        );
        for alpha in [f64::NAN, -1.0, 1.1] {
            assert!(flaps.interpolate(alpha).is_err());
        }
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS0 LEFT ANGLE").unwrap().0,
            -0.5
        );
    }

    #[test]
    fn commands_are_bounded_and_sparse_detents_use_preceding_definition() {
        let mut primary = definition();
        primary.positions = positions(&[(0, 0.0), (1, 10.0), (2, 20.0), (3, 30.0)]);
        let sparse = FlapSet {
            index: 1,
            kind: 2,
            extending_time: Some(2.0),
            system_type: None,
            positions: positions(&[(0, 0.0), (1, 0.5), (3, 1.0)]),
        };
        let mut flaps = Flaps::new(&[primary, sparse]).unwrap();
        flaps.event("FLAPS_DECR", 0.0).unwrap();
        assert_eq!(flaps.read("FLAPS HANDLE INDEX").unwrap().0, 0.0);
        flaps.event("FLAPS_INCR", 0.0).unwrap();
        flaps.advance(2.0, 0.0).unwrap();
        assert_eq!(flaps.read("LEADING EDGE FLAPS0 LEFT ANGLE").unwrap().0, 0.5);
        flaps.event("FLAPS_SET", 10922.0).unwrap();
        assert_eq!(flaps.read("FLAPS HANDLE INDEX").unwrap().0, 2.0);
        flaps.advance(1.0, 0.0).unwrap();
        assert_eq!(flaps.read("LEADING EDGE FLAPS0 LEFT ANGLE").unwrap().0, 0.5);
        flaps.event("FLAPS_SET", 1e9).unwrap();
        flaps.advance(20.0, 0.0).unwrap();
        assert_eq!(flaps.read("LEADING EDGE FLAPS0 LEFT ANGLE").unwrap().0, 1.0);
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS1 LEFT ANGLE").unwrap().0,
            0.0
        );
        assert!(flaps.read("TRAILING EDGE FLAPS-1 LEFT ANGLE").is_none());
        assert!(flaps.read("TRAILING EDGE FLAPS0 LEFT UNKNOWN").is_none());
        flaps.event("FLAPS_UP", 0.0).unwrap();
        flaps.advance(20.0, 0.0).unwrap();
        flaps.event("FLAPS_SET", 10922.0).unwrap();
        flaps.advance(1.0, 0.0).unwrap();
        assert_eq!(flaps.read("LEADING EDGE FLAPS0 LEFT ANGLE").unwrap().0, 0.5);
    }

    #[test]
    fn detent_speed_limit_inhibits_motion_until_airspeed_decreases() {
        let mut set = definition();
        set.positions[2].speed_limit = Some(120.0);
        let mut flaps = Flaps::new(&[set]).unwrap();
        flaps.event("FLAPS_DOWN", 0.0).unwrap();
        flaps.advance(10.0, 121.0).unwrap();
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS0 LEFT ANGLE").unwrap().0,
            -5.0
        );
        flaps.advance(5.0, 120.0).unwrap();
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS0 LEFT ANGLE").unwrap().0,
            10.0
        );
        flaps.advance(5.0, 121.0).unwrap();
        assert_eq!(
            flaps.read("TRAILING EDGE FLAPS0 LEFT ANGLE").unwrap().0,
            10.0
        );
        assert!(flaps.advance(1.0, f64::NAN).is_err());
    }

    #[test]
    fn empty_and_incomplete_aircraft_data_are_explicit() {
        let mut empty = Flaps::new(&[]).unwrap();
        assert_eq!(empty.read("FLAPS AVAILABLE").unwrap().0, 0.0);
        assert!(empty.event("FLAPS_DOWN", 0.0).is_err());
        let mut set = definition();
        set.extending_time = None;
        assert!(Flaps::new(&[set]).is_err());
        let mut set = definition();
        set.positions.remove(0);
        assert!(Flaps::new(&[set]).is_err());
    }
}

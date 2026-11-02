use anyhow::{Context, Result, ensure};
use esp_assets::flight::FlightData;
use esp_formats::cfg::{self, contact::ContactPoints};
use glam::{DMat3, DVec3};

use crate::{
    flight::{State, Wrench},
    loading::WeightBalance,
};

pub struct LandingGear {
    wheels: Vec<Wheel>,
    brake_scale: f64,
}

struct Wheel {
    index: u32,
    position: DVec3,
    spring: f64,
    damper: f64,
    maximum_compression: f64,
    steering: f64,
    brake_map: u32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Controls {
    pub steering: f64,
    pub brakes: [f64; 2],
    pub parking_brake: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Surface {
    /// Elevation above the local NED origin, in meters.
    pub elevation: f64,
    pub rolling_resistance: f64,
    pub tyre_friction: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Output {
    pub wrench: Wrench,
    pub on_ground: bool,
    /// Largest fraction of maximum suspension travel used by any wheel.
    pub maximum_compression_ratio: f64,
}

impl LandingGear {
    pub fn from_data(data: &FlightData) -> Result<Self> {
        let config = &data.configuration;
        let contacts = ContactPoints::from_config(config)?;
        for rejected in &contacts.rejected {
            let class = rejected
                .source
                .split(',')
                .next()
                .and_then(|v| v.trim().parse::<u32>().ok());
            ensure!(
                matches!(class, Some(0 | 2)),
                "invalid landing gear: {}",
                rejected.error
            );
            eprintln!(
                "ground contact {} omitted: {}",
                rejected.index, rejected.error
            );
        }
        ensure!(
            contacts.points.iter().all(|p| matches!(p.class, 0..=2)),
            "only wheel and scrape ground contacts are supported"
        );
        let weights = WeightBalance::from_config(config)?;
        let empty = weights.load(0.0, false)?;
        let datum = cfg::aircraft::ReferencePoints::from_config(config)?
            .datum_feet
            .unwrap_or([0.0; 3]);
        let position =
            |p: [f64; 3]| DVec3::new(p[0] + datum[0], p[1] + datum[1], -p[2] - datum[2]) * 0.3048;
        let definitions = contacts
            .points
            .iter()
            .filter(|p| p.class == 1)
            .collect::<Vec<_>>();
        ensure!(
            definitions.len() >= 3,
            "at least three wheel contact points are required"
        );
        // The empty-aircraft support loads determine the spring rates from the
        // configured static compressions. For additional wheels, solve the
        // minimum-norm load distribution. Unloaded supports need additional
        // stiffness data rather than an invented spring rate.
        let arms = definitions
            .iter()
            .map(|p| {
                let r = position(p.position_feet) - empty.cg;
                DVec3::new(1.0, r.x, r.y)
            })
            .collect::<Vec<_>>();
        let matrix = arms.iter().fold(DMat3::ZERO, |m, a| {
            m + DMat3::from_cols(*a * a.x, *a * a.y, *a * a.z)
        });
        ensure!(
            matrix.determinant().abs() > 1e-12,
            "wheel supports do not span the aircraft center of gravity"
        );
        let multipliers = matrix.inverse() * DVec3::new(empty.mass.kilograms() * 9.80665, 0.0, 0.0);
        let support = arms.iter().map(|a| a.dot(multipliers));
        let mut wheels = Vec::new();
        for (point, load) in definitions.into_iter().zip(support) {
            ensure!(
                load > 0.0,
                "cannot derive suspension stiffness for unloaded contact {}",
                point.index
            );
            ensure!(
                point.static_compression_feet > 0.0 && point.maximum_compression_ratio >= 1.0,
                "invalid suspension for contact {}",
                point.index
            );
            let spring = load / (point.static_compression_feet * 0.3048);
            let damper = 2.0 * point.damping_ratio * (spring * load / 9.80665).sqrt();
            wheels.push(Wheel {
                index: point.index,
                position: position(point.position_feet),
                spring,
                damper,
                maximum_compression: point.static_compression_feet
                    * point.maximum_compression_ratio
                    * 0.3048,
                steering: point.steering_degrees.to_radians(),
                brake_map: point.brake_map,
            });
        }
        let brake_scale = config.number("brakes", "toe_brakes_scale")?.unwrap_or(1.0);
        ensure!(brake_scale >= 0.0, "invalid toe brake scalar");
        Ok(Self {
            wheels,
            brake_scale,
        })
    }

    pub fn forces(
        &self,
        state: &State,
        cg: DVec3,
        surface: Surface,
        controls: Controls,
    ) -> Result<Output> {
        ensure!(
            surface.elevation.is_finite()
                && surface.rolling_resistance.is_finite()
                && surface.rolling_resistance >= 0.0
                && surface.tyre_friction.is_finite()
                && surface.tyre_friction >= 0.0,
            "invalid ground surface"
        );
        ensure!(
            cg.is_finite()
                && state.position.is_finite()
                && state.velocity.is_finite()
                && state.orientation.is_finite()
                && state.angular_velocity.is_finite(),
            "invalid ground contact state"
        );
        ensure!(
            controls.steering.is_finite()
                && controls.steering.abs() <= 1.0
                && controls
                    .brakes
                    .iter()
                    .all(|v| v.is_finite() && (0.0..=1.0).contains(v)),
            "invalid ground controls"
        );
        let mut output = Output::default();
        for wheel in &self.wheels {
            let arm = wheel.position - cg;
            let world_arm = state.orientation * arm;
            let compression = state.position.z + world_arm.z + surface.elevation;
            if compression < 0.0 {
                continue;
            }
            let velocity = state.velocity + state.orientation * state.angular_velocity.cross(arm);
            let excess = (compression - wheel.maximum_compression).max(0.0);
            // Progressive bump stop beyond the authored suspension travel.
            let normal =
                (wheel.spring * (compression + 19.0 * excess) + wheel.damper * velocity.z).max(0.0);
            if normal == 0.0 {
                continue;
            }
            let caster = (wheel.steering.abs() - std::f64::consts::PI).abs() < 1e-6;
            let direction = if caster && velocity.truncate().length_squared() > 1e-12 {
                DVec3::new(velocity.x, velocity.y, 0.0).normalize()
            } else {
                let angle = controls.steering * wheel.steering;
                let forward = state.orientation * DVec3::new(angle.cos(), angle.sin(), 0.0);
                DVec3::new(forward.x, forward.y, 0.0)
                    .try_normalize()
                    .context("wheel has no horizontal rolling direction")?
            };
            let lateral = DVec3::Z.cross(direction);
            let brake = if wheel.brake_map == 0 {
                0.0
            } else if controls.parking_brake {
                1.0
            } else {
                controls.brakes[wheel.brake_map as usize - 1]
            };
            let rolling =
                surface.rolling_resistance + surface.tyre_friction * brake * self.brake_scale;
            // Regularized Coulomb friction approaches its limit without a
            // sign discontinuity at zero wheel speed. Surface coefficients are
            // supplied by the scene rather than guessed from aircraft type.
            let along = -normal * rolling * (velocity.dot(direction) / 0.25).tanh();
            let across = if caster {
                0.0
            } else {
                -normal * surface.tyre_friction * (velocity.dot(lateral) / 0.25).tanh()
            };
            let mut friction = direction * along + lateral * across;
            let limit = normal * surface.tyre_friction;
            if friction.length() > limit {
                friction = friction.normalize() * limit;
            }
            let force = state.orientation.conjugate() * (friction - DVec3::Z * normal);
            output.wrench.force += force;
            output.wrench.moment += arm.cross(force);
            output.on_ground = true;
            output.maximum_compression_ratio = output
                .maximum_compression_ratio
                .max(compression / wheel.maximum_compression);
        }
        ensure!(
            output.wrench.force.is_finite() && output.wrench.moment.is_finite(),
            "non-finite landing gear forces"
        );
        Ok(output)
    }

    pub fn clearance(&self, state: &State, cg: DVec3, elevation: f64) -> f64 {
        self.wheels
            .iter()
            .map(|w| -(state.position + state.orientation * (w.position - cg)).z - elevation)
            .fold(f64::INFINITY, f64::min)
    }

    pub fn wheel_indices(&self) -> impl Iterator<Item = u32> + '_ {
        self.wheels.iter().map(|w| w.index)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flight::MassProperties;

    fn tripod() -> LandingGear {
        let spring = 100.0 * 9.80665 / 0.1;
        LandingGear {
            wheels: [
                DVec3::new(2.0, 0.0, 1.0),
                DVec3::new(-1.0, -1.0, 1.0),
                DVec3::new(-1.0, 1.0, 1.0),
            ]
            .into_iter()
            .enumerate()
            .map(|(i, position)| Wheel {
                index: i as u32,
                position,
                spring,
                damper: 2.0 * 0.8 * (spring * 100.0).sqrt(),
                maximum_compression: 0.25,
                steering: if i == 0 { 30.0_f64.to_radians() } else { 0.0 },
                brake_map: i as u32,
            })
            .collect(),
            brake_scale: 1.0,
        }
    }

    const SURFACE: Surface = Surface {
        elevation: 0.0,
        rolling_resistance: 0.02,
        tyre_friction: 0.8,
    };

    #[test]
    fn suspension_balances_weight_and_never_pulls_the_aircraft_down() {
        let gear = tripod();
        let mut state = State {
            position: DVec3::new(0.0, 0.0, -0.9),
            ..State::default()
        };
        let output = gear
            .forces(&state, DVec3::ZERO, SURFACE, Controls::default())
            .unwrap();
        assert!((output.wrench.force.z + 300.0 * 9.80665).abs() < 1e-10);
        assert!(output.wrench.moment.length() < 1e-10);
        assert!(output.on_ground);
        state.velocity.z = -10.0;
        assert!(
            !gear
                .forces(&state, DVec3::ZERO, SURFACE, Controls::default())
                .unwrap()
                .on_ground
        );
        state.position.z = -2.0;
        assert_eq!(gear.clearance(&state, DVec3::ZERO, 0.0), 1.0);
        assert_eq!(
            gear.forces(&state, DVec3::ZERO, SURFACE, Controls::default())
                .unwrap()
                .wrench
                .force,
            DVec3::ZERO
        );
    }

    #[test]
    fn drop_settles_on_the_configured_springs_without_energy_growth() {
        let gear = tripod();
        let mass =
            MassProperties::new(300.0, DMat3::from_diagonal(DVec3::new(400.0, 600.0, 600.0)))
                .unwrap();
        let mut state = State {
            position: DVec3::new(0.0, 0.0, -1.5),
            ..State::default()
        };
        for _ in 0..2400 {
            state
                .advance(&mass, 1.0 / 240.0, DVec3::Z * 9.80665, |s| {
                    Ok(gear
                        .forces(s, DVec3::ZERO, SURFACE, Controls::default())?
                        .wrench)
                })
                .unwrap();
        }
        assert!((state.position.z + 0.9).abs() < 1e-6, "{state:?}");
        assert!(state.velocity.length() < 1e-6, "{state:?}");
        assert!(state.angular_velocity.length() < 1e-6, "{state:?}");
    }

    #[test]
    fn braking_opposes_motion_and_respects_the_friction_circle() {
        let gear = tripod();
        let mut state = State {
            position: DVec3::new(0.0, 0.0, -0.9),
            velocity: DVec3::new(15.0, 0.0, 0.0),
            ..State::default()
        };
        let coasting = gear
            .forces(&state, DVec3::ZERO, SURFACE, Controls::default())
            .unwrap();
        let braking = gear
            .forces(
                &state,
                DVec3::ZERO,
                SURFACE,
                Controls {
                    parking_brake: true,
                    ..Controls::default()
                },
            )
            .unwrap();
        assert!(braking.wrench.force.x < coasting.wrench.force.x * 10.0);
        state.velocity.y = 15.0;
        let output = gear
            .forces(
                &state,
                DVec3::ZERO,
                SURFACE,
                Controls {
                    parking_brake: true,
                    ..Controls::default()
                },
            )
            .unwrap();
        assert!(output.wrench.force.dot(state.velocity) < 0.0);
        assert!(
            output.wrench.force.truncate().length()
                <= 300.0 * 9.80665 * SURFACE.tyre_friction + 1e-9
        );
    }
}

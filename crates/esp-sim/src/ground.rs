use anyhow::{Result, ensure};
use esp_assets::flight::FlightData;
use esp_formats::cfg::{self, contact::ContactPoints};
use glam::{DMat3, DVec3};

use crate::{
    flight::{MassProperties, State, Wrench},
    loading::WeightBalance,
};

mod friction;

pub struct LandingGear {
    wheels: Vec<Wheel>,
    brake_scale: f64,
    contacts: Vec<Contact>,
}

struct Contact {
    index: u32,
    position: DVec3,
    damage_speed: f64,
    scrape: bool,
}

#[derive(Debug)]
pub struct Impact {
    pub index: u32,
    pub speed: f64,
    pub damage_speed: f64,
    pub scrape: bool,
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
        let collision_contacts = contacts
            .points
            .iter()
            .filter(|p| p.class != 0)
            .map(|p| Contact {
                index: p.index,
                position: position(p.position_feet),
                damage_speed: p.impact_feet_per_minute * 0.3048 / 60.0,
                scrape: p.class == 2,
            })
            .collect();
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
            contacts: collision_contacts,
        })
    }

    pub fn impact(
        &self,
        before: &State,
        after: &State,
        cg: DVec3,
        surface: Surface,
    ) -> Option<Impact> {
        for contact in &self.contacts {
            let arm = contact.position - cg;
            let old = (before.position + before.orientation * arm).z + surface.elevation;
            let new = (after.position + after.orientation * arm).z + surface.elevation;
            let speed = (before.velocity + before.orientation * before.angular_velocity.cross(arm))
                .z
                .max(0.0);
            if new >= 0.0
                && (contact.scrape
                    || (old < 0.0 && contact.damage_speed > 0.0 && speed > contact.damage_speed))
            {
                return Some(Impact {
                    index: contact.index,
                    speed,
                    damage_speed: contact.damage_speed,
                    scrape: contact.scrape,
                });
            }
        }
        None
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
            let force = state.orientation.conjugate() * (-DVec3::Z * normal);
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

    /// Apply tyre impulses and their midpoint pose correction after integrating
    /// the non-friction forces over `dt`.
    pub fn solve_friction(
        &self,
        state: &mut State,
        mass: &MassProperties,
        cg: DVec3,
        surface: Surface,
        controls: Controls,
        dt: f64,
    ) -> Result<()> {
        friction::solve(self, state, mass, cg, surface, controls, dt)
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
            contacts: vec![],
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
        let mass =
            MassProperties::new(300.0, DMat3::from_diagonal(DVec3::new(400.0, 600.0, 600.0)))
                .unwrap();
        let mut state = State {
            position: DVec3::new(0.0, 0.0, -0.9),
            velocity: DVec3::new(15.0, 0.0, 0.0),
            ..State::default()
        };
        let mut coasting = state;
        let mut braking = state;
        gear.solve_friction(
            &mut coasting,
            &mass,
            DVec3::ZERO,
            SURFACE,
            Controls::default(),
            1.0 / 240.0,
        )
        .unwrap();
        gear.solve_friction(
            &mut braking,
            &mass,
            DVec3::ZERO,
            SURFACE,
            Controls {
                parking_brake: true,
                ..Controls::default()
            },
            1.0 / 240.0,
        )
        .unwrap();
        assert!(braking.velocity.x < coasting.velocity.x);
        state.velocity.y = 15.0;
        let initial = state;
        gear.solve_friction(
            &mut state,
            &mass,
            DVec3::ZERO,
            SURFACE,
            Controls {
                parking_brake: true,
                ..Controls::default()
            },
            1.0 / 240.0,
        )
        .unwrap();
        let impulse = (state.velocity - initial.velocity) * mass.kilograms();
        assert!(impulse.dot(initial.velocity) < 0.0);
        assert!(
            impulse.truncate().length() <= 300.0 * 9.80665 * SURFACE.tyre_friction / 240.0 + 1e-9
        );
    }

    #[test]
    fn swept_contacts_detect_hard_touchdowns_and_scrapes() {
        let mut gear = tripod();
        gear.contacts.push(Contact {
            index: 0,
            position: DVec3::Z,
            damage_speed: 5.0,
            scrape: false,
        });
        let before = State {
            position: -DVec3::Z * 1.01,
            velocity: DVec3::Z * 6.0,
            ..State::default()
        };
        let after = State {
            position: -DVec3::Z * 0.99,
            velocity: DVec3::ZERO,
            ..before
        };
        let impact = gear.impact(&before, &after, DVec3::ZERO, SURFACE).unwrap();
        assert_eq!(impact.index, 0);
        assert_eq!(
            impact.speed, 6.0,
            "collision must retain approach speed before suspension damping"
        );
        assert!(
            gear.impact(
                &State {
                    velocity: DVec3::Z,
                    ..before
                },
                &after,
                DVec3::ZERO,
                SURFACE
            )
            .is_none()
        );
        gear.contacts[0].scrape = true;
        assert!(
            gear.impact(&after, &after, DVec3::ZERO, SURFACE)
                .unwrap()
                .scrape
        );
    }

    #[test]
    fn tyre_impulses_dissipate_translation_and_rotation_energy() {
        let gear = tripod();
        let mass =
            MassProperties::new(300.0, DMat3::from_diagonal(DVec3::new(400.0, 600.0, 600.0)))
                .unwrap();
        let energy = |s: &State| {
            0.5 * mass.kilograms() * s.velocity.length_squared()
                + 0.5 * s.angular_velocity.dot(mass.inertia * s.angular_velocity)
        };
        for index in 0..24 {
            let i = f64::from(index);
            let mut state = State {
                position: -DVec3::Z * 0.9,
                velocity: DVec3::new(5.0 * i - 40.0, 11.0 - i, 0.0),
                angular_velocity: DVec3::new(0.17 * i - 2.0, 1.0 - 0.11 * i, 0.07 * i - 0.5),
                ..State::default()
            };
            let before = energy(&state);
            gear.solve_friction(
                &mut state,
                &mass,
                DVec3::ZERO,
                SURFACE,
                Controls {
                    steering: 0.4,
                    brakes: [0.7, 0.1],
                    parking_brake: false,
                },
                1.0 / 240.0,
            )
            .unwrap();
            assert!(
                energy(&state) <= before + 1e-8,
                "tyre impulses added energy for case {index}"
            );
        }
    }

    #[test]
    fn static_grip_holds_a_steady_force_and_yields_when_overloaded() {
        let gear = tripod();
        let mass =
            MassProperties::new(300.0, DMat3::from_diagonal(DVec3::new(400.0, 600.0, 600.0)))
                .unwrap();
        let controls = Controls {
            parking_brake: true,
            ..Controls::default()
        };
        let mut state = State {
            position: -DVec3::Z * 0.9,
            ..State::default()
        };
        let step = |state: &mut State, push: f64| {
            state
                .advance(&mass, 1.0 / 240.0, DVec3::Z * 9.80665, |s| {
                    let mut loads = gear.forces(s, DVec3::ZERO, SURFACE, controls)?.wrench;
                    loads.force += DVec3::X * push;
                    Ok(loads)
                })
                .unwrap();
            gear.solve_friction(state, &mass, DVec3::ZERO, SURFACE, controls, 1.0 / 240.0)
                .unwrap();
        };
        for _ in 0..2400 {
            step(&mut state, 300.0);
        }
        let before = state.position;
        for _ in 0..2400 {
            step(&mut state, 300.0);
        }
        assert!(
            (state.position - before).truncate().length() < 0.001,
            "static tyre contact crept: {state:?}"
        );
        step(&mut state, 1_000_000.0);
        assert!(
            state.velocity.x > 1.0,
            "overloaded brakes incorrectly pinned the aircraft"
        );
    }
}

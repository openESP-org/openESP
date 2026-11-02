use anyhow::{Result, ensure};
use glam::{DMat2, DQuat, DVec2, DVec3};

use super::{Controls, LandingGear, Surface};
use crate::flight::{MassProperties, State};

struct Constraint {
    arm: DVec3,
    along: DVec3,
    across: DVec3,
    inverse: DMat2,
    limit: DVec2,
    impulse: DVec2,
}

pub(super) fn solve(
    gear: &LandingGear,
    state: &mut State,
    mass: &MassProperties,
    cg: DVec3,
    surface: Surface,
    controls: Controls,
    dt: f64,
) -> Result<()> {
    ensure!(dt.is_finite() && dt >= 0.0, "invalid tyre contact timestep");
    // The force evaluation validates the state, surface and control inputs.
    gear.forces(state, cg, surface, controls)?;
    if dt == 0.0 {
        return Ok(());
    }
    let inverse_orientation = state.orientation.conjugate();
    let mut constraints = Vec::with_capacity(gear.wheels.len());
    for wheel in &gear.wheels {
        if wheel.extension < 1.0 {
            continue;
        }
        let extended_arm = wheel.position - cg;
        let world_arm = state.orientation * extended_arm;
        let compression = state.position.z + world_arm.z + surface.elevation;
        if compression < 0.0 {
            continue;
        }
        let vertical_speed =
            (state.velocity + state.orientation * state.angular_velocity.cross(extended_arm)).z;
        let excess = (compression - wheel.maximum_compression).max(0.0);
        let normal =
            (wheel.spring * (compression + 19.0 * excess) + wheel.damper * vertical_speed).max(0.0);
        if normal == 0.0 {
            continue;
        }
        // Tyre forces act at the surface, after suspension compression.
        let arm = inverse_orientation * (world_arm - DVec3::Z * compression);
        let velocity = state.velocity + state.orientation * state.angular_velocity.cross(arm);
        let caster = (wheel.steering - std::f64::consts::PI).abs() < 1e-6;
        let direction = if caster && velocity.truncate().length_squared() > 1e-12 {
            DVec3::new(velocity.x, velocity.y, 0.0).normalize()
        } else {
            let angle = controls.steering * wheel.steering;
            let direction = state.orientation * DVec3::new(angle.cos(), angle.sin(), 0.0);
            let Some(direction) = DVec3::new(direction.x, direction.y, 0.0).try_normalize() else {
                continue;
            };
            direction
        };
        let along = inverse_orientation * direction;
        let across = inverse_orientation * DVec3::Z.cross(direction);
        let brake = if wheel.brake_map == 0 {
            0.0
        } else if controls.parking_brake {
            1.0
        } else {
            controls.brakes[wheel.brake_map as usize - 1]
        };
        let rolling = (surface.rolling_resistance
            + surface.tyre_friction * brake * gear.brake_scale)
            .min(surface.tyre_friction);
        let limit =
            DVec2::new(rolling, if caster { 0.0 } else { surface.tyre_friction }) * normal * dt;
        if limit == DVec2::ZERO {
            continue;
        }
        let a = arm.cross(along);
        let b = arm.cross(across);
        let kaa = 1.0 / mass.kilograms() + a.dot(mass.inverse_inertia * a);
        let kab = a.dot(mass.inverse_inertia * b);
        let kbb = 1.0 / mass.kilograms() + b.dot(mass.inverse_inertia * b);
        let inverse = if limit.x > 0.0 && limit.y > 0.0 {
            DMat2::from_cols(DVec2::new(kaa, kab), DVec2::new(kab, kbb)).inverse()
        } else {
            DMat2::from_diagonal(DVec2::new(
                if limit.x > 0.0 { 1.0 / kaa } else { 0.0 },
                if limit.y > 0.0 { 1.0 / kbb } else { 0.0 },
            ))
        };
        ensure!(inverse.is_finite(), "invalid tyre effective mass");
        constraints.push(Constraint {
            arm,
            along,
            across,
            inverse,
            limit,
            impulse: DVec2::ZERO,
        });
    }
    // Project accumulated impulses onto the wheel's anisotropic friction
    // ellipse. A stationary contact can balance a finite external force;
    // once its friction budget is exhausted, the tyre slips instead.
    let velocity_before = state.velocity;
    let angular_before = state.angular_velocity;
    for _ in 0..12 {
        for contact in &mut constraints {
            let velocity =
                inverse_orientation * state.velocity + state.angular_velocity.cross(contact.arm);
            let speed = DVec2::new(velocity.dot(contact.along), velocity.dot(contact.across));
            let candidate = contact.impulse - contact.inverse * speed;
            let mut normalized = DVec2::new(
                if contact.limit.x > 0.0 {
                    candidate.x / contact.limit.x
                } else {
                    0.0
                },
                if contact.limit.y > 0.0 {
                    candidate.y / contact.limit.y
                } else {
                    0.0
                },
            );
            if normalized.length_squared() > 1.0 {
                normalized = normalized.normalize();
            }
            let impulse = normalized * contact.limit;
            let delta = impulse - contact.impulse;
            contact.impulse = impulse;
            let body_impulse = contact.along * delta.x + contact.across * delta.y;
            state.velocity += state.orientation * body_impulse / mass.kilograms();
            state.angular_velocity += mass.inverse_inertia * contact.arm.cross(body_impulse);
        }
    }
    ensure!(
        state.velocity.is_finite() && state.angular_velocity.is_finite(),
        "non-finite tyre impulse response"
    );
    // The force integrator already advanced the pose using the unconstrained
    // velocities. Apply the impulse's midpoint contribution to that pose as
    // well, otherwise a parked aircraft drifts by O(dt²) on every step.
    state.position += (state.velocity - velocity_before) * (dt * 0.5);
    state.orientation = (state.orientation
        * DQuat::from_scaled_axis((state.angular_velocity - angular_before) * (dt * 0.5)))
    .normalize();
    Ok(())
}

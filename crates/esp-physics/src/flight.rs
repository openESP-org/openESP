use anyhow::{Result, ensure};
use glam::{DMat3, DQuat, DVec3};

#[derive(Debug, Clone, Copy)]
pub struct MassProperties {
    kilograms: f64,
    pub(crate) inertia: DMat3,
    pub(crate) inverse_inertia: DMat3,
}

impl MassProperties {
    pub fn new(kilograms: f64, inertia: DMat3) -> Result<Self> {
        ensure!(
            kilograms.is_finite() && kilograms > 0.0,
            "invalid aircraft mass"
        );
        ensure!(
            inertia.is_finite() && inertia.abs_diff_eq(inertia.transpose(), 1e-10),
            "inertia tensor must be finite and symmetric"
        );
        ensure!(
            inertia.x_axis.x > 0.0
                && inertia.x_axis.x * inertia.y_axis.y - inertia.x_axis.y.powi(2) > 0.0
                && inertia.determinant() > 0.0,
            "inertia tensor must be positive definite"
        );
        let inverse_inertia = inertia.inverse();
        ensure!(
            inverse_inertia.is_finite(),
            "inertia tensor is not invertible"
        );
        Ok(Self {
            kilograms,
            inertia,
            inverse_inertia,
        })
    }

    pub fn kilograms(&self) -> f64 {
        self.kilograms
    }
}

// Body axes are forward, right, down. Position and velocity use a local NED frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    pub position: DVec3,
    pub velocity: DVec3,
    pub orientation: DQuat,
    pub angular_velocity: DVec3,
}

impl Default for State {
    fn default() -> Self {
        Self {
            position: DVec3::ZERO,
            velocity: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            angular_velocity: DVec3::ZERO,
        }
    }
}

impl State {
    pub fn rebase(&mut self, offset: DVec3) -> Result<()> {
        ensure!(offset.is_finite(), "invalid aircraft reference offset");
        self.position += self.orientation * offset;
        self.velocity += self.orientation * self.angular_velocity.cross(offset);
        Ok(())
    }

    pub fn air_velocity(&self, wind: DVec3) -> DVec3 {
        self.orientation.conjugate() * (self.velocity - wind)
    }

    pub fn interpolate(&self, other: &Self, alpha: f64) -> Result<Self> {
        ensure!(
            alpha.is_finite() && (0.0..=1.0).contains(&alpha),
            "invalid flight interpolation"
        );
        Ok(Self {
            position: self.position.lerp(other.position, alpha),
            velocity: self.velocity.lerp(other.velocity, alpha),
            orientation: self.orientation.slerp(other.orientation, alpha),
            angular_velocity: self.angular_velocity.lerp(other.angular_velocity, alpha),
        })
    }

    fn validate(&self) -> Result<()> {
        ensure!(
            self.position.is_finite()
                && self.velocity.is_finite()
                && self.angular_velocity.is_finite()
                && self.orientation.is_finite()
                && (self.orientation.length_squared() - 1.0).abs() < 1e-8,
            "invalid flight state"
        );
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Wrench {
    pub force: DVec3,
    pub moment: DVec3,
}

struct Derivative {
    position: DVec3,
    velocity: DVec3,
    orientation: DQuat,
    angular_velocity: DVec3,
}

impl State {
    pub fn advance(
        &mut self,
        mass: &MassProperties,
        dt: f64,
        gravity: DVec3,
        loads: impl Fn(&Self) -> Result<Wrench>,
    ) -> Result<()> {
        ensure!(
            dt.is_finite() && dt >= 0.0 && gravity.is_finite(),
            "invalid flight timestep or gravity"
        );
        self.validate()?;
        if dt == 0.0 {
            return Ok(());
        }
        let derivative = |state: &Self| -> Result<Derivative> {
            let wrench = loads(state)?;
            ensure!(
                wrench.force.is_finite() && wrench.moment.is_finite(),
                "invalid aircraft forces"
            );
            let omega = state.angular_velocity;
            Ok(Derivative {
                position: state.velocity,
                velocity: state.orientation * wrench.force / mass.kilograms + gravity,
                orientation: (state.orientation * DQuat::from_xyzw(omega.x, omega.y, omega.z, 0.0))
                    * 0.5,
                angular_velocity: mass.inverse_inertia
                    * (wrench.moment - omega.cross(mass.inertia * omega)),
            })
        };
        let at = |derivative: &Derivative, interval: f64| Self {
            position: self.position + derivative.position * interval,
            velocity: self.velocity + derivative.velocity * interval,
            orientation: (self.orientation + derivative.orientation * interval).normalize(),
            angular_velocity: self.angular_velocity + derivative.angular_velocity * interval,
        };
        let a = derivative(self)?;
        let b = derivative(&at(&a, dt * 0.5))?;
        let c = derivative(&at(&b, dt * 0.5))?;
        let d = derivative(&at(&c, dt))?;
        let next = Self {
            position: self.position
                + (a.position + 2.0 * b.position + 2.0 * c.position + d.position) * (dt / 6.0),
            velocity: self.velocity
                + (a.velocity + 2.0 * b.velocity + 2.0 * c.velocity + d.velocity) * (dt / 6.0),
            orientation: (self.orientation
                + (a.orientation + b.orientation * 2.0 + c.orientation * 2.0 + d.orientation)
                    * (dt / 6.0))
                .normalize(),
            angular_velocity: self.angular_velocity
                + (a.angular_velocity
                    + 2.0 * b.angular_velocity
                    + 2.0 * c.angular_velocity
                    + d.angular_velocity)
                    * (dt / 6.0),
        };
        next.validate()?;
        *self = next;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::time::STEP;

    #[test]
    fn rebasing_preserves_airframe_position_and_material_point_velocity() {
        let mut state = State {
            position: DVec3::splat(1e6),
            velocity: DVec3::new(100.0, 2.0, -3.0),
            orientation: DQuat::from_rotation_y(0.3),
            angular_velocity: DVec3::new(0.1, 0.2, -0.3),
        };
        let before = state;
        let offset = DVec3::new(0.5, -0.2, 0.1);
        state.rebase(offset).unwrap();
        assert!((state.position - state.orientation * offset).abs_diff_eq(before.position, 1e-9));
        let point = DVec3::new(-4.0, 2.0, 1.0);
        let velocity =
            state.velocity + state.orientation * state.angular_velocity.cross(point - offset);
        let expected = before.velocity + before.orientation * before.angular_velocity.cross(point);
        assert!(velocity.abs_diff_eq(expected, 1e-12));
    }

    fn mass() -> MassProperties {
        MassProperties::new(
            1000.0,
            DMat3::from_diagonal(DVec3::new(800.0, 1200.0, 1600.0)),
        )
        .unwrap()
    }

    #[test]
    fn follows_ballistic_trajectory_in_world_coordinates() {
        let mut state = State {
            position: DVec3::new(100.0, 200.0, -1000.0),
            velocity: DVec3::new(50.0, -5.0, 0.0),
            orientation: DQuat::from_rotation_z(1.0),
            ..State::default()
        };
        for _ in 0..600 {
            state
                .advance(&mass(), STEP, DVec3::Z * 9.80665, |_| Ok(Wrench::default()))
                .unwrap();
        }
        assert!(
            state
                .position
                .abs_diff_eq(DVec3::new(600.0, 150.0, -509.6675), 1e-8)
        );
        assert!(
            state
                .velocity
                .abs_diff_eq(DVec3::new(50.0, -5.0, 98.0665), 1e-8)
        );
        assert!(
            state
                .orientation
                .abs_diff_eq(DQuat::from_rotation_z(1.0), 1e-12)
        );
    }

    #[test]
    fn rotates_body_forces_and_integrates_applied_torque() {
        let mut state = State {
            orientation: DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2),
            ..State::default()
        };
        for _ in 0..60 {
            state
                .advance(&mass(), STEP, DVec3::ZERO, |_| {
                    Ok(Wrench {
                        force: DVec3::X * 2000.0,
                        ..Wrench::default()
                    })
                })
                .unwrap();
        }
        assert!(state.velocity.abs_diff_eq(DVec3::Y * 2.0, 1e-10));
        assert!(state.position.abs_diff_eq(DVec3::Y, 1e-10));
        let mut state = State::default();
        for _ in 0..60 {
            state
                .advance(&mass(), STEP, DVec3::ZERO, |_| {
                    Ok(Wrench {
                        moment: DVec3::Y * 1200.0,
                        ..Wrench::default()
                    })
                })
                .unwrap();
        }
        assert!(state.angular_velocity.abs_diff_eq(DVec3::Y, 1e-10));
        assert!((state.orientation.angle_between(DQuat::from_rotation_y(0.5))).abs() < 1e-7);
    }

    #[test]
    fn conserves_torque_free_angular_momentum_and_energy() {
        let mass = mass();
        let mut state = State {
            angular_velocity: DVec3::new(0.3, -0.4, 0.7),
            ..State::default()
        };
        let momentum = mass.inertia * state.angular_velocity;
        let energy = 0.5 * state.angular_velocity.dot(momentum);
        for _ in 0..3600 {
            state
                .advance(&mass, STEP, DVec3::ZERO, |_| Ok(Wrench::default()))
                .unwrap();
        }
        let after = mass.inertia * state.angular_velocity;
        assert!((state.orientation * after).abs_diff_eq(momentum, 1e-5));
        assert!((0.5 * state.angular_velocity.dot(after) - energy).abs() < 1e-6);
        assert!((state.orientation.length_squared() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn interpolation_does_not_change_physical_state() {
        let before = State::default();
        let mut after = before;
        after
            .advance(&mass(), 1.0, DVec3::Z * 10.0, |_| Ok(Wrench::default()))
            .unwrap();
        let snapshot = after;
        let middle = before.interpolate(&after, 0.5).unwrap();
        assert_eq!(middle.position, DVec3::Z * 2.5);
        assert_eq!(after, snapshot);
    }

    #[test]
    fn rejects_invalid_state_and_mass_without_advancing() {
        for inertia in [
            DMat3::ZERO,
            DMat3::from_diagonal(DVec3::new(1.0, -1.0, -1.0)),
            DMat3::from_cols(DVec3::X, DVec3::X + DVec3::Y, DVec3::Z),
        ] {
            assert!(MassProperties::new(1000.0, inertia).is_err());
        }
        assert!(MassProperties::new(f64::NAN, DMat3::IDENTITY).is_err());
        let mut state = State::default();
        assert!(
            state
                .advance(&mass(), -1.0, DVec3::ZERO, |_| Ok(Wrench::default()))
                .is_err()
        );
        assert!(
            state
                .advance(&mass(), STEP, DVec3::ZERO, |_| Ok(Wrench {
                    force: DVec3::NAN,
                    ..Wrench::default()
                }))
                .is_err()
        );
        assert_eq!(state, State::default());
    }
}

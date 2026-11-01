use anyhow::{Context, Result, ensure};
use esp_assets::flight::FlightData;
use esp_formats::{
    air::{
        self,
        parameters::LegacyParameters,
        table::{Point, Table},
    },
    cfg::Config,
};
use glam::DVec3;

use crate::{flight::Wrench, loading::WeightBalance};

const FOOT: f64 = 0.3048;

pub struct Aerodynamics {
    area: f64,
    span: f64,
    chord: f64,
    induced_drag: f64,
    lift_scalar: f64,
    drag_scalar: f64,
    stability: DVec3,
    effectiveness: DVec3,
    elevator: [f64; 2],
    aileron: f64,
    rudder: f64,
    incidence: f64,
    pitch_reference: f64,
    lift: Curve,
    pitch: Curve,
    coefficients: LegacyParameters,
    flaps: Vec<[f64; 3]>,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Controls {
    pub elevator: f64,
    pub aileron: f64,
    pub rudder: f64,
}

struct Curve(Vec<Point>);

impl Curve {
    fn new(points: Vec<Point>) -> Result<Self> {
        ensure!(
            points.len() >= 2
                && points
                    .iter()
                    .all(|p| p.input.is_finite() && p.output.is_finite())
                && points.windows(2).all(|p| p[0].input < p[1].input),
            "invalid aerodynamic curve"
        );
        Ok(Self(points))
    }

    fn sample(&self, input: f64) -> f64 {
        let index = self.0.partition_point(|point| point.input < input);
        if index == 0 {
            return self.0[0].output;
        }
        if index == self.0.len() {
            return self.0[index - 1].output;
        }
        let [a, b] = [self.0[index - 1], self.0[index]];
        a.output + (b.output - a.output) * (input - a.input) / (b.input - a.input)
    }

    fn gradient(&self) -> f64 {
        (self.sample(10f64.to_radians()) - self.sample(0.0)) / 10f64.to_radians()
    }
}

fn required(config: &Config, section: &str, key: &str) -> Result<f64> {
    config
        .number(section, key)?
        .with_context(|| format!("missing [{section}] {key}"))
}

fn tuning(config: &Config, key: &str) -> Result<f64> {
    Ok(config.number("flight_tuning", key)?.unwrap_or(1.0))
}

impl Aerodynamics {
    pub fn from_data(data: &FlightData) -> Result<Self> {
        let config = &data.configuration;
        let records = air::parse(&data.air)?;
        ensure!(
            !records.iter().any(|r| matches!(r.id, 0x1539..=0x1546)),
            "extended AIR aerodynamics are not simulated yet"
        );
        let record = |id| -> Result<&air::Record<'_>> {
            let mut matches = records.iter().filter(|r| r.id == id);
            let result = matches
                .next()
                .with_context(|| format!("missing AIR record {id:#x}"))?;
            ensure!(matches.next().is_none(), "duplicate AIR record {id:#x}");
            Ok(result)
        };
        let curve = |id| -> Result<Curve> {
            let Some(Table::Curve(curve)) = record(id)?.table()? else {
                anyhow::bail!("AIR record {id:#x} is not a curve");
            };
            Curve::new(curve.points).with_context(|| format!("AIR record {id:#x}"))
        };
        let lift = curve(0x404)?;
        let pitch = curve(0x473)?;
        let coefficients = record(0x1101)?.legacy_aerodynamics()?.unwrap();
        let geometry = |key| required(config, "airplane_geometry", key);
        let area = geometry("wing_area")? * FOOT * FOOT;
        let span = geometry("wing_span")? * FOOT;
        let root = geometry("wing_root_chord")? * FOOT;
        let efficiency = geometry("oswald_efficiency_factor")?;
        ensure!(
            [area, span, root, efficiency].iter().all(|n| *n > 0.0),
            "invalid wing geometry"
        );
        let mean = area / span;
        let tip = 2.0 * mean - root;
        ensure!(tip >= 0.0, "wing geometry implies a negative tip chord");
        let taper = tip / root;
        let chord = 2.0 * root * (1.0 + taper + taper * taper) / (3.0 * (1.0 + taper));
        let weights = WeightBalance::from_config(config)?;
        let pitch_reference = if records.iter().any(|r| r.id == 0x1534) {
            let reference = record(0x1534)?;
            ensure!(
                reference.data.len() == 24,
                "invalid AIR aerodynamic center record"
            );
            f64::from_le_bytes(reference.data[16..24].try_into().unwrap()) * FOOT
        } else {
            ensure!(
                lift.gradient().abs() > 1e-10,
                "lift curve has no linear gradient"
            );
            weights.load(0.0, true)?.cg.x - pitch.gradient() / lift.gradient() * chord
        };
        ensure!(
            pitch_reference.is_finite(),
            "invalid aerodynamic pitch reference"
        );
        let flaps = esp_formats::cfg::aircraft::flaps(config)?
            .iter()
            .filter(|flap| flap.kind != 0)
            .map(|flap| {
                let section = format!("flaps.{}", flap.index);
                Ok([
                    config.number(&section, "lift_scalar")?.unwrap_or(1.0),
                    config.number(&section, "drag_scalar")?.unwrap_or(1.0),
                    config.number(&section, "pitch_scalar")?.unwrap_or(1.0),
                ])
            })
            .collect::<Result<Vec<_>>>()?;
        let result = Self {
            area,
            span,
            chord,
            induced_drag: tuning(config, "induced_drag_scalar")?
                / (std::f64::consts::PI * efficiency * (span * span / area)),
            lift_scalar: tuning(config, "cruise_lift_scalar")?,
            drag_scalar: tuning(config, "parasite_drag_scalar")?,
            stability: DVec3::new(
                tuning(config, "roll_stability")?,
                tuning(config, "pitch_stability")?,
                tuning(config, "yaw_stability")?,
            ),
            effectiveness: DVec3::new(
                tuning(config, "aileron_effectiveness")?,
                tuning(config, "elevator_effectiveness")?,
                tuning(config, "rudder_effectiveness")?,
            ),
            elevator: [
                geometry("elevator_up_limit")?.to_radians(),
                geometry("elevator_down_limit")?.to_radians(),
            ],
            aileron: (geometry("aileron_up_limit")? + geometry("aileron_down_limit")?).to_radians()
                * 0.5,
            rudder: geometry("rudder_limit")?.to_radians(),
            incidence: geometry("htail_incidence")?.to_radians(),
            pitch_reference,
            lift,
            pitch,
            coefficients,
            flaps,
        };
        ensure!(
            result.induced_drag >= 0.0 && result.drag_scalar >= 0.0,
            "invalid aerodynamic drag scalars"
        );
        ensure!(
            result
                .elevator
                .iter()
                .chain([&result.aileron, &result.rudder])
                .all(|angle| *angle >= 0.0),
            "invalid control surface limits"
        );
        result.forces(
            DVec3::X * 50.0,
            DVec3::ZERO,
            1.225,
            Controls::default(),
            &vec![0.0; result.flaps.len()],
            weights.load(0.0, false)?.cg,
        )?;
        Ok(result)
    }

    pub fn forces(
        &self,
        velocity: DVec3,
        angular_velocity: DVec3,
        density: f64,
        controls: Controls,
        flaps: &[f64],
        cg: DVec3,
    ) -> Result<Wrench> {
        ensure!(
            velocity.is_finite()
                && angular_velocity.is_finite()
                && density.is_finite()
                && density >= 0.0
                && cg.is_finite(),
            "invalid airflow"
        );
        ensure!(
            [controls.elevator, controls.aileron, controls.rudder]
                .iter()
                .all(|n| n.is_finite() && n.abs() <= 1.0),
            "invalid flight controls"
        );
        ensure!(
            flaps.len() == self.flaps.len() && flaps.iter().all(|n| n.is_finite()),
            "invalid flap deflections"
        );
        let speed = velocity.length();
        if speed == 0.0 || density == 0.0 {
            return Ok(Wrench::default());
        }
        let alpha = velocity.z.atan2(velocity.x);
        let beta = (velocity.y / speed).clamp(-1.0, 1.0).asin();
        let qbar_area = 0.5 * density * speed * speed * self.area;
        let p = -angular_velocity.x * self.span / (2.0 * speed);
        let q = -angular_velocity.y * self.chord / (2.0 * speed);
        let r = angular_velocity.z * self.span / (2.0 * speed);
        let elevator = controls.elevator
            * self.elevator[usize::from(controls.elevator < 0.0)]
            * self.effectiveness.y;
        let aileron = controls.aileron * self.aileron * self.effectiveness.x;
        let rudder = controls.rudder * self.rudder * self.effectiveness.z;
        let flap = flaps
            .iter()
            .zip(&self.flaps)
            .fold(DVec3::ZERO, |sum, (angle, scalars)| {
                sum + DVec3::from_array(*scalars) * angle
            });
        let c = &self.coefficients;
        let cl = self.lift_scalar * (self.lift.sample(alpha) + c.lift.cl_flaps * flap.x)
            + c.lift.cl_de * elevator
            + c.lift.cl_ih * self.incidence
            + c.pitch.cl_q * q;
        let cd = self.drag_scalar * (c.drag.cd0 + c.drag.cd_flaps * flap.y)
            + self.induced_drag * cl * cl;
        let cy = c.side.cy_beta * beta
            + c.side.cy_delta_rudder * rudder
            + c.side.cy_p * p
            + c.side.cy_r * r;
        let roll = c.roll.cl_beta * beta
            + c.roll.cl_delta_aileron * aileron
            + c.roll.cl_delta_rudder * rudder
            + c.roll.cl_p * p * self.stability.x
            + c.roll.cl_r * r;
        let pitch = c.pitch.cm0
            + self.pitch.sample(alpha)
            + c.pitch.cm_de * elevator
            + c.pitch.cm_ih * self.incidence
            + c.pitch.cm_flap * flap.z
            + c.pitch.cm_q * q * self.stability.y;
        let yaw = c.yaw.cn_beta * beta
            + c.yaw.cn_delta_aileron * aileron
            + c.yaw.cn_delta_rudder * rudder
            + c.yaw.cn_p * p
            + c.yaw.cn_r * r * self.stability.z;
        let lift_direction = DVec3::new(alpha.sin(), 0.0, -alpha.cos());
        let force = (lift_direction * cl - velocity / speed * cd + DVec3::Y * cy) * qbar_area;
        // AIR roll/pitch moments use the opposite signs to forward-right-down body axes.
        let mut moment =
            DVec3::new(-roll * self.span, -pitch * self.chord, yaw * self.span) * qbar_area;
        let offset = -cg.cross(force);
        moment += DVec3::new(
            offset.x,
            offset.y - self.pitch_reference * force.z,
            offset.z,
        );
        ensure!(
            force.is_finite() && moment.is_finite() && cd >= 0.0,
            "invalid aerodynamic forces"
        );
        Ok(Wrench { force, moment })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolates_curves_and_clamps_to_defined_domain() {
        let curve = Curve::new(vec![
            Point {
                input: -1.0,
                output: -2.0,
            },
            Point {
                input: 1.0,
                output: 4.0,
            },
        ])
        .unwrap();
        assert_eq!(curve.sample(0.0), 1.0);
        assert_eq!(curve.sample(-2.0), -2.0);
        assert_eq!(curve.sample(2.0), 4.0);
        assert!(
            Curve::new(vec![
                Point {
                    input: 0.0,
                    output: 1.0
                };
                2
            ])
            .is_err()
        );
        assert!(Curve::new(vec![]).is_err());
    }
}

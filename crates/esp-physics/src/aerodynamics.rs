use anyhow::{Context, Result, ensure};
use esp_assets::flight::FlightData;
use esp_formats::{
    air::{
        self,
        parameters::{LegacyParameters, Parameters},
    },
    cfg::Config,
};
use glam::DVec3;

use crate::{flight::Wrench, lookup::Curve};

const FOOT: f64 = 0.3048;

pub struct Aerodynamics {
    area: f64,
    span: f64,
    wing_reference: DVec3,
    chord: f64,
    induced_drag: f64,
    lift_scalar: f64,
    drag_scalar: f64,
    stability: DVec3,
    effectiveness: DVec3,
    control_scaling: [Option<Curve>; 3],
    control_elasticity: [Option<Curve>; 3],
    trim_elasticity: Option<Curve>,
    /// Angle-of-attack scalars for roll sideslip, roll damping, yaw sideslip and yaw damping.
    angular_effects: [Option<Curve>; 4],
    elevator: [f64; 2],
    elevator_trim: f64,
    elevator_trim_limit: f64,
    lateral_trim: [f64; 2],
    lateral_trim_limits: [f64; 2],
    spoiler_limit: f64,
    spoileron_gain: f64,
    spoileron_threshold: f64,
    spoileron_min_flaps: f64,
    aileron: f64,
    rudder: f64,
    incidence: f64,
    pitch_reference: f64,
    lift: Curve,
    pitch: Curve,
    ground_effect: Option<Curve>,
    lift_mach: Option<Curve>,
    drag_mach: Option<Curve>,
    coefficients: LegacyParameters,
    flaps: Vec<[f64; 3]>,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Controls {
    pub elevator: f64,
    pub aileron: f64,
    pub rudder: f64,
    pub elevator_trim: f64,
    pub aileron_trim: f64,
    pub rudder_trim: f64,
    pub spoilers: f64,
    pub spoiler_difference: f64,
    pub gear: f64,
    pub ground_height: Option<f64>,
    /// Local speed of sound in m/s; sea-level standard is used when absent.
    pub sound_speed: Option<f64>,
    /// Normalized structural ice accumulation.
    pub structural_ice: f64,
}

impl Curve {
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

fn aerodynamic_coefficients(records: &[air::Record<'_>]) -> Result<LegacyParameters> {
    let unique = |id| -> Result<Option<&air::Record<'_>>> {
        let mut matching = records.iter().filter(|record| record.id == id);
        let first = matching.next();
        ensure!(matching.next().is_none(), "duplicate AIR record {id:#x}");
        Ok(first)
    };
    if unique(0x1539)?.is_some() {
        let read = |id| -> Result<Parameters> {
            unique(id)?
                .with_context(|| format!("missing AIR record {id:#x}"))?
                .aerodynamic_parameters()?
                .with_context(|| format!("invalid AIR record {id:#x}"))
        };
        let Parameters::Lift(lift) = read(0x1539)? else {
            unreachable!()
        };
        let Parameters::Drag(drag) = read(0x1540)? else {
            unreachable!()
        };
        let Parameters::Pitch(pitch) = read(0x1541)? else {
            unreachable!()
        };
        let Parameters::SideForce(side) = read(0x1542)? else {
            unreachable!()
        };
        let Parameters::Roll(roll) = read(0x1543)? else {
            unreachable!()
        };
        let Parameters::Yaw(yaw) = read(0x1544)? else {
            unreachable!()
        };
        Ok(LegacyParameters {
            lift,
            drag,
            pitch,
            side,
            roll,
            yaw,
        })
    } else {
        unique(0x1101)?
            .context("missing AIR record 0x1101")?
            .legacy_aerodynamics()?
            .context("invalid AIR record 0x1101")
    }
}

impl Aerodynamics {
    pub fn from_data(
        data: &FlightData,
        loaded_cg: impl FnOnce() -> Result<DVec3>,
        empty_cg: impl FnOnce() -> Result<DVec3>,
    ) -> Result<Self> {
        let config = &data.configuration;
        let records = air::parse(&data.air)?;
        ensure!(
            !records.iter().any(|r| matches!(r.id, 0x1545 | 0x1546)),
            "extended AIR lift and pitch tables are not simulated yet"
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
            crate::content_tables::curve(record(id)?).with_context(|| format!("AIR record {id:#x}"))
        };
        let optional_curve = |id| -> Result<Option<Curve>> {
            if records.iter().any(|r| r.id == id) {
                Ok(Some(curve(id)?))
            } else {
                Ok(None)
            }
        };
        let lift = curve(0x404)?;
        let pitch = curve(0x473)?;
        let coefficients = aerodynamic_coefficients(&records)?;
        let lateral_trim = if records.iter().any(|r| r.id == 0x516) {
            let definition = record(0x516)?;
            ensure!(
                definition.data.len() == 16,
                "invalid AIR lateral trim record"
            );
            let aileron = f64::from_le_bytes(definition.data[..8].try_into().unwrap());
            let rudder = f64::from_le_bytes(definition.data[8..].try_into().unwrap());
            ensure!(
                aileron.is_finite() && rudder.is_finite(),
                "invalid AIR lateral trim coefficients"
            );
            [
                aileron * tuning(config, "aileron_trim_effectiveness")?,
                rudder * tuning(config, "rudder_trim_effectiveness")?,
            ]
        } else {
            [0.0; 2]
        };
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
        let datum = esp_formats::cfg::aircraft::ReferencePoints::from_config(config)?
            .datum_feet
            .context("missing reference datum")?;
        let wing_reference = DVec3::new(
            datum[0]
                + config
                    .number("airplane_geometry", "wing_pos_apex_lon")?
                    .unwrap_or(0.0),
            datum[1],
            -datum[2]
                - config
                    .number("airplane_geometry", "wing_pos_apex_vert")?
                    .unwrap_or(0.0),
        ) * FOOT;
        let ground_effect = if records.iter().any(|r| r.id == 0x400) {
            Some(curve(0x400)?)
        } else {
            None
        };
        ensure!(
            ground_effect.as_ref().is_none_or(|curve| curve
                .0
                .iter()
                .all(|point| point.input >= 0.0 && point.output >= 0.0)),
            "invalid ground effect table"
        );
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
            loaded_cg()?.x - pitch.gradient() / lift.gradient() * chord
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
        let elevator_trim_limit = config
            .number("airplane_geometry", "elevator_trim_limit")?
            .unwrap_or(0.0)
            .to_radians();
        // Legacy AIR files do not specify lateral trim travel; use ten degrees
        // when the aircraft does not provide a rudder limit.
        let lateral_trim_limits = [
            10f64.to_radians(),
            config
                .number("airplane_geometry", "rudder_trim_limit")?
                .unwrap_or(10.0)
                .to_radians(),
        ];
        let spoilerons = config
            .number("airplane_geometry", "spoilerons_available")?
            .unwrap_or(0.0)
            != 0.0;
        let result = Self {
            area,
            span,
            wing_reference,
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
            control_scaling: [
                optional_curve(0x342)?,
                optional_curve(0x341)?,
                optional_curve(0x343)?,
            ],
            control_elasticity: [
                optional_curve(0x518)?,
                optional_curve(0x517)?,
                optional_curve(0x519)?,
            ],
            trim_elasticity: optional_curve(0x1525)?,
            angular_effects: [
                optional_curve(0x451)?,
                optional_curve(0x456)?,
                optional_curve(0x460)?,
                optional_curve(0x464)?,
            ],
            elevator: [
                geometry("elevator_up_limit")?.to_radians(),
                geometry("elevator_down_limit")?.to_radians(),
            ],
            elevator_trim: elevator_trim_limit * tuning(config, "elevator_trim_effectiveness")?,
            elevator_trim_limit,
            lateral_trim,
            lateral_trim_limits,
            spoiler_limit: config
                .number("airplane_geometry", "spoiler_limit")?
                .unwrap_or(0.0)
                .to_radians(),
            spoileron_gain: if spoilerons {
                config
                    .number("airplane_geometry", "aileron_to_spoileron_gain")?
                    .unwrap_or(0.0)
            } else {
                0.0
            },
            spoileron_threshold: config
                .number("airplane_geometry", "min_ailerons_for_spoilerons")?
                .unwrap_or(0.0)
                .to_radians(),
            spoileron_min_flaps: config
                .number("airplane_geometry", "min_flaps_for_spoilerons")?
                .unwrap_or(0.0),
            aileron: (geometry("aileron_up_limit")? + geometry("aileron_down_limit")?).to_radians()
                * 0.5,
            rudder: geometry("rudder_limit")?.to_radians(),
            incidence: geometry("htail_incidence")?.to_radians(),
            pitch_reference,
            lift,
            pitch,
            ground_effect,
            lift_mach: optional_curve(0x401)?,
            drag_mach: optional_curve(0x154a)?,
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
                .chain([
                    &result.aileron,
                    &result.rudder,
                    &result.elevator_trim,
                    &result.elevator_trim_limit,
                    &result.lateral_trim_limits[0],
                    &result.lateral_trim_limits[1],
                    &result.spoiler_limit,
                    &result.spoileron_gain,
                    &result.spoileron_threshold,
                    &result.spoileron_min_flaps
                ])
                .all(|angle| *angle >= 0.0),
            "invalid control surface limits"
        );
        result.forces(
            DVec3::X * 50.0,
            DVec3::ZERO,
            1.225,
            Controls::default(),
            &vec![0.0; result.flaps.len()],
            empty_cg()?,
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
            [
                controls.elevator,
                controls.aileron,
                controls.rudder,
                controls.elevator_trim,
                controls.aileron_trim,
                controls.rudder_trim,
                controls.spoiler_difference
            ]
            .iter()
            .all(|n| n.is_finite() && n.abs() <= 1.0),
            "invalid flight controls"
        );
        ensure!(
            controls.spoilers.is_finite() && (0.0..=1.0).contains(&controls.spoilers),
            "invalid spoiler position"
        );
        ensure!(
            controls.gear.is_finite() && (0.0..=1.0).contains(&controls.gear),
            "invalid gear position"
        );
        ensure!(
            controls.structural_ice.is_finite() && (0.0..=1.0).contains(&controls.structural_ice),
            "invalid structural ice"
        );
        ensure!(
            controls
                .ground_height
                .is_none_or(|height| height.is_finite() && height >= 0.0),
            "invalid height above ground"
        );
        ensure!(
            controls
                .sound_speed
                .is_none_or(|speed| speed.is_finite() && speed > 0.0),
            "invalid speed of sound"
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
        let mach = speed / controls.sound_speed.unwrap_or(340.293_988);
        let lift_mach = self
            .lift_mach
            .as_ref()
            .map_or(1.0, |curve| curve.sample(mach));
        let drag_mach = self
            .drag_mach
            .as_ref()
            .map_or(0.0, |curve| curve.sample(mach));
        let beta = (velocity.y / speed).clamp(-1.0, 1.0).asin();
        let angle_factor = |index: usize| {
            self.angular_effects[index]
                .as_ref()
                .map_or(1.0, |curve| curve.sample(alpha.to_degrees()))
        };
        let dynamic_pressure = 0.5 * density * speed * speed;
        let qbar_area = dynamic_pressure * self.area;
        let pressure_psf = dynamic_pressure / 47.88025898033584;
        let p = -angular_velocity.x * self.span / (2.0 * speed);
        let q = -angular_velocity.y * self.chord / (2.0 * speed);
        let r = angular_velocity.z * self.span / (2.0 * speed);
        let scaled = |index: usize, angle: f64, effectiveness: f64| {
            let angle_factor = self.control_scaling[index]
                .as_ref()
                .map_or(1.0, |curve| curve.sample(angle));
            let pressure_factor = self.control_elasticity[index]
                .as_ref()
                .map_or(1.0, |curve| curve.sample(pressure_psf));
            angle * effectiveness * angle_factor * pressure_factor
        };
        let elevator = scaled(
            1,
            controls.elevator * self.elevator[usize::from(controls.elevator < 0.0)],
            self.effectiveness.y,
        );
        let aileron = scaled(0, controls.aileron * self.aileron, self.effectiveness.x);
        let rudder = scaled(2, controls.rudder * self.rudder, self.effectiveness.z);
        let trim_factor = self
            .trim_elasticity
            .as_ref()
            .map_or(1.0, |curve| curve.sample(pressure_psf));
        let flap = flaps
            .iter()
            .zip(&self.flaps)
            .fold(DVec3::ZERO, |sum, (angle, scalars)| {
                sum + DVec3::from_array(*scalars) * angle
            });
        let c = &self.coefficients;
        let spoiler = controls.spoilers * self.spoiler_limit;
        // The authored curve approaches unity as height approaches one span.
        let ground_scalar = controls
            .ground_height
            .zip(self.ground_effect.as_ref())
            .map_or(1.0, |(height, curve)| curve.sample(height / self.span));
        let cl = (1.0 - 0.3 * controls.structural_ice)
            * ground_scalar
            * (self.lift_scalar * (self.lift.sample(alpha) * lift_mach + c.lift.cl_flaps * flap.x)
                + c.lift.cl_spoiler * spoiler
                + c.lift.cl_de * elevator
                + c.lift.cl_ih * self.incidence
                + c.pitch.cl_q * q);
        let cd = self.drag_scalar
            * (c.drag.cd0
                + drag_mach
                + c.drag.cd_flaps * flap.y
                + c.drag.cd_spoiler * spoiler
                + c.drag.cd_gear * controls.gear)
            + self.induced_drag * cl * cl
            + 0.08 * controls.structural_ice;
        let cy = c.side.cy_beta * beta
            + c.side.cy_delta_rudder * rudder
            + c.side.cy_p * p
            + c.side.cy_r * r;
        let roll = c.roll.cl_beta * beta * angle_factor(0)
            + c.roll.cl_delta_aileron * aileron
            + self.lateral_trim[0] * controls.aileron_trim * self.lateral_trim_limits[0]
            + c.roll.cl_delta_spoiler * controls.spoiler_difference * self.spoiler_limit
            + c.roll.cl_delta_rudder * rudder
            + c.roll.cl_p * p * self.stability.x * angle_factor(1)
            + c.roll.cl_r * r;
        let pitch = c.pitch.cm0
            + self.pitch.sample(alpha)
            + c.pitch.cm_de * elevator
            + c.pitch.cm_delta_trim * controls.elevator_trim * self.elevator_trim * trim_factor
            + c.pitch.cm_spoiler * spoiler
            + c.pitch.cm_gear * controls.gear
            + c.pitch.cm_ih * self.incidence
            + c.pitch.cm_flap * flap.z
            + c.pitch.cm_q * q * self.stability.y;
        let yaw = c.yaw.cn_beta * beta * angle_factor(2)
            + c.yaw.cn_delta_aileron * aileron
            + c.yaw.cn_delta_rudder * rudder
            + self.lateral_trim[1] * controls.rudder_trim * self.lateral_trim_limits[1]
            + c.yaw.cn_p * p
            + c.yaw.cn_r * r * self.stability.z * angle_factor(3);
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

    /// Estimated control and directional loads from the propeller wake.
    pub fn tail_wash_forces(
        &self,
        flow: (DVec3, crate::propeller::TailWash),
        angular_velocity: DVec3,
        density: f64,
        controls: Controls,
        flaps: &[f64],
        cg: DVec3,
    ) -> Result<Wrench> {
        let (velocity, wash) = flow;
        ensure!(
            wash.horizontal.is_finite() && wash.vertical.is_finite() && wash.swirl.is_finite(),
            "invalid propeller wash"
        );
        if wash == crate::propeller::TailWash::default() {
            return Ok(Wrench::default());
        }
        let mut neutral = controls;
        neutral.elevator = 0.0;
        neutral.elevator_trim = 0.0;
        neutral.rudder = 0.0;
        neutral.rudder_trim = 0.0;
        let increment = |flow, active| -> Result<Wrench> {
            let active = self.forces(flow, angular_velocity, density, active, flaps, cg)?;
            let neutral = self.forces(flow, angular_velocity, density, neutral, flaps, cg)?;
            Ok(Wrench {
                force: active.force - neutral.force,
                moment: active.moment - neutral.moment,
            })
        };
        let mut added = Wrench::default();
        for (enabled, flow, elevator) in [
            (
                controls.elevator != 0.0 || controls.elevator_trim != 0.0,
                wash.horizontal,
                true,
            ),
            (
                controls.rudder != 0.0 || controls.rudder_trim != 0.0,
                wash.vertical + wash.swirl,
                false,
            ),
        ] {
            if !enabled || flow.length_squared() == 0.0 {
                continue;
            }
            let mut active = neutral;
            if elevator {
                active.elevator = controls.elevator;
                active.elevator_trim = controls.elevator_trim;
            } else {
                active.rudder = controls.rudder;
                active.rudder_trim = controls.rudder_trim;
            }
            let base = increment(velocity, active)?;
            let washed = increment(velocity + flow, active)?;
            added.force += (washed.force - base.force) * 0.5;
            added.moment += (washed.moment - base.moment) * 0.5;
        }
        if wash.swirl.length_squared() != 0.0 {
            let axial = self.forces(
                velocity + wash.vertical,
                angular_velocity,
                density,
                neutral,
                flaps,
                cg,
            )?;
            let rotating = self.forces(
                velocity + wash.vertical + wash.swirl,
                angular_velocity,
                density,
                neutral,
                flaps,
                cg,
            )?;
            added.force.y += (rotating.force.y - axial.force.y) * 0.5;
            added.moment.z += (rotating.moment.z - axial.moment.z) * 0.5;
        }
        Ok(added)
    }
    pub fn elevator_trim_limit(&self) -> f64 {
        self.elevator_trim_limit
    }

    pub fn wing_height(
        &self,
        state: &crate::flight::Motion,
        cg: DVec3,
        ground_elevation: f64,
    ) -> f64 {
        (-(state.position + state.orientation * (self.wing_reference - cg)).z - ground_elevation)
            .max(0.0)
    }

    pub fn lateral_trim_angles(&self, controls: Controls) -> [f64; 2] {
        [
            controls.aileron_trim * self.lateral_trim_limits[0],
            controls.rudder_trim * self.lateral_trim_limits[1],
        ]
    }

    pub fn has_spoilers(&self) -> bool {
        self.spoiler_limit > 0.0
    }

    pub fn spoiler_targets(&self, handle: f64, aileron: f64, flap_handle: f64) -> [f64; 2] {
        if !self.has_spoilers() {
            return [0.0; 2];
        }
        let mut target = [handle; 2];
        if flap_handle >= self.spoileron_min_flaps && self.spoileron_gain > 0.0 {
            let angle = (aileron.abs() * self.aileron - self.spoileron_threshold).max(0.0)
                * self.spoileron_gain;
            let index = usize::from(aileron > 0.0);
            target[index] = (target[index] + angle / self.spoiler_limit).min(1.0);
        }
        target
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lookup::Point;

    #[test]
    fn uses_complete_modern_aerodynamic_blocks_in_preference_to_legacy() {
        let data: Vec<Vec<u8>> = [6, 4, 13, 4, 6, 9]
            .into_iter()
            .enumerate()
            .map(|(i, n)| vec![(i as u8) + 1; n * 8])
            .collect();
        let ids = [0x1539, 0x1540, 0x1541, 0x1542, 0x1543, 0x1544];
        let records: Vec<_> = ids
            .into_iter()
            .zip(&data)
            .map(|(id, data)| air::Record {
                id,
                offset: 0,
                data,
            })
            .collect();
        let coefficients = aerodynamic_coefficients(&records).unwrap();
        assert_eq!(coefficients.lift.cl_de, f64::from_le_bytes([1; 8]));
        assert_eq!(coefficients.drag.cd0, f64::from_le_bytes([2; 8]));
        assert_eq!(coefficients.yaw.cn_r, f64::from_le_bytes([6; 8]));
        assert!(aerodynamic_coefficients(&records[..5]).is_err());
        let mut duplicates = records;
        let legacy = [0u8; 250];
        duplicates.push(air::Record {
            id: 0x1101,
            offset: 0,
            data: &legacy,
        });
        assert_eq!(
            aerodynamic_coefficients(&duplicates).unwrap().drag.cd0,
            f64::from_le_bytes([2; 8])
        );
        duplicates.push(air::Record {
            id: 0x1540,
            offset: 0,
            data: &data[1],
        });
        assert!(
            aerodynamic_coefficients(&duplicates)
                .unwrap_err()
                .to_string()
                .contains("duplicate")
        );
    }

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

use anyhow::{Context, Result, ensure};
use esp_assets::flight::FlightData;
use esp_formats::air;
use glam::DVec3;
use std::f64::consts::{PI, TAU};

use crate::{atmosphere::Atmosphere, lookup::Grid};

/// Approximate flow increments reaching each tail surface, in body axes.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TailWash {
    pub horizontal: DVec3,
    pub vertical: DVec3,
    pub swirl: DVec3,
}
pub struct Propeller {
    efficiency: Grid,
    power: Grid,
    diameter: f64,
    inertia: f64,
    reduction: f64,
    beta: f64,
    low_speed_limit: f64,
    thrust_scalar: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Output {
    pub rpm: f64,
    pub advance_ratio: f64,
    pub efficiency: f64,
    pub absorbed_power: f64,
    /// Torque reflected to the engine shaft, in N m.
    pub absorbed_torque: f64,
    pub thrust: f64,
}

impl Propeller {
    pub fn from_data(data: &FlightData) -> Result<Self> {
        let config = &data.configuration;
        ensure!(
            config.number("propeller", "propeller_type")? == Some(1.0),
            "only fixed-pitch propellers are supported yet"
        );
        let records = air::parse(&data.air)?;
        let grid = |id| -> Result<Grid> {
            let mut matches = records.iter().filter(|r| r.id == id);
            let record = matches
                .next()
                .with_context(|| format!("missing propeller AIR record {id:#x}"))?;
            ensure!(
                matches.next().is_none(),
                "duplicate propeller AIR record {id:#x}"
            );
            Grid::from_record(record)
        };
        let required = |key| -> Result<f64> {
            config
                .number("propeller", key)?
                .with_context(|| format!("missing [propeller] {key}"))
        };
        let diameter = required("propeller_diameter")? * 0.3048;
        let inertia = required("propeller_moi")? * 1.3558179483314;
        let reduction = config
            .number("propeller", "gear_reduction_ratio")?
            .unwrap_or(1.0);
        let beta = required("fixed_pitch_beta")?;
        let low_speed_limit = config
            .number("propeller", "low_speed_theory_limit")?
            .unwrap_or(80.0)
            * 0.3048;
        let thrust_scalar = config.number("propeller", "thrust_scalar")?.unwrap_or(1.0);
        ensure!(
            diameter > 0.0
                && inertia > 0.0
                && reduction > 0.0
                && low_speed_limit > 0.0
                && thrust_scalar >= 0.0,
            "invalid propeller dimensions, inertia or tuning"
        );
        Ok(Self {
            efficiency: grid(0x511)?,
            power: grid(0x512)?,
            diameter,
            inertia,
            reduction,
            beta,
            low_speed_limit,
            thrust_scalar,
        })
    }

    pub fn radius(&self) -> f64 {
        self.diameter * 0.5
    }

    pub fn angular_momentum(&self, engine_rpm: f64) -> f64 {
        self.inertia * (engine_rpm / self.reduction) * PI / 30.0
    }

    pub fn engine_shaft_inertia(&self) -> f64 {
        self.inertia / self.reduction.powi(2)
    }

    pub fn gear_reduction_ratio(&self) -> f64 {
        self.reduction
    }

    pub fn sample(&self, air: &Atmosphere, engine_rpm: f64, axial_speed: f64) -> Result<Output> {
        ensure!(
            engine_rpm.is_finite()
                && engine_rpm >= 0.0
                && axial_speed.is_finite()
                && axial_speed >= 0.0,
            "invalid propeller RPM or forward axial speed"
        );
        ensure!(
            air.density.is_finite() && air.density > 0.0,
            "invalid propeller air density"
        );
        if engine_rpm == 0.0 {
            return Ok(Output::default());
        }
        let rpm = engine_rpm / self.reduction;
        let turns = rpm / 60.0;
        let advance_ratio = axial_speed / (turns * self.diameter);
        let efficiency = self.efficiency.sample(self.beta, advance_ratio);
        ensure!(
            (0.0..=1.0).contains(&efficiency),
            "unsupported propeller efficiency outside physical range"
        );
        let cp = self.power.sample(self.beta, advance_ratio);
        let absorbed_power = cp * air.density * turns.powi(3) * self.diameter.powi(5);
        let absorbed_torque = absorbed_power / (TAU * turns * self.reduction);
        let high = if axial_speed > 0.0 {
            efficiency * absorbed_power / axial_speed
        } else {
            0.0
        };
        let thrust = if axial_speed >= self.low_speed_limit || absorbed_power <= 0.0 {
            high
        } else {
            // FSX does not publish its low-speed fallback equations. Use an
            // actuator disk with the authored efficiency curve to keep static
            // thrust finite, then blend into the AIR P*eta/V relation at the
            // configured low-speed limit. This is a momentum-model fallback,
            // not a claim of bit-for-bit FSX low-speed behavior.
            let area = PI * (self.diameter * 0.5).powi(2);
            let mut lo = 0.0;
            let mut hi = (absorbed_power / (2.0 * air.density * area)).cbrt() + 1.0;
            for _ in 0..40 {
                let induced = (lo + hi) * 0.5;
                let flow = axial_speed + induced;
                let eta = self
                    .efficiency
                    .sample(self.beta, flow / (turns * self.diameter));
                ensure!(
                    (0.0..=1.0).contains(&eta),
                    "unsupported low-speed propeller efficiency"
                );
                if 2.0 * air.density * area * induced * flow.powi(2) > eta * absorbed_power {
                    hi = induced;
                } else {
                    lo = induced;
                }
            }
            let induced = (lo + hi) * 0.5;
            let momentum = 2.0 * air.density * area * induced * (axial_speed + induced);
            let t = (axial_speed / self.low_speed_limit).clamp(0.0, 1.0);
            let blend = t * t * (3.0 - 2.0 * t);
            momentum + (high - momentum) * blend
        } * self.thrust_scalar;
        ensure!(
            [
                rpm,
                advance_ratio,
                efficiency,
                absorbed_power,
                absorbed_torque,
                thrust
            ]
            .iter()
            .all(|v| v.is_finite()),
            "non-finite propeller output"
        );
        Ok(Output {
            rpm,
            advance_ratio,
            efficiency,
            absorbed_power,
            absorbed_torque,
            thrust,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn propeller(cp: f64) -> Propeller {
        let grid = |id, value| {
            let values = [0.0_f64, 0.0, 2.0, 15.0, value, value, 25.0, value, value];
            let bytes = [
                3_u32.to_le_bytes().to_vec(),
                3_u32.to_le_bytes().to_vec(),
                values.into_iter().flat_map(f64::to_le_bytes).collect(),
            ]
            .concat();
            Grid::from_record(&air::Record {
                id,
                offset: 0,
                data: &bytes,
            })
            .unwrap()
        };
        Propeller {
            efficiency: grid(0x511, 0.7),
            power: grid(0x512, cp),
            diameter: 2.0,
            inertia: 5.0,
            reduction: 1.0,
            beta: 20.0,
            low_speed_limit: 24.0,
            thrust_scalar: 1.0,
        }
    }

    #[test]
    fn propeller_loading_obeys_density_rpm_and_gear_scaling() {
        let prop = propeller(0.08);
        let air = Atmosphere::standard(0.0).unwrap();
        let base = prop.sample(&air, 1200.0, 30.0).unwrap();
        let fast = prop.sample(&air, 2400.0, 60.0).unwrap();
        assert_eq!(base.advance_ratio, fast.advance_ratio);
        assert!((fast.absorbed_power / base.absorbed_power - 8.0).abs() < 1e-12);
        assert!((fast.absorbed_torque / base.absorbed_torque - 4.0).abs() < 1e-12);
        assert!((fast.thrust / base.thrust - 4.0).abs() < 1e-12);
        assert!((base.thrust * 30.0 / base.absorbed_power - 0.7).abs() < 1e-12);
        let mut geared = propeller(0.08);
        geared.reduction = 2.0;
        let reduced = geared.sample(&air, 2400.0, 30.0).unwrap();
        assert_eq!(base.rpm, reduced.rpm);
        assert_eq!(base.absorbed_power, reduced.absorbed_power);
        assert_eq!(base.absorbed_torque, reduced.absorbed_torque * 2.0);
        assert_eq!(
            geared.engine_shaft_inertia(),
            prop.engine_shaft_inertia() / 4.0
        );
        let thin = Atmosphere {
            density: air.density * 0.5,
            ..air
        };
        let high = prop.sample(&thin, 1200.0, 30.0).unwrap();
        assert!((high.thrust / base.thrust - 0.5).abs() < 1e-12);
    }

    #[test]
    fn static_fallback_is_finite_and_joins_the_forward_flight_relation() {
        let prop = propeller(0.08);
        let air = Atmosphere::standard(0.0).unwrap();
        let static_output = prop.sample(&air, 2400.0, 0.0).unwrap();
        let expected =
            (2.0 * air.density * PI).cbrt() * (static_output.absorbed_power * 0.7).powf(2.0 / 3.0);
        assert!((static_output.thrust / expected - 1.0).abs() < 1e-9);
        for boundary in [0.0, prop.low_speed_limit] {
            let a = prop
                .sample(&air, 2400.0, (boundary - 1e-6).max(0.0))
                .unwrap();
            let b = prop.sample(&air, 2400.0, boundary + 1e-6).unwrap();
            assert!((a.thrust - b.thrust).abs() < 0.01);
        }
        assert_eq!(prop.sample(&air, 0.0, 0.0).unwrap().thrust, 0.0);
    }

    #[test]
    fn negative_power_coefficients_drive_the_shaft_and_generate_drag() {
        let prop = propeller(-0.08);
        let output = prop
            .sample(&Atmosphere::standard(0.0).unwrap(), 2400.0, 60.0)
            .unwrap();
        assert!(output.absorbed_torque < 0.0 && output.absorbed_power < 0.0 && output.thrust < 0.0);
    }
}

use anyhow::{Result, ensure};
use std::f64::consts::PI;

use crate::lookup::Curve;

mod engine;
pub use engine::{Command, EngineOutput, Pistons};

pub struct PistonParameters {
    pub efficiency: Curve,
    pub friction: Curve,
    pub displacement: f64,
    pub cylinders: f64,
    pub compression: f64,
    pub maximum_rpm: f64,
    pub power_scalar: f64,
    pub mechanical_scalars: [f64; 2],
    pub friction_scalars: [f64; 2],
    pub fuel_scalar: f64,
    pub specific_consumption: f64,
}
pub struct PistonCycle {
    efficiency: Curve,
    friction: Curve,
    displacement: f64,
    cylinders: f64,
    compression: f64,
    maximum_rpm: f64,
    power_scalar: f64,
    mechanical_scalars: [f64; 2],
    friction_scalars: [f64; 2],
    fuel_scalar: f64,
    specific_consumption: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct Charge {
    pub pressure: f64,
    pub temperature: f64,
    /// Combustion heat released per kilogram of inducted air, in J/kg.
    /// Zero represents an engine that is not firing.
    pub heat_release: f64,
    pub oil_viscosity: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Output {
    pub gross_torque: f64,
    pub friction_torque: f64,
    pub shaft_torque: f64,
    pub shaft_power: f64,
    pub air_kilograms_per_second: f64,
    /// Reference fuel flow at the configured best-power specific consumption.
    pub best_power_fuel_kilograms_per_second: f64,
}

impl PistonCycle {
    pub fn new(parameters: PistonParameters) -> Result<Self> {
        let PistonParameters {
            efficiency,
            friction,
            displacement,
            cylinders,
            compression,
            maximum_rpm,
            power_scalar,
            mechanical_scalars,
            friction_scalars,
            fuel_scalar,
            specific_consumption,
        } = parameters;
        ensure!(
            displacement > 0.0
                && cylinders > 0.0
                && cylinders.fract() == 0.0
                && compression > 1.0
                && maximum_rpm > 0.0,
            "invalid piston geometry or rated RPM"
        );
        ensure!(
            [power_scalar, fuel_scalar, specific_consumption]
                .iter()
                .chain(&mechanical_scalars)
                .chain(&friction_scalars)
                .all(|v| *v >= 0.0),
            "invalid piston tuning scalar"
        );
        ensure!(
            efficiency.outputs_nonnegative(),
            "negative piston mechanical efficiency"
        );
        Ok(Self {
            efficiency,
            friction,
            displacement,
            cylinders,
            compression,
            maximum_rpm,
            power_scalar,
            mechanical_scalars,
            friction_scalars,
            fuel_scalar,
            specific_consumption,
        })
    }
    pub fn sample(&self, rpm: f64, charge: Charge) -> Result<Output> {
        ensure!(
            rpm.is_finite()
                && rpm >= 0.0
                && charge.pressure.is_finite()
                && charge.pressure >= 0.0
                && charge.temperature.is_finite()
                && charge.temperature > 0.0
                && charge.heat_release.is_finite()
                && charge.heat_release >= 0.0
                && charge.oil_viscosity.is_finite()
                && charge.oil_viscosity >= 0.0,
            "invalid piston operating point"
        );
        // Ideal air-standard four-stroke Otto cycle. The AIR mechanical
        // efficiency and friction curves supply the authored loss model.
        // Intake/mixture/ignition models must provide the actual charge; this
        // calculation does not invent a manifold pressure from a lever value.
        let mass = charge.pressure / (287.05287 * charge.temperature) * self.displacement;
        let thermal = 1.0 - self.compression.powf(-0.4);
        let cycle_torque = mass * charge.heat_release * thermal * self.cylinders / (4.0 * PI);
        let blend = (rpm / self.maximum_rpm).clamp(0.0, 1.0);
        let scalar = |values: [f64; 2]| values[0] + (values[1] - values[0]) * blend;
        let gross_torque = cycle_torque
            * self.efficiency.sample(rpm)
            * scalar(self.mechanical_scalars)
            * self.power_scalar;
        let friction_torque = self.friction.sample(rpm)
            * 1.3558179483314
            * self.cylinders
            * charge.oil_viscosity
            * scalar(self.friction_scalars)
            * self.power_scalar;
        let shaft_torque = gross_torque - friction_torque;
        let shaft_power = shaft_torque * rpm * PI / 30.0;
        let air_kilograms_per_second = mass * self.cylinders * rpm / 120.0;
        let best_power_fuel_kilograms_per_second = shaft_power.max(0.0) / 745.69987158227
            * self.specific_consumption
            * self.fuel_scalar
            * 0.45359237
            / 3600.0;
        ensure!(
            [
                gross_torque,
                friction_torque,
                shaft_torque,
                shaft_power,
                air_kilograms_per_second,
                best_power_fuel_kilograms_per_second
            ]
            .iter()
            .all(|v| v.is_finite()),
            "non-finite piston cycle output"
        );
        Ok(Output {
            gross_torque,
            friction_torque,
            shaft_torque,
            shaft_power,
            air_kilograms_per_second,
            best_power_fuel_kilograms_per_second,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_formats::air::table::Point;

    fn cycle() -> PistonCycle {
        let curve = |a, b| {
            Curve::new(vec![
                Point {
                    input: 0.0,
                    output: a,
                },
                Point {
                    input: 3000.0,
                    output: b,
                },
            ])
            .unwrap()
        };
        PistonCycle {
            efficiency: curve(0.7, 0.6),
            friction: curve(0.0, 25.0),
            displacement: 0.0015,
            cylinders: 4.0,
            compression: 8.0,
            maximum_rpm: 3000.0,
            power_scalar: 1.0,
            mechanical_scalars: [1.0; 2],
            friction_scalars: [1.0; 2],
            fuel_scalar: 1.0,
            specific_consumption: 0.49,
        }
    }

    const CHARGE: Charge = Charge {
        pressure: 101325.0,
        temperature: 288.15,
        heat_release: 3_500_000.0,
        oil_viscosity: 1.0,
    };

    #[test]
    fn charge_density_and_compression_control_work_before_friction() {
        let mut cycle = cycle();
        let full = cycle.sample(2700.0, CHARGE).unwrap();
        let thin = cycle
            .sample(
                2700.0,
                Charge {
                    pressure: CHARGE.pressure * 0.5,
                    ..CHARGE
                },
            )
            .unwrap();
        assert_eq!(thin.gross_torque, full.gross_torque * 0.5);
        assert_eq!(thin.friction_torque, full.friction_torque);
        let hot = cycle
            .sample(
                2700.0,
                Charge {
                    temperature: CHARGE.temperature * 2.0,
                    ..CHARGE
                },
            )
            .unwrap();
        assert_eq!(hot.gross_torque, thin.gross_torque);
        cycle.compression = 16.0;
        let high = cycle.sample(2700.0, CHARGE).unwrap();
        assert!(high.gross_torque > full.gross_torque);
        assert_eq!(high.friction_torque, full.friction_torque);
    }

    #[test]
    fn an_unfired_engine_consumes_shaft_work_and_colder_oil_increases_the_loss() {
        let cycle = cycle();
        let stopped = cycle
            .sample(
                0.0,
                Charge {
                    heat_release: 0.0,
                    ..CHARGE
                },
            )
            .unwrap();
        assert_eq!(stopped.shaft_power, 0.0);
        let off = cycle
            .sample(
                1200.0,
                Charge {
                    heat_release: 0.0,
                    ..CHARGE
                },
            )
            .unwrap();
        assert!(off.shaft_torque < 0.0 && off.shaft_power < 0.0);
        assert_eq!(off.best_power_fuel_kilograms_per_second, 0.0);
        let cold = cycle
            .sample(
                1200.0,
                Charge {
                    heat_release: 0.0,
                    oil_viscosity: 2.0,
                    ..CHARGE
                },
            )
            .unwrap();
        assert_eq!(cold.friction_torque, off.friction_torque * 2.0);
        assert!(cycle.sample(f64::NAN, CHARGE).is_err());
        assert!(
            cycle
                .sample(
                    1200.0,
                    Charge {
                        temperature: 0.0,
                        ..CHARGE
                    }
                )
                .is_err()
        );
    }
}

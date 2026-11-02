use anyhow::{Context, Result, ensure};
use esp_assets::flight::FlightData;
use esp_formats::{air, air::table::Point, cfg};
use glam::DVec3;
use std::f64::consts::PI;

use super::{Charge, Output, PistonCycle};
use crate::{
    atmosphere::Atmosphere,
    flight::Wrench,
    lookup::Curve,
    propeller::{self, Propeller},
};

/// Warm, naturally aspirated four-stroke engines with fixed-pitch propellers.
/// Oil viscosity is held at its warm reference value; starter, thermal and
/// propeller gyroscopic/reaction loads are not connected yet.
pub struct Pistons {
    cycle: PistonCycle,
    propeller: Propeller,
    manifold: Curve,
    mixture: Curve,
    mixture_efficiency: Curve,
    mixture_consumption: Curve,
    automatic_mixture: bool,
    automatic_ignition: bool,
    engines: Vec<Engine>,
}

struct Engine {
    index: u32,
    position: DVec3,
    direction: DVec3,
    rpm: f64,
    angle: f64,
    command: Command,
    fuel_available: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Command {
    pub throttle: f64,
    pub mixture: f64,
    pub fuel_valve: bool,
    pub magnetos: [bool; 2],
}

impl Default for Command {
    fn default() -> Self {
        Self {
            throttle: 0.0,
            mixture: 1.0,
            fuel_valve: true,
            magnetos: [true; 2],
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct EngineOutput {
    pub rpm: f64,
    pub rpm_percent: f64,
    pub propeller_angle: f64,
    pub manifold_pressure: f64,
    pub mixture_ratio: f64,
    pub cycle: Output,
    pub propeller: propeller::Output,
    pub fuel_kilograms_per_second: f64,
    pub combustion: bool,
}

fn curve(config: &cfg::Config, key: &str, defaults: &[(f64, f64)]) -> Result<Curve> {
    let points = if let Some(value) = config.value("piston_engine", key)? {
        value
            .split(',')
            .map(|pair| -> Result<Point> {
                let (x, y) = pair
                    .trim()
                    .split_once(':')
                    .context("invalid piston curve point")?;
                Ok(Point {
                    input: x.trim().parse()?,
                    output: y.trim().parse()?,
                })
            })
            .collect::<Result<Vec<_>>>()?
    } else {
        defaults
            .iter()
            .map(|&(input, output)| Point { input, output })
            .collect()
    };
    ensure!(
        points.iter().all(|p| p.output >= 0.0),
        "negative piston curve output: {key}"
    );
    Curve::new(points).with_context(|| format!("piston {key}"))
}

impl Pistons {
    pub fn from_data(data: &FlightData) -> Result<Self> {
        let config = &data.configuration;
        for key in [
            "turbocharged",
            "supercharged",
            "diesel",
            "two_stroke_cycle",
            "manifold_pressure_regulator",
            "emergency_boost_type",
        ] {
            ensure!(
                config.number("piston_engine", key)?.unwrap_or(0.0) == 0.0,
                "piston {key} is not implemented yet"
            );
        }
        let records = air::parse(&data.air)?;
        let mut throttle = records.iter().filter(|r| r.id == 0x506);
        let manifold = if let Some(record) = throttle.next() {
            ensure!(
                throttle.next().is_none(),
                "duplicate piston AIR throttle profile"
            );
            Curve::from_record(record)?
        } else {
            // Published SDK defaults when a legacy AIR throttle table is absent.
            // docs.flightsimulator.com/html/Content_Configuration/SimObjects/Aircraft_SimO/engines_cfg.htm
            curve(
                config,
                "manifold_efficiency_table",
                &[(0.0, 0.2), (1.0, 0.97)],
            )?
        };
        ensure!(
            manifold.0.iter().all(|p| (0.0..=1.0).contains(&p.output)),
            "naturally aspirated manifold efficiency is outside 0..1"
        );
        let datum = cfg::aircraft::ReferencePoints::from_config(config)?
            .datum_feet
            .context("missing engine reference datum")?;
        let engines = cfg::engine::engines(config)?
            .into_iter()
            .map(|definition| -> Result<Engine> {
                let [x, y, z] = definition.position_feet;
                let pitch = definition.pitch_degrees.to_radians();
                let heading = definition.heading_degrees.to_radians();
                Ok(Engine {
                    index: definition
                        .index
                        .checked_add(1)
                        .context("engine index is too large")?,
                    position: DVec3::new(x + datum[0], y + datum[1], -z - datum[2]) * 0.3048,
                    direction: DVec3::new(
                        pitch.cos() * heading.cos(),
                        pitch.cos() * heading.sin(),
                        pitch.sin(),
                    ),
                    rpm: 0.0,
                    angle: 0.0,
                    command: Command::default(),
                    fuel_available: false,
                })
            })
            .collect::<Result<Vec<_>>>()?;
        ensure!(!engines.is_empty(), "aircraft has no configured engines");
        Ok(Self {
            cycle: PistonCycle::from_data(data)?,
            propeller: Propeller::from_data(data)?,
            manifold,
            mixture: curve(
                config,
                "mixture_lever_to_ratio_table",
                &[
                    (0.0, 0.0),
                    (0.21, 0.049),
                    (0.45, 0.067),
                    (0.6, 0.078),
                    (0.75, 0.083),
                    (0.9, 0.086),
                    (1.0, 0.09),
                ],
            )?,
            mixture_efficiency: curve(
                config,
                "mixture_ratio_to_engine_efficiency_table",
                &[
                    (0.024, 0.0),
                    (0.049, 0.705),
                    (0.06, 0.889),
                    (0.067, 0.94),
                    (0.083, 1.0),
                    (0.11, 0.95),
                    (0.12, 0.78),
                    (0.133, 0.0),
                ],
            )?,
            mixture_consumption: curve(
                config,
                "mixture_ratio_to_sfc_scalar_table",
                &[
                    (0.0, 0.0),
                    (0.059, 0.786),
                    (0.067, 0.932),
                    (0.083, 1.0),
                    (0.09, 1.066),
                    (0.105, 1.1),
                ],
            )?,
            automatic_mixture: config
                .number("piston_engine", "fuel_air_auto_mixture")?
                .unwrap_or(0.0)
                != 0.0,
            automatic_ignition: config
                .number("piston_engine", "auto_ignition")?
                .unwrap_or(0.0)
                != 0.0,
            engines,
        })
    }

    pub fn indices(&self) -> impl Iterator<Item = u32> + '_ {
        self.engines.iter().map(|e| e.index)
    }

    fn sample(
        &self,
        rpm: f64,
        air: &Atmosphere,
        speed: f64,
        command: Command,
        fuel: bool,
    ) -> Result<EngineOutput> {
        ensure!(
            air.pressure.is_finite()
                && air.pressure > 0.0
                && air.temperature.is_finite()
                && air.temperature > 0.0,
            "invalid piston inlet conditions"
        );
        let ratio = if !command.fuel_valve || !fuel || command.mixture == 0.0 {
            0.0
        } else if self.automatic_mixture {
            0.083
        } else {
            self.mixture.sample(command.mixture) * 1.225 / air.density
        };
        let ignition = self.automatic_ignition || command.magnetos.iter().any(|v| *v);
        let combustion = rpm > 0.0 && ignition && self.mixture_efficiency.sample(ratio) > 0.0;
        let ignition_efficiency = if self.automatic_ignition || command.magnetos.iter().all(|v| *v)
        {
            1.0
        } else {
            0.97
        };
        let manifold_pressure = air.pressure * self.manifold.sample(command.throttle);
        let cycle = self.cycle.sample(
            rpm,
            Charge {
                pressure: manifold_pressure,
                temperature: air.temperature,
                heat_release: if combustion {
                    0.083
                        * 43_500_000.0
                        * self.mixture_efficiency.sample(ratio)
                        * ignition_efficiency
                } else {
                    0.0
                },
                oil_viscosity: 1.0,
            },
        )?;
        let fuel_kilograms_per_second = if combustion {
            cycle.best_power_fuel_kilograms_per_second * self.mixture_consumption.sample(ratio)
        } else {
            0.0
        };
        Ok(EngineOutput {
            rpm,
            rpm_percent: rpm / self.cycle.maximum_rpm * 100.0,
            propeller_angle: 0.0,
            manifold_pressure,
            mixture_ratio: ratio,
            cycle,
            propeller: self.propeller.sample(air, rpm, speed)?,
            fuel_kilograms_per_second,
            combustion,
        })
    }

    pub fn output(&self, index: usize, air: &Atmosphere, velocity: DVec3) -> Result<EngineOutput> {
        ensure!(velocity.is_finite(), "invalid piston inlet velocity");
        let engine = self.engines.get(index).context("engine is absent")?;
        let mut output = self.sample(
            engine.rpm,
            air,
            velocity.dot(engine.direction).max(0.0),
            engine.command,
            engine.fuel_available,
        )?;
        output.propeller_angle = engine.angle;
        Ok(output)
    }

    pub fn start_running(&mut self, air: &Atmosphere, velocity: DVec3) -> Result<()> {
        ensure!(velocity.is_finite(), "invalid piston inlet velocity");
        let mut speeds = Vec::with_capacity(self.engines.len());
        for engine in &self.engines {
            let speed = velocity.dot(engine.direction).max(0.0);
            let torque = |rpm| -> Result<f64> {
                let output = self.sample(rpm, air, speed, Command::default(), true)?;
                Ok(output.cycle.shaft_torque - output.propeller.absorbed_torque)
            };
            let (mut lo, mut hi) = (1e-6, self.cycle.maximum_rpm * 2.0);
            ensure!(
                torque(lo)? > 0.0 && torque(hi)? < 0.0,
                "no running piston idle equilibrium"
            );
            for _ in 0..50 {
                let rpm = (lo + hi) * 0.5;
                if torque(rpm)? > 0.0 {
                    lo = rpm;
                } else {
                    hi = rpm;
                }
            }
            speeds.push((lo + hi) * 0.5);
        }
        for (engine, rpm) in self.engines.iter_mut().zip(speeds) {
            engine.rpm = rpm;
            engine.command = Command::default();
            engine.fuel_available = true;
        }
        Ok(())
    }

    pub fn advance(
        &mut self,
        dt: f64,
        air: &Atmosphere,
        velocity: DVec3,
        commands: &[Command],
        fuel_available: bool,
    ) -> Result<()> {
        ensure!(
            dt.is_finite()
                && (0.0..=0.25).contains(&dt)
                && velocity.is_finite()
                && commands.len() == self.engines.len(),
            "invalid piston timestep or command count"
        );
        ensure!(
            commands.iter().all(|c| c.throttle.is_finite()
                && (0.0..=1.0).contains(&c.throttle)
                && c.mixture.is_finite()
                && (0.0..=1.0).contains(&c.mixture)),
            "invalid piston engine controls"
        );
        let steps = (dt * 240.0).ceil().max(1.0) as u32;
        let h = dt / f64::from(steps);
        for (index, &command) in commands.iter().enumerate() {
            let speed = velocity.dot(self.engines[index].direction).max(0.0);
            let derivative = |rpm: f64| -> Result<f64> {
                let output = self.sample(rpm.max(0.0), air, speed, command, fuel_available)?;
                Ok(
                    (output.cycle.shaft_torque - output.propeller.absorbed_torque)
                        / self.propeller.engine_shaft_inertia()
                        * 30.0
                        / PI,
                )
            };
            let mut rpm = self.engines[index].rpm;
            let mut angle = self.engines[index].angle;
            for _ in 0..steps {
                let a = derivative(rpm)?;
                let b = derivative(rpm + h * a * 0.5)?;
                let c = derivative(rpm + h * b * 0.5)?;
                let d = derivative(rpm + h * c)?;
                let next = (rpm + h * (a + 2.0 * b + 2.0 * c + d) / 6.0).max(0.0);
                angle = (angle
                    + h * (rpm
                        + 2.0 * (rpm + h * a * 0.5).max(0.0)
                        + 2.0 * (rpm + h * b * 0.5).max(0.0)
                        + (rpm + h * c).max(0.0))
                        / 6.0
                        * PI
                        / 30.0
                        / self.propeller.gear_reduction_ratio())
                .rem_euclid(2.0 * PI);
                rpm = next;
                ensure!(rpm.is_finite(), "non-finite piston shaft speed");
            }
            let engine = &mut self.engines[index];
            engine.rpm = rpm;
            engine.angle = angle;
            engine.command = command;
            engine.fuel_available = fuel_available;
        }
        Ok(())
    }

    pub fn wrench(
        &self,
        air: &Atmosphere,
        velocity: DVec3,
        cg: DVec3,
        power_fraction: f64,
    ) -> Result<Wrench> {
        ensure!(
            cg.is_finite() && power_fraction.is_finite() && (0.0..=1.0).contains(&power_fraction),
            "invalid piston CG or power fraction"
        );
        let mut wrench = Wrench::default();
        for (index, engine) in self.engines.iter().enumerate() {
            let output = self.output(index, air, velocity)?;
            let thrust = if output.propeller.thrust > 0.0 {
                output.propeller.thrust * power_fraction
            } else {
                output.propeller.thrust
            };
            let force = engine.direction * thrust;
            wrench.force += force;
            wrench.moment += (engine.position - cg).cross(force);
        }
        Ok(wrench)
    }
}

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

/// Naturally aspirated four-stroke engines with fixed-pitch propellers.
/// Oil temperature, pressure and viscosity use an approximate thermal model.
pub struct Pistons {
    cycle: PistonCycle,
    propeller: Propeller,
    manifold: Curve,
    mixture: Curve,
    mixture_efficiency: Curve,
    mixture_consumption: Curve,
    automatic_mixture: bool,
    automatic_ignition: bool,
    starter_torque: f64,
    rated_power: f64,
    torque_on_roll: f64,
    gyro_pitch: f64,
    gyro_yaw: f64,
    p_factor_on_yaw: f64,
    vacuum_max_pressure: Option<f64>,
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
    oil_temperature: Option<f64>,
    exhaust_temperature: Option<f64>,
    rotation: f64,
    horizontal_wake_overlap: f64,
    vertical_wake_overlap: f64,
    vacuum_pump: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Command {
    pub throttle: f64,
    pub mixture: f64,
    pub fuel_valve: bool,
    pub magnetos: [bool; 2],
    pub starter: bool,
}

impl Default for Command {
    fn default() -> Self {
        Self {
            throttle: 0.0,
            mixture: 1.0,
            fuel_valve: true,
            magnetos: [true; 2],
            starter: false,
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
    pub exhaust_temperature_c: f64,
    pub oil_temperature_c: f64,
    pub oil_pressure_psi: f64,
    pub suction_inhg: Option<f64>,
}

// A qualitative exhaust temperature estimate: peak near stoichiometric mixture,
// falling on both sides of peak. Actual readings depend on probe placement.
fn exhaust_temperature(
    ambient_c: f64,
    ratio: f64,
    manifold_fraction: f64,
    combustion: bool,
) -> f64 {
    if !combustion {
        return ambient_c;
    }
    let mixture_effect = (-((ratio - 0.067) / 0.035).powi(2)).exp();
    ambient_c + 760.0 * mixture_effect * (0.65 + 0.35 * manifold_fraction.clamp(0.0, 1.0))
}
// Approximate the shifted thrust center from crossflow relative to blade-tip
// speed. The quarter-radius bound prevents an unphysical offset near zero RPM.
fn asymmetric_thrust_moment(
    axis: DVec3,
    velocity: DVec3,
    rotation: f64,
    radius: f64,
    prop_rpm: f64,
    thrust: f64,
) -> DVec3 {
    if prop_rpm <= 1e-9 || thrust <= 0.0 {
        return DVec3::ZERO;
    }
    let crossflow = velocity - axis * velocity.dot(axis);
    let tip_speed = radius * prop_rpm * PI / 30.0;
    let displacement = (-axis.cross(crossflow) * (rotation * radius * 0.5 / tip_speed))
        .clamp_length_max(radius * 0.25);
    displacement.cross(axis * thrust)
}
#[derive(Clone, Copy)]
struct TailSurface {
    root: DVec3,
    span: f64,
    vertical: bool,
}

// Integrate uniformly along a simple spanwise tail line. The wake is a
// cylindrical approximation aligned behind the thrust axis.
fn tail_overlap(surface: TailSurface, origin: DVec3, axis: DVec3, radius: f64) -> f64 {
    if surface.span == 0.0 {
        return 0.0;
    }
    let samples = 32;
    let mut hits = 0;
    for i in 0..samples {
        let fraction = (f64::from(i) + 0.5) / f64::from(samples);
        let point = if surface.vertical {
            surface.root - DVec3::Z * (surface.span * fraction)
        } else {
            surface.root + DVec3::Y * (surface.span * (fraction - 0.5))
        };
        let delta = point - origin;
        let longitudinal = delta.dot(axis);
        let radial = delta - axis * longitudinal;
        if longitudinal < 0.0 && radial.length_squared() < radius * radius {
            hits += 1;
        }
    }
    f64::from(hits) / f64::from(samples)
}

fn tail_surface(
    config: &cfg::Config,
    datum: [f64; 3],
    vertical: bool,
) -> Result<Option<TailSurface>> {
    let prefix = if vertical { "vtail" } else { "htail" };
    let span = config.number("airplane_geometry", &format!("{prefix}_span"))?;
    let longitudinal = config.number("airplane_geometry", &format!("{prefix}_pos_lon"))?;
    let height = config.number("airplane_geometry", &format!("{prefix}_pos_vert"))?;
    let (Some(span), Some(longitudinal), Some(height)) = (span, longitudinal, height) else {
        return Ok(None);
    };
    ensure!(
        span.is_finite() && span >= 0.0 && longitudinal.is_finite() && height.is_finite(),
        "invalid tail geometry"
    );
    Ok(Some(TailSurface {
        root: DVec3::new(datum[0] + longitudinal, datum[1], -datum[2] - height) * 0.3048,
        span: span * 0.3048,
        vertical,
    }))
}
fn oil_viscosity(temperature_c: f64) -> f64 {
    (-(temperature_c - 80.0) * 0.006).exp().clamp(0.65, 1.6)
}

fn oil_pressure(rpm: f64, rated_rpm: f64, temperature_c: f64) -> f64 {
    (75.0 * (rpm / rated_rpm).max(0.0).sqrt() * oil_viscosity(temperature_c)).clamp(0.0, 100.0)
}

fn oil_temperature_step(
    previous: f64,
    ambient: f64,
    speed: f64,
    load: f64,
    running: bool,
    dt: f64,
) -> f64 {
    let cooling = 1.0 + speed / 250.0;
    let target = ambient
        + if running {
            (45.0 + 45.0 * load) / cooling
        } else {
            0.0
        };
    let tau = (if running { 600.0 } else { 900.0 }) / cooling;
    previous + (target - previous) * (1.0 - (-dt / tau).exp())
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
        ensure!(
            config
                .number("GeneralEngineData", "starter_type")?
                .unwrap_or(0.0)
                == 0.0,
            "only electric piston starters are implemented yet"
        );
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
        let rotations = config
            .value("propeller", "rotation")?
            .map(|value| {
                value
                    .split(',')
                    .map(|v| v.trim().parse::<f64>())
                    .collect::<std::result::Result<Vec<_>, _>>()
            })
            .transpose()?
            .unwrap_or_default();
        ensure!(
            rotations
                .iter()
                .all(|&direction| direction == 1.0 || direction == -1.0),
            "invalid propeller rotation direction"
        );
        let torque_on_roll = config
            .number("flight_tuning", "torque_on_roll")?
            .unwrap_or(1.0);
        ensure!(
            torque_on_roll.is_finite() && torque_on_roll >= 0.0,
            "invalid torque_on_roll scalar"
        );
        let gyro_pitch = config
            .number("flight_tuning", "gyro_precession_on_pitch")?
            .unwrap_or(1.0);
        let gyro_yaw = config
            .number("flight_tuning", "gyro_precession_on_yaw")?
            .unwrap_or(1.0);
        ensure!(
            [gyro_pitch, gyro_yaw]
                .iter()
                .all(|v| v.is_finite() && *v >= 0.0),
            "invalid gyroscopic tuning scalar"
        );
        let p_factor_on_yaw = config
            .number("flight_tuning", "p_factor_on_yaw")?
            .unwrap_or(1.0);
        ensure!(
            p_factor_on_yaw.is_finite() && p_factor_on_yaw >= 0.0,
            "invalid p_factor_on_yaw scalar"
        );
        let vacuum_type = config
            .number("vacuum_system", "vacuum_type")?
            .unwrap_or(0.0);
        let vacuum_max = config
            .number("vacuum_system", "max_pressure")?
            .unwrap_or(0.0);
        ensure!(
            vacuum_max.is_finite() && vacuum_max >= 0.0,
            "invalid vacuum pressure"
        );
        let vacuum_max_pressure = match vacuum_type {
            0.0 => Some(0.0),
            1.0 => Some(vacuum_max),
            _ => None,
        };
        let vacuum_map = config
            .value("vacuum_system", "engine_map")?
            .map(|value| {
                value
                    .split(',')
                    .map(|v| match v.trim() {
                        "0" => Ok(false),
                        "1" => Ok(true),
                        _ => anyhow::bail!("invalid vacuum engine map"),
                    })
                    .collect::<Result<Vec<_>>>()
            })
            .transpose()?
            .unwrap_or_default();
        let datum = cfg::aircraft::ReferencePoints::from_config(config)?
            .datum_feet
            .context("missing engine reference datum")?;
        let propeller = Propeller::from_data(data)?;
        let horizontal_tail = tail_surface(config, datum, false)?;
        let vertical_tail = tail_surface(config, datum, true)?;
        let engines = cfg::engine::engines(config)?
            .into_iter()
            .map(|definition| -> Result<Engine> {
                let [x, y, z] = definition.position_feet;
                let pitch = definition.pitch_degrees.to_radians();
                let heading = definition.heading_degrees.to_radians();
                let position = DVec3::new(x + datum[0], y + datum[1], -z - datum[2]) * 0.3048;
                let direction = DVec3::new(
                    pitch.cos() * heading.cos(),
                    pitch.cos() * heading.sin(),
                    pitch.sin(),
                );
                let fallback = (1.0 - (position.y.abs() / propeller.radius()).powi(2)).max(0.0);
                let horizontal_wake_overlap = horizontal_tail.map_or(fallback, |tail| {
                    tail_overlap(tail, position, direction, propeller.radius())
                });
                let vertical_wake_overlap = vertical_tail.map_or(fallback, |tail| {
                    tail_overlap(tail, position, direction, propeller.radius())
                });
                Ok(Engine {
                    index: definition
                        .index
                        .checked_add(1)
                        .context("engine index is too large")?,
                    position,
                    direction,
                    rpm: 0.0,
                    angle: 0.0,
                    command: Command::default(),
                    fuel_available: false,
                    oil_temperature: None,
                    exhaust_temperature: None,
                    horizontal_wake_overlap,
                    vertical_wake_overlap,
                    vacuum_pump: vacuum_map
                        .get(definition.index as usize)
                        .copied()
                        .unwrap_or(false),
                    rotation: rotations
                        .get(definition.index as usize)
                        .copied()
                        .unwrap_or(1.0),
                })
            })
            .collect::<Result<Vec<_>>>()?;
        ensure!(!engines.is_empty(), "aircraft has no configured engines");
        let cycle = PistonCycle::from_data(data)?;
        let rated_power = config
            .number("piston_engine", "max_rated_hp")?
            .context("missing piston rated power")?
            * 745.69987158227;
        let starter_scalar = config
            .number("piston_engine", "normalized_starter_torque")?
            .unwrap_or(0.3);
        ensure!(
            rated_power > 0.0 && starter_scalar >= 0.0,
            "invalid starter power or torque scalar"
        );
        let starter_torque = starter_scalar * rated_power / (cycle.maximum_rpm * PI / 30.0);
        Ok(Self {
            cycle,
            propeller,
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
            starter_torque,
            rated_power,
            torque_on_roll,
            gyro_pitch,
            gyro_yaw,
            p_factor_on_yaw,
            vacuum_max_pressure,
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
        oil_temperature_c: f64,
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
        // Approximate minimum cranking speed until ignition timing is simulated.
        let combustion = rpm >= self.cycle.maximum_rpm * 0.04
            && ignition
            && self.mixture_efficiency.sample(ratio) > 0.0;
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
                // Only part of total mechanical friction is oil-dependent.
                oil_viscosity: 1.0 + 0.25 * (oil_viscosity(oil_temperature_c) - 1.0),
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
            exhaust_temperature_c: exhaust_temperature(
                air.temperature - 273.15,
                ratio,
                manifold_pressure / air.pressure,
                combustion,
            ),
            oil_temperature_c: 0.0,
            oil_pressure_psi: 0.0,
            suction_inhg: None,
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
            engine.oil_temperature.unwrap_or(air.temperature - 273.15),
        )?;
        output.propeller_angle = engine.angle;
        output.exhaust_temperature_c = engine
            .exhaust_temperature
            .unwrap_or(air.temperature - 273.15);
        output.oil_temperature_c = engine.oil_temperature.unwrap_or(air.temperature - 273.15);
        output.oil_pressure_psi =
            oil_pressure(engine.rpm, self.cycle.maximum_rpm, output.oil_temperature_c);
        // Approximate an engine-driven pump reaching regulated suction by
        // 1,000 RPM. A windmilling engine can still turn the pump.
        output.suction_inhg = self.vacuum_max_pressure.map(|maximum| {
            if engine.vacuum_pump {
                maximum * (engine.rpm / 1000.0).clamp(0.0, 1.0)
            } else {
                0.0
            }
        });
        Ok(output)
    }

    pub fn start_running(&mut self, air: &Atmosphere, velocity: DVec3) -> Result<()> {
        ensure!(velocity.is_finite(), "invalid piston inlet velocity");
        let mut speeds = Vec::with_capacity(self.engines.len());
        for engine in &self.engines {
            let speed = velocity.dot(engine.direction).max(0.0);
            let torque = |rpm| -> Result<f64> {
                let output = self.sample(
                    rpm,
                    air,
                    speed,
                    Command::default(),
                    true,
                    air.temperature - 273.15 + 60.0,
                )?;
                Ok(output.cycle.shaft_torque - output.propeller.absorbed_torque)
            };
            let (mut lo, mut hi) = (self.cycle.maximum_rpm * 0.04, self.cycle.maximum_rpm * 2.0);
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
            let rpm = (lo + hi) * 0.5;
            let exhaust = self
                .sample(
                    rpm,
                    air,
                    speed,
                    Command::default(),
                    true,
                    air.temperature - 273.15 + 60.0,
                )?
                .exhaust_temperature_c;
            speeds.push((rpm, exhaust));
        }
        for (engine, (rpm, exhaust)) in self.engines.iter_mut().zip(speeds) {
            engine.rpm = rpm;
            engine.command = Command::default();
            engine.fuel_available = true;
            engine.oil_temperature = Some(air.temperature - 273.15 + 60.0);
            engine.exhaust_temperature = Some(exhaust);
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
            let oil_temperature_c = self.engines[index]
                .oil_temperature
                .unwrap_or(air.temperature - 273.15);
            let derivative = |rpm: f64| -> Result<f64> {
                let output = self.sample(
                    rpm.max(0.0),
                    air,
                    speed,
                    command,
                    fuel_available,
                    oil_temperature_c,
                )?;
                Ok(
                    (output.cycle.shaft_torque - output.propeller.absorbed_torque
                        + if command.starter {
                            self.starter_torque
                                * (1.0 - rpm.max(0.0) / (self.cycle.maximum_rpm * 0.15))
                                    .clamp(0.0, 1.0)
                        } else {
                            0.0
                        })
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
            let output =
                self.sample(rpm, air, speed, command, fuel_available, oil_temperature_c)?;
            let ambient = air.temperature - 273.15;
            let load = (output.cycle.shaft_power.max(0.0) / self.rated_power).clamp(0.0, 1.0);
            let engine = &mut self.engines[index];
            // Approximate the thermocouple's finite response with a five-second
            // time constant; the exhaust target itself is a simple estimate.
            let previous_exhaust = engine.exhaust_temperature.unwrap_or(ambient);
            engine.exhaust_temperature = Some(
                previous_exhaust
                    + (output.exhaust_temperature_c - previous_exhaust) * (1.0 - (-dt / 5.0).exp()),
            );
            let initial = engine.oil_temperature.unwrap_or(ambient);
            engine.oil_temperature = Some(oil_temperature_step(
                initial,
                ambient,
                speed,
                load,
                output.combustion,
                dt,
            ));
            engine.rpm = rpm;
            engine.angle = angle;
            engine.command = command;
            engine.fuel_available = fuel_available;
        }
        Ok(())
    }

    /// Approximate axial wake and upper-tail swirl using actuator-disk
    /// momentum theory and approximate tail geometry.
    pub fn tail_wash(
        &self,
        air: &Atmosphere,
        velocity: DVec3,
        power_fractions: &[f64],
    ) -> Result<propeller::TailWash> {
        ensure!(
            velocity.is_finite()
                && air.density.is_finite()
                && air.density > 0.0
                && power_fractions.len() == self.engines.len()
                && power_fractions
                    .iter()
                    .all(|v| v.is_finite() && (0.0..=1.0).contains(v)),
            "invalid propeller wash conditions"
        );
        let radius = self.propeller.radius();
        let area = PI * radius * radius;
        let mut wash = propeller::TailWash::default();
        for (index, engine) in self.engines.iter().enumerate() {
            if engine.horizontal_wake_overlap == 0.0 && engine.vertical_wake_overlap == 0.0 {
                continue;
            }
            let output = self.output(index, air, velocity)?;
            let thrust = output.propeller.thrust.max(0.0) * power_fractions[index];
            let axial_speed = velocity.dot(engine.direction).max(0.0);
            let wake_speed = (axial_speed * axial_speed + 2.0 * thrust / (air.density * area))
                .sqrt()
                - axial_speed;
            wash.horizontal += engine.direction * (wake_speed * engine.horizontal_wake_overlap);
            wash.vertical += engine.direction * (wake_speed * engine.vertical_wake_overlap);
            // Wake swirl follows angular-momentum flux. The representative
            // half-radius and attenuation approximate the upper vertical tail.
            let mass_flow = air.density * area * (axial_speed + wake_speed * 0.5);
            if mass_flow > 1e-9 && thrust > 0.0 {
                let torque = output.propeller.absorbed_torque.max(0.0)
                    * self.propeller.gear_reduction_ratio()
                    * power_fractions[index];
                let tangential = (torque / (mass_flow * radius * 0.5))
                    .min((axial_speed + wake_speed) * 0.5)
                    * 0.05;
                wash.swirl += engine.direction.cross(DVec3::Z)
                    * (engine.rotation * tangential * engine.vertical_wake_overlap);
            }
        }
        ensure!(
            wash.horizontal.is_finite() && wash.vertical.is_finite() && wash.swirl.is_finite(),
            "invalid propeller wake"
        );
        Ok(wash)
    }
    pub fn wrench(
        &self,
        air: &Atmosphere,
        velocity: DVec3,
        angular_velocity: DVec3,
        cg: DVec3,
        power_fractions: &[f64],
    ) -> Result<Wrench> {
        ensure!(
            cg.is_finite()
                && angular_velocity.is_finite()
                && power_fractions.len() == self.engines.len()
                && power_fractions
                    .iter()
                    .all(|v| v.is_finite() && (0.0..=1.0).contains(v)),
            "invalid piston CG or power fraction"
        );
        let mut wrench = Wrench::default();
        for (index, engine) in self.engines.iter().enumerate() {
            let output = self.output(index, air, velocity)?;
            let thrust = if output.propeller.thrust > 0.0 {
                output.propeller.thrust * power_fractions[index]
            } else {
                output.propeller.thrust
            };
            let force = engine.direction * thrust;
            wrench.force += force;
            wrench.moment += (engine.position - cg).cross(force);
            let asymmetry = asymmetric_thrust_moment(
                engine.direction,
                velocity,
                engine.rotation,
                self.propeller.radius(),
                output.propeller.rpm,
                thrust,
            );
            wrench.moment +=
                DVec3::new(asymmetry.x, asymmetry.y, asymmetry.z * self.p_factor_on_yaw);
            // In body coordinates, the airframe reacts to the changing
            // direction of the spinning propeller's angular momentum.
            let rotor_momentum =
                engine.direction * (engine.rotation * self.propeller.angular_momentum(output.rpm));
            let gyro = -angular_velocity.cross(rotor_momentum);
            wrench.moment += DVec3::new(gyro.x, gyro.y * self.gyro_pitch, gyro.z * self.gyro_yaw);
            // The rotor inertia is reflected through the reduction gearbox. Its
            // casing reaction includes the starter and changes sign when the
            // rotating engine is driven by a windmilling propeller.
            let starter = if engine.command.starter {
                self.starter_torque
                    * (1.0 - output.rpm / (self.cycle.maximum_rpm * 0.15)).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let engine_torque = if output.cycle.shaft_torque > 0.0 {
                output.cycle.shaft_torque * power_fractions[index]
            } else {
                output.cycle.shaft_torque
            };
            let shaft_torque = if output.rpm == 0.0 {
                (engine_torque + starter).max(0.0)
            } else {
                engine_torque + starter
            };
            wrench.moment -= engine.direction
                * (engine.rotation
                    * shaft_torque
                    * self.propeller.gear_reduction_ratio()
                    * self.torque_on_roll);
        }
        Ok(wrench)
    }
}

#[cfg(test)]
mod oil_tests {
    use super::*;

    #[test]
    fn pressure_tracks_shaft_speed_and_oil_warms_and_cools() {
        assert_eq!(oil_pressure(0.0, 2700.0, 15.0), 0.0);
        assert!(oil_pressure(1000.0, 2700.0, 80.0) < oil_pressure(2000.0, 2700.0, 80.0));
        assert!(oil_pressure(2000.0, 2700.0, 15.0) > oil_pressure(2000.0, 2700.0, 80.0));
        let warmed = oil_temperature_step(15.0, 15.0, 0.0, 0.7, true, 600.0);
        assert!(warmed > 15.0 && warmed < 100.0);
        let cooled = oil_temperature_step(warmed, 15.0, 50.0, 0.0, false, 600.0);
        assert!(cooled > 15.0 && cooled < warmed);
        assert_eq!(
            oil_temperature_step(warmed, 15.0, 0.0, 0.7, true, 0.0),
            warmed
        );
    }
}

#[cfg(test)]
mod exhaust_tests {
    use super::exhaust_temperature;

    #[test]
    fn exhaust_temperature_peaks_near_stoichiometric_and_drops_without_combustion() {
        let peak = exhaust_temperature(15.0, 0.067, 0.8, true);
        assert!(peak > exhaust_temperature(15.0, 0.05, 0.8, true));
        assert!(peak > exhaust_temperature(15.0, 0.085, 0.8, true));
        assert!(peak > exhaust_temperature(15.0, 0.067, 0.3, true));
        assert_eq!(exhaust_temperature(15.0, 0.067, 0.8, false), 15.0);
    }
}

#[cfg(test)]
mod asymmetry_tests {
    use super::*;

    #[test]
    fn crossflow_moves_thrust_center_and_reverses_with_propeller_direction() {
        let axis = DVec3::X;
        let straight = DVec3::new(40.0, 0.0, 0.0);
        assert_eq!(
            asymmetric_thrust_moment(axis, straight, 1.0, 1.0, 2000.0, 1000.0),
            DVec3::ZERO
        );
        let pitched = straight + DVec3::Z * 5.0;
        let clockwise = asymmetric_thrust_moment(axis, pitched, 1.0, 1.0, 2000.0, 1000.0);
        let reverse = asymmetric_thrust_moment(axis, pitched, -1.0, 1.0, 2000.0, 1000.0);
        assert!(clockwise.z < 0.0);
        assert!((clockwise + reverse).length() < 1e-12);
        assert_eq!(
            asymmetric_thrust_moment(axis, pitched, 1.0, 1.0, 0.0, 1000.0),
            DVec3::ZERO
        );
        assert_eq!(
            asymmetric_thrust_moment(axis, pitched, 1.0, 1.0, 2000.0, 0.0),
            DVec3::ZERO
        );
        assert!(
            asymmetric_thrust_moment(axis, pitched * 1e6, 1.0, 1.0, 1.0, 1000.0).length()
                <= 250.0 + 1e-9
        );
    }
}

#[cfg(test)]
mod wake_geometry_tests {
    use super::*;

    #[test]
    fn tail_coverage_accounts_for_span_offset_height_and_engine_direction() {
        let tail = TailSurface {
            root: DVec3::new(-5.0, 0.0, 0.0),
            span: 4.0,
            vertical: false,
        };
        let centered = tail_overlap(tail, DVec3::ZERO, DVec3::X, 1.0);
        assert_eq!(centered, 0.5);
        assert_eq!(
            tail_overlap(tail, DVec3::new(-10.0, 0.0, 0.0), DVec3::X, 1.0),
            0.0
        );
        assert_eq!(
            tail_overlap(tail, DVec3::new(0.0, 0.0, 2.0), DVec3::X, 1.0),
            0.0
        );
        assert!(tail_overlap(tail, DVec3::new(0.0, 1.5, 0.0), DVec3::X, 1.0) > 0.0);
        let vertical = TailSurface {
            root: DVec3::new(-5.0, 0.0, 0.0),
            span: 2.0,
            vertical: true,
        };
        assert_eq!(tail_overlap(vertical, DVec3::ZERO, DVec3::X, 1.0), 0.5);
    }
}

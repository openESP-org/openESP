use anyhow::{Context, Result, ensure};
use glam::DVec3;

use crate::{
    atmosphere::Atmosphere,
    flight::Wrench,
    lookup::{Curve, Grid},
};

const POUND: f64 = 0.45359237;
const POUND_FORCE: f64 = 4.4482216152605;

pub struct TurbineEngine {
    pub index: u32,
    pub position: DVec3,
    pub direction: DVec3,
}

pub struct TurbineParameters {
    pub n1: Grid,
    pub low_n2: Grid,
    pub high_n2: Grid,
    pub spool: Curve,
    pub thrust: Grid,
    pub airflow: Grid,
    pub static_thrust: f64,
    pub inlet_area: f64,
    pub thrust_scalar: f64,
    pub fuel_scalar: f64,
    pub consumption: f64,
    pub gain: f64,
    pub throttle_lower_limit: f64,
    pub reverse_scalar: f64,
    pub engines: Vec<TurbineEngine>,
}
pub struct Turbines {
    n1: Grid,
    low_n2: Grid,
    high_n2: Grid,
    spool: Curve,
    thrust: Grid,
    airflow: Grid,
    static_thrust: f64,
    inlet_area: f64,
    thrust_scalar: f64,
    fuel_scalar: f64,
    consumption: f64,
    gain: f64,
    throttle_lower_limit: f64,
    reverse_scalar: f64,
    engines: Vec<Engine>,
}

struct Engine {
    index: u32,
    position: DVec3,
    direction: DVec3,
    coordinate: f64,
    lit: bool,
    reversing: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Command {
    pub throttle: f64,
    pub fuel_valve: bool,
    pub starter: bool,
    pub ignition: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Output {
    pub n1: f64,
    pub n2: f64,
    pub gross_thrust: f64,
    pub ram_drag: f64,
    pub net_thrust: f64,
    pub fuel_kilograms_per_second: f64,
    pub combustion: bool,
    pub reverse_nozzle: f64,
}

impl Turbines {
    pub fn new(parameters: TurbineParameters) -> Result<Self> {
        let TurbineParameters {
            n1,
            low_n2,
            high_n2,
            spool,
            thrust,
            airflow,
            static_thrust,
            inlet_area,
            thrust_scalar,
            fuel_scalar,
            consumption,
            gain,
            throttle_lower_limit,
            reverse_scalar,
            engines,
        } = parameters;
        ensure!(
            spool.outputs_nondecreasing(),
            "non-monotonic turbine spool table"
        );
        ensure!(!engines.is_empty(), "aircraft has no configured engines");
        let coordinate = spool.first_input();
        let engines = engines
            .into_iter()
            .map(|engine| Engine {
                index: engine.index,
                position: engine.position,
                direction: engine.direction,
                coordinate,
                lit: false,
                reversing: false,
            })
            .collect();
        let model = Self {
            n1,
            low_n2,
            high_n2,
            spool,
            thrust,
            airflow,
            static_thrust,
            inlet_area,
            thrust_scalar,
            fuel_scalar,
            consumption,
            gain,
            throttle_lower_limit,
            reverse_scalar,
            engines,
        };
        ensure!(
            model.low_n2.corner < model.high_n2.corner && model.low_n2.corner >= 0.0,
            "invalid commanded N2 Mach range"
        );
        ensure!(
            model.static_thrust > 0.0 && model.gain > 0.0,
            "invalid rated thrust or fuel flow gain"
        );
        ensure!(
            model.throttle_lower_limit.is_finite()
                && (-1.0..=0.0).contains(&model.throttle_lower_limit)
                && model.reverse_scalar.is_finite()
                && model.reverse_scalar >= 0.0,
            "invalid thrust reverser configuration"
        );
        ensure!(
            [
                model.static_thrust,
                model.inlet_area,
                model.thrust_scalar,
                model.fuel_scalar,
                model.consumption,
                model.gain
            ]
            .iter()
            .all(|v| v.is_finite() && *v >= 0.0),
            "invalid engine parameters"
        );
        Ok(model)
    }

    pub fn indices(&self) -> impl Iterator<Item = u32> + '_ {
        self.engines.iter().map(|e| e.index)
    }

    pub fn throttle_lower_limit(&self) -> f64 {
        if self.reverse_scalar > 0.0 {
            self.throttle_lower_limit
        } else {
            0.0
        }
    }

    fn target_n2(&self, throttle: f64, air: &Atmosphere, speed: f64) -> f64 {
        let mach = speed / air.sound_speed;
        let inverse_pressure = 101325.0 / air.pressure;
        let low = self.low_n2.sample(throttle, inverse_pressure);
        let high = self.high_n2.sample(throttle, inverse_pressure);
        let alpha = ((mach - self.low_n2.corner) / (self.high_n2.corner - self.low_n2.corner))
            .clamp(0.0, 1.0);
        low + (high - low) * alpha
    }

    pub fn start_running(&mut self, air: &Atmosphere, speed: f64) -> Result<()> {
        validate_air(air, speed)?;
        let coordinate = self.spool.inverse(self.target_n2(0.0, air, speed));
        for engine in &mut self.engines {
            engine.coordinate = coordinate;
            engine.lit = true;
        }
        Ok(())
    }

    pub fn advance(
        &mut self,
        dt: f64,
        air: &Atmosphere,
        speed: f64,
        commands: &[Command],
        fuel_available: bool,
    ) -> Result<()> {
        validate_air(air, speed)?;
        ensure!(
            dt.is_finite() && dt >= 0.0 && commands.len() == self.engines.len(),
            "invalid engine timestep or command count"
        );
        ensure!(
            commands.iter().all(|c| c.throttle.is_finite()
                && (self.throttle_lower_limit()..=1.0).contains(&c.throttle)),
            "invalid engine throttle"
        );
        let travel = self.gain * (2048.0 / 60.0) * dt;
        for (i, command) in commands.iter().enumerate() {
            let reversing = command.throttle < 0.0;
            let throttle = if reversing {
                command.throttle / self.throttle_lower_limit
            } else {
                command.throttle
            };
            let mach = speed / air.sound_speed;
            let theta = air.temperature / 288.15 * (1.0 + 0.2 * mach * mach);
            let current_n2 = self.spool.sample(self.engines[i].coordinate) * theta.sqrt();
            // Starter torque and ignition thresholds are approximated until pneumatic
            // supply and engine-specific start schedules are available.
            let lit = (self.engines[i].lit || (command.ignition && current_n2 >= 19.4))
                && fuel_available
                && command.fuel_valve;
            let target = if lit {
                self.spool.inverse(self.target_n2(throttle, air, speed))
            } else if command.starter {
                self.spool.inverse(25.0 / theta.sqrt())
            } else {
                self.spool.0[0].input
            };
            let engine = &mut self.engines[i];
            engine.lit = lit;
            engine.reversing = reversing;
            if command.starter && !lit {
                let next_n2 = (current_n2 + dt).min(25.0);
                let start_coordinate = self.spool.inverse(next_n2 / theta.sqrt());
                engine.coordinate = engine
                    .coordinate
                    .max(start_coordinate)
                    .min(target.max(engine.coordinate));
            } else {
                engine.coordinate += (target - engine.coordinate).clamp(-travel, travel);
            }
        }
        Ok(())
    }

    pub fn output(&self, index: usize, air: &Atmosphere, speed: f64) -> Result<Output> {
        validate_air(air, speed)?;
        let engine = self.engines.get(index).context("engine is absent")?;
        let mach = speed / air.sound_speed;
        let ram_temperature = 1.0 + 0.2 * mach * mach;
        let theta = air.temperature / 288.15 * ram_temperature;
        let delta = air.pressure / 101325.0 * ram_temperature.powf(3.5);
        let cn2 = self.spool.sample(engine.coordinate);
        let cn1 = self.n1.sample(cn2, mach);
        let gross = if engine.lit {
            self.static_thrust * self.thrust.sample(cn1, mach) * delta
        } else {
            0.0
        };
        let ram_drag =
            speed * self.inlet_area * self.airflow.sample(cn1, mach) * delta / theta.sqrt() * POUND;
        let output = Output {
            n1: cn1 * theta.sqrt(),
            n2: cn2 * theta.sqrt(),
            gross_thrust: gross * self.thrust_scalar,
            ram_drag: ram_drag * self.thrust_scalar,
            net_thrust: (gross
                * if engine.reversing {
                    -self.reverse_scalar
                } else {
                    1.0
                }
                - ram_drag)
                * self.thrust_scalar,
            fuel_kilograms_per_second: gross / POUND_FORCE * self.consumption * self.fuel_scalar
                / theta.sqrt()
                * POUND
                / 3600.0,
            combustion: engine.lit,
            reverse_nozzle: f64::from(engine.reversing),
        };
        ensure!(
            [
                output.n1,
                output.n2,
                output.gross_thrust,
                output.ram_drag,
                output.net_thrust,
                output.fuel_kilograms_per_second
            ]
            .iter()
            .all(|v| v.is_finite()),
            "invalid turbine output"
        );
        ensure!(
            output.fuel_kilograms_per_second >= 0.0,
            "negative turbine fuel consumption"
        );
        Ok(output)
    }

    pub fn wrench(
        &self,
        air: &Atmosphere,
        velocity: DVec3,
        cg: DVec3,
        power_fractions: &[f64],
    ) -> Result<Wrench> {
        ensure!(
            velocity.is_finite()
                && cg.is_finite()
                && power_fractions.len() == self.engines.len()
                && power_fractions
                    .iter()
                    .all(|v| v.is_finite() && (0.0..=1.0).contains(v)),
            "invalid engine flow or CG"
        );
        let mut wrench = Wrench::default();
        for (index, engine) in self.engines.iter().enumerate() {
            let speed = velocity.length();
            let output = self.output(index, air, speed)?;
            let drag = if speed > 0.0 {
                velocity / speed * output.ram_drag
            } else {
                DVec3::ZERO
            };
            let direction = if engine.reversing {
                -self.reverse_scalar
            } else {
                1.0
            };
            let force =
                engine.direction * output.gross_thrust * power_fractions[index] * direction - drag;
            wrench.force += force;
            wrench.moment += (engine.position - cg).cross(force);
        }
        Ok(wrench)
    }
}

fn validate_air(air: &Atmosphere, speed: f64) -> Result<()> {
    ensure!(
        [air.temperature, air.pressure, air.sound_speed]
            .iter()
            .all(|v| v.is_finite() && *v > 0.0)
            && speed.is_finite()
            && speed >= 0.0,
        "invalid turbine inlet conditions"
    );
    Ok(())
}

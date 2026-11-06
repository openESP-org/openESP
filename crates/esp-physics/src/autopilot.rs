use anyhow::{Result, ensure};
use esp_script::{Error, Reference};

use crate::{flight::State, variables::VariableStore};

const FEET: f64 = 0.3048;
const FEET_PER_MINUTE: f64 = 0.3048 / 60.0;

#[derive(Debug, Clone, Copy, Default)]
pub struct Inputs {
    pub heading: f64,
    pub pitch: f64,
    pub bank: f64,
    pub altitude: f64,
    pub vertical_speed: f64,
    pub roll_rate: f64,
    pub pitch_rate: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct Output {
    pub aileron: Option<f64>,
    pub elevator: Option<f64>,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
enum PitchMode {
    #[default]
    Off,
    Altitude,
    VerticalSpeed,
}

#[derive(Debug, Default)]
pub struct Autopilot {
    pitch_mode: PitchMode,
    pitch_integral: f64,
}

impl Inputs {
    pub fn from_state(state: &State) -> Result<Self> {
        use glam::EulerRot;
        let (heading, pitch, bank) = state.orientation.to_euler(EulerRot::ZYX);
        let inputs = Self {
            heading: heading.to_degrees().rem_euclid(360.0),
            pitch: pitch.to_degrees(),
            bank: bank.to_degrees(),
            altitude: -state.position.z,
            vertical_speed: -state.velocity.z,
            roll_rate: state.angular_velocity.x.to_degrees(),
            pitch_rate: state.angular_velocity.y.to_degrees(),
        };
        ensure!(
            [
                inputs.heading,
                inputs.pitch,
                inputs.bank,
                inputs.altitude,
                inputs.vertical_speed,
                inputs.roll_rate,
                inputs.pitch_rate,
            ]
            .iter()
            .all(|value| value.is_finite()),
            "invalid autopilot inputs"
        );
        Ok(inputs)
    }
}

impl Autopilot {
    pub fn configure(variables: &mut dyn VariableStore, altitude: f64) -> Result<()> {
        ensure!(altitude.is_finite(), "invalid autopilot altitude");
        for name in [
            "AUTOPILOT MASTER",
            "AUTOPILOT HEADING LOCK",
            "AUTOPILOT ALTITUDE LOCK",
            "AUTOPILOT VERTICAL HOLD",
            "AUTOPILOT WING LEVELER",
            "AUTOPILOT FLIGHT DIRECTOR ACTIVE",
        ] {
            variables.set_number(name, "bool", 0.0);
        }
        variables.set_number("AUTOPILOT MAX BANK", "degrees", 25.0);
        variables.set_number("AUTOPILOT ALTITUDE LOCK VAR", "feet", altitude / FEET);
        variables.set_number("AUTOPILOT VERTICAL HOLD VAR", "feet per minute", 0.0);
        Ok(())
    }

    pub fn update(
        &mut self,
        dt: f64,
        inputs: Inputs,
        variables: &mut dyn VariableStore,
    ) -> Result<Output> {
        ensure!(dt.is_finite() && dt >= 0.0, "invalid autopilot timestep");
        let master = Self::boolean(variables, "AUTOPILOT MASTER")? != 0.0;
        let powered = Self::powered(variables)?;
        if master && !powered {
            self.disengage(variables);
        }
        if !master || !powered {
            self.pitch_mode = PitchMode::Off;
            self.pitch_integral = 0.0;
            return Ok(Output::default());
        }

        let mut output = Output::default();
        let heading_hold = Self::boolean(variables, "AUTOPILOT HEADING LOCK")? != 0.0;
        let wing_leveler = Self::boolean(variables, "AUTOPILOT WING LEVELER")? != 0.0;
        if heading_hold || wing_leveler {
            let desired_bank = if heading_hold {
                let variation = Self::number(variables, "MAGVAR", "degrees")?.unwrap_or(0.0);
                let drift = Self::number(variables, "GYRO DRIFT ERROR", "degrees")?.unwrap_or(0.0);
                let heading = (inputs.heading + variation + drift).rem_euclid(360.0);
                let selected = Self::number(variables, "AUTOPILOT HEADING LOCK DIR", "degrees")?
                    .unwrap_or(heading)
                    .rem_euclid(360.0);
                let error = wrap_degrees(selected - heading);
                let max_bank = Self::number(variables, "AUTOPILOT MAX BANK", "degrees")?
                    .unwrap_or(25.0)
                    .clamp(5.0, 45.0);
                (error * 2.0).clamp(-max_bank, max_bank)
            } else {
                0.0
            };
            output.aileron = Some(bank_aileron(inputs, desired_bank));
        }

        let alt_hold = Self::boolean(variables, "AUTOPILOT ALTITUDE LOCK")? != 0.0;
        let vs_hold = Self::boolean(variables, "AUTOPILOT VERTICAL HOLD")? != 0.0;
        let pitch_mode = if alt_hold {
            PitchMode::Altitude
        } else if vs_hold {
            PitchMode::VerticalSpeed
        } else {
            PitchMode::Off
        };
        if pitch_mode != self.pitch_mode {
            self.pitch_mode = pitch_mode;
            self.pitch_integral = 0.0;
        }
        let target_vertical_speed = match pitch_mode {
            PitchMode::Off => None,
            PitchMode::Altitude => {
                let target = Self::number(variables, "AUTOPILOT ALTITUDE LOCK VAR", "meters")?
                    .unwrap_or(inputs.altitude);
                let error = target - inputs.altitude;
                Some(if error.abs() < 2.0 {
                    0.0
                } else {
                    (error * 0.15).clamp(-3.0, 3.0)
                })
            }
            PitchMode::VerticalSpeed => Some(
                Self::number(
                    variables,
                    "AUTOPILOT VERTICAL HOLD VAR",
                    "meters per second",
                )?
                .unwrap_or(0.0)
                .clamp(-15.0, 15.0),
            ),
        };
        if let Some(target) = target_vertical_speed {
            let error = (target - inputs.vertical_speed).clamp(-30.0, 30.0);
            self.pitch_integral = (self.pitch_integral + error * dt * 0.01).clamp(-0.35, 0.35);
            let command = 0.12 * error + self.pitch_integral - 0.02 * inputs.pitch_rate;
            output.elevator = Some(command.clamp(-1.0, 1.0));
        }
        Ok(output)
    }

    fn disengage(&mut self, variables: &mut dyn VariableStore) {
        for name in [
            "AUTOPILOT MASTER",
            "AUTOPILOT HEADING LOCK",
            "AUTOPILOT ALTITUDE LOCK",
            "AUTOPILOT VERTICAL HOLD",
            "AUTOPILOT WING LEVELER",
        ] {
            variables.set_number(name, "bool", 0.0);
        }
        self.pitch_mode = PitchMode::Off;
        self.pitch_integral = 0.0;
    }

    fn powered(variables: &mut dyn VariableStore) -> Result<bool> {
        if let Some(value) = Self::number(variables, "CIRCUIT AUTOPILOT ON", "bool")? {
            return Ok(value != 0.0);
        }
        if let Some(value) = Self::number(variables, "ELECTRICAL AVIONICS BUS VOLTAGE", "volts")? {
            return Ok(value > 0.0);
        }
        Ok(true)
    }

    fn boolean(variables: &mut dyn VariableStore, name: &str) -> Result<f64> {
        Ok(Self::number(variables, name, "bool")?.unwrap_or(0.0))
    }

    fn number(variables: &mut dyn VariableStore, name: &str, unit: &str) -> Result<Option<f64>> {
        match variables.read(&Reference::new('A', name, Some(unit))) {
            Ok(value) => Ok(Some(value.number()?)),
            Err(Error::Variable(_)) => Ok(None),
            Err(error) => Err(error.into()),
        }
    }
}

fn wrap_degrees(degrees: f64) -> f64 {
    (degrees + 180.0).rem_euclid(360.0) - 180.0
}

fn bank_aileron(inputs: Inputs, desired_bank: f64) -> f64 {
    let bank_error = desired_bank - inputs.bank;
    let desired_rate = (bank_error * 1.5).clamp(-15.0, 15.0);
    let rate_error = desired_rate - inputs.roll_rate;
    (rate_error * 0.04).clamp(-1.0, 1.0)
}

pub fn selected_altitude_increment(current: f64, increment: f64) -> Result<f64> {
    ensure!(
        current.is_finite() && increment.is_finite(),
        "invalid selected altitude"
    );
    Ok((current + increment).clamp(-1000.0, 100_000.0))
}

pub fn selected_vertical_speed_increment(current: f64, increment: f64) -> Result<f64> {
    ensure!(
        current.is_finite() && increment.is_finite(),
        "invalid selected vertical speed"
    );
    Ok((current + increment).clamp(-6000.0, 6000.0))
}

pub fn selected_altitude_meters(meters: f64) -> Result<f64> {
    ensure!(meters.is_finite(), "invalid selected altitude");
    Ok(meters.clamp(-1000.0 * FEET, 100_000.0 * FEET) / FEET)
}

pub fn selected_vertical_speed_meters_per_second(meters_per_second: f64) -> Result<f64> {
    ensure!(
        meters_per_second.is_finite(),
        "invalid selected vertical speed"
    );
    Ok(
        meters_per_second.clamp(-6000.0 * FEET_PER_MINUTE, 6000.0 * FEET_PER_MINUTE)
            / FEET_PER_MINUTE,
    )
}

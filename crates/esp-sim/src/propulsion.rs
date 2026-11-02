use anyhow::{Result, bail};
use esp_assets::flight::FlightData;
use glam::DVec3;

use crate::{atmosphere::Atmosphere, flight::Wrench, piston, turbine};

pub enum Propulsion {
    Piston {
        model: Box<piston::Pistons>,
        commands: Vec<piston::Command>,
    },
    Turbine {
        model: Box<turbine::Turbines>,
        commands: Vec<turbine::Command>,
    },
}

#[derive(Clone, Copy, Debug)]
pub struct Command {
    pub throttle: f64,
    pub fuel_valve: bool,
    pub mixture: f64,
    pub magnetos: [bool; 2],
}

#[derive(Clone, Copy, Debug)]
pub enum Output {
    Piston(piston::EngineOutput),
    Turbine(turbine::Output),
}

impl Output {
    pub fn fuel_kilograms_per_second(&self) -> f64 {
        match self {
            Self::Piston(output) => output.fuel_kilograms_per_second,
            Self::Turbine(output) => output.fuel_kilograms_per_second,
        }
    }

    pub fn combustion(&self) -> bool {
        match self {
            Self::Piston(output) => output.combustion,
            Self::Turbine(output) => output.combustion,
        }
    }
}

impl Propulsion {
    pub fn from_data(data: &FlightData) -> Result<Self> {
        match data
            .configuration
            .number("GeneralEngineData", "engine_type")?
        {
            Some(0.0) => Ok(Self::Piston {
                model: Box::new(piston::Pistons::from_data(data)?),
                commands: Vec::new(),
            }),
            Some(1.0) => Ok(Self::Turbine {
                model: Box::new(turbine::Turbines::from_data(data)?),
                commands: Vec::new(),
            }),
            kind => bail!("unsupported running engine type: {kind:?}"),
        }
    }

    pub fn is_piston(&self) -> bool {
        matches!(self, Self::Piston { .. })
    }

    pub fn indices(&self) -> impl Iterator<Item = u32> + '_ {
        let (piston, turbine) = match self {
            Self::Piston { model, .. } => (Some(model), None),
            Self::Turbine { model, .. } => (None, Some(model)),
        };
        piston
            .into_iter()
            .flat_map(|p| p.indices())
            .chain(turbine.into_iter().flat_map(|t| t.indices()))
    }

    pub fn start_running(&mut self, air: &Atmosphere, velocity: DVec3) -> Result<()> {
        match self {
            Self::Piston { model, .. } => model.start_running(air, velocity),
            Self::Turbine { model, .. } => model.start_running(air, velocity.length()),
        }
    }

    pub fn advance(
        &mut self,
        dt: f64,
        air: &Atmosphere,
        velocity: DVec3,
        controls: &[Command],
        fuel_available: bool,
    ) -> Result<()> {
        match self {
            Self::Piston { model, commands } => {
                commands.clear();
                commands.extend(controls.iter().map(|c| piston::Command {
                    throttle: c.throttle,
                    fuel_valve: c.fuel_valve,
                    mixture: c.mixture,
                    magnetos: c.magnetos,
                }));
                model.advance(dt, air, velocity, commands, fuel_available)
            }
            Self::Turbine { model, commands } => {
                commands.clear();
                commands.extend(controls.iter().map(|c| turbine::Command {
                    throttle: c.throttle,
                    fuel_valve: c.fuel_valve,
                }));
                model.advance(dt, air, velocity.length(), commands, fuel_available)
            }
        }
    }

    pub fn output(&self, index: usize, air: &Atmosphere, velocity: DVec3) -> Result<Output> {
        match self {
            Self::Piston { model, .. } => Ok(Output::Piston(model.output(index, air, velocity)?)),
            Self::Turbine { model, .. } => Ok(Output::Turbine(model.output(
                index,
                air,
                velocity.length(),
            )?)),
        }
    }

    pub fn wrench(
        &self,
        air: &Atmosphere,
        velocity: DVec3,
        cg: DVec3,
        power_fraction: f64,
    ) -> Result<Wrench> {
        match self {
            Self::Piston { model, .. } => model.wrench(air, velocity, cg, power_fraction),
            Self::Turbine { model, .. } => model.wrench(air, velocity, cg, power_fraction),
        }
    }
}

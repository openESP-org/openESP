pub use self::state::SessionState;
use crate::autopilot::{Autopilot, Inputs as AutopilotInputs};
use crate::hydraulics::Hydraulics;
use crate::loading::{Loading, WeightBalance};
use crate::variables::Variables;
use anyhow::{Context, Result, ensure};
use esp_assets::flight::FlightData;
use esp_physics::{
    aerodynamics::{Aerodynamics, Controls},
    atmosphere::Atmosphere,
    flight::Motion as State,
    ground::{Controls as GroundControls, LandingGear, Surface},
    propulsion::{Command, Output as EngineOutput, Propulsion},
};
use esp_script::{Host, Reference};
use glam::{DMat3, DQuat, DVec3, EulerRot, Quat};

#[cfg(test)]
#[path = "../../../../tests/support/content.rs"]
mod content;

pub struct Flight {
    model: Aerodynamics,
    loading: Loading,
    weights: WeightBalance,
    payload: bool,
    engines: Option<Propulsion>,
    commands: Vec<Command>,
    fuel_selectors: u32,
    master_ignition_available: bool,
    pub current: SessionState,
    previous: SessionState,
    previous_cg: DVec3,
    flaps: Vec<f64>,
    spoiler_surfaces: [f64; 2],
    spoiler_extension_time: f64,
    spoiler_system_type: Option<u32>,
    auto_spoiler_available: bool,
    auto_spoiler_min_speed: f64,
    auto_spoiler_auto_retracts: bool,
    spoiler_airborne: bool,
    spoiler_auto_deployed: bool,
    gear: LandingGear,
    hydraulics: Option<Hydraulics>,
    hydraulic_boost: [f64; 3],
    aileron_up_limit: f64,
    autopilot: Autopilot,
    on_ground: bool,
    ground: Option<ground::Ground>,
}

pub mod ground;

// Physical surface properties of the local test grid. Scenery surfaces will
// supply their own coefficients when terrain contact is connected.
const TEST_SURFACE: Surface = Surface {
    elevation: 0.0,
    rolling_resistance: 0.02,
    tyre_friction: 0.8,
};

impl Flight {
    pub fn set_ground(&mut self, ground: ground::Ground) {
        self.ground = Some(ground);
    }

    pub fn set_terrain(&mut self, mesh: std::sync::Arc<esp_terrain::mesh::Mesh>) {
        if let Some(ground) = &mut self.ground {
            ground.set_terrain(mesh);
        }
    }

    fn surface_at(&self, state: &State) -> Surface {
        self.ground.as_ref().map_or(TEST_SURFACE, |ground| {
            ground.surface(state.position.x, state.position.y)
        })
    }

    pub fn new(
        data: &FlightData,
        knots: f64,
        height: f64,
        fuel_fraction: f64,
        payload: bool,
    ) -> Result<Self> {
        ensure!(
            knots.is_finite()
                && knots >= 0.0
                && height.is_finite()
                && height >= 0.0
                && (height == 0.0 || knots > 0.0),
            "invalid flight starting state"
        );
        let model = crate::loading::aerodynamic_model(data)?;
        let aileron_up_limit = data
            .configuration
            .number("airplane_geometry", "aileron_up_limit")?
            .unwrap_or(20.0);
        let hydraulics = Hydraulics::from_config(&data.configuration)?;
        let boost = |key| -> Result<f64> {
            let value = data
                .configuration
                .number("airplane_geometry", key)?
                .unwrap_or(0.0);
            ensure!((0.0..=1.0).contains(&value), "invalid {key}");
            Ok(value)
        };
        let hydraulic_boost = [
            boost("elevator_hydraulic_boost_percent")?,
            boost("aileron_hydraulic_boost_percent")?,
            boost("rudder_hydraulic_boost_percent")?,
        ];
        let spoiler_extension_time = data
            .configuration
            .number("airplane_geometry", "spoiler_extension_time")?
            .unwrap_or(0.0);
        let spoiler_type = data
            .configuration
            .number("airplane_geometry", "spoiler_system_type")?;
        ensure!(
            spoiler_type.is_none_or(|kind| kind.fract() == 0.0 && (0.0..=4.0).contains(&kind)),
            "invalid spoiler system type"
        );
        let spoiler_system_type = spoiler_type.map(|kind| kind as u32);
        let auto_spoiler_available = data
            .configuration
            .number("airplane_geometry", "auto_spoiler_available")?
            .unwrap_or(0.0)
            != 0.0;
        let auto_spoiler_min_speed = data
            .configuration
            .number("airplane_geometry", "auto_spoiler_min_speed")?
            .unwrap_or(0.0);
        let auto_spoiler_auto_retracts = data
            .configuration
            .number("airplane_geometry", "auto_spoiler_auto_retracts")?
            .unwrap_or(1.0)
            != 0.0;
        ensure!(
            spoiler_extension_time >= 0.0 && auto_spoiler_min_speed >= 0.0,
            "invalid spoiler extension time"
        );
        let weights = WeightBalance::from_config(&data.configuration)?;
        let master_ignition = data
            .configuration
            .number("GeneralEngineData", "master_ignition_switch")?
            .unwrap_or(0.0);
        ensure!(
            master_ignition == 0.0 || master_ignition == 1.0,
            "invalid master ignition switch configuration"
        );
        let fuel_selectors = data
            .configuration
            .number("fuel", "number_of_tank_selectors")?
            .unwrap_or(1.0);
        ensure!(
            fuel_selectors.fract() == 0.0 && (0.0..=4.0).contains(&fuel_selectors),
            "invalid fuel selector count"
        );
        let loading = weights.load(fuel_fraction, payload)?;
        let previous_cg = loading.cg;
        Atmosphere::standard(height)?;
        let gear = crate::loading::landing_gear(data)?;
        let mut current: SessionState = State {
            position: DVec3::new(0.0, 0.0, -height),
            velocity: DVec3::X * (knots * 1852.0 / 3600.0),
            ..State::default()
        }
        .into();
        if height == 0.0 {
            let pitch = data
                .configuration
                .number("contact_points", "static_pitch")?
                .context("missing ground starting pitch")?
                .to_radians();
            let cg_height = data
                .configuration
                .number("contact_points", "static_cg_height")?
                .context("missing ground starting height")?
                * 0.3048;
            current.position.z = -cg_height;
            current.orientation = DQuat::from_rotation_y(pitch);
        }
        let on_ground = gear
            .forces(
                &current,
                loading.cg,
                TEST_SURFACE,
                GroundControls::default(),
            )?
            .on_ground;
        Ok(Self {
            model,
            loading,
            weights,
            payload,
            engines: None,
            commands: Vec::new(),
            fuel_selectors: fuel_selectors as u32,
            master_ignition_available: master_ignition != 0.0,
            previous_cg,
            current,
            previous: current,
            flaps: Vec::new(),
            spoiler_surfaces: [0.0; 2],
            spoiler_extension_time,
            spoiler_system_type,
            auto_spoiler_available,
            auto_spoiler_min_speed,
            auto_spoiler_auto_retracts,
            spoiler_airborne: !on_ground,
            spoiler_auto_deployed: false,
            gear,
            hydraulics,
            hydraulic_boost,
            aileron_up_limit,
            autopilot: Autopilot::default(),
            on_ground,
            ground: None,
        })
    }

    pub fn configure_ground(variables: &mut Variables) {
        variables.set_number("ELEVATOR TRIM PCT", "percent", 0.0);
        variables.set_number("AILERON TRIM PCT", "percent", 0.0);
        variables.set_number("RUDDER TRIM PCT", "percent", 0.0);
        variables.set_number("GEAR HANDLE POSITION", "bool", 1.0);
        variables.set_number("HYDRAULIC SWITCH", "bool", 0.0);
        variables.set_number("HYDRAULIC FAILURE", "bool", 0.0);
        variables.set_number("SPOILERS ARMED", "bool", 0.0);
        variables.set_number("ELEVATOR TRIM POSITION", "radians", 0.0);
        for name in [
            "SPOILERS HANDLE POSITION",
            "SPOILERS LEFT POSITION",
            "SPOILERS RIGHT POSITION",
        ] {
            variables.set_number(name, "percent over 100", 0.0);
        }
        for name in [
            "BRAKE LEFT POSITION",
            "BRAKE RIGHT POSITION",
            "BRAKE PARKING POSITION",
        ] {
            variables.set_number(name, "percent over 100", 0.0);
        }
    }

    pub fn configure_controls(&self, variables: &mut Variables) -> Result<()> {
        Autopilot::configure(variables, -self.current.position.z)?;
        variables.configure_aileron_travel(self.aileron_up_limit)
    }

    pub fn add_initial_wind(&mut self, wind: DVec3) {
        self.current.velocity += wind;
        self.previous = self.current;
    }

    pub fn place_initial(&mut self, placement: crate::navigation::InitialPlacement) {
        placement.apply(&mut self.current);
        self.previous = self.current;
    }
}

mod dynamics;
mod engines;
mod instruments;
mod loading;
mod pose;
mod report;
mod state;

#[cfg(test)]
mod tests;

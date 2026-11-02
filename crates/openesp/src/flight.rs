use anyhow::{Context, Result, ensure};
use esp_assets::flight::FlightData;
use esp_script::{Host, Reference};
use esp_sim::{
    aerodynamics::{Aerodynamics, Controls},
    atmosphere::Atmosphere,
    flight::State,
    ground::{Controls as GroundControls, LandingGear, Surface},
    hydraulics::Hydraulics,
    loading::{Loading, WeightBalance},
    propulsion::{Command, Output as EngineOutput, Propulsion},
    variables::Variables,
};
use glam::{DMat3, DQuat, DVec3, EulerRot, Quat};

#[cfg(test)]
#[path = "../../../tests/support/content.rs"]
mod content;

pub struct Flight {
    model: Aerodynamics,
    loading: Loading,
    weights: WeightBalance,
    payload: bool,
    engines: Option<Propulsion>,
    commands: Vec<Command>,
    pub current: State,
    previous: State,
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
    on_ground: bool,
}

// Physical surface properties of the local test grid. Scenery surfaces will
// supply their own coefficients when terrain contact is connected.
const TEST_SURFACE: Surface = Surface {
    elevation: 0.0,
    rolling_resistance: 0.02,
    tyre_friction: 0.8,
};

impl Flight {
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
        let model = Aerodynamics::from_data(data)?;
        let hydraulics = Hydraulics::from_config(&data.configuration)?;
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
        let loading = weights.load(fuel_fraction, payload)?;
        let previous_cg = loading.cg;
        Atmosphere::standard(height)?;
        let gear = LandingGear::from_data(data)?;
        let mut current = State {
            position: DVec3::new(0.0, 0.0, -height),
            velocity: DVec3::X * (knots * 1852.0 / 3600.0),
            ..State::default()
        };
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
            on_ground,
        })
    }

    pub fn configure_ground(variables: &mut Variables) {
        variables.set_number("ELEVATOR TRIM PCT", "percent", 0.0);
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

    pub fn start_running(&mut self, data: &FlightData) -> Result<()> {
        ensure!(
            self.loading.usable_fuel_gallons > 0.0,
            "cannot start running engines without fuel"
        );
        let mut engines = Propulsion::from_data(data)?;
        engines.start_running(
            &Atmosphere::standard(-self.current.position.z)?,
            self.current.air_velocity(DVec3::ZERO),
        )?;
        self.engines = Some(engines);
        Ok(())
    }

    pub fn prepare_piston(&mut self, data: &FlightData) -> Result<()> {
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")?
            == Some(0.0)
        {
            self.engines = Some(Propulsion::from_data(data)?);
        }
        Ok(())
    }

    pub fn configure_engines(&self, variables: &mut Variables) -> Result<()> {
        if let Some(engines) = &self.engines {
            variables.configure_engines(engines.indices())?;
            if engines.is_piston() {
                let air = Atmosphere::standard(-self.current.position.z)?;
                for (index, id) in engines.indices().enumerate() {
                    let running = engines
                        .output(index, &air, self.current.air_velocity(DVec3::ZERO))?
                        .combustion();
                    variables.set_number(
                        &format!("RECIP ENG MAGNETO:{id}"),
                        "enum",
                        if running { 3.0 } else { 0.0 },
                    );
                    variables.set_number(&format!("GENERAL ENG STARTER:{id}"), "bool", 0.0);
                    variables.set_number(&format!("GENERAL ENG STARTER ACTIVE:{id}"), "bool", 0.0);
                    variables.set_number(
                        &format!("GENERAL ENG MIXTURE LEVER POSITION:{id}"),
                        "percent",
                        100.0,
                    );
                    for side in ["LEFT", "RIGHT"] {
                        variables.set_number(
                            &format!("RECIP ENG {side} MAGNETO:{id}"),
                            "bool",
                            f64::from(running),
                        );
                    }
                }
            }
            self.publish_engines(variables)?;
        }
        Ok(())
    }

    pub fn publish_engines(&self, variables: &mut Variables) -> Result<()> {
        let Some(engines) = &self.engines else {
            return Ok(());
        };
        let air = Atmosphere::standard(-self.current.position.z)?;
        for (index, id) in engines.indices().enumerate() {
            let output = engines.output(index, &air, self.current.air_velocity(DVec3::ZERO))?;
            match output {
                EngineOutput::Turbine(output) => {
                    for (name, unit, value) in [
                        ("TURB ENG N1", "percent", output.n1),
                        ("TURB ENG N2", "percent", output.n2),
                        ("TURB ENG JET THRUST", "newtons", output.net_thrust),
                        (
                            "TURB ENG FUEL FLOW PPH",
                            "pounds per hour",
                            output.fuel_kilograms_per_second / 0.45359237 * 3600.0,
                        ),
                    ] {
                        variables.set_number(&format!("{name}:{id}"), unit, value);
                    }
                }
                EngineOutput::Piston(output) => {
                    for (name, unit, value) in [
                        ("GENERAL ENG RPM", "rpm", output.rpm),
                        ("GENERAL ENG PCT MAX RPM", "percent", output.rpm_percent),
                        ("PROP RPM", "rpm", output.propeller.rpm),
                        ("PROP ROTATION ANGLE", "radians", output.propeller_angle),
                        ("PROP MAX RPM PERCENT", "percent", output.rpm_percent),
                        ("PROP THRUST", "newtons", output.propeller.thrust),
                        (
                            "RECIP ENG MANIFOLD PRESSURE",
                            "pascals",
                            output.manifold_pressure,
                        ),
                        ("RECIP ENG BRAKE POWER", "watts", output.cycle.shaft_power),
                        ("RECIP ENG FUEL AIR RATIO", "number", output.mixture_ratio),
                    ] {
                        variables.set_number(&format!("{name}:{id}"), unit, value);
                    }
                }
            }
            for (name, unit, value) in [
                (
                    "ENG FUEL FLOW PPH",
                    "pounds per hour",
                    output.fuel_kilograms_per_second() / 0.45359237 * 3600.0,
                ),
                (
                    "GENERAL ENG COMBUSTION",
                    "bool",
                    f64::from(output.combustion()),
                ),
            ] {
                variables.set_number(&format!("{name}:{id}"), unit, value);
            }
        }
        Ok(())
    }

    pub fn advance(&mut self, dt: f64, variables: &mut Variables) -> Result<()> {
        ensure!(
            dt.is_finite() && (0.0..=0.25).contains(&dt),
            "invalid aircraft timestep"
        );
        let hydraulic_fraction = self
            .hydraulics
            .as_ref()
            .map(|h| h.update(variables))
            .transpose()?;
        variables.advance_flaps(dt, hydraulic_fraction)?;
        let spoiler_power = match self.spoiler_system_type {
            Some(0) => f64::from(
                variables
                    .read(&Reference::new(
                        'A',
                        "ELECTRICAL MAIN BUS VOLTAGE",
                        Some("volts"),
                    ))
                    .ok()
                    .and_then(|v| v.number().ok())
                    .is_some_and(|v| v > 0.0),
            ),
            Some(1) => hydraulic_fraction.unwrap_or(1.0),
            Some(4) => 0.0,
            _ => 1.0,
        };
        let armed = variables
            .read(&Reference::new('A', "SPOILERS ARMED", Some("bool")))?
            .number()?
            != 0.0;
        let mut throttles_idle = true;
        if let Some(engines) = &self.engines {
            for index in engines.indices() {
                let position = variables
                    .read(&Reference::new(
                        'A',
                        &format!("GENERAL ENG THROTTLE LEVER POSITION:{index}"),
                        Some("percent over 100"),
                    ))?
                    .number()?;
                throttles_idle &= position <= 0.05;
            }
        }
        let ground_speed = self.current.velocity.truncate().length() * 3600.0 / 1852.0;
        let handle = variables
            .read(&Reference::new(
                'A',
                "SPOILERS HANDLE POSITION",
                Some("percent over 100"),
            ))?
            .number()?;
        ensure!((0.0..=1.0).contains(&handle), "invalid spoiler handle");
        if self.spoiler_auto_deployed && handle < 1.0 {
            self.spoiler_auto_deployed = false;
        }
        if self.spoiler_auto_deployed
            && (!armed
                || !throttles_idle
                || (self.auto_spoiler_auto_retracts && ground_speed < self.auto_spoiler_min_speed))
        {
            self.spoiler_auto_deployed = false;
            variables.set_number("SPOILERS HANDLE POSITION", "percent over 100", 0.0);
        }
        if !self.on_ground && !self.spoiler_auto_deployed {
            self.spoiler_airborne = true;
        }
        if self.on_ground && self.spoiler_airborne {
            self.spoiler_airborne = false;
            if self.auto_spoiler_available
                && armed
                && throttles_idle
                && ground_speed >= self.auto_spoiler_min_speed
            {
                self.spoiler_auto_deployed = true;
                variables.set_number("SPOILERS HANDLE POSITION", "percent over 100", 1.0);
            }
        }
        let flap_handle = variables.flap_handle_fraction();
        let mut control = |name| -> Result<f64> {
            Ok(variables
                .read(&Reference::new('A', name, Some("percent over 100")))?
                .number()?)
        };
        let spoiler_handle = control("SPOILERS HANDLE POSITION")?;
        let gear_handle = control("GEAR HANDLE POSITION")? != 0.0;
        let air = Atmosphere::standard(-self.current.position.z)?;
        let ias = air.calibrated_airspeed(self.current.air_velocity(DVec3::ZERO).length())?
            * 3600.0
            / 1852.0;
        let gear_power = match self.gear.system_type() {
            Some(0) => f64::from(
                control("CIRCUIT GEAR MOTOR ON")
                    .ok()
                    .is_some_and(|v| v != 0.0),
            ),
            Some(1) => hydraulic_fraction.unwrap_or(1.0),
            Some(4) => 0.0,
            _ => 1.0,
        };
        if gear_power > 0.0 {
            self.gear
                .advance(gear_handle || self.on_ground, ias, dt * gear_power)?;
        }
        ensure!(
            (0.0..=1.0).contains(&spoiler_handle),
            "invalid spoiler handle"
        );
        let aileron = control("AILERON LEFT DEFLECTION PCT")?;
        let targets = self
            .model
            .spoiler_targets(spoiler_handle, aileron, flap_handle);
        let travel = if dt == 0.0 || spoiler_power == 0.0 {
            0.0
        } else if self.spoiler_extension_time > 0.0 {
            dt / self.spoiler_extension_time * spoiler_power
        } else {
            1.0
        };
        for (surface, target) in self.spoiler_surfaces.iter_mut().zip(targets) {
            *surface += (target - *surface).clamp(-travel, travel);
        }
        let spoilers = (self.spoiler_surfaces[0] + self.spoiler_surfaces[1]) * 0.5;
        let controls = Controls {
            elevator: control("ELEVATOR DEFLECTION PCT")?,
            aileron,
            rudder: control("RUDDER DEFLECTION PCT")?,
            elevator_trim: control("ELEVATOR TRIM PCT")?,
            spoilers,
            spoiler_difference: self.spoiler_surfaces[0] - self.spoiler_surfaces[1],
            gear: self.gear.extension(),
        };
        let ground_controls = GroundControls {
            steering: controls.rudder,
            brakes: [
                control("BRAKE LEFT POSITION")?,
                control("BRAKE RIGHT POSITION")?,
            ],
            parking_brake: control("BRAKE PARKING POSITION")? != 0.0,
        };
        variables.set_number(
            "ELEVATOR TRIM POSITION",
            "radians",
            controls.elevator_trim * self.model.elevator_trim_limit(),
        );
        for (name, position) in ["SPOILERS LEFT POSITION", "SPOILERS RIGHT POSITION"]
            .into_iter()
            .zip(self.spoiler_surfaces)
        {
            variables.set_number(name, "percent over 100", position);
        }
        self.flaps.clear();
        self.flaps.extend(variables.flap_angles());
        let mut fuel_used = 0.0;
        let mut power_fraction = 1.0;
        if let Some(engines) = &mut self.engines {
            self.commands.clear();
            for index in engines.indices() {
                let throttle = variables
                    .read(&Reference::new(
                        'A',
                        &format!("GENERAL ENG THROTTLE LEVER POSITION:{index}"),
                        Some("percent over 100"),
                    ))?
                    .number()?;
                let fuel_valve = variables
                    .read(&Reference::new(
                        'A',
                        &format!("GENERAL ENG FUEL VALVE:{index}"),
                        Some("bool"),
                    ))?
                    .number()?
                    != 0.0;
                let (mixture, magnetos) = if engines.is_piston() {
                    let mixture = variables
                        .read(&Reference::new(
                            'A',
                            &format!("GENERAL ENG MIXTURE LEVER POSITION:{index}"),
                            Some("percent over 100"),
                        ))?
                        .number()?;
                    let mut magnetos = [false; 2];
                    for (active, side) in magnetos.iter_mut().zip(["LEFT", "RIGHT"]) {
                        *active = variables
                            .read(&Reference::new(
                                'A',
                                &format!("RECIP ENG {side} MAGNETO:{index}"),
                                Some("bool"),
                            ))?
                            .number()?
                            != 0.0;
                    }
                    (mixture, magnetos)
                } else {
                    (1.0, [true; 2])
                };
                let starter = if engines.is_piston() {
                    let requested = variables
                        .read(&Reference::new(
                            'A',
                            &format!("GENERAL ENG STARTER:{index}"),
                            Some("bool"),
                        ))?
                        .number()?
                        != 0.0;
                    let powered = variables
                        .read(&Reference::new(
                            'A',
                            &format!("CIRCUIT STARTER ON:{index}"),
                            Some("bool"),
                        ))
                        .ok()
                        .and_then(|v| v.number().ok())
                        .is_some_and(|v| v != 0.0);
                    let active = requested && powered;
                    variables.set_number(
                        &format!("GENERAL ENG STARTER ACTIVE:{index}"),
                        "bool",
                        f64::from(active),
                    );
                    active
                } else {
                    false
                };
                self.commands.push(Command {
                    throttle,
                    fuel_valve,
                    mixture,
                    magnetos,
                    starter,
                });
            }
            let air = Atmosphere::standard(-self.current.position.z)?;
            let velocity = self.current.air_velocity(DVec3::ZERO);
            engines.advance(
                dt,
                &air,
                velocity,
                &self.commands,
                self.loading.usable_fuel_gallons > 0.0,
            )?;
            let mut rate = 0.0;
            for (index, _) in engines.indices().enumerate() {
                rate += engines
                    .output(index, &air, velocity)?
                    .fuel_kilograms_per_second();
            }
            let available =
                self.loading.usable_fuel_gallons * self.loading.fuel_pounds_per_gallon * 0.45359237;
            let demand = rate * dt;
            fuel_used = demand.min(available);
            if demand > 0.0 {
                power_fraction = fuel_used / demand;
            }
        }
        let mut next = self.current;
        let max_step = if self
            .gear
            .clearance(&next, self.loading.cg, TEST_SURFACE.elevation)
            < 10.0
        {
            1.0 / 240.0
        } else {
            1.0 / 60.0
        };
        let steps = (dt / max_step).ceil().max(1.0) as u32;
        for _ in 0..steps {
            let before = next;
            next.advance(
                &self.loading.mass,
                dt / f64::from(steps),
                DVec3::Z * 9.80665,
                |state| {
                    let air = Atmosphere::standard(-state.position.z)?;
                    let mut loads = self.model.forces(
                        state.air_velocity(DVec3::ZERO),
                        state.angular_velocity,
                        air.density,
                        controls,
                        &self.flaps,
                        self.loading.cg,
                    )?;
                    if let Some(engines) = &self.engines {
                        let thrust = engines.wrench(
                            &air,
                            state.air_velocity(DVec3::ZERO),
                            self.loading.cg,
                            power_fraction,
                        )?;
                        loads.force += thrust.force;
                        loads.moment += thrust.moment;
                    }
                    let ground =
                        self.gear
                            .forces(state, self.loading.cg, TEST_SURFACE, ground_controls)?;
                    loads.force += ground.wrench.force;
                    loads.moment += ground.wrench.moment;
                    Ok(loads)
                },
            )?;
            self.gear.solve_friction(
                &mut next,
                &self.loading.mass,
                self.loading.cg,
                TEST_SURFACE,
                ground_controls,
                dt / f64::from(steps),
            )?;
            self.gear.advance_wheel_motion(
                &next,
                self.loading.cg,
                TEST_SURFACE,
                ground_controls,
                dt / f64::from(steps),
            )?;
            if let Some(impact) = self
                .gear
                .impact(&before, &next, self.loading.cg, TEST_SURFACE)
            {
                ensure!(
                    !impact.scrape,
                    "scrape contact {} reached the ground; scrape forces are not simulated yet",
                    impact.index
                );
                anyhow::bail!(
                    "ground impact at contact {}: {:.2} m/s exceeds configured {:.2} m/s damage threshold",
                    impact.index,
                    impact.speed,
                    impact.damage_speed
                );
            }
        }
        ensure!(
            next.position.z < 0.0,
            "aircraft center of gravity penetrated the ground"
        );
        self.previous = self.current;
        self.previous_cg = self.loading.cg;
        if fuel_used > 0.0 {
            let fuel_mass_per_gallon = self.loading.fuel_pounds_per_gallon * 0.45359237;
            let consumed = if fuel_used >= self.loading.usable_fuel_gallons * fuel_mass_per_gallon {
                self.loading.usable_fuel_gallons
            } else {
                fuel_used / fuel_mass_per_gallon
            };
            let loading = self.weights.burn(&self.loading, consumed, self.payload)?;
            next.rebase(loading.cg - self.loading.cg)?;
            self.loading = loading;
            if self.loading.usable_fuel_gallons == 0.0
                && let Some(engines) = &mut self.engines
            {
                engines.advance(
                    0.0,
                    &Atmosphere::standard(-next.position.z)?,
                    next.air_velocity(DVec3::ZERO),
                    &self.commands,
                    false,
                )?;
            }
        }
        self.current = next;
        self.on_ground = self
            .gear
            .forces(
                &self.current,
                self.loading.cg,
                TEST_SURFACE,
                ground_controls,
            )?
            .on_ground;
        self.publish_loading(variables);
        self.publish_engines(variables)?;
        Ok(())
    }

    pub fn sample(&self, alpha: f64) -> Result<State> {
        let mut previous = self.previous;
        previous.rebase(self.loading.cg - self.previous_cg)?;
        previous.interpolate(&self.current, alpha)
    }

    pub fn pose(&self, state: &State) -> (DVec3, Quat) {
        let basis = DMat3::from_cols(DVec3::Z, DVec3::X, -DVec3::Y);
        let orientation =
            DQuat::from_mat3(&(basis * DMat3::from_quat(state.orientation) * basis.transpose()))
                .as_quat();
        let origin = basis * (state.position - state.orientation * self.loading.cg);
        (origin, orientation)
    }

    pub fn publish_loading(&self, variables: &mut Variables) {
        variables.set_number("SIM ON GROUND", "bool", f64::from(self.on_ground));
        variables.set_number(
            "AUTO SPOILERS AVAILABLE",
            "bool",
            f64::from(self.auto_spoiler_available),
        );
        variables.set_number(
            "IS GEAR RETRACTABLE",
            "bool",
            f64::from(self.gear.retractable()),
        );
        variables.set_number(
            "GEAR TOTAL PCT EXTENDED",
            "percent over 100",
            self.gear.extension(),
        );
        for (index, position) in
            self.gear
                .animation_positions(&self.current, self.loading.cg, TEST_SURFACE)
        {
            variables.set_number(
                &format!("GEAR ANIMATION POSITION:{index}"),
                "keyframe",
                position,
            );
        }
        for (index, position) in self.gear.extensions() {
            let state = if position == 0.0 {
                1.0
            } else if position == 1.0 {
                2.0
            } else {
                0.0
            };
            variables.set_number(&format!("GEAR POSITION:{index}"), "enum", state);
            if let Some(name) = [
                "GEAR CENTER POSITION",
                "GEAR LEFT POSITION",
                "GEAR RIGHT POSITION",
                "GEAR AUX POSITION",
            ]
            .get(index as usize)
            {
                variables.set_number(name, "percent over 100", position);
            }
        }
        for wheel in self.gear.wheel_motion() {
            variables.set_number(
                &format!("WHEEL ROTATION ANGLE:{}", wheel.index),
                "radians",
                wheel.rotation,
            );
            variables.set_number(&format!("WHEEL RPM:{}", wheel.index), "rpm", wheel.rpm);
            variables.set_number(
                &format!("GEAR STEER ANGLE:{}", wheel.index),
                "radians",
                wheel.steer_angle,
            );
            if let Some(side) = ["CENTER", "LEFT", "RIGHT", "AUX"].get(wheel.index as usize) {
                variables.set_number(
                    &format!("{side} WHEEL ROTATION ANGLE"),
                    "radians",
                    wheel.rotation,
                );
                variables.set_number(&format!("{side} WHEEL RPM"), "rpm", wheel.rpm);
                variables.set_number(
                    &format!("GEAR {side} STEER ANGLE"),
                    "radians",
                    wheel.steer_angle,
                );
            }
        }
        for (tank, &quantity) in self.weights.tanks().iter().zip(&self.loading.tank_gallons) {
            let capacity = tank.capacity_gallons + tank.unusable_gallons;
            let fraction = if capacity > 0.0 {
                quantity / capacity
            } else {
                0.0
            };
            let label = match tank.name {
                "Center1" => "CENTER",
                "Center2" => "CENTER2",
                "Center3" => "CENTER3",
                "LeftMain" => "LEFT MAIN",
                "LeftAux" => "LEFT AUX",
                "LeftTip" => "LEFT TIP",
                "RightMain" => "RIGHT MAIN",
                "RightAux" => "RIGHT AUX",
                "RightTip" => "RIGHT TIP",
                "External1" => "EXTERNAL1",
                "External2" => "EXTERNAL2",
                _ => continue,
            };
            variables.set_number(&format!("FUEL TANK {label} CAPACITY"), "gallons", capacity);
            variables.set_number(&format!("FUEL TANK {label} QUANTITY"), "gallons", quantity);
            variables.set_number(
                &format!("FUEL TANK {label} LEVEL"),
                "percent over 100",
                fraction,
            );
        }
        variables.set_number("TOTAL WEIGHT", "kilograms", self.loading.mass.kilograms());
        variables.set_number("EMPTY WEIGHT", "kilograms", self.loading.empty_kilograms);
        variables.set_number("FUEL TOTAL QUANTITY", "gallons", self.loading.fuel_gallons);
        variables.set_number(
            "FUEL TOTAL CAPACITY",
            "gallons",
            self.loading.fuel_capacity_gallons,
        );
        variables.set_number(
            "FUEL TOTAL QUANTITY WEIGHT",
            "pounds",
            self.loading.fuel_gallons * self.loading.fuel_pounds_per_gallon,
        );
        variables.set_number(
            "FUEL WEIGHT PER GALLON",
            "pounds",
            self.loading.fuel_pounds_per_gallon,
        );
    }

    pub fn report(&self) -> Result<()> {
        eprintln!("flight ground: on_ground={}", self.on_ground);
        eprintln!("flight fuel: {} gallons", self.loading.fuel_gallons);
        if let Some(engines) = &self.engines {
            let air = Atmosphere::standard(-self.current.position.z)?;
            for (index, id) in engines.indices().enumerate() {
                let output = engines.output(index, &air, self.current.air_velocity(DVec3::ZERO))?;
                match output {
                    EngineOutput::Turbine(output) => eprintln!(
                        "engine {id}: N1={} N2={} thrust={}N fuel={}kg/s",
                        output.n1, output.n2, output.net_thrust, output.fuel_kilograms_per_second
                    ),
                    EngineOutput::Piston(output) => eprintln!(
                        "engine {id}: RPM={} prop_RPM={} power={}W thrust={}N fuel={}kg/s",
                        output.rpm,
                        output.propeller.rpm,
                        output.cycle.shaft_power,
                        output.propeller.thrust,
                        output.fuel_kilograms_per_second
                    ),
                }
            }
        }
        Ok(())
    }

    pub fn publish(state: &State, variables: &mut Variables) -> Result<()> {
        let air = state.air_velocity(DVec3::ZERO);
        let speed = air.length();
        let atmosphere = Atmosphere::standard(-state.position.z)?;
        let (heading, pitch, bank) = state.orientation.to_euler(EulerRot::ZYX);
        variables.set_number("AIRSPEED TRUE", "meters per second", speed);
        variables.set_number(
            "AIRSPEED INDICATED",
            "meters per second",
            atmosphere.calibrated_airspeed(speed)?,
        );
        variables.set_number("AIRSPEED MACH", "mach", speed / atmosphere.sound_speed);
        variables.set_number("AMBIENT PRESSURE", "pascals", atmosphere.pressure);
        variables.set_number(
            "AMBIENT TEMPERATURE",
            "celsius",
            atmosphere.temperature - 273.15,
        );
        variables.set_number(
            "TOTAL AIR TEMPERATURE",
            "celsius",
            atmosphere.temperature * (1.0 + 0.2 * (speed / atmosphere.sound_speed).powi(2))
                - 273.15,
        );
        for name in [
            "PLANE ALTITUDE",
            "INDICATED ALTITUDE",
            "PLANE ALT ABOVE GROUND",
        ] {
            variables.set_number(name, "meters", -state.position.z);
        }
        for name in ["PLANE PITCH DEGREES", "ATTITUDE INDICATOR PITCH DEGREES"] {
            variables.set_number(name, "radians", -pitch);
        }
        for name in ["PLANE BANK DEGREES", "ATTITUDE INDICATOR BANK DEGREES"] {
            variables.set_number(name, "radians", -bank);
        }
        variables.set_number(
            "PLANE HEADING DEGREES TRUE",
            "radians",
            heading.rem_euclid(std::f64::consts::TAU),
        );
        variables.set_number("INCIDENCE ALPHA", "radians", air.z.atan2(air.x));
        variables.set_number(
            "INCIDENCE BETA",
            "radians",
            if speed > 0.0 {
                (air.y / speed).clamp(-1.0, 1.0).asin()
            } else {
                0.0
            },
        );
        variables.set_number("VERTICAL SPEED", "meters per second", -state.velocity.z);
        variables.set_number(
            "GROUND VELOCITY",
            "meters per second",
            state.velocity.truncate().length(),
        );
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_script::Value;

    #[test]
    #[ignore = "requires original aircraft with spoilerons"]
    fn spoilerons_follow_aileron_direction_and_add_roll_moment() {
        let mut verified = 0;
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            if data
                .configuration
                .number("airplane_geometry", "spoilerons_available")
                .unwrap()
                .unwrap_or(0.0)
                == 0.0
                || data
                    .configuration
                    .number("airplane_geometry", "min_flaps_for_spoilerons")
                    .unwrap()
                    .unwrap_or(0.0)
                    > 0.0
            {
                continue;
            }
            let mut flight = Flight::new(&data, 200.0, 5000.0, 0.5, true).unwrap();
            let mut gated_data = FlightData::load(&path, 0).unwrap();
            let source = std::fs::read_to_string(path.join("aircraft.cfg")).unwrap();
            let mut changed = false;
            let source = source
                .lines()
                .map(|line| {
                    if line
                        .split('=')
                        .next()
                        .unwrap_or("")
                        .trim()
                        .eq_ignore_ascii_case("min_flaps_for_spoilerons")
                    {
                        changed = true;
                        "min_flaps_for_spoilerons=0.5".to_owned()
                    } else {
                        line.to_owned()
                    }
                })
                .collect::<Vec<_>>()
                .join("\n");
            assert!(changed);
            gated_data.configuration = esp_formats::cfg::Config::parse(&source).unwrap();
            let gated = Aerodynamics::from_data(&gated_data).unwrap();
            assert_eq!(gated.spoiler_targets(0.0, 1.0, 0.0), [0.0; 2]);
            assert!(gated.spoiler_targets(0.0, 1.0, 0.5)[1] > 0.0);
            let mut vars = controls(&data);
            vars.set_number("AILERON LEFT DEFLECTION PCT", "percent", 25.0);
            for _ in 0..60 {
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert_eq!(flight.spoiler_surfaces, [0.0, 0.0]);
            vars.set_number("AILERON LEFT DEFLECTION PCT", "percent", 100.0);
            for _ in 0..60 {
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert!(flight.spoiler_surfaces[1] > 0.0 && flight.spoiler_surfaces[0] == 0.0);
            let reference = |name| Reference::new('A', name, Some("percent over 100"));
            let left = vars
                .read(&reference("SPOILERS LEFT POSITION"))
                .unwrap()
                .number()
                .unwrap();
            let right = vars
                .read(&reference("SPOILERS RIGHT POSITION"))
                .unwrap()
                .number()
                .unwrap();
            assert_eq!([left, right], flight.spoiler_surfaces);
            let mut base = Controls {
                aileron: 1.0,
                spoilers: (left + right) * 0.5,
                ..Controls::default()
            };
            let flow = DVec3::X * 80.0;
            let clean = flight
                .model
                .forces(
                    flow,
                    DVec3::ZERO,
                    1.0,
                    base,
                    &flight.flaps,
                    flight.loading.cg,
                )
                .unwrap();
            base.spoiler_difference = left - right;
            let asymmetric = flight
                .model
                .forces(
                    flow,
                    DVec3::ZERO,
                    1.0,
                    base,
                    &flight.flaps,
                    flight.loading.cg,
                )
                .unwrap();
            assert!(asymmetric.moment.x > clean.moment.x);
            vars.set_number("AILERON LEFT DEFLECTION PCT", "percent", -100.0);
            for _ in 0..180 {
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert!(flight.spoiler_surfaces[0] > 0.0 && flight.spoiler_surfaces[1] < 1e-9);
            vars.dispatch_event("SPOILERS_ON", Value::Number(0.0))
                .unwrap();
            for _ in 0..180 {
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert!(flight.spoiler_surfaces.iter().all(|v| *v > 0.99));
            verified += 1;
        }
        assert!(verified > 0);
    }

    #[test]
    #[ignore = "requires an original aircraft with automatic spoilers"]
    fn automatic_spoilers_deploy_on_touchdown_and_retract_for_go_around() {
        let mut verified = 0;
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            if data
                .configuration
                .number("airplane_geometry", "auto_spoiler_available")
                .unwrap()
                .unwrap_or(0.0)
                == 0.0
                || data
                    .configuration
                    .number("GeneralEngineData", "engine_type")
                    .unwrap()
                    != Some(1.0)
            {
                continue;
            }
            let mut ground = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
            let mut ground_vars = controls(&data);
            for _ in 0..600 {
                ground.advance(1.0 / 60.0, &mut ground_vars).unwrap();
            }
            for mode in 0..3 {
                let armed = mode != 0;
                let mut flight =
                    Flight::new(&data, 1.0, -ground.current.position.z + 0.5, 0.5, true).unwrap();
                flight.current.velocity = DVec3::ZERO;
                flight.current.orientation = ground.current.orientation;
                flight.start_running(&data).unwrap();
                let mut vars = controls(&data);
                flight.configure_engines(&mut vars).unwrap();
                if armed {
                    vars.dispatch_key_event(0x1013d, Value::Number(0.0))
                        .unwrap();
                }
                for _ in 0..900 {
                    flight.advance(1.0 / 60.0, &mut vars).unwrap();
                }
                assert!(flight.on_ground);
                if armed {
                    assert!(
                        flight
                            .spoiler_surfaces
                            .iter()
                            .all(|position| *position > 0.9)
                    );
                    assert_eq!(
                        vars.read(&Reference::new(
                            'A',
                            "SPOILERS HANDLE POSITION",
                            Some("percent over 100")
                        ))
                        .unwrap(),
                        Value::Number(1.0)
                    );
                    if mode == 2 {
                        vars.dispatch_event("SPOILERS_SET", Value::Number(8191.5))
                            .unwrap();
                        flight.advance(1.0 / 60.0, &mut vars).unwrap();
                        assert!(!flight.spoiler_auto_deployed);
                        assert_eq!(
                            vars.read(&Reference::new(
                                'A',
                                "SPOILERS HANDLE POSITION",
                                Some("percent over 100")
                            ))
                            .unwrap(),
                            Value::Number(0.5)
                        );
                        continue;
                    }
                    vars.dispatch_event("THROTTLE_FULL", Value::Number(0.0))
                        .unwrap();
                    flight.advance(1.0 / 60.0, &mut vars).unwrap();
                    assert!(!flight.spoiler_auto_deployed);
                    assert_eq!(
                        vars.read(&Reference::new(
                            'A',
                            "SPOILERS HANDLE POSITION",
                            Some("percent over 100")
                        ))
                        .unwrap(),
                        Value::Number(0.0)
                    );
                    for _ in 0..180 {
                        flight.advance(1.0 / 60.0, &mut vars).unwrap();
                    }
                    assert!(
                        flight
                            .spoiler_surfaces
                            .iter()
                            .all(|position| *position < 1e-9)
                    );
                } else {
                    assert_eq!(flight.spoiler_surfaces, [0.0; 2]);
                }
            }
            verified += 1;
        }
        assert!(verified > 0);
    }

    #[test]
    #[ignore = "requires original electric and hydraulic flap aircraft"]
    fn configured_flaps_stop_without_power_and_resume() {
        let mut verified = [false; 2];
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            let sets = esp_formats::cfg::aircraft::flaps(&data.configuration).unwrap();
            let Some(kind @ (0 | 1)) = sets.first().and_then(|set| set.system_type) else {
                continue;
            };
            let mut flight = Flight::new(&data, 80.0, 5000.0, 0.5, true).unwrap();
            let mut vars = controls(&data);
            vars.configure_electrical(
                &esp_formats::cfg::electrical::Electrical::from_config(&data.configuration)
                    .unwrap(),
                false,
            )
            .unwrap();
            if kind == 1 {
                flight.start_running(&data).unwrap();
                flight.configure_engines(&mut vars).unwrap();
                vars.dispatch_key_event(0x10305, Value::Number(0.0))
                    .unwrap();
            }
            vars.dispatch_event("FLAPS_DOWN", Value::Number(0.0))
                .unwrap();
            for _ in 0..60 {
                vars.update_electrical().unwrap();
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            let position = || {
                Reference::new(
                    'A',
                    "TRAILING EDGE FLAPS LEFT PERCENT",
                    Some("percent over 100"),
                )
            };
            assert_eq!(vars.read(&position()).unwrap().number().unwrap(), 0.0);
            if kind == 0 {
                vars.dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
                    .unwrap();
            } else {
                vars.dispatch_key_event(0x10305, Value::Number(0.0))
                    .unwrap();
            }
            for _ in 0..60 {
                vars.update_electrical().unwrap();
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert!(vars.read(&position()).unwrap().number().unwrap() > 0.0);
            verified[kind as usize] = true;
        }
        assert_eq!(verified, [true, true]);
    }

    #[test]
    #[ignore = "requires original hydraulic aircraft"]
    fn configured_spoilers_stop_without_power_and_resume() {
        let mut verified = false;
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            if esp_formats::cfg::contact::ContactPoints::from_config(&data.configuration)
                .unwrap()
                .gear_system_type
                != Some(1)
            {
                continue;
            }
            for kind in [0, 1] {
                let mut flight = Flight::new(&data, 160.0, 5000.0, 0.5, true).unwrap();
                flight.spoiler_system_type = Some(kind);
                let mut vars = controls(&data);
                vars.configure_electrical(
                    &esp_formats::cfg::electrical::Electrical::from_config(&data.configuration)
                        .unwrap(),
                    false,
                )
                .unwrap();
                if kind == 1 {
                    flight.start_running(&data).unwrap();
                    flight.configure_engines(&mut vars).unwrap();
                    vars.dispatch_key_event(0x10305, Value::Number(0.0))
                        .unwrap();
                }
                vars.dispatch_event("SPOILERS_ON", Value::Number(0.0))
                    .unwrap();
                for _ in 0..60 {
                    vars.update_electrical().unwrap();
                    flight.advance(1.0 / 60.0, &mut vars).unwrap();
                }
                assert_eq!(flight.spoiler_surfaces, [0.0; 2]);
                if kind == 0 {
                    vars.dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
                        .unwrap();
                } else {
                    vars.dispatch_key_event(0x10305, Value::Number(0.0))
                        .unwrap();
                }
                for _ in 0..60 {
                    vars.update_electrical().unwrap();
                    flight.advance(1.0 / 60.0, &mut vars).unwrap();
                }
                assert!(flight.spoiler_surfaces.iter().all(|surface| *surface > 0.0));
            }
            verified = true;
        }
        assert!(verified);
    }

    #[test]
    #[ignore = "requires original hydraulic aircraft"]
    fn hydraulic_failure_stops_gear_until_pressure_is_restored() {
        let mut verified = 0;
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            if esp_formats::cfg::contact::ContactPoints::from_config(&data.configuration)
                .unwrap()
                .gear_system_type
                != Some(1)
                || data
                    .configuration
                    .number("GeneralEngineData", "engine_type")
                    .unwrap()
                    != Some(1.0)
            {
                continue;
            }
            let mut flight = Flight::new(&data, 160.0, 5000.0, 0.5, true).unwrap();
            flight.start_running(&data).unwrap();
            let mut vars = controls(&data);
            flight.configure_engines(&mut vars).unwrap();
            vars.dispatch_event("GEAR_UP", Value::Number(0.0)).unwrap();
            vars.dispatch_key_event(0x10305, Value::Number(0.0))
                .unwrap();
            for _ in 0..60 {
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert_eq!(flight.gear.extension(), 1.0);
            assert_eq!(
                vars.read(&Reference::new('A', "HYDRAULIC PRESSURE:1", Some("psi")))
                    .unwrap(),
                Value::Number(0.0)
            );
            vars.dispatch_key_event(0x10305, Value::Number(0.0))
                .unwrap();
            for _ in 0..60 {
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert!(flight.gear.extension() < 1.0);
            assert!(
                vars.read(&Reference::new('A', "HYDRAULIC PRESSURE:1", Some("psi")))
                    .unwrap()
                    .number()
                    .unwrap()
                    > 0.0
            );
            verified += 1;
        }
        assert!(verified > 0);
    }

    #[test]
    #[ignore = "requires original retractable aircraft"]
    fn electric_gear_stops_without_power_and_resumes_when_power_returns() {
        let mut verified = 0;
        for path in content::aircraft_paths() {
            let mut data = FlightData::load(&path, 0).unwrap();
            if esp_formats::cfg::contact::ContactPoints::from_config(&data.configuration)
                .unwrap()
                .gear_system_type
                != Some(1)
            {
                continue;
            }
            let source = std::fs::read_to_string(path.join("aircraft.cfg")).unwrap();
            let mut changed = false;
            let source = source
                .lines()
                .map(|line| {
                    if line
                        .split('=')
                        .next()
                        .unwrap_or("")
                        .trim()
                        .eq_ignore_ascii_case("gear_system_type")
                    {
                        changed = true;
                        "gear_system_type=0".to_owned()
                    } else {
                        line.to_owned()
                    }
                })
                .collect::<Vec<_>>()
                .join("\n");
            assert!(changed);
            data.configuration = esp_formats::cfg::Config::parse(&source).unwrap();
            let mut flight = Flight::new(&data, 160.0, 5000.0, 0.5, true).unwrap();
            let mut vars = controls(&data);
            vars.configure_electrical(
                &esp_formats::cfg::electrical::Electrical::from_config(&data.configuration)
                    .unwrap(),
                false,
            )
            .unwrap();
            vars.dispatch_event("GEAR_UP", Value::Number(0.0)).unwrap();
            for _ in 0..60 {
                vars.update_electrical().unwrap();
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert_eq!(flight.gear.extension(), 1.0);
            vars.dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
                .unwrap();
            for _ in 0..60 {
                vars.update_electrical().unwrap();
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            let partial = flight.gear.extension();
            assert!(partial > 0.0 && partial < 1.0);
            vars.dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
                .unwrap();
            for _ in 0..60 {
                vars.update_electrical().unwrap();
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert_eq!(flight.gear.extension(), partial);
            vars.dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
                .unwrap();
            for _ in 0..480 {
                vars.update_electrical().unwrap();
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert_eq!(flight.gear.extension(), 0.0);
            verified += 1;
        }
        assert!(verified > 0);
    }

    #[test]
    #[ignore = "requires original fixed and retractable aircraft"]
    fn landing_gear_retracts_and_extends_at_authored_rates() {
        let mut retractable = 0;
        let mut fixed = 0;
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            let mut flight = Flight::new(&data, 160.0, 5000.0, 0.5, true).unwrap();
            let mut vars = controls(&data);
            vars.set_number("HYDRAULIC SWITCH", "bool", 1.0);
            vars.set_number("CIRCUIT HYDRAULIC PUMP ON", "bool", 1.0);
            vars.dispatch_key_event(0x10022, Value::Number(0.0))
                .unwrap();
            for _ in 0..120 {
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            if flight.gear.retractable() {
                let positions = flight.gear.extensions().collect::<Vec<_>>();
                assert!(
                    positions
                        .iter()
                        .all(|(_, position)| *position > 0.0 && *position < 1.0)
                );
                let contacts =
                    esp_formats::cfg::contact::ContactPoints::from_config(&data.configuration)
                        .unwrap();
                for (index, position) in positions {
                    let wheel = contacts
                        .points
                        .iter()
                        .find(|point| point.index == index)
                        .unwrap();
                    assert!((position - (1.0 - 2.0 / wheel.retraction_seconds)).abs() < 1e-8);
                }
                for _ in 0..480 {
                    flight.advance(1.0 / 60.0, &mut vars).unwrap();
                }
                assert!(flight.gear.extension() < 1e-9);
                vars.dispatch_event("GEAR_DOWN", Value::Number(0.0))
                    .unwrap();
                for _ in 0..540 {
                    flight.advance(1.0 / 60.0, &mut vars).unwrap();
                }
                assert!((flight.gear.extension() - 1.0).abs() < 1e-9);
                retractable += 1;
            } else {
                assert_eq!(flight.gear.extension(), 1.0);
                fixed += 1;
            }
        }
        assert!(retractable > 0 && fixed > 0);
    }

    #[test]
    #[ignore = "requires original aircraft with wheel contacts"]
    fn gear_animation_tracks_actual_suspension_compression() {
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            let airborne = Flight::new(&data, 100.0, 1000.0, 0.5, true).unwrap();
            let mut ground = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
            let mut vars = controls(&data);
            airborne.publish_loading(&mut vars);
            let position = |vars: &mut Variables, index| {
                vars.read(&Reference::new(
                    'A',
                    &format!("GEAR ANIMATION POSITION:{index}"),
                    Some("keyframe"),
                ))
                .unwrap()
                .number()
                .unwrap()
            };
            for index in 0..3 {
                assert_eq!(position(&mut vars, index), 100.0);
            }
            for _ in 0..600 {
                ground.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            for index in 0..3 {
                let value = position(&mut vars, index);
                assert!(
                    value > 100.0 && value <= 200.0,
                    "{} gear {index} frame {value}",
                    path.display()
                );
            }
            assert_eq!(
                vars.read(&Reference::new(
                    'A',
                    "GEAR CENTER POSITION",
                    Some("percent over 100")
                ))
                .unwrap(),
                Value::Number(1.0)
            );
        }
    }

    #[test]
    #[ignore = "requires original aircraft with configured spoilers"]
    fn spoiler_handle_moves_surfaces_and_changes_aircraft_forces() {
        let mut verified = 0;
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            let limit = data
                .configuration
                .number("airplane_geometry", "spoiler_limit")
                .unwrap()
                .unwrap_or(0.0);
            let time = data
                .configuration
                .number("airplane_geometry", "spoiler_extension_time")
                .unwrap()
                .unwrap_or(0.0);
            if limit <= 0.0 || time <= 0.5 {
                continue;
            }
            let mut clean = Flight::new(&data, 200.0, 5000.0, 0.5, true).unwrap();
            let mut dirty = Flight::new(&data, 200.0, 5000.0, 0.5, true).unwrap();
            let mut clean_vars = controls(&data);
            let mut dirty_vars = controls(&data);
            dirty_vars
                .dispatch_key_event(0x10210, Value::Number(0.0))
                .unwrap();
            for _ in 0..30 {
                clean.advance(1.0 / 60.0, &mut clean_vars).unwrap();
                dirty.advance(1.0 / 60.0, &mut dirty_vars).unwrap();
            }
            let left = dirty_vars
                .read(&Reference::new(
                    'A',
                    "SPOILERS LEFT POSITION",
                    Some("percent over 100"),
                ))
                .unwrap()
                .number()
                .unwrap();
            assert!((left - 0.5 / time).abs() < 1e-9);
            assert!(dirty.current.velocity.x < clean.current.velocity.x);
            dirty_vars
                .dispatch_key_event(0x10211, Value::Number(0.0))
                .unwrap();
            for _ in 0..30 {
                dirty.advance(1.0 / 60.0, &mut dirty_vars).unwrap();
            }
            assert!(
                dirty
                    .spoiler_surfaces
                    .iter()
                    .all(|position| *position < 1e-9)
            );
            verified += 1;
        }
        assert!(verified > 0);
    }

    #[test]
    #[ignore = "requires original aircraft and AIR data"]
    fn elevator_trim_events_change_flight_pitch() {
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            let mut neutral = Flight::new(&data, 120.0, 2000.0, 0.5, true).unwrap();
            let mut trimmed = Flight::new(&data, 120.0, 2000.0, 0.5, true).unwrap();
            let mut base_vars = controls(&data);
            let mut trim_vars = controls(&data);
            trim_vars
                .dispatch_key_event(0x100e6, Value::Number(8191.5))
                .unwrap();
            for _ in 0..60 {
                neutral.advance(1.0 / 60.0, &mut base_vars).unwrap();
                trimmed.advance(1.0 / 60.0, &mut trim_vars).unwrap();
            }
            assert!(
                trimmed.current.angular_velocity.y > neutral.current.angular_velocity.y,
                "{}",
                path.display()
            );
        }
    }

    #[test]
    #[ignore = "requires original fixed-pitch piston aircraft"]
    fn piston_aircraft_accelerates_and_lifts_off_from_the_ground() {
        let mut verified = 0;
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            if data
                .configuration
                .number("GeneralEngineData", "engine_type")
                .unwrap()
                != Some(0.0)
                || data
                    .configuration
                    .number("propeller", "propeller_type")
                    .unwrap()
                    != Some(1.0)
            {
                continue;
            }
            let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
            flight.start_running(&data).unwrap();
            let mut vars = controls(&data);
            flight.configure_engines(&mut vars).unwrap();
            vars.dispatch_event("THROTTLE_FULL", Value::Number(0.0))
                .unwrap();
            let mut lifted = false;
            for _ in 0..3600 {
                if flight.current.velocity.x > 28.0 {
                    vars.set_number("ELEVATOR DEFLECTION PCT", "percent", 35.0);
                }
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
                if !flight.on_ground
                    && flight.current.position.z < -3.0
                    && flight.current.velocity.z < 0.0
                {
                    lifted = true;
                    break;
                }
            }
            assert!(
                lifted,
                "{}: failed to lift off: {:?}",
                path.display(),
                flight.current
            );
            verified += 1;
        }
        assert!(verified > 0);
    }

    #[test]
    #[ignore = "requires original fixed-pitch piston aircraft"]
    fn cold_piston_start_requires_electrical_power_and_continues_after_release() {
        let mut verified = 0;
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            if data
                .configuration
                .number("GeneralEngineData", "engine_type")
                .unwrap()
                != Some(0.0)
                || data
                    .configuration
                    .number("propeller", "propeller_type")
                    .unwrap()
                    != Some(1.0)
            {
                continue;
            }
            let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
            flight.prepare_piston(&data).unwrap();
            let mut vars = controls(&data);
            flight.configure_engines(&mut vars).unwrap();
            vars.configure_electrical(
                &esp_formats::cfg::electrical::Electrical::from_config(&data.configuration)
                    .unwrap(),
                false,
            )
            .unwrap();
            vars.dispatch_event("PARKING_BRAKES", Value::Number(0.0))
                .unwrap();
            vars.dispatch_event("MAGNETO1_BOTH", Value::Number(0.0))
                .unwrap();
            vars.dispatch_key_event(0x1018c, Value::Number(1.0))
                .unwrap();
            for _ in 0..60 {
                vars.update_electrical().unwrap();
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            let read = |vars: &mut Variables, name, unit| {
                vars.read(&Reference::new('A', name, Some(unit)))
                    .unwrap()
                    .number()
                    .unwrap()
            };
            assert_eq!(read(&mut vars, "GENERAL ENG RPM:1", "rpm"), 0.0);
            assert_eq!(read(&mut vars, "GENERAL ENG STARTER ACTIVE:1", "bool"), 0.0);
            vars.dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
                .unwrap();
            for _ in 0..480 {
                vars.update_electrical().unwrap();
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            let rpm = read(&mut vars, "GENERAL ENG RPM:1", "rpm");
            assert!(rpm > 400.0, "rpm={rpm}");
            assert_eq!(read(&mut vars, "GENERAL ENG STARTER ACTIVE:1", "bool"), 1.0);
            assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION:1", "bool"), 1.0);
            vars.dispatch_key_event(0x1018c, Value::Number(0.0))
                .unwrap();
            for _ in 0..120 {
                vars.update_electrical().unwrap();
                flight.advance(1.0 / 60.0, &mut vars).unwrap();
            }
            assert_eq!(read(&mut vars, "GENERAL ENG STARTER ACTIVE:1", "bool"), 0.0);
            assert!(read(&mut vars, "GENERAL ENG RPM:1", "rpm") > 400.0);
            assert_eq!(read(&mut vars, "GENERAL ENG COMBUSTION:1", "bool"), 1.0);
            verified += 1;
        }
        assert!(verified > 0);
    }

    #[test]
    #[ignore = "requires original fixed-pitch piston aircraft and AIR flight data"]
    fn piston_aircraft_idle_taxi_and_shutdown_follow_live_engine_state() {
        let mut verified = 0;
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            if data
                .configuration
                .number("GeneralEngineData", "engine_type")
                .unwrap()
                != Some(0.0)
                || data
                    .configuration
                    .number("propeller", "propeller_type")
                    .unwrap()
                    != Some(1.0)
            {
                continue;
            }
            let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
            flight.start_running(&data).unwrap();
            let mut variables = controls(&data);
            flight.configure_engines(&mut variables).unwrap();
            variables
                .dispatch_event("PARKING_BRAKES", Value::Number(0.0))
                .unwrap();
            for _ in 0..600 {
                flight.advance(1.0 / 60.0, &mut variables).unwrap();
            }
            let parked = flight.current.position;
            for _ in 0..600 {
                flight.advance(1.0 / 60.0, &mut variables).unwrap();
            }
            assert!((flight.current.position - parked).truncate().length() < 0.001);
            let read = |variables: &mut Variables, name, unit| {
                variables
                    .read(&Reference::new('A', name, Some(unit)))
                    .unwrap()
                    .number()
                    .unwrap()
            };
            let idle = read(&mut variables, "GENERAL ENG RPM:1", "rpm");
            assert!(idle > 300.0 && idle < 1500.0);
            assert_eq!(read(&mut variables, "GENERAL ENG1 RPM", "rpm"), idle);
            assert_eq!(read(&mut variables, "PROP RPM:1", "rpm"), idle);
            let angle = read(&mut variables, "PROP ROTATION ANGLE:1", "radians");
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
            assert_ne!(
                read(&mut variables, "PROP ROTATION ANGLE:1", "radians"),
                angle
            );
            assert!(read(&mut variables, "PROP MAX RPM PERCENT:1", "percent") > 0.0);
            let fuel = flight.loading.fuel_gallons;
            variables
                .dispatch_event("PARKING_BRAKES", Value::Number(0.0))
                .unwrap();
            variables
                .dispatch_event("THROTTLE_FULL", Value::Number(0.0))
                .unwrap();
            for _ in 0..300 {
                flight.advance(1.0 / 60.0, &mut variables).unwrap();
            }
            assert!(flight.current.velocity.x > 2.0);
            assert!(read(&mut variables, "CENTER WHEEL RPM", "rpm") > 0.0);
            assert_eq!(
                read(&mut variables, "CENTER WHEEL ROTATION ANGLE", "radians"),
                read(&mut variables, "WHEEL ROTATION ANGLE:0", "radians")
            );
            variables.set_number("RUDDER DEFLECTION PCT", "percent", 50.0);
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
            assert!(read(&mut variables, "GEAR CENTER STEER ANGLE", "degrees") > 0.0);
            variables.set_number("RUDDER DEFLECTION PCT", "percent", 0.0);
            assert!(flight.loading.fuel_gallons < fuel);
            assert!(read(&mut variables, "GENERAL ENG RPM:1", "rpm") > idle);
            assert!(read(&mut variables, "RECIP ENG BRAKE POWER:1", "horsepower") > 50.0);
            assert!(read(&mut variables, "RECIP ENG MANIFOLD PRESSURE:1", "inhg") > 20.0);
            variables
                .dispatch_key_event(0x100f2, Value::Number(0.0))
                .unwrap();
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
            assert_eq!(
                read(&mut variables, "GENERAL ENG COMBUSTION:1", "bool"),
                0.0
            );
            variables
                .dispatch_event("MIXTURE_RICH", Value::Number(0.0))
                .unwrap();
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
            assert_eq!(
                read(&mut variables, "GENERAL ENG COMBUSTION:1", "bool"),
                1.0
            );
            variables
                .dispatch_key_event(0x10187, Value::Number(0.0))
                .unwrap();
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
            assert_eq!(
                read(&mut variables, "GENERAL ENG COMBUSTION:1", "bool"),
                0.0
            );
            variables
                .dispatch_event("MAGNETO1_BOTH", Value::Number(0.0))
                .unwrap();
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
            assert_eq!(
                read(&mut variables, "GENERAL ENG COMBUSTION:1", "bool"),
                1.0
            );
            variables
                .dispatch_event("ENGINE_AUTO_SHUTDOWN", Value::Number(0.0))
                .unwrap();
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
            assert_eq!(
                read(&mut variables, "GENERAL ENG COMBUSTION:1", "bool"),
                0.0
            );
            assert_eq!(
                read(&mut variables, "ENG FUEL FLOW PPH:1", "pounds per hour"),
                0.0
            );
            assert!(read(&mut variables, "PROP RPM:1", "rpm") > idle);
            let fuel = flight.loading.fuel_gallons;
            for _ in 0..120 {
                flight.advance(1.0 / 60.0, &mut variables).unwrap();
            }
            assert_eq!(flight.loading.fuel_gallons, fuel);
            eprintln!(
                "{}: parked idle {:.2} RPM, taxi velocity {:?}",
                path.display(),
                idle,
                flight.current.velocity
            );
            verified += 1;
        }
        assert!(verified > 0);
    }

    #[test]
    #[ignore = "requires original wheeled aircraft and AIR flight data"]
    fn parked_turbines_hold_the_airframe_stationary_against_idle_thrust() {
        let mut verified = 0;
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            if data
                .configuration
                .number("GeneralEngineData", "engine_type")
                .unwrap()
                != Some(1.0)
            {
                continue;
            }
            let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
            flight.start_running(&data).unwrap();
            let mut variables = controls(&data);
            flight.configure_engines(&mut variables).unwrap();
            variables
                .dispatch_event("PARKING_BRAKES", Value::Number(0.0))
                .unwrap();
            for _ in 0..600 {
                flight.advance(1.0 / 60.0, &mut variables).unwrap();
            }
            let origin = flight.current.position - flight.current.orientation * flight.loading.cg;
            for _ in 0..600 {
                flight.advance(1.0 / 60.0, &mut variables).unwrap();
            }
            let after = flight.current.position - flight.current.orientation * flight.loading.cg;
            let drift = (after - origin).truncate().length();
            assert!(
                drift < 0.001,
                "{path:?}: parked airframe crept {drift} m under idle thrust"
            );
            verified += 1;
        }
        assert!(verified > 0);
    }

    #[test]
    #[ignore = "requires original wheeled aircraft and AIR flight data"]
    fn ground_start_braking_and_landing_share_the_flight_model() {
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            let mut variables = controls(&data);
            let mut ground = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
            for _ in 0..600 {
                ground.advance(1.0 / 60.0, &mut variables).unwrap();
            }
            assert!(ground.on_ground, "{path:?}");
            assert!(
                ground.current.velocity.length() < 0.03,
                "{path:?}: {:?}",
                ground.current
            );
            assert_eq!(
                variables
                    .read(&Reference::new('A', "SIM ON GROUND", Some("bool")))
                    .unwrap(),
                Value::Number(1.0)
            );
            ground.current.velocity = DVec3::X * 15.0;
            variables
                .dispatch_event("PARKING_BRAKES", Value::Number(0.0))
                .unwrap();
            for _ in 0..600 {
                ground.advance(1.0 / 60.0, &mut variables).unwrap();
            }
            assert!(
                ground.current.velocity.length() < 0.03,
                "{path:?}: brakes did not stop flight state: {:?}",
                ground.current
            );
            let mut landing =
                Flight::new(&data, 1.0, -ground.current.position.z + 0.5, 0.5, true).unwrap();
            landing.current.velocity = DVec3::ZERO;
            landing.current.orientation = ground.current.orientation;
            for _ in 0..1200 {
                landing.advance(1.0 / 60.0, &mut variables).unwrap();
            }
            assert!(
                landing.on_ground,
                "{path:?}: landing did not produce weight on wheels"
            );
            assert!(
                landing.current.position.z < 0.0 && landing.current.velocity.length() < 0.03,
                "{path:?}: landing did not settle: {:?}",
                landing.current
            );
        }
    }

    #[test]
    #[ignore = "requires original wheeled aircraft and AIR flight data"]
    fn brake_axis_events_turn_the_aircraft_toward_the_braked_side() {
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            let mut variables = controls(&data);
            let mut resting = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
            for _ in 0..600 {
                resting.advance(1.0 / 60.0, &mut variables).unwrap();
            }
            for (event, sign) in [("AXIS_LEFT_BRAKE_SET", -1.0), ("AXIS_RIGHT_BRAKE_SET", 1.0)] {
                let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
                flight.current = resting.current;
                flight.current.velocity = DVec3::X * 15.0;
                let mut variables = controls(&data);
                variables
                    .dispatch_event(event, Value::Number(16383.0))
                    .unwrap();
                for _ in 0..120 {
                    flight.advance(1.0 / 60.0, &mut variables).unwrap();
                }
                let yaw = flight.current.orientation.to_euler(EulerRot::ZYX).0;
                assert!(yaw * sign > 0.0001, "{path:?}: {event} produced yaw {yaw}");
            }
        }
    }

    #[test]
    #[ignore = "requires original PFD artwork and installed gauge fonts"]
    fn original_attitude_horizon_tracks_flight_orientation_within_its_mask() {
        let mut verified = 0;
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            let panel = esp_assets::panel::Panel::load(&path, None)
                .unwrap()
                .unwrap();
            let layout = panel.layout().unwrap();
            let Some((window, gauge)) = layout
                .windows
                .iter()
                .filter(|w| w.texture.is_some())
                .find_map(|w| {
                    w.gauges
                        .iter()
                        .find(|g| g.reference.to_ascii_lowercase().ends_with("!pfd"))
                        .map(|g| (w, g))
                })
            else {
                continue;
            };
            let mut image = esp_assets::gauge::load_panel(&panel, window.texture.unwrap())
                .unwrap()
                .unwrap();
            let mut variables = controls(&data);
            variables
                .configure_electrical(
                    &esp_formats::cfg::electrical::Electrical::from_config(&data.configuration)
                        .unwrap(),
                    true,
                )
                .unwrap();
            variables.set('P', "UNITS OF MEASURE", Some("enum"), Value::Number(0.0));
            let flight = Flight::new(&data, 200.0, 10000.0, 0.5, true).unwrap();
            let sx =
                window.pixel_size.as_ref().unwrap().width / window.size_mm.as_ref().unwrap().width;
            let sy = window.pixel_size.as_ref().unwrap().height.unwrap()
                / window.size_mm.as_ref().unwrap().height.unwrap();
            let gx = gauge.width.unwrap_or(260.0) / 260.0;
            let gy = gauge.height.unwrap_or(260.0) / 260.0;
            let point = |x: f64, y: f64| {
                (
                    ((gauge.x + x * gx) * sx).round() as u32,
                    ((gauge.y + y * gy) * sy).round() as u32,
                )
            };
            let sample = |image: &image::RgbaImage, x, y| {
                let (x, y) = point(x, y);
                *image.get_pixel(x, y)
            };
            let crop = |image: &image::RgbaImage| {
                let (x, y) = point(47.0, 55.0);
                let (right, bottom) = point(179.0, 194.0);
                image::imageops::crop_imm(image, x, y, right - x, bottom - y).to_image()
            };
            let mut pictures = Vec::new();
            for (name, pitch, bank) in [
                ("level", 0.0_f64, 0.0_f64),
                ("pitch", 20.0, 0.0),
                ("bank", 0.0, 30.0),
            ] {
                let state = State {
                    orientation: DQuat::from_euler(
                        EulerRot::ZYX,
                        0.0,
                        pitch.to_radians(),
                        bank.to_radians(),
                    ),
                    ..flight.current
                };
                Flight::publish(&state, &mut variables).unwrap();
                assert!(image.update(&mut variables));
                if let Some(directory) = std::env::var_os("OPENESP_TEST_CAPTURE") {
                    let directory = std::path::PathBuf::from(directory);
                    std::fs::create_dir_all(&directory).unwrap();
                    let (x, y) = point(0.0, 0.0);
                    let (right, bottom) = point(260.0, 260.0);
                    image::imageops::crop_imm(&image.image, x, y, right - x, bottom - y)
                        .to_image()
                        .save(directory.join(format!("pfd-{name}.png")))
                        .unwrap();
                }
                pictures.push(crop(&image.image));
                if name == "level" {
                    let sky = sample(&image.image, 80.0, 90.0);
                    let ground = sample(&image.image, 80.0, 150.0);
                    assert!(
                        sky[2] > sky[0] + 20,
                        "sky is outside its expected aperture: {sky:?}"
                    );
                    assert!(
                        ground[0] > ground[2] + 20,
                        "ground is outside its expected aperture: {ground:?}"
                    );
                }
                if name == "pitch" {
                    let sky = sample(&image.image, 80.0, 150.0);
                    assert!(
                        sky[2] > sky[0] + 20,
                        "nose-up pitch did not move the horizon down: {sky:?}"
                    );
                }
                assert!(!image.update(&mut variables));
            }
            for picture in &pictures[1..] {
                assert!(
                    pictures[0]
                        .pixels()
                        .zip(picture.pixels())
                        .filter(|(a, b)| a != b)
                        .count()
                        > 1000,
                    "flight attitude did not move original horizon artwork"
                );
            }
            verified += 1;
        }
        assert!(verified > 0);
    }

    #[test]
    #[ignore = "requires original EICAS XML artwork and its installed Arial font"]
    fn original_engine_text_readouts_follow_air_table_engine_state() {
        let mut count = 0;
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            if data
                .configuration
                .number("GeneralEngineData", "engine_type")
                .unwrap()
                != Some(1.0)
            {
                continue;
            }
            let panel = esp_assets::panel::Panel::load(&path, None)
                .unwrap()
                .unwrap();
            let layout = panel.layout().unwrap();
            let Some((window, gauge)) = layout
                .windows
                .iter()
                .filter(|w| w.texture.is_some())
                .find_map(|w| {
                    w.gauges
                        .iter()
                        .find(|g| g.reference.to_ascii_lowercase().ends_with("!eicas"))
                        .map(|g| (w, g))
                })
            else {
                continue;
            };
            if window.texture.is_none() {
                continue;
            }
            let mut image = esp_assets::gauge::load_panel(&panel, window.texture.unwrap())
                .unwrap()
                .unwrap();
            let mut flight = Flight::new(&data, 200.0, 10000.0, 0.5, true).unwrap();
            flight.start_running(&data).unwrap();
            let mut variables = controls(&data);
            flight.configure_engines(&mut variables).unwrap();
            Flight::publish(&flight.current, &mut variables).unwrap();
            flight.publish_loading(&mut variables);
            variables
                .configure_electrical(
                    &esp_formats::cfg::electrical::Electrical::from_config(&data.configuration)
                        .unwrap(),
                    true,
                )
                .unwrap();
            variables.set('P', "UNITS OF MEASURE", Some("enum"), Value::Number(0.0));
            image.update(&mut variables);
            let scale_x =
                window.pixel_size.as_ref().unwrap().width / window.size_mm.as_ref().unwrap().width;
            let scale_y = window.pixel_size.as_ref().unwrap().height.unwrap()
                / window.size_mm.as_ref().unwrap().height.unwrap();
            let gauge_x = gauge.width.unwrap_or(260.0) / 260.0;
            let gauge_y = gauge.height.unwrap_or(260.0) / 260.0;
            let rectangle = (
                ((gauge.x + 44.0 * gauge_x) * scale_x).round() as u32,
                ((gauge.y + 42.0 * gauge_y) * scale_y).round() as u32,
                (42.0 * gauge_x * scale_x).round() as u32,
                (16.0 * gauge_y * scale_y).round() as u32,
            );
            let crop = |image: &image::RgbaImage| {
                image::imageops::crop_imm(image, rectangle.0, rectangle.1, rectangle.2, rectangle.3)
                    .to_image()
            };
            let before = crop(&image.image);
            assert!(
                before
                    .pixels()
                    .filter(|p| p.0[0] > 160 && p.0[1] > 160 && p.0[2] > 160)
                    .count()
                    > 5,
                "N1 text was not drawn"
            );
            variables
                .dispatch_event("THROTTLE_FULL", Value::Number(0.0))
                .unwrap();
            for _ in 0..600 {
                flight.advance(1.0 / 60.0, &mut variables).unwrap();
            }
            image.update(&mut variables);
            let after = crop(&image.image);
            assert_ne!(
                before, after,
                "AIR-derived N1 did not change original readout pixels"
            );
            assert!(!image.update(&mut variables));
            for event in [
                "TOGGLE_MASTER_BATTERY",
                "TOGGLE_ALTERNATOR1",
                "TOGGLE_ALTERNATOR2",
            ] {
                variables.dispatch_event(event, Value::Number(0.0)).unwrap();
            }
            variables.update_electrical().unwrap();
            assert!(image.update(&mut variables));
            let off = crop(&image.image);
            assert!(
                !off.pixels()
                    .any(|p| p.0[0] > 160 && p.0[1] > 160 && p.0[2] > 160),
                "unpowered N1 readout remained lit"
            );
            variables
                .dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
                .unwrap();
            variables.update_electrical().unwrap();
            assert!(image.update(&mut variables));
            assert_eq!(
                crop(&image.image),
                after,
                "battery power did not restore the readout"
            );
            count += 1;
        }
        assert!(count > 0);
    }

    fn controls(data: &FlightData) -> Variables {
        let mut variables = Variables::default();
        Flight::configure_ground(&mut variables);
        variables
            .configure_flaps(&esp_formats::cfg::aircraft::flaps(&data.configuration).unwrap())
            .unwrap();
        for name in [
            "ELEVATOR DEFLECTION PCT",
            "AILERON LEFT DEFLECTION PCT",
            "RUDDER DEFLECTION PCT",
        ] {
            variables.set_number(name, "percent", 0.0);
        }
        variables
    }

    #[test]
    #[ignore = "requires original jet AIR records and OPENESP_AIRCRAFT_PATHS"]
    fn powered_flight_accelerates_burns_fuel_and_publishes_engine_instruments() {
        let mut count = 0;
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            if data
                .configuration
                .number("GeneralEngineData", "engine_type")
                .unwrap()
                != Some(1.0)
            {
                continue;
            }
            let mut powered = Flight::new(&data, 200.0, 10000.0, 0.5, true).unwrap();
            let mut glide = Flight::new(&data, 200.0, 10000.0, 0.5, true).unwrap();
            powered.start_running(&data).unwrap();
            let mut variables = controls(&data);
            powered.configure_engines(&mut variables).unwrap();
            variables
                .dispatch_event("THROTTLE_FULL", esp_script::Value::Number(0.0))
                .unwrap();
            let initial_fuel = powered.loading.fuel_gallons;
            let initial_mass = powered.loading.mass.kilograms();
            let mut integrated_fuel = 0.0;
            for _ in 0..600 {
                powered.advance(1.0 / 60.0, &mut variables).unwrap();
                glide.advance(1.0 / 60.0, &mut variables).unwrap();
                for index in [1, 2] {
                    integrated_fuel += variables
                        .read(&Reference::new(
                            'A',
                            &format!("TURB ENG FUEL FLOW PPH:{index}"),
                            Some("kilograms per second"),
                        ))
                        .unwrap()
                        .number()
                        .unwrap()
                        / 60.0;
                }
            }
            assert!(powered.current.position.x > glide.current.position.x + 5.0);
            let consumed = (initial_fuel - powered.loading.fuel_gallons)
                * powered.loading.fuel_pounds_per_gallon
                * 0.45359237;
            assert!(consumed > 0.0);
            assert!((consumed - (initial_mass - powered.loading.mass.kilograms())).abs() < 1e-8);
            assert!((consumed - integrated_fuel).abs() < 0.1);
            let n1 = variables
                .read(&Reference::new('A', "TURB ENG N1:1", Some("percent")))
                .unwrap()
                .number()
                .unwrap();
            assert!(n1 > 80.0);
            eprintln!(
                "{}: powered x={}m, glide x={}m, fuel used={}kg, N1={}%",
                path.display(),
                powered.current.position.x,
                glide.current.position.x,
                consumed,
                n1
            );
            count += 1;
        }
        assert!(count > 0);
    }

    #[test]
    #[ignore = "requires original jet AIR records and OPENESP_AIRCRAFT_PATHS"]
    fn unusable_fuel_remains_after_engine_exhaustion() {
        let mut count = 0;
        for path in content::aircraft_paths() {
            let mut data = FlightData::load(&path, 0).unwrap();
            if data
                .configuration
                .number("GeneralEngineData", "engine_type")
                .unwrap()
                != Some(1.0)
            {
                continue;
            }
            let source = std::fs::read_to_string(path.join("aircraft.cfg")).unwrap();
            let mut section = false;
            let source = source
                .lines()
                .map(|line| {
                    let trimmed = line.trim();
                    if trimmed.starts_with('[') {
                        section = trimmed.eq_ignore_ascii_case("[fuel]");
                    }
                    if section
                        && let Some((name, value)) = line.split_once('=')
                        && [
                            "center1",
                            "center2",
                            "center3",
                            "leftmain",
                            "leftaux",
                            "lefttip",
                            "rightmain",
                            "rightaux",
                            "righttip",
                            "external1",
                            "external2",
                        ]
                        .iter()
                        .any(|tank| name.trim().eq_ignore_ascii_case(tank))
                    {
                        let value = value.split("//").next().unwrap();
                        if let Some((fields, _)) = value.rsplit_once(',') {
                            return format!("{name}={fields},1");
                        }
                    }
                    line.to_owned()
                })
                .collect::<Vec<_>>()
                .join("\n");
            data.configuration = esp_formats::cfg::Config::parse(&source).unwrap();
            let mut flight = Flight::new(&data, 200.0, 10000.0, 1.0, false).unwrap();
            flight.start_running(&data).unwrap();
            flight.loading = flight
                .weights
                .burn(
                    &flight.loading,
                    flight.loading.usable_fuel_gallons - 1e-7,
                    false,
                )
                .unwrap();
            let mut variables = controls(&data);
            flight.configure_engines(&mut variables).unwrap();
            for _ in 0..120 {
                flight.advance(1.0 / 60.0, &mut variables).unwrap();
            }
            assert_eq!(flight.loading.usable_fuel_gallons, 0.0);
            assert!(flight.loading.fuel_gallons > 0.0);
            assert_eq!(
                variables
                    .read(&Reference::new(
                        'A',
                        "GENERAL ENG COMBUSTION:1",
                        Some("bool")
                    ))
                    .unwrap()
                    .number()
                    .unwrap(),
                0.0
            );
            assert_eq!(
                variables
                    .read(&Reference::new('A', "FUEL TOTAL QUANTITY", Some("gallons")))
                    .unwrap()
                    .number()
                    .unwrap(),
                flight.loading.fuel_gallons
            );
            count += 1;
        }
        assert!(count > 0);
    }

    #[test]
    #[ignore = "requires original jet AIR records and OPENESP_AIRCRAFT_PATHS"]
    fn fuel_exhaustion_extinguishes_combustion_without_negative_quantity() {
        let mut count = 0;
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            if data
                .configuration
                .number("GeneralEngineData", "engine_type")
                .unwrap()
                != Some(1.0)
            {
                continue;
            }
            let mut flight = Flight::new(&data, 200.0, 10000.0, 1e-8, false).unwrap();
            flight.start_running(&data).unwrap();
            let mut variables = controls(&data);
            flight.configure_engines(&mut variables).unwrap();
            variables
                .dispatch_event("THROTTLE_FULL", esp_script::Value::Number(0.0))
                .unwrap();
            for _ in 0..120 {
                flight.advance(1.0 / 60.0, &mut variables).unwrap();
            }
            assert_eq!(flight.loading.fuel_gallons, 0.0);
            for index in [1, 2] {
                assert_eq!(
                    variables
                        .read(&Reference::new(
                            'A',
                            &format!("GENERAL ENG COMBUSTION:{index}"),
                            Some("bool")
                        ))
                        .unwrap()
                        .number()
                        .unwrap(),
                    0.0
                );
            }
            count += 1;
        }
        assert!(count > 0);
    }

    #[test]
    #[ignore = "requires original aircraft AIR data and OPENESP_AIRCRAFT_PATHS"]
    fn transforms_aircraft_about_its_cg_and_preserves_simulation_during_interpolation() {
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            let mut flight = Flight::new(&data, 100.0, 1000.0, 0.0, false).unwrap();
            let mut variables = Variables::default();
            Flight::configure_ground(&mut variables);
            variables
                .configure_flaps(&esp_formats::cfg::aircraft::flaps(&data.configuration).unwrap())
                .unwrap();
            for name in [
                "ELEVATOR DEFLECTION PCT",
                "AILERON LEFT DEFLECTION PCT",
                "RUDDER DEFLECTION PCT",
            ] {
                variables.set_number(name, "percent", 0.0);
            }
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
            let physical = flight.current;
            for alpha in [0.0, 0.25, 0.5, 1.0] {
                let mut state = flight.sample(alpha).unwrap();
                state.orientation = DQuat::from_euler(EulerRot::ZYX, 0.4, 0.2, 0.3);
                let (origin, orientation) = flight.pose(&state);
                let cg = flight.loading.cg;
                let rendered_cg = DVec3::new(cg.y, -cg.z, cg.x);
                let expected = DVec3::new(state.position.y, -state.position.z, state.position.x);
                assert!(
                    (origin + orientation.as_dquat() * rendered_cg).abs_diff_eq(expected, 1e-6)
                );
                let forward = state.orientation * DVec3::X;
                assert!(
                    (orientation * glam::Vec3::Z)
                        .as_dvec3()
                        .abs_diff_eq(DVec3::new(forward.y, -forward.z, forward.x), 1e-6)
                );
            }
            assert_eq!(flight.current, physical);
        }
    }

    #[test]
    #[ignore = "requires original aircraft AIR data and OPENESP_AIRCRAFT_PATHS"]
    fn altitude_changes_forces_and_indicated_airspeed() {
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            let mut low = Flight::new(&data, 150.0, 1000.0, 0.0, false).unwrap();
            let mut high = Flight::new(&data, 150.0, 10_000.0, 0.0, false).unwrap();
            let mut variables = Variables::default();
            Flight::configure_ground(&mut variables);
            variables
                .configure_flaps(&esp_formats::cfg::aircraft::flaps(&data.configuration).unwrap())
                .unwrap();
            for name in [
                "ELEVATOR DEFLECTION PCT",
                "AILERON LEFT DEFLECTION PCT",
                "RUDDER DEFLECTION PCT",
            ] {
                variables.set_number(name, "percent", 0.0);
            }
            low.advance(1.0 / 60.0, &mut variables).unwrap();
            high.advance(1.0 / 60.0, &mut variables).unwrap();
            assert!(low.current.velocity.x < high.current.velocity.x);
            assert!(low.current.velocity.z < high.current.velocity.z);
            let reference = Reference::new('A', "AIRSPEED INDICATED", Some("knots"));
            Flight::publish(&low.current, &mut variables).unwrap();
            let low_ias = variables.read(&reference).unwrap().number().unwrap();
            Flight::publish(&high.current, &mut variables).unwrap();
            let high_ias = variables.read(&reference).unwrap().number().unwrap();
            assert!(high_ias < low_ias && low_ias < 150.0);
            let pressure = variables
                .read(&Reference::new('A', "AMBIENT PRESSURE", Some("millibars")))
                .unwrap()
                .number()
                .unwrap();
            assert!((pressure - 264.999).abs() < 0.01);
        }
    }

    #[test]
    #[ignore = "requires original aircraft AIR data and OPENESP_AIRCRAFT_PATHS"]
    fn configured_loading_changes_flight_response_and_weight_variables() {
        for path in content::aircraft_paths() {
            let data = FlightData::load(&path, 0).unwrap();
            let mut empty = Flight::new(&data, 150.0, 1000.0, 0.0, false).unwrap();
            let mut full = Flight::new(&data, 150.0, 1000.0, 1.0, true).unwrap();
            assert!(full.loading.mass.kilograms() > empty.loading.mass.kilograms());
            assert!(full.loading.payload_kilograms > 0.0 && full.loading.fuel_gallons > 0.0);
            assert!(!full.loading.cg.abs_diff_eq(empty.loading.cg, 1e-6));
            let mut variables = Variables::default();
            Flight::configure_ground(&mut variables);
            variables
                .configure_flaps(&esp_formats::cfg::aircraft::flaps(&data.configuration).unwrap())
                .unwrap();
            for name in [
                "ELEVATOR DEFLECTION PCT",
                "AILERON LEFT DEFLECTION PCT",
                "RUDDER DEFLECTION PCT",
            ] {
                variables.set_number(name, "percent", 0.0);
            }
            full.publish_loading(&mut variables);
            let total = variables
                .read(&Reference::new('A', "TOTAL WEIGHT", Some("pounds")))
                .unwrap()
                .number()
                .unwrap();
            let fuel = variables
                .read(&Reference::new(
                    'A',
                    "FUEL TOTAL QUANTITY WEIGHT",
                    Some("kilograms"),
                ))
                .unwrap()
                .number()
                .unwrap();
            assert!((total * 0.45359237 - full.loading.mass.kilograms()).abs() < 1e-8);
            assert!(
                (fuel + full.loading.payload_kilograms + full.loading.empty_kilograms
                    - full.loading.mass.kilograms())
                .abs()
                    < 1e-8
            );
            empty.advance(1.0 / 60.0, &mut variables).unwrap();
            full.advance(1.0 / 60.0, &mut variables).unwrap();
            assert!(full.current.velocity.z > empty.current.velocity.z);
        }
    }

    #[test]
    fn publishes_attitude_and_airspeed_using_simulator_units() {
        let state = State {
            position: DVec3::new(0.0, 0.0, -1000.0),
            velocity: DVec3::X * 50.0,
            orientation: DQuat::from_euler(EulerRot::ZYX, 0.4, 0.1, 0.2),
            ..State::default()
        };
        let mut variables = Variables::default();
        Flight::publish(&state, &mut variables).unwrap();
        for (name, unit, expected) in [
            ("AIRSPEED TRUE", "knots", 50.0 * 3600.0 / 1852.0),
            ("PLANE ALTITUDE", "feet", 1000.0 / 0.3048),
            ("PLANE PITCH DEGREES", "radians", -0.1),
            ("PLANE BANK DEGREES", "radians", -0.2),
            ("PLANE HEADING DEGREES TRUE", "radians", 0.4),
        ] {
            let actual = variables
                .read(&Reference::new('A', name, Some(unit)))
                .unwrap()
                .number()
                .unwrap();
            assert!((actual - expected).abs() < 1e-9, "{name}: {actual}");
        }
    }
}

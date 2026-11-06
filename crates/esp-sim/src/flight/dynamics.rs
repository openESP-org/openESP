use super::*;

impl Flight {
    pub fn advance(&mut self, dt: f64, variables: &mut Variables) -> Result<()> {
        ensure!(
            dt.is_finite() && (0.0..=0.25).contains(&dt),
            "invalid aircraft timestep"
        );
        let wind = crate::weather::wind(variables)?;
        let weather = crate::weather::conditions(variables)?;
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
        let elevator_input = variables.control_input("ELEVATOR DEFLECTION PCT")?;
        let aileron_input = variables.control_input("AILERON LEFT DEFLECTION PCT")?;
        let right_input = if variables.has_control_input("AILERON RIGHT DEFLECTION PCT") {
            variables.control_input("AILERON RIGHT DEFLECTION PCT")?
        } else {
            -aileron_input
        };
        let rudder_input = variables.control_input("RUDDER DEFLECTION PCT")?;
        ensure!(
            [elevator_input, aileron_input, right_input, rudder_input]
                .iter()
                .all(|value| value.is_finite() && (-1.0..=1.0).contains(value)),
            "invalid flight control input"
        );
        let autopilot =
            self.autopilot
                .update(dt, AutopilotInputs::from_state(&self.current)?, variables)?;
        let elevator_input = autopilot.elevator.unwrap_or(elevator_input);
        let aileron_input = autopilot.aileron.unwrap_or(aileron_input);
        let right_input = autopilot.aileron.map_or(right_input, |aileron| -aileron);
        let mut control = |name| -> Result<f64> {
            Ok(variables
                .read(&Reference::new('A', name, Some("percent over 100")))?
                .number()?)
        };
        let spoiler_handle = control("SPOILERS HANDLE POSITION")?;
        let gear_handle = control("GEAR HANDLE POSITION")? != 0.0;
        let air = weather.at(-self.current.position.z)?;
        let ias =
            air.calibrated_airspeed(self.current.air_velocity(wind).length())? * 3600.0 / 1852.0;
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
        let pressure = hydraulic_fraction.unwrap_or(1.0);
        let authority = self
            .hydraulic_boost
            .map(|boost| 1.0 - boost * (1.0 - pressure));
        let aileron = aileron_input * authority[1];
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
            elevator: elevator_input * authority[0],
            aileron,
            rudder: rudder_input * authority[2],
            elevator_trim: control("ELEVATOR TRIM PCT")?,
            aileron_trim: control("AILERON TRIM PCT")?,
            rudder_trim: control("RUDDER TRIM PCT")?,
            spoilers,
            spoiler_difference: self.spoiler_surfaces[0] - self.spoiler_surfaces[1],
            gear: self.gear.extension(),
            ground_height: None,
            sound_speed: None,
        };
        let ground_controls = GroundControls {
            steering: rudder_input,
            brakes: [
                control("BRAKE LEFT POSITION")?,
                control("BRAKE RIGHT POSITION")?,
            ],
            parking_brake: control("BRAKE PARKING POSITION")? != 0.0,
        };
        variables.set_control_surfaces([
            controls.elevator,
            aileron,
            right_input * authority[1],
            controls.rudder,
        ]);
        variables.set_number(
            "ELEVATOR TRIM POSITION",
            "radians",
            controls.elevator_trim * self.model.elevator_trim_limit(),
        );
        for (name, angle) in ["AILERON TRIM", "RUDDER TRIM"]
            .into_iter()
            .zip(self.model.lateral_trim_angles(controls))
        {
            variables.set_number(name, "radians", angle);
        }
        for (name, position) in ["SPOILERS LEFT POSITION", "SPOILERS RIGHT POSITION"]
            .into_iter()
            .zip(self.spoiler_surfaces)
        {
            variables.set_number(name, "percent over 100", position);
        }
        self.flaps.clear();
        self.flaps.extend(variables.flap_angles());
        let mut new_loading = None;
        let mut power_fractions = Vec::new();
        let mut selectors = Vec::new();
        if let Some(engines) = &mut self.engines {
            self.commands.clear();
            let master_ignition = !self.master_ignition_available
                || variables
                    .read(&Reference::new('A', "MASTER IGNITION SWITCH", Some("bool")))?
                    .number()?
                    != 0.0;
            let ignition_mode = if engines.is_piston() {
                0.0
            } else {
                variables
                    .read(&Reference::new(
                        'A',
                        "TURB ENG IGNITION SWITCH",
                        Some("enum"),
                    ))?
                    .number()?
            };
            ensure!(
                ignition_mode.fract() == 0.0 && (0.0..=2.0).contains(&ignition_mode),
                "invalid turbine ignition switch position"
            );
            let ignition_power = if engines.is_piston() {
                false
            } else {
                match variables.read(&Reference::new(
                    'A',
                    "ELECTRICAL MAIN BUS VOLTAGE",
                    Some("volts"),
                )) {
                    Ok(value) => value.number()? > 0.0,
                    Err(esp_script::Error::Variable(_)) => false,
                    Err(error) => return Err(error.into()),
                }
            };
            let mut any_ignition = false;
            for index in engines.indices() {
                let throttle = variables
                    .read(&Reference::new(
                        'A',
                        &format!("GENERAL ENG THROTTLE LEVER POSITION:{index}"),
                        Some("percent over 100"),
                    ))?
                    .number()?;
                let selector_index = if self.fuel_selectors <= 1 {
                    1
                } else {
                    index.min(self.fuel_selectors)
                };
                let selector = if self.fuel_selectors == 0 {
                    1
                } else {
                    variables
                        .read(&Reference::new(
                            'A',
                            &format!("FUEL TANK SELECTOR:{selector_index}"),
                            Some("enum"),
                        ))?
                        .number()? as u32
                };
                selectors.push(selector);
                let fuel_valve = master_ignition
                    && self.weights.selected_fuel(&self.loading, selector) > 0.0
                    && variables
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
                    if !master_ignition {
                        magnetos = [false; 2];
                    }
                    (mixture, magnetos)
                } else {
                    (1.0, [true; 2])
                };
                let starter = {
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
                };
                let ignition = master_ignition
                    && !engines.is_piston()
                    && (ignition_mode == 2.0 && ignition_power || ignition_mode == 1.0 && starter);
                any_ignition |= ignition;
                self.commands.push(Command {
                    throttle,
                    fuel_valve,
                    mixture,
                    magnetos,
                    starter,
                    ignition,
                });
            }
            if !engines.is_piston() {
                variables.set_number("TURB ENG IGNITION ON", "bool", f64::from(any_ignition));
            }
            let air = weather.at(-self.current.position.z)?;
            let velocity = self.current.air_velocity(wind);
            engines.advance(
                dt,
                &air,
                velocity,
                &self.commands,
                self.loading.usable_fuel_gallons > 0.0,
            )?;
            let mass_per_gallon = self.loading.fuel_pounds_per_gallon * 0.45359237;
            let mut demands = Vec::new();
            let mut loading = None;
            for (index, _) in engines.indices().enumerate() {
                demands.push(
                    engines
                        .output(index, &air, velocity)?
                        .fuel_kilograms_per_second()
                        * dt,
                );
            }
            power_fractions.resize(demands.len(), 1.0);
            for (index, &selector) in selectors.iter().enumerate() {
                if selectors[..index].contains(&selector) {
                    continue;
                }
                let demand: f64 = demands
                    .iter()
                    .zip(&selectors)
                    .filter(|(_, route)| **route == selector)
                    .map(|(&demand, _)| demand)
                    .sum();
                if demand == 0.0 {
                    continue;
                }
                let previous = loading.as_ref().unwrap_or(&self.loading);
                let requested = demand / mass_per_gallon;
                let supplied = self
                    .weights
                    .selected_fuel(previous, selector)
                    .min(requested);
                let fraction = (supplied / requested).clamp(0.0, 1.0);
                for (engine_fraction, &route) in power_fractions.iter_mut().zip(&selectors) {
                    if route == selector {
                        *engine_fraction = fraction;
                    }
                }
                loading = Some(self.weights.burn_selected(
                    previous,
                    selector,
                    requested,
                    self.payload,
                )?);
            }
            new_loading = loading;
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
                    let air = weather.at(-state.position.z)?;
                    let mut controls = controls;
                    controls.sound_speed = Some(air.sound_speed);
                    controls.ground_height = Some(self.model.wing_height(
                        state,
                        self.loading.cg,
                        TEST_SURFACE.elevation,
                    ));
                    let mut loads = self.model.forces(
                        state.air_velocity(wind),
                        state.angular_velocity,
                        air.density,
                        controls,
                        &self.flaps,
                        self.loading.cg,
                    )?;
                    if let Some(engines) = &self.engines {
                        let thrust = engines.wrench(
                            &air,
                            state.air_velocity(wind),
                            state.angular_velocity,
                            self.loading.cg,
                            &power_fractions,
                        )?;
                        loads.force += thrust.force;
                        loads.moment += thrust.moment;
                        let velocity = state.air_velocity(wind);
                        let wash = engines.tail_wash(&air, velocity, &power_fractions)?;
                        let wash_loads = self.model.tail_wash_forces(
                            (velocity, wash),
                            state.angular_velocity,
                            air.density,
                            controls,
                            &self.flaps,
                            self.loading.cg,
                        )?;
                        loads.force += wash_loads.force;
                        loads.moment += wash_loads.moment;
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
        crate::navigation::publish_acceleration(&self.current, &next, dt, variables)?;
        self.previous = self.current;
        self.previous_cg = self.loading.cg;
        if let Some(loading) = new_loading {
            next.rebase(loading.cg - self.loading.cg)?;
            self.loading = loading;
            let mut exhausted = false;
            for (command, &selector) in self.commands.iter_mut().zip(&selectors) {
                if command.fuel_valve && self.weights.selected_fuel(&self.loading, selector) == 0.0
                {
                    command.fuel_valve = false;
                    exhausted = true;
                }
            }
            if exhausted && let Some(engines) = &mut self.engines {
                engines.advance(
                    0.0,
                    &weather.at(-next.position.z)?,
                    next.air_velocity(wind),
                    &self.commands,
                    self.loading.usable_fuel_gallons > 0.0,
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
}

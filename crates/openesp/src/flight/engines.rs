use super::*;

impl Flight {
    #[cfg(test)]
    pub fn start_running(&mut self, data: &FlightData) -> Result<()> {
        self.start_running_with_weather(data, crate::weather::Conditions::new(1013.25, 15.0)?)
    }

    pub fn start_running_with_weather(
        &mut self,
        data: &FlightData,
        weather: crate::weather::Conditions,
    ) -> Result<()> {
        ensure!(
            self.loading.usable_fuel_gallons > 0.0,
            "cannot start running engines without fuel"
        );
        let mut engines = Propulsion::from_data(data)?;
        engines.start_running(
            &weather.at(-self.current.position.z)?,
            self.current.air_velocity(DVec3::ZERO),
        )?;
        self.engines = Some(engines);
        Ok(())
    }

    pub fn prepare_engines(&mut self, data: &FlightData) -> Result<()> {
        if matches!(
            data.configuration
                .number("GeneralEngineData", "engine_type")?,
            Some(0.0 | 1.0)
        ) {
            self.engines = Some(Propulsion::from_data(data)?);
        }
        Ok(())
    }

    pub fn configure_engines(&self, variables: &mut Variables) -> Result<()> {
        let wind = crate::weather::wind(variables)?;
        let weather = crate::weather::conditions(variables)?;
        if let Some(engines) = &self.engines {
            variables.configure_engines(engines.indices())?;
            variables.configure_throttle_limit(engines.throttle_lower_limit())?;
            variables.configure_fuel_selectors(self.fuel_selectors);
            variables.set_number("MASTER IGNITION SWITCH", "bool", 1.0);
            if !engines.is_piston() {
                variables.set_number("TURB ENG IGNITION SWITCH", "enum", 1.0);
                variables.set_number("TURB ENG IGNITION ON", "bool", 0.0);
            }
            for id in engines.indices() {
                variables.set_number(&format!("GENERAL ENG STARTER:{id}"), "bool", 0.0);
                variables.set_number(&format!("GENERAL ENG STARTER ACTIVE:{id}"), "bool", 0.0);
            }
            if engines.is_piston() {
                let air = weather.at(-self.current.position.z)?;
                for (index, id) in engines.indices().enumerate() {
                    let running = engines
                        .output(index, &air, self.current.air_velocity(wind))?
                        .combustion();
                    variables.set_number(
                        &format!("RECIP ENG MAGNETO:{id}"),
                        "enum",
                        if running { 3.0 } else { 0.0 },
                    );
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
        let air = crate::weather::conditions(variables)?.at(-self.current.position.z)?;
        let wind = crate::weather::wind(variables)?;
        let mut suction: Option<f64> = None;
        for (index, id) in engines.indices().enumerate() {
            let output = engines.output(index, &air, self.current.air_velocity(wind))?;
            match output {
                EngineOutput::Turbine(output) => {
                    for (name, unit, value) in [
                        ("TURB ENG N1", "percent", output.n1),
                        ("TURB ENG N2", "percent", output.n2),
                        ("TURB ENG JET THRUST", "newtons", output.net_thrust),
                        (
                            "TURB ENG REVERSE NOZZLE PERCENT",
                            "percent over 100",
                            output.reverse_nozzle,
                        ),
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
                    if let Some(pressure) = output.suction_inhg {
                        suction = Some(suction.unwrap_or(0.0).max(pressure));
                    }
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
                        (
                            "GENERAL ENG EXHAUST GAS TEMPERATURE",
                            "celsius",
                            output.exhaust_temperature_c,
                        ),
                        (
                            "ENG EXHAUST GAS TEMPERATURE",
                            "celsius",
                            output.exhaust_temperature_c,
                        ),
                        (
                            "GENERAL ENG OIL TEMPERATURE",
                            "celsius",
                            output.oil_temperature_c,
                        ),
                        ("GENERAL ENG OIL PRESSURE", "psi", output.oil_pressure_psi),
                        ("ENG OIL TEMPERATURE", "celsius", output.oil_temperature_c),
                        ("ENG OIL PRESSURE", "psi", output.oil_pressure_psi),
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
        if let Some(pressure) = suction {
            let failed = variables
                .read(&Reference::new('A', "PARTIAL PANEL VACUUM", Some("enum")))?
                .number()?
                != 0.0;
            variables.set_number(
                "SUCTION PRESSURE",
                "inhg",
                if failed { 0.0 } else { pressure },
            );
        }
        Ok(())
    }
}

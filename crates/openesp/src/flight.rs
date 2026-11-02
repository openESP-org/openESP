use anyhow::{Result, ensure};
use esp_assets::flight::FlightData;
use esp_script::{Host, Reference};
use esp_sim::{
    aerodynamics::{Aerodynamics, Controls},
    atmosphere::Atmosphere,
    flight::State,
    loading::{Loading, WeightBalance},
    turbine::{Command, Turbines},
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
    engines: Option<Turbines>,
    commands: Vec<Command>,
    pub current: State,
    previous: State,
    previous_cg: DVec3,
    flaps: Vec<f64>,
}

impl Flight {
    pub fn new(
        data: &FlightData,
        knots: f64,
        height: f64,
        fuel_fraction: f64,
        payload: bool,
    ) -> Result<Self> {
        ensure!(
            knots.is_finite() && knots > 0.0 && height.is_finite() && height > 0.0,
            "invalid airborne starting state"
        );
        let model = Aerodynamics::from_data(data)?;
        let weights = WeightBalance::from_config(&data.configuration)?;
        let loading = weights.load(fuel_fraction, payload)?;
        let previous_cg = loading.cg;
        Atmosphere::standard(height)?;
        let current = State {
            position: DVec3::new(0.0, 0.0, -height),
            velocity: DVec3::X * (knots * 1852.0 / 3600.0),
            ..State::default()
        };
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
        })
    }

    pub fn start_running(&mut self, data: &FlightData) -> Result<()> {
        ensure!(
            self.loading.fuel_gallons > 0.0,
            "cannot start running engines without fuel"
        );
        let mut engines = Turbines::from_data(data)?;
        engines.start_running(
            &Atmosphere::standard(-self.current.position.z)?,
            self.current.velocity.length(),
        )?;
        self.engines = Some(engines);
        Ok(())
    }

    pub fn configure_engines(&self, variables: &mut Variables) -> Result<()> {
        if let Some(engines) = &self.engines {
            variables.configure_engines(engines.indices())?;
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
            let output = engines.output(index, &air, self.current.velocity.length())?;
            for (name, unit, value) in [
                ("TURB ENG N1", "percent", output.n1),
                ("TURB ENG N2", "percent", output.n2),
                ("TURB ENG JET THRUST", "newtons", output.net_thrust),
                (
                    "ENG FUEL FLOW PPH",
                    "pounds per hour",
                    output.fuel_kilograms_per_second / 0.45359237 * 3600.0,
                ),
                (
                    "TURB ENG FUEL FLOW PPH",
                    "pounds per hour",
                    output.fuel_kilograms_per_second / 0.45359237 * 3600.0,
                ),
                (
                    "GENERAL ENG COMBUSTION",
                    "bool",
                    f64::from(output.combustion),
                ),
            ] {
                variables.set_number(&format!("{name}:{id}"), unit, value);
            }
        }
        Ok(())
    }

    pub fn advance(&mut self, dt: f64, variables: &mut Variables) -> Result<()> {
        ensure!(dt.is_finite() && dt >= 0.0, "invalid aircraft timestep");
        let mut control = |name| -> Result<f64> {
            Ok(variables
                .read(&Reference::new('A', name, Some("percent over 100")))?
                .number()?)
        };
        let controls = Controls {
            elevator: control("ELEVATOR DEFLECTION PCT")?,
            aileron: control("AILERON LEFT DEFLECTION PCT")?,
            rudder: control("RUDDER DEFLECTION PCT")?,
        };
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
                self.commands.push(Command {
                    throttle,
                    fuel_valve,
                });
            }
            let air = Atmosphere::standard(-self.current.position.z)?;
            let speed = self.current.velocity.length();
            engines.advance(
                dt,
                &air,
                speed,
                &self.commands,
                self.loading.fuel_gallons > 0.0,
            )?;
            let mut rate = 0.0;
            for (index, _) in engines.indices().enumerate() {
                rate += engines
                    .output(index, &air, speed)?
                    .fuel_kilograms_per_second;
            }
            let available =
                self.loading.fuel_gallons * self.loading.fuel_pounds_per_gallon * 0.45359237;
            let demand = rate * dt;
            fuel_used = demand.min(available);
            if demand > 0.0 {
                power_fraction = fuel_used / demand;
            }
        }
        let mut next = self.current;
        next.advance(&self.loading.mass, dt, DVec3::Z * 9.80665, |state| {
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
            Ok(loads)
        })?;
        ensure!(
            next.position.z < 0.0,
            "airborne test reached the ground; ground contact is not simulated yet"
        );
        self.previous = self.current;
        self.previous_cg = self.loading.cg;
        if fuel_used > 0.0 {
            let remaining = (self.loading.fuel_gallons
                - fuel_used / (self.loading.fuel_pounds_per_gallon * 0.45359237))
                .max(0.0);
            let loading = self
                .weights
                .load(remaining / self.loading.fuel_capacity_gallons, self.payload)?;
            next.rebase(loading.cg - self.loading.cg)?;
            self.loading = loading;
            if remaining == 0.0
                && let Some(engines) = &mut self.engines
            {
                engines.advance(
                    0.0,
                    &Atmosphere::standard(-next.position.z)?,
                    next.velocity.length(),
                    &self.commands,
                    false,
                )?;
            }
        }
        self.current = next;
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
        let fraction = if self.loading.fuel_capacity_gallons > 0.0 {
            self.loading.fuel_gallons / self.loading.fuel_capacity_gallons
        } else {
            0.0
        };
        for tank in self.weights.tanks() {
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
            variables.set_number(
                &format!("FUEL TANK {label} CAPACITY"),
                "gallons",
                tank.capacity_gallons,
            );
            variables.set_number(
                &format!("FUEL TANK {label} QUANTITY"),
                "gallons",
                tank.capacity_gallons * fraction,
            );
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
        eprintln!("flight fuel: {} gallons", self.loading.fuel_gallons);
        if let Some(engines) = &self.engines {
            let air = Atmosphere::standard(-self.current.position.z)?;
            for (index, id) in engines.indices().enumerate() {
                let output = engines.output(index, &air, self.current.velocity.length())?;
                eprintln!(
                    "engine {id}: N1={} N2={} thrust={}N fuel={}kg/s",
                    output.n1, output.n2, output.net_thrust, output.fuel_kilograms_per_second
                );
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
        variables.set_number("SIM ON GROUND", "bool", 0.0);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_script::Value;

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
            variables.set_number("CIRCUIT GENERAL PANEL ON", "bool", 1.0);
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
            count += 1;
        }
        assert!(count > 0);
    }

    fn controls(data: &FlightData) -> Variables {
        let mut variables = Variables::default();
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

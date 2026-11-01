use anyhow::{Result, ensure};
use esp_assets::flight::FlightData;
use esp_script::{Host, Reference};
use esp_sim::{
    aerodynamics::{Aerodynamics, Controls},
    atmosphere::Atmosphere,
    flight::State,
    loading::{Loading, WeightBalance},
    variables::Variables,
};
use glam::{DMat3, DQuat, DVec3, EulerRot, Quat};

#[cfg(test)]
#[path = "../../../tests/support/content.rs"]
mod content;

pub struct Flight {
    model: Aerodynamics,
    loading: Loading,
    pub current: State,
    previous: State,
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
        let loading =
            WeightBalance::from_config(&data.configuration)?.load(fuel_fraction, payload)?;
        Atmosphere::standard(height)?;
        let current = State {
            position: DVec3::new(0.0, 0.0, -height),
            velocity: DVec3::X * (knots * 1852.0 / 3600.0),
            ..State::default()
        };
        Ok(Self {
            model,
            loading,
            current,
            previous: current,
            flaps: Vec::new(),
        })
    }

    pub fn advance(&mut self, dt: f64, variables: &mut Variables) -> Result<()> {
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
        let mut next = self.current;
        next.advance(&self.loading.mass, dt, DVec3::Z * 9.80665, |state| {
            let air = Atmosphere::standard(-state.position.z)?;
            self.model.forces(
                state.air_velocity(DVec3::ZERO),
                state.angular_velocity,
                air.density,
                controls,
                &self.flaps,
                self.loading.cg,
            )
        })?;
        ensure!(
            next.position.z < 0.0,
            "airborne test reached the ground; ground contact is not simulated yet"
        );
        self.previous = self.current;
        self.current = next;
        Ok(())
    }

    pub fn sample(&self, alpha: f64) -> Result<State> {
        self.previous.interpolate(&self.current, alpha)
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

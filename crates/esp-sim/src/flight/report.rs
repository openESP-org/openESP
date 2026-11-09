use super::*;

impl Flight {
    pub fn report(&self, wind: DVec3, weather: crate::weather::Conditions) -> Result<()> {
        eprintln!("flight ground: on_ground={}", self.on_ground);
        eprintln!("flight fuel: {} gallons", self.loading.fuel_gallons);
        if let Some(engines) = &self.engines {
            let air = weather.at(self.msl_altitude(&self.current))?;
            for (index, id) in engines.indices().enumerate() {
                let output = engines.output(index, &air, self.current.air_velocity(wind))?;
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
}

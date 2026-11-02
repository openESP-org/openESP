use anyhow::{Context, Result, ensure};
use esp_formats::cfg::Config;
use esp_script::{Host, Reference};

use crate::variables::Variables;

/// Ideal pump routing. Fluid storage, leaks and accumulator dynamics are not modeled.
pub struct Hydraulics {
    normal_pressure: f64,
    engine_pumps: Vec<u32>,
    electric_pumps: u32,
    full_pressure_rpm: Option<f64>,
}

impl Hydraulics {
    pub fn from_config(config: &Config) -> Result<Option<Self>> {
        let Some(normal_pressure) = config.number("hydraulic_system", "normal_pressure")? else {
            return Ok(None);
        };
        let electric = config
            .number("hydraulic_system", "electric_pumps")?
            .unwrap_or(0.0);
        let full_pressure_rpm =
            config.number("hydraulic_system", "min_pct_rpm_for_max_pressure")?;
        ensure!(
            normal_pressure >= 0.0
                && electric >= 0.0
                && electric.fract() == 0.0
                && electric <= f64::from(u32::MAX)
                && full_pressure_rpm.is_none_or(|v| v > 0.0),
            "invalid hydraulic parameters"
        );
        let mut engine_pumps = Vec::new();
        if let Some(map) = config.value("hydraulic_system", "engine_map")? {
            for (index, value) in map.split(',').enumerate() {
                ensure!(index < 16, "too many hydraulic pump entries");
                let count: u32 = value
                    .trim()
                    .parse()
                    .context("invalid hydraulic engine map")?;
                if count > 0 {
                    engine_pumps.push(index as u32 + 1);
                }
            }
        }
        Ok(Some(Self {
            normal_pressure,
            engine_pumps,
            electric_pumps: electric as u32,
            full_pressure_rpm,
        }))
    }

    pub fn update(&self, variables: &mut Variables) -> Result<f64> {
        let number = |variables: &mut Variables, name: &str, unit: &str| -> Result<Option<f64>> {
            match variables.read(&Reference::new('A', name, Some(unit))) {
                Ok(value) => Ok(Some(value.number()?)),
                Err(esp_script::Error::Variable(_)) => Ok(None),
                Err(error) => Err(error.into()),
            }
        };
        let mut fraction: f64 = 0.0;
        for index in &self.engine_pumps {
            let spinning = number(
                variables,
                &format!("GENERAL ENG PCT MAX RPM:{index}"),
                "percent over 100",
            )?
            .or(number(
                variables,
                &format!("TURB ENG N2:{index}"),
                "percent over 100",
            )?);
            let combustion = number(
                variables,
                &format!("GENERAL ENG COMBUSTION:{index}"),
                "bool",
            )?
            .unwrap_or(0.0)
                != 0.0;
            let pump = match (self.full_pressure_rpm, spinning) {
                (Some(rated), Some(rpm)) => (rpm / rated).clamp(0.0, 1.0),
                _ => f64::from(combustion),
            };
            fraction = fraction.max(pump);
        }
        let electric = self.electric_pumps > 0
            && number(variables, "HYDRAULIC SWITCH", "bool")?.unwrap_or(0.0) != 0.0
            && number(variables, "CIRCUIT HYDRAULIC PUMP ON", "bool")?.unwrap_or(0.0) != 0.0;
        if electric {
            fraction = 1.0;
        }
        if number(variables, "HYDRAULIC FAILURE", "bool")?.unwrap_or(0.0) != 0.0 {
            fraction = 0.0;
        }
        let pressure = self.normal_pressure * fraction;
        variables.set_number("HYDRAULIC PRESSURE:1", "psi", pressure);
        Ok(if self.normal_pressure > 0.0 {
            fraction
        } else {
            0.0
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_script::Value;

    #[test]
    fn configured_pumps_and_failure_determine_pressure() {
        let config = Config::parse("[hydraulic_system]\nnormal_pressure=3000\nengine_map=1,0\nelectric_pumps=1\nmin_pct_rpm_for_max_pressure=0.2").unwrap();
        let hydraulics = Hydraulics::from_config(&config).unwrap().unwrap();
        let mut vars = Variables::default();
        vars.set_number("GENERAL ENG PCT MAX RPM:1", "percent", 10.0);
        vars.set_number("GENERAL ENG PCT MAX RPM:2", "percent", 100.0);
        assert_eq!(hydraulics.update(&mut vars).unwrap(), 0.5);
        assert!(
            (vars
                .read(&Reference::new('A', "HYDRAULIC PRESSURE:1", Some("psi")))
                .unwrap()
                .number()
                .unwrap()
                - 1500.0)
                .abs()
                < 1e-9
        );
        vars.set_number("GENERAL ENG PCT MAX RPM:1", "percent", 0.0);
        assert_eq!(hydraulics.update(&mut vars).unwrap(), 0.0);
        vars.set_number("HYDRAULIC SWITCH", "bool", 1.0);
        vars.set_number("CIRCUIT HYDRAULIC PUMP ON", "bool", 1.0);
        assert_eq!(hydraulics.update(&mut vars).unwrap(), 1.0);
        vars.dispatch_key_event(0x10305, Value::Number(0.0))
            .unwrap();
        assert_eq!(hydraulics.update(&mut vars).unwrap(), 0.0);
        vars.dispatch_key_event(0x10305, Value::Number(0.0))
            .unwrap();
        assert_eq!(hydraulics.update(&mut vars).unwrap(), 1.0);
    }
}

//! Helicopter-specific aircraft.cfg fields, in their authored units.
use super::{Config, ValueError, camera};

#[derive(Debug, PartialEq)]
pub struct MainRotor {
    /// Longitudinal, lateral, vertical position relative to the datum, in feet.
    pub position_feet: Option<[f64; 3]>,
    pub radius_feet: Option<f64>,
    pub static_pitch_degrees: Option<f64>,
    pub static_bank_degrees: Option<f64>,
    pub max_disc_degrees: Option<f64>,
    pub rated_rpm: Option<f64>,
    pub blades: Option<f64>,
    pub blade_weight_pounds: Option<f64>,
    pub weight_to_moi_factor: Option<f64>,
    pub inflow_velocity_reference: Option<f64>,
}

#[derive(Debug, PartialEq)]
pub struct SecondaryRotor {
    pub position_feet: Option<[f64; 3]>,
    pub radius_feet: Option<f64>,
    pub tail_rotor: Option<bool>,
}

impl MainRotor {
    pub fn from_config(config: &Config) -> Result<Self, ValueError> {
        let n = |key| camera::number(config, "mainrotor", key);
        Ok(Self {
            position_feet: camera::vector(config, "mainrotor", "position")?,
            radius_feet: n("radius")?,
            static_pitch_degrees: n("static_pitch_angle")?,
            static_bank_degrees: n("static_bank_angle")?,
            max_disc_degrees: n("max_disc_angle")?,
            rated_rpm: n("ratedrpm")?,
            blades: n("number_of_blades")?,
            blade_weight_pounds: n("weight_per_blade")?,
            weight_to_moi_factor: n("weight_to_moi_factor")?,
            inflow_velocity_reference: n("inflow_vel_reference")?,
        })
    }
}

impl SecondaryRotor {
    pub fn from_config(config: &Config) -> Result<Self, ValueError> {
        let tail_rotor = match config.value("secondaryrotor", "tailrotor")? {
            None => None,
            Some(value) if value.trim() == "0" => Some(false),
            Some(value) if value.trim() == "1" => Some(true),
            Some(value) => {
                return Err(ValueError::Invalid {
                    section: "secondaryrotor".into(),
                    key: "tailrotor".into(),
                    value: value.into(),
                });
            }
        };
        Ok(Self {
            position_feet: camera::vector(config, "secondaryrotor", "position")?,
            radius_feet: camera::number(config, "secondaryrotor", "radius")?,
            tail_rotor,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_rotors_and_preserves_missing_values() {
        let config = Config::parse("[MAINROTOR]\nPosition=-8.5, 0, 4.91\nRadius=12.583\nRatedRpm=510\nNumber_of_blades=2\nWeight_per_blade=26\nWeight_to_moi_factor=0.58\nInflow_vel_reference=34\n[secondaryrotor]\nPosition=-22.8,-0.74,1.8\nTailRotor=1\nRadius=1.75").unwrap();
        let main = MainRotor::from_config(&config).unwrap();
        assert_eq!(main.position_feet, Some([-8.5, 0.0, 4.91]));
        assert_eq!(main.radius_feet, Some(12.583));
        assert_eq!(main.rated_rpm, Some(510.0));
        assert_eq!(main.blades, Some(2.0));
        assert_eq!(main.static_pitch_degrees, None);
        let tail = SecondaryRotor::from_config(&config).unwrap();
        assert_eq!(tail.position_feet, Some([-22.8, -0.74, 1.8]));
        assert_eq!(tail.tail_rotor, Some(true));
        assert_eq!(tail.radius_feet, Some(1.75));
    }

    #[test]
    fn rejects_bad_vectors_flags_and_non_finite_values() {
        for text in [
            "[mainrotor]\nposition=1,2",
            "[mainrotor]\nradius=NaN",
            "[mainrotor]\nratedrpm=inf",
        ] {
            assert!(MainRotor::from_config(&Config::parse(text).unwrap()).is_err());
        }
        assert!(
            SecondaryRotor::from_config(&Config::parse("[secondaryrotor]\ntailrotor=2").unwrap())
                .is_err()
        );
        assert_eq!(
            SecondaryRotor::from_config(&Config::parse("[secondaryrotor]\ntailrotor=0").unwrap())
                .unwrap()
                .tail_rotor,
            Some(false)
        );
    }
}

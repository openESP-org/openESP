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

#[derive(Debug, PartialEq)]
pub struct Helicopter {
    pub lift_aero_center_feet: Option<f64>,
    pub side_aero_center_feet: Option<f64>,
    pub reference_length_feet: Option<f64>,
    pub reference_frontal_area_sq_feet: Option<f64>,
    pub reference_side_area_sq_feet: Option<f64>,
    pub low_realism_stability_scale: Option<[f64; 3]>,
    pub correlator_available: Option<bool>,
    pub governed_pct_rpm_ref: Option<f64>,
    pub governor_pid: Option<[f64; 5]>,
    pub right_trim_scalar: Option<f64>,
    pub rotor_brake_scalar: Option<f64>,
    pub torque_scalar: Option<f64>,
    pub cyclic_roll_control_scalar: Option<f64>,
    pub cyclic_pitch_control_scalar: Option<f64>,
    pub pedal_control_scalar: Option<f64>,
    pub collective_on_rotor_torque_scalar: Option<f64>,
}

#[derive(Debug, PartialEq)]
pub struct FuselageAerodynamics {
    pub drag_force: Option<f64>,
    pub side_drag_force: Option<f64>,
    pub pitch_damping: Option<f64>,
    pub roll_damping: Option<f64>,
    pub yaw_damping: Option<f64>,
    pub yaw_stability: Option<f64>,
}

fn flag(config: &Config, section: &str, key: &str) -> Result<Option<bool>, ValueError> {
    match config.value(section, key)? {
        None => Ok(None),
        Some(value) if value.trim() == "0" => Ok(Some(false)),
        Some(value) if value.trim() == "1" => Ok(Some(true)),
        Some(value) => Err(ValueError::Invalid {
            section: section.into(),
            key: key.into(),
            value: value.into(),
        }),
    }
}

fn governor_pid(config: &Config) -> Result<Option<[f64; 5]>, ValueError> {
    let Some(value) = config.value("helicopter", "governor_pid")? else {
        return Ok(None);
    };
    let values = value
        .split(',')
        .map(|item| camera::parse_number("helicopter", "governor_pid", item))
        .collect::<Result<Vec<_>, _>>()?;
    values
        .try_into()
        .map(Some)
        .map_err(|_| ValueError::Invalid {
            section: "helicopter".into(),
            key: "governor_pid".into(),
            value: value.into(),
        })
}

impl Helicopter {
    pub fn from_config(config: &Config) -> Result<Self, ValueError> {
        let n = |key| camera::number(config, "helicopter", key);
        Ok(Self {
            lift_aero_center_feet: n("lift_aero_center")?,
            side_aero_center_feet: n("side_aero_center")?,
            reference_length_feet: n("reference_length")?,
            reference_frontal_area_sq_feet: n("reference_frontal_area")?,
            reference_side_area_sq_feet: n("reference_side_area")?,
            low_realism_stability_scale: camera::vector(
                config,
                "helicopter",
                "low_realism_stability_scale",
            )?,
            correlator_available: flag(config, "helicopter", "correlator_available")?,
            governed_pct_rpm_ref: n("governed_pct_rpm_ref")?,
            governor_pid: governor_pid(config)?,
            right_trim_scalar: n("right_trim_scalar")?,
            rotor_brake_scalar: n("rotor_brake_scalar")?,
            torque_scalar: n("torque_scalar")?,
            cyclic_roll_control_scalar: n("cyclic_roll_control_scalar")?,
            cyclic_pitch_control_scalar: n("cyclic_pitch_control_scalar")?,
            pedal_control_scalar: n("pedal_control_scalar")?,
            collective_on_rotor_torque_scalar: n("collective_on_rotor_torque_scalar")?,
        })
    }
}

impl FuselageAerodynamics {
    pub fn from_config(config: &Config) -> Result<Self, ValueError> {
        let n = |key| camera::number(config, "fuselage_aerodynamics", key);
        Ok(Self {
            drag_force: n("drag_force_cf")?,
            side_drag_force: n("side_drag_force_cf")?,
            pitch_damping: n("pitch_damp_cf")?,
            roll_damping: n("roll_damp_cf")?,
            yaw_damping: n("yaw_damp_cf")?,
            yaw_stability: n("yaw_stability_cf")?,
        })
    }
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
        Ok(Self {
            position_feet: camera::vector(config, "secondaryrotor", "position")?,
            radius_feet: camera::number(config, "secondaryrotor", "radius")?,
            tail_rotor: flag(config, "secondaryrotor", "tailrotor")?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_helicopter_controls_governor_and_fuselage() {
        let config = Config::parse("[helicopter]\nreference_length=21.58\nreference_frontal_area=17.7\nside_aero_center=-12.5\nlow_realism_stability_scale=1,1.1,0.9\ncorrelator_available=1\ngoverned_pct_rpm_ref=1.04\ngovernor_pid=0.4,0,0.1,0,0.2\ncyclic_roll_control_scalar=1\n[fuselage_aerodynamics]\ndrag_force_cf=0.55\nside_drag_force_cf=10\npitch_damp_cf=-2\nyaw_stability_cf=0.27").unwrap();
        let helicopter = Helicopter::from_config(&config).unwrap();
        assert_eq!(helicopter.reference_length_feet, Some(21.58));
        assert_eq!(helicopter.reference_frontal_area_sq_feet, Some(17.7));
        assert_eq!(helicopter.side_aero_center_feet, Some(-12.5));
        assert_eq!(
            helicopter.low_realism_stability_scale,
            Some([1.0, 1.1, 0.9])
        );
        assert_eq!(helicopter.correlator_available, Some(true));
        assert_eq!(helicopter.governor_pid, Some([0.4, 0.0, 0.1, 0.0, 0.2]));
        assert_eq!(helicopter.pedal_control_scalar, None);
        let fuselage = FuselageAerodynamics::from_config(&config).unwrap();
        assert_eq!(fuselage.drag_force, Some(0.55));
        assert_eq!(fuselage.pitch_damping, Some(-2.0));
        assert_eq!(fuselage.yaw_stability, Some(0.27));
        assert_eq!(fuselage.yaw_damping, None);
        for text in [
            "[helicopter]\ngovernor_pid=1,2,3,4",
            "[helicopter]\ngovernor_pid=1,2,3,4,NaN",
            "[helicopter]\ncorrelator_available=2",
            "[helicopter]\nlow_realism_stability_scale=1,2",
        ] {
            assert!(Helicopter::from_config(&Config::parse(text).unwrap()).is_err());
        }
        assert!(
            FuselageAerodynamics::from_config(
                &Config::parse("[fuselage_aerodynamics]\npitch_damp_cf=inf").unwrap()
            )
            .is_err()
        );
    }

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

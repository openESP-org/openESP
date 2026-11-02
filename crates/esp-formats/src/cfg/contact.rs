use super::{Config, ValueError, camera::parse_number, indexed_name};

#[derive(Debug, Clone, PartialEq)]
pub struct ContactPoint {
    pub index: u32,
    pub class: u32,
    pub position_feet: [f64; 3],
    pub impact_feet_per_minute: f64,
    pub brake_map: u32,
    pub wheel_radius_feet: f64,
    pub steering_degrees: f64,
    pub static_compression_feet: f64,
    pub maximum_compression_ratio: f64,
    pub damping_ratio: f64,
    pub extension_seconds: f64,
    pub retraction_seconds: f64,
    pub sound_type: u32,
    pub extension_limit_knots: f64,
    pub damage_limit_knots: f64,
}

#[derive(Debug)]
pub struct RejectedPoint {
    pub index: u32,
    pub source: String,
    pub error: ValueError,
}

#[derive(Debug)]
pub struct ContactPoints {
    pub static_pitch_degrees: Option<f64>,
    pub static_cg_height_feet: Option<f64>,
    pub gear_system_type: Option<u32>,
    pub points: Vec<ContactPoint>,
    /// Malformed rows remain available to the caller for compatibility
    /// diagnostics; they must not silently become collision geometry.
    pub rejected: Vec<RejectedPoint>,
}

fn invalid(key: &str, value: &str) -> ValueError {
    ValueError::Invalid {
        section: "contact_points".into(),
        key: key.into(),
        value: value.into(),
    }
}

fn integer(key: &str, value: f64) -> Result<u32, ValueError> {
    if value.fract() != 0.0 || !(0.0..=f64::from(u32::MAX)).contains(&value) {
        return Err(invalid(key, &value.to_string()));
    }
    Ok(value as u32)
}

impl ContactPoint {
    pub fn parse(index: u32, source: &str) -> Result<Self, ValueError> {
        let key = format!("point.{index}");
        let fields = source
            .split(',')
            .map(|s| parse_number("contact_points", &key, s))
            .collect::<Result<Vec<_>, _>>()?;
        if !(14..=16).contains(&fields.len()) {
            return Err(invalid(&key, source));
        }
        let nonnegative = |index| -> Result<f64, ValueError> {
            let value = fields[index];
            if value < 0.0 {
                return Err(invalid(&key, source));
            }
            Ok(value)
        };
        let brake_map = integer(&key, fields[5])?;
        if brake_map > 2 {
            return Err(invalid(&key, source));
        }
        Ok(Self {
            index,
            class: integer(&key, fields[0])?,
            position_feet: [fields[1], fields[2], fields[3]],
            impact_feet_per_minute: nonnegative(4)?,
            brake_map,
            wheel_radius_feet: nonnegative(6)?,
            steering_degrees: nonnegative(7)?,
            static_compression_feet: nonnegative(8)?,
            maximum_compression_ratio: nonnegative(9)?,
            damping_ratio: nonnegative(10)?,
            extension_seconds: nonnegative(11)?,
            retraction_seconds: nonnegative(12)?,
            sound_type: integer(&key, fields[13])?,
            extension_limit_knots: if fields.len() > 14 {
                nonnegative(14)?
            } else {
                0.0
            },
            damage_limit_knots: if fields.len() > 15 {
                nonnegative(15)?
            } else {
                0.0
            },
        })
    }
}

impl ContactPoints {
    pub fn from_config(config: &Config) -> Result<Self, ValueError> {
        let mut rows = Vec::new();
        for (section, properties) in &config.sections {
            let Some(section) = section.filter(|s| s.eq_ignore_ascii_case("contact_points")) else {
                continue;
            };
            for (key, value) in properties {
                if let Some(index) = indexed_name(key, "point.")? {
                    config.value(section, key)?;
                    rows.push((index, value));
                }
            }
        }
        rows.sort_by_key(|r| r.0);
        if rows.windows(2).any(|p| p[0].0 == p[1].0) {
            return Err(ValueError::Duplicate {
                section: "contact_points".into(),
                key: "point".into(),
            });
        }
        let mut points = Vec::new();
        let mut rejected = Vec::new();
        for (index, source) in rows {
            match ContactPoint::parse(index, source) {
                Ok(point) => points.push(point),
                Err(error) => rejected.push(RejectedPoint {
                    index,
                    source: source.into(),
                    error,
                }),
            }
        }
        let static_cg_height_feet = config.number("contact_points", "static_cg_height")?;
        if static_cg_height_feet.is_some_and(|v| v < 0.0) {
            return Err(invalid(
                "static_cg_height",
                &static_cg_height_feet.unwrap().to_string(),
            ));
        }
        Ok(Self {
            static_pitch_degrees: config.number("contact_points", "static_pitch")?,
            static_cg_height_feet,
            gear_system_type: config
                .number("contact_points", "gear_system_type")?
                .map(|v| integer("gear_system_type", v))
                .transpose()?,
            points,
            rejected,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn retains_landing_gear_geometry_suspension_and_optional_speed_limits() {
        let point =
            ContactPoint::parse(3, "1,2,-3,-4,1200,2,0.6,35,0.3,2.5,0.8,4,5,3,180,220").unwrap();
        assert_eq!(point.position_feet, [2.0, -3.0, -4.0]);
        assert_eq!((point.brake_map, point.sound_type), (2, 3));
        assert_eq!(
            (
                point.static_compression_feet,
                point.maximum_compression_ratio,
                point.damping_ratio
            ),
            (0.3, 2.5, 0.8)
        );
        assert_eq!(
            (point.extension_seconds, point.retraction_seconds),
            (4.0, 5.0)
        );
        assert_eq!(
            (point.extension_limit_knots, point.damage_limit_knots),
            (180.0, 220.0)
        );
        let fixed = ContactPoint::parse(0, "1,1,0,-2,1200,0,0.5,20,0.2,2.5,0.7,0,0,0").unwrap();
        assert_eq!(
            (fixed.extension_limit_knots, fixed.damage_limit_knots),
            (0.0, 0.0)
        );
    }

    #[test]
    fn reports_malformed_rows_without_guessing_coordinates() {
        let config = Config::parse("[CONTACT_POINTS]\nstatic_pitch=2\nstatic_cg_height=3\ngear_system_type=1\npoint.4=2,-2 5,3,1200,0,0,0,0,0,0,0,0,5\npoint.1=1,1,0,-2,1200,0,0.5,20,0.2,2.5,0.7,0,0,0").unwrap();
        let contacts = ContactPoints::from_config(&config).unwrap();
        assert_eq!(contacts.static_pitch_degrees, Some(2.0));
        assert_eq!(contacts.static_cg_height_feet, Some(3.0));
        assert_eq!(contacts.gear_system_type, Some(1));
        assert_eq!(contacts.points.len(), 1);
        assert_eq!(contacts.rejected[0].index, 4);
        assert!(contacts.rejected[0].source.contains("-2 5"));
        let duplicate = Config::parse("[contact_points]\npoint.1=0\npoint.01=0").unwrap();
        assert!(ContactPoints::from_config(&duplicate).is_err());
        for source in [
            "1,0,0,0,1200,3,0,0,0,0,0,0,0,0",
            "1,0,0,0,1200,0,-1,0,0,0,0,0,0,0",
            "1,0,NaN,0,1200,0,1,0,0,0,0,0,0,0",
        ] {
            assert!(ContactPoint::parse(0, source).is_err());
        }
    }
}

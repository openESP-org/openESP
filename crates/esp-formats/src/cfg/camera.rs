use super::{Config, ValueError, indexed_name};
use std::collections::HashSet;
use uuid::Uuid;

#[derive(Debug, PartialEq)]
pub struct Camera<'a> {
    pub index: u32,
    pub section: &'a str,
    pub title: &'a str,
    pub guid: Uuid,
    pub origin: Option<&'a str>,
    pub category: Option<&'a str>,
    pub description: Option<&'a str>,
    pub initial_xyz_meters: Option<[f64; 3]>,
    pub initial_pbh_degrees: Option<[f64; 3]>,
    pub initial_zoom: Option<f64>,
}

#[derive(Debug, PartialEq)]
pub struct Views {
    // Aircraft eyepoints use longitudinal, lateral, vertical order, unlike camera XYZ.
    pub eyepoint_feet: Option<[f64; 3]>,
    pub zoom: Option<f64>,
}

impl Views {
    pub fn from_config(config: &Config) -> Result<Self, ValueError> {
        Ok(Self {
            eyepoint_feet: vector(config, "views", "eyepoint")?,
            zoom: number(config, "views", "zoom")?,
        })
    }
}

pub fn definitions(config: &Config) -> Result<Vec<Camera<'_>>, ValueError> {
    let mut cameras = Vec::new();
    let mut indices = HashSet::new();
    let mut guids = HashSet::new();
    for (section, _) in &config.sections {
        let Some(section) = section else { continue };
        let Some(index) = indexed_name(section, "cameradefinition.")? else {
            continue;
        };
        if !indices.insert(index) {
            return Err(ValueError::Duplicate {
                section: section.into(),
                key: "section".into(),
            });
        }
        let title = required(config, section, "title")?;
        let value = required(config, section, "guid")?;
        let guid = Uuid::parse_str(value).map_err(|_| invalid(section, "guid", value))?;
        if !guids.insert(guid) {
            return Err(ValueError::Duplicate {
                section: section.into(),
                key: "guid".into(),
            });
        }
        cameras.push(Camera {
            index,
            section,
            title,
            guid,
            origin: config.value(section, "origin")?,
            category: config.value(section, "category")?,
            description: config.value(section, "description")?,
            initial_xyz_meters: vector(config, section, "initialxyz")?,
            initial_pbh_degrees: vector(config, section, "initialpbh")?,
            initial_zoom: number(config, section, "initialzoom")?,
        });
    }
    cameras.sort_by_key(|camera| camera.index);
    Ok(cameras)
}

fn required<'a>(config: &'a Config, section: &str, key: &str) -> Result<&'a str, ValueError> {
    config
        .value(section, key)?
        .filter(|value| !value.is_empty())
        .ok_or_else(|| ValueError::Missing {
            section: section.into(),
            key: key.into(),
        })
}

fn invalid(section: &str, key: &str, value: &str) -> ValueError {
    ValueError::Invalid {
        section: section.into(),
        key: key.into(),
        value: value.into(),
    }
}

fn parse_number(section: &str, key: &str, value: &str) -> Result<f64, ValueError> {
    value
        .trim()
        .parse::<f64>()
        .ok()
        .filter(|value| value.is_finite())
        .ok_or_else(|| invalid(section, key, value))
}

fn number(config: &Config, section: &str, key: &str) -> Result<Option<f64>, ValueError> {
    config
        .value(section, key)?
        .map(|value| parse_number(section, key, value))
        .transpose()
}

fn vector(config: &Config, section: &str, key: &str) -> Result<Option<[f64; 3]>, ValueError> {
    let Some(value) = config.value(section, key)? else {
        return Ok(None);
    };
    let mut fields = value.split(',');
    let mut next = || {
        fields
            .next()
            .ok_or_else(|| invalid(section, key, value))
            .and_then(|value| parse_number(section, key, value))
    };
    let result = [next()?, next()?, next()?];
    if fields.next().is_some() {
        return Err(invalid(section, key, value));
    }
    Ok(Some(result))
}

#[cfg(test)]
mod tests {
    use super::*;

    const CAMERA: &str = "[CameraDefinition.3]\nTitle=Pilot\nGuid={01020304-0506-0708-090a-0b0c0d0e0f10}\nOrigin=Virtual Cockpit\nCategory=Cockpit\nInitialXyz=-0.45, 1.2, 3\nInitialPbh=10, 0, -90\nInitialZoom=0.8\nCustom=retained";

    #[test]
    fn reads_camera_positions_in_their_original_units() {
        let config = Config::parse(&format!(
            "{CAMERA}\n[Views]\neyepoint=43.7,-1.7,4.3\nzoom=1.1"
        ))
        .unwrap();
        let cameras = definitions(&config).unwrap();
        let camera = &cameras[0];
        assert_eq!(camera.index, 3);
        assert_eq!(
            camera.guid,
            Uuid::from_u128(0x0102030405060708090a0b0c0d0e0f10)
        );
        assert_eq!(camera.title, "Pilot");
        assert_eq!(camera.origin, Some("Virtual Cockpit"));
        assert_eq!(camera.initial_xyz_meters, Some([-0.45, 1.2, 3.0]));
        assert_eq!(camera.initial_pbh_degrees, Some([10.0, 0.0, -90.0]));
        assert_eq!(camera.initial_zoom, Some(0.8));
        assert_eq!(camera.description, None);
        assert_eq!(
            Views::from_config(&config).unwrap(),
            Views {
                eyepoint_feet: Some([43.7, -1.7, 4.3]),
                zoom: Some(1.1)
            }
        );
        assert_eq!(
            config.value(camera.section, "custom").unwrap(),
            Some("retained")
        );
    }

    #[test]
    fn orders_definitions_and_keeps_optional_values_absent() {
        let config = Config::parse(&format!("{CAMERA}\n[CAMERADEFINITION.01]\nTITLE=Other\nGUID={{00000000-0000-0000-0000-000000000001}}")).unwrap();
        let cameras = definitions(&config).unwrap();
        assert_eq!(
            cameras
                .iter()
                .map(|camera| camera.index)
                .collect::<Vec<_>>(),
            [1, 3]
        );
        assert_eq!(cameras[0].initial_xyz_meters, None);
        let empty = Config::parse("").unwrap();
        assert!(definitions(&empty).unwrap().is_empty());
        assert_eq!(
            Views::from_config(&empty).unwrap(),
            Views {
                eyepoint_feet: None,
                zoom: None
            }
        );
    }

    #[test]
    fn rejects_ambiguous_and_malformed_camera_metadata() {
        for extra in [
            "\nTitle=Duplicate",
            "\nGuid=bad",
            "\nInitialZoom=NaN",
            "\n[CameraDefinition.03]",
            "\n[CameraDefinition.4]\nTitle=Same Guid\nGuid={01020304-0506-0708-090a-0b0c0d0e0f10}",
        ] {
            assert!(definitions(&Config::parse(&format!("{CAMERA}{extra}")).unwrap()).is_err());
        }
        for metadata in [
            "Title=Pilot",
            "Guid={00000000-0000-0000-0000-000000000001}",
            "Title=\nGuid=invalid",
            "Title=Pilot\nGuid=invalid",
        ] {
            assert!(
                definitions(&Config::parse(&format!("[CameraDefinition.0]\n{metadata}")).unwrap())
                    .is_err()
            );
        }
        for value in [
            "", "1,2", "1,2,3,", "1,2,3,4", "NaN,2,3", "1,inf,3", "1,2,no",
        ] {
            let config = Config::parse(&format!("[views]\neyepoint={value}")).unwrap();
            assert!(matches!(
                Views::from_config(&config),
                Err(ValueError::Invalid { .. })
            ));
        }
    }
}

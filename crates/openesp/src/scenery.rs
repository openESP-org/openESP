use anyhow::{Context, Result, ensure};
use glam::DVec3;
use std::path::Path;

use crate::navigation::Origin;

pub struct AirportScene {
    pub runways: Vec<esp_render::Runway>,
    pub taxiways: Vec<esp_render::Taxiway>,
    pub aprons: Vec<esp_render::Apron>,
}

/// Place BGL runways and taxiways on the current local ground plane.
pub fn airport(
    path: &Path,
    ident: Option<&str>,
    latitude: f64,
    longitude: f64,
) -> Result<AirportScene> {
    let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    let airports = esp_formats::bgl::Bgl::parse(&bytes)?.airports()?;
    let airport = airports
        .iter()
        .find(|airport| {
            ident.map_or(!airport.runways.is_empty(), |ident| {
                airport.ident.eq_ignore_ascii_case(ident)
            })
        })
        .with_context(|| {
            format!(
                "airport {} not found in {}",
                ident.unwrap_or("with runways"),
                path.display()
            )
        })?;
    ensure!(
        !airport.runways.is_empty(),
        "airport {} has no runways",
        airport.ident
    );
    let origin = Origin::new(latitude, longitude)?;
    let runways = airport
        .runways
        .iter()
        .map(|runway| {
            ensure!(
                runway.length_m.is_finite()
                    && (1.0..=20_000.0).contains(&runway.length_m)
                    && runway.width_m.is_finite()
                    && (1.0..=1_000.0).contains(&runway.width_m)
                    && runway.heading_deg.is_finite(),
                "invalid runway geometry at {}",
                airport.ident
            );
            let (north, east) = origin.local_position(runway.latitude, runway.longitude)?;
            let heading = f64::from(runway.heading_deg).to_radians();
            let half_length = f64::from(runway.length_m) * 0.5;
            Ok(esp_render::Runway {
                threshold: DVec3::new(
                    east - heading.sin() * half_length,
                    0.0,
                    north - heading.cos() * half_length,
                ),
                heading,
                length: runway.length_m,
                width: runway.width_m,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    let mut taxiways = Vec::new();
    for path in &airport.taxi_paths {
        // Parking links use a different endpoint index space; runway links are
        // already covered by the dedicated runway surface.
        if path.kind != 1
            || !path.draw_surface
            || !path.width_m.is_finite()
            || !(0.1..=1000.0).contains(&path.width_m)
        {
            continue;
        }
        let (Some(start), Some(end)) = (
            airport.taxi_points.get(path.start as usize),
            airport.taxi_points.get(path.end as usize),
        ) else {
            continue;
        };
        let (start_north, start_east) = origin.local_position(start.latitude, start.longitude)?;
        let (end_north, end_east) = origin.local_position(end.latitude, end.longitude)?;
        taxiways.push(esp_render::Taxiway {
            start: DVec3::new(start_east, 0.0, start_north),
            end: DVec3::new(end_east, 0.0, end_north),
            width: path.width_m,
            centerline: path.centerline,
        });
    }
    let aprons = airport
        .aprons
        .iter()
        .filter(|apron| apron.draw_surface)
        .map(|apron| {
            let vertices = apron
                .vertices
                .iter()
                .map(|&(latitude, longitude)| {
                    let (north, east) = origin.local_position(latitude, longitude)?;
                    Ok(DVec3::new(east, 0.0, north))
                })
                .collect::<Result<Vec<_>>>()?;
            Ok(esp_render::Apron {
                vertices,
                indices: apron.indices.clone(),
            })
        })
        .collect::<Result<Vec<_>>>()?;
    Ok(AirportScene {
        runways,
        taxiways,
        aprons,
    })
}

/// Load explicitly supplied model libraries and their placed objects.
pub fn objects(
    placement_path: &Path,
    libraries: &[std::path::PathBuf],
    texture_directory: Option<&Path>,
    latitude: f64,
    longitude: f64,
) -> Result<Vec<esp_render::SceneryObject>> {
    use std::{collections::HashMap, sync::Arc};
    let mut models = HashMap::new();
    for path in std::iter::once(placement_path).chain(libraries.iter().map(|p| p.as_path())) {
        let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
        let bgl = esp_formats::bgl::Bgl::parse(&bytes)?;
        let parent = path.parent().unwrap_or(Path::new("."));
        let textures = texture_directory.map(Path::to_path_buf).unwrap_or_else(|| {
            parent
                .parent()
                .and_then(|root| esp_assets::resolve_path(root, "texture").ok())
                .unwrap_or_else(|| parent.to_path_buf())
        });
        for section in bgl
            .sections
            .iter()
            .filter(|section| section.kind == 0x2b && section.class == 5)
        {
            for record in section.guid_records()? {
                match esp_assets::Model::from_bytes(record.data) {
                    Ok(model) => {
                        models.insert(*record.guid.as_bytes(), (Arc::new(model), textures.clone()));
                    }
                    Err(error) => eprintln!("scenery model {}: {error:#}", record.guid),
                }
            }
        }
    }
    let bytes = std::fs::read(placement_path)?;
    let bgl = esp_formats::bgl::Bgl::parse(&bytes)?;
    let origin = Origin::new(latitude, longitude)?;
    let mut result = Vec::new();
    for section in bgl.sections.iter().filter(|section| section.kind == 0x25) {
        let esp_formats::bgl::Directory::Direct(subsections) = &section.directory else {
            continue;
        };
        for subsection in subsections {
            for record in esp_formats::bgl::scenery::parse(subsection.records.data)? {
                let Some(object) = record.decode()? else {
                    continue;
                };
                let esp_formats::bgl::scenery::Definition::Library { guid, scale } =
                    object.definition
                else {
                    continue;
                };
                let Some((model, textures)) = models.get(guid.as_bytes()) else {
                    continue;
                };
                ensure!(
                    scale.is_finite() && scale > 0.0,
                    "invalid scenery object scale"
                );
                let (north, east) = origin.local_position(
                    object.placement.latitude_degrees(),
                    object.placement.longitude_degrees(),
                )?;
                let [pitch, bank, heading] =
                    object.placement.orientation_degrees().map(f64::to_radians);
                // Ground elevation is currently zero. Absolute MSL objects will require terrain elevation.
                let height = if object.placement.flags & 1 != 0 {
                    object.placement.altitude_meters()
                } else {
                    0.0
                };
                let transform = glam::Mat4::from_translation(glam::Vec3::new(
                    east as f32,
                    height as f32,
                    north as f32,
                )) * glam::Mat4::from_rotation_y(heading as f32)
                    * glam::Mat4::from_rotation_x(pitch as f32)
                    * glam::Mat4::from_rotation_z(bank as f32)
                    * glam::Mat4::from_scale(glam::Vec3::splat(scale));
                result.push(esp_render::SceneryObject {
                    model: model.clone(),
                    transform,
                    textures: textures.clone(),
                });
            }
        }
    }
    eprintln!("scenery: loaded {} placed model(s)", result.len());
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore = "requires a P3D airport BGL in OPENESP_AIRPORT_BGL"]
    fn loads_real_airport_runway_positions() {
        let path = std::env::var_os("OPENESP_AIRPORT_BGL").expect("set OPENESP_AIRPORT_BGL");
        let scene = airport(Path::new(&path), Some("KLAX"), 33.9377117, -118.4010096).unwrap();
        let runways = scene.runways;
        assert_eq!(scene.taxiways.len(), 406);
        assert_eq!(scene.aprons.len(), 22);
        assert_eq!(runways.len(), 4);
        assert!(runways.iter().any(|r| (r.length - 3938.93).abs() < 0.1));
        let first = runways
            .iter()
            .find(|r| (r.length - 3938.93).abs() < 0.1)
            .unwrap();
        let center = first.threshold
            + DVec3::new(first.heading.sin(), 0.0, first.heading.cos())
                * f64::from(first.length)
                * 0.5;
        assert!(center.length() < 2.0);
        assert!(super::airport(Path::new(&path), Some("ZZZZ"), 33.9, -118.4).is_err());
    }
}

#[cfg(test)]
mod object_tests {
    use super::*;

    #[test]
    #[ignore = "requires scenery BGL with a placed model in OPENESP_SCENERY_OBJECT_BGL"]
    fn loads_real_placed_model_from_embedded_library() {
        let path =
            std::env::var_os("OPENESP_SCENERY_OBJECT_BGL").expect("set OPENESP_SCENERY_OBJECT_BGL");
        let objects = objects(Path::new(&path), &[], None, 48.63158628, 9.43264246).unwrap();
        assert_eq!(objects.len(), 1);
        assert!(
            objects[0]
                .transform
                .transform_point3(glam::Vec3::ZERO)
                .length()
                < 1.0
        );
        assert!(!objects[0].model.lods[0].meshes.is_empty());
    }
}

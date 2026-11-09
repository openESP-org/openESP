use anyhow::{Context, Result, ensure};
use glam::DVec3;
use std::path::Path;

use crate::navigation::Origin;

pub struct AirportScene {
    pub elevation_m: f64,
    pub runways: Vec<esp_render::Runway>,
    pub taxiways: Vec<esp_render::Taxiway>,
    pub aprons: Vec<esp_render::Apron>,
    pub runway_surfaces: Vec<u16>,
    pub taxiway_surfaces: Vec<u16>,
    pub apron_surfaces: Vec<u16>,
}

/// Place BGL airport surfaces on the current local ground plane.
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
            ident.map_or(
                !airport.runways.is_empty() || !airport.helipads.is_empty(),
                |ident| airport.ident.eq_ignore_ascii_case(ident),
            )
        })
        .with_context(|| {
            format!(
                "airport {} not found in {}",
                ident.unwrap_or("with runways or helipads"),
                path.display()
            )
        })?;
    airport_scene(airport, latitude, longitude)
}

fn airport_scene(
    airport: &esp_formats::bgl::airport::Airport,
    latitude: f64,
    longitude: f64,
) -> Result<AirportScene> {
    ensure!(
        !airport.runways.is_empty() || !airport.helipads.is_empty(),
        "airport {} has no runways or helipads",
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
    let mut taxiway_surfaces = Vec::new();
    for path in &airport.taxi_paths {
        // Runway links are already covered by the dedicated runway surface.
        if !matches!(path.kind, 1 | 3)
            || !path.draw_surface
            || !path.width_m.is_finite()
            || !(0.1..=1000.0).contains(&path.width_m)
        {
            continue;
        }
        let Some(start) = airport.taxi_points.get(path.start as usize) else {
            continue;
        };
        let end = match path.kind {
            1 => airport
                .taxi_points
                .get(path.end as usize)
                .map(|p| (p.latitude, p.longitude)),
            3 => airport
                .taxi_parkings
                .get(path.end as usize)
                .map(|p| (p.latitude, p.longitude)),
            _ => None,
        };
        let Some((end_latitude, end_longitude)) = end else {
            continue;
        };
        let (start_north, start_east) = origin.local_position(start.latitude, start.longitude)?;
        let (end_north, end_east) = origin.local_position(end_latitude, end_longitude)?;
        taxiways.push(esp_render::Taxiway {
            start: DVec3::new(start_east, 0.0, start_north),
            end: DVec3::new(end_east, 0.0, end_north),
            width: path.width_m,
            centerline: path.centerline,
        });
        taxiway_surfaces.push(u16::from(path.surface));
    }
    let mut aprons = airport
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
    let mut apron_surfaces: Vec<u16> = airport
        .aprons
        .iter()
        .filter(|apron| apron.draw_surface)
        .map(|apron| u16::from(apron.surface))
        .collect();
    for pad in airport.helipads.iter().filter(|pad| !pad.transparent) {
        ensure!(
            pad.length_m.is_finite()
                && (0.1..=1000.0).contains(&pad.length_m)
                && pad.width_m.is_finite()
                && (0.1..=1000.0).contains(&pad.width_m)
                && pad.heading_deg.is_finite(),
            "invalid helipad geometry at {}",
            airport.ident
        );
        let (north, east) = origin.local_position(pad.latitude, pad.longitude)?;
        let heading = f64::from(pad.heading_deg).to_radians();
        let along = DVec3::new(heading.sin(), 0.0, heading.cos()) * f64::from(pad.length_m) * 0.5;
        let across = DVec3::new(heading.cos(), 0.0, -heading.sin()) * f64::from(pad.width_m) * 0.5;
        let center = DVec3::new(east, 0.0, north);
        aprons.push(esp_render::Apron {
            vertices: vec![
                center - along - across,
                center + along - across,
                center + along + across,
                center - along + across,
            ],
            indices: vec![0, 1, 2, 0, 2, 3],
        });
        apron_surfaces.push(u16::from(pad.surface));
    }
    Ok(AirportScene {
        elevation_m: airport.elevation_m,
        runways,
        taxiways,
        aprons,
        runway_surfaces: airport
            .runways
            .iter()
            .map(|runway| runway.surface)
            .collect(),
        taxiway_surfaces,
        apron_surfaces,
    })
}

/// Load explicitly supplied model libraries and their placed objects.
#[allow(clippy::too_many_arguments)]
pub fn objects(
    placement_path: &Path,
    libraries: &[std::path::PathBuf],
    texture_directory: Option<&Path>,
    latitude: f64,
    longitude: f64,
    ground: Option<&esp_sim::flight::ground::Ground>,
    reference_msl: Option<f64>,
    simulator_root: Option<&Path>,
) -> Result<Vec<esp_render::SceneryObject>> {
    use std::{
        collections::{HashMap, HashSet},
        sync::Arc,
    };
    let mut seen = HashSet::new();
    let mut paths = Vec::new();
    for path in std::iter::once(placement_path).chain(libraries.iter().map(|p| p.as_path())) {
        let identity =
            std::fs::canonicalize(path).with_context(|| format!("resolving {}", path.display()))?;
        if seen.insert(identity) {
            paths.push(path);
        }
    }
    let mut models = HashMap::new();
    let origin = Origin::new(latitude, longitude)?;
    let mut airport_elevation: Option<(f64, f64)> = None;
    for path in &paths {
        let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
        let bgl = esp_formats::bgl::Bgl::parse(&bytes)?;
        if let Ok(airports) = bgl.airports() {
            for airport in airports {
                let Ok((north, east)) = origin.local_position(airport.latitude, airport.longitude)
                else {
                    continue;
                };
                let distance = north.hypot(east);
                if distance <= 25_000.0
                    && airport.elevation_m.is_finite()
                    && airport_elevation.is_none_or(|(best, _)| distance < best)
                {
                    airport_elevation = Some((distance, airport.elevation_m));
                }
            }
        }
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
    // The airport scene uses a flat ground plane at the selected airport's
    // elevation. Object-only libraries do not contain a terrain datum.
    let airport_elevation =
        reference_msl.or_else(|| airport_elevation.map(|(_, elevation)| elevation));
    let mut missing = HashSet::new();
    if simulator_root.is_some() {
        for path in &paths {
            let bytes =
                std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
            let bgl = esp_formats::bgl::Bgl::parse(&bytes)?;
            for section in bgl.sections.iter().filter(|section| section.kind == 0x25) {
                let esp_formats::bgl::Directory::Direct(subsections) = &section.directory else {
                    continue;
                };
                for subsection in subsections {
                    for record in esp_formats::bgl::scenery::parse(subsection.records.data)? {
                        if let Some(object) = record.decode()?
                            && let esp_formats::bgl::scenery::Definition::Library { guid, .. } =
                                object.definition
                            && !models.contains_key(guid.as_bytes())
                        {
                            missing.insert(*guid.as_bytes());
                        }
                    }
                }
            }
        }
        if let Some(root) = simulator_root {
            global_models::load(root, &mut missing, &mut models);
        }
    }
    let mut result = Vec::new();
    for path in paths {
        let bytes = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
        let bgl = esp_formats::bgl::Bgl::parse(&bytes)
            .with_context(|| format!("parsing {}", path.display()))?;
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
                    let local_ground = ground.map(|ground| ground.surface(north, east).elevation);
                    let height =
                        placement_height(&object.placement, airport_elevation, local_ground);
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
    }
    eprintln!("scenery: loaded {} placed model(s)", result.len());
    Ok(result)
}

mod global_models;

fn placement_height(
    placement: &esp_formats::bgl::scenery::Placement,
    ground_msl: Option<f64>,
    local_ground: Option<f64>,
) -> f64 {
    if placement.flags & 1 != 0 {
        local_ground.unwrap_or(0.0) + placement.altitude_meters()
    } else {
        ground_msl.map_or(0.0, |ground| placement.altitude_meters() - ground)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn places_agl_and_msl_objects_against_airport_ground() {
        let mut placement = esp_formats::bgl::scenery::Placement {
            longitude: 0,
            latitude: 0,
            altitude: 105_000,
            flags: 0,
            pitch: 0,
            bank: 0,
            heading: 0,
            image_complexity: 0,
            unknown: 0,
            instance: Default::default(),
        };
        assert_eq!(placement_height(&placement, Some(100.0), Some(70.0)), 5.0);
        assert_eq!(placement_height(&placement, None, Some(70.0)), 0.0);
        placement.flags = 1;
        assert_eq!(placement_height(&placement, Some(100.0), Some(70.0)), 175.0);
        assert_eq!(placement_height(&placement, None, None), 105.0);
    }

    #[test]
    fn builds_heliport_without_runways() {
        use esp_formats::bgl::airport::{Airport, Helipad};
        let pad = Helipad {
            surface: 0,
            kind: 1,
            transparent: false,
            closed: false,
            latitude: 0.0,
            longitude: 0.0,
            elevation_m: 0.0,
            length_m: 30.0,
            width_m: 20.0,
            heading_deg: 90.0,
        };
        let mut airport = Airport {
            ident: "HEL1".into(),
            latitude: 0.0,
            longitude: 0.0,
            elevation_m: 0.0,
            runways: vec![],
            helipads: vec![pad],
            taxi_points: vec![],
            taxi_parkings: vec![],
            taxi_paths: vec![],
            aprons: vec![],
        };
        let scene = airport_scene(&airport, 0.0, 0.0).unwrap();
        assert!(scene.runways.is_empty());
        assert_eq!(scene.aprons.len(), 1);
        let vertices = &scene.aprons[0].vertices;
        assert!(((vertices[1] - vertices[0]).length() - 30.0).abs() < 1e-6);
        assert!(((vertices[2] - vertices[1]).length() - 20.0).abs() < 1e-6);
        assert!((vertices[1].x - vertices[0].x - 30.0).abs() < 1e-6);
        airport.helipads[0].transparent = true;
        assert!(airport_scene(&airport, 0.0, 0.0).unwrap().aprons.is_empty());
    }

    #[test]
    #[ignore = "requires a P3D airport BGL in OPENESP_AIRPORT_BGL"]
    fn loads_real_airport_runway_positions() {
        let path = std::env::var_os("OPENESP_AIRPORT_BGL").expect("set OPENESP_AIRPORT_BGL");
        let scene = airport(Path::new(&path), Some("KLAX"), 33.9377117, -118.4010096).unwrap();
        let runways = scene.runways;
        assert!(scene.taxiways.len() >= 406);
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

    #[test]
    #[ignore = "requires LTFO airport BGL in OPENESP_SCENERY_PLACEMENT_BGL"]
    fn loads_real_parking_taxi_links() {
        let path = std::env::var_os("OPENESP_SCENERY_PLACEMENT_BGL").unwrap();
        let bytes = std::fs::read(&path).unwrap();
        let airports = esp_formats::bgl::Bgl::parse(&bytes)
            .unwrap()
            .airports()
            .unwrap();
        let airport_data = airports
            .iter()
            .find(|airport| airport.ident == "LTFO")
            .unwrap();
        let parking_links = airport_data
            .taxi_paths
            .iter()
            .filter(|path| path.kind == 3)
            .collect::<Vec<_>>();
        assert!(!airport_data.taxi_parkings.is_empty());
        assert!(!parking_links.is_empty());
        for path in &parking_links {
            let stand = &airport_data.taxi_parkings[path.end as usize];
            assert!((stand.latitude - airport_data.latitude).abs() < 0.1);
            assert!((stand.longitude - airport_data.longitude).abs() < 0.1);
        }
        let scene = airport(
            Path::new(&path),
            Some("LTFO"),
            airport_data.latitude,
            airport_data.longitude,
        )
        .unwrap();
        assert_eq!(
            scene.runway_surfaces,
            airport_data
                .runways
                .iter()
                .map(|runway| runway.surface)
                .collect::<Vec<_>>()
        );
        assert_eq!(scene.taxiway_surfaces.len(), scene.taxiways.len());
        assert_eq!(scene.apron_surfaces.len(), scene.aprons.len());
        let normal_links = airport_data
            .taxi_paths
            .iter()
            .filter(|path| path.kind == 1 && path.draw_surface)
            .count();
        assert_eq!(
            scene.taxiways.len(),
            normal_links
                + parking_links
                    .iter()
                    .filter(|path| path.draw_surface)
                    .count()
        );
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
        let objects = objects(
            Path::new(&path),
            &[],
            None,
            48.63158628,
            9.43264246,
            None,
            None,
            None,
        )
        .unwrap();
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

    #[test]
    #[ignore = "requires P3D placement and library BGLs"]
    fn loads_p3d_scenery_objects_from_multiple_libraries() {
        let placement = std::env::var_os("OPENESP_SCENERY_PLACEMENT_BGL")
            .expect("set OPENESP_SCENERY_PLACEMENT_BGL");
        let libraries = std::env::var_os("OPENESP_SCENERY_OBJECT_BGLS")
            .expect("set OPENESP_SCENERY_OBJECT_BGLS");
        let libraries: Vec<_> = std::env::split_paths(&libraries).collect();
        let objects = objects(
            Path::new(&placement),
            &libraries,
            None,
            41.168,
            40.835,
            None,
            None,
            None,
        )
        .unwrap();
        assert!(!objects.is_empty(), "no placed objects found");
        let mut reversed = libraries.clone();
        reversed.push(placement.clone().into());
        reversed.push(placement.clone().into());
        let objects_from_library = super::objects(
            &libraries[0],
            &reversed,
            None,
            41.168,
            40.835,
            None,
            None,
            None,
        )
        .unwrap();
        assert_eq!(
            objects_from_library.len(),
            objects.len(),
            "placements in additional BGLs must load once"
        );
        let distinct: std::collections::HashSet<_> = objects
            .iter()
            .map(|object| std::sync::Arc::as_ptr(&object.model))
            .collect();
        assert!(
            distinct.len() >= 2,
            "expected models from multiple libraries"
        );
        for object in objects {
            assert!(object.transform.is_finite());
            assert!(
                object
                    .model
                    .lods
                    .iter()
                    .any(|lod| lod.meshes.iter().any(|mesh| !mesh.indices.is_empty()))
            );
        }
    }
}

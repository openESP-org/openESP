use anyhow::Result;
use glam::DVec3;

use crate::{cli::Arguments, navigation::Origin, scenery::AirportScene};

pub(super) fn ground(
    mesh: std::sync::Arc<esp_terrain::mesh::Mesh>,
    scene: &AirportScene,
) -> esp_sim::flight::ground::Ground {
    let mut ground = esp_sim::flight::ground::Ground::new(mesh);
    for runway in &scene.runways {
        let start = [runway.threshold.x, runway.threshold.z];
        let end = [
            start[0] + runway.heading.sin() * f64::from(runway.length),
            start[1] + runway.heading.cos() * f64::from(runway.length),
        ];
        ground.add_segment(start, end, f64::from(runway.width));
    }
    for taxiway in &scene.taxiways {
        ground.add_segment(
            [taxiway.start.x, taxiway.start.z],
            [taxiway.end.x, taxiway.end.z],
            f64::from(taxiway.width),
        );
    }
    for apron in &scene.aprons {
        for triangle in apron.indices.as_chunks::<3>().0 {
            let points = triangle.map(|index| {
                apron
                    .vertices
                    .get(index as usize)
                    .map(|point| [point.x, point.z])
            });
            if let [Some(a), Some(b), Some(c)] = points {
                ground.add_triangle([a, b, c]);
            }
        }
    }
    ground
}

pub(super) fn load(
    arguments: &Arguments,
    airport_elevation: f64,
) -> Result<Option<esp_terrain::mesh::Mesh>> {
    let roots = arguments.fsx_root.clone().map_or_else(
        || {
            esp_installs::discover()
                .into_iter()
                .map(|install| install.root)
                .collect()
        },
        |root| vec![root],
    );
    let mut selected = None;
    for root in roots {
        let terrain = esp_terrain::elevation::Terrain::load(
            &root,
            arguments.latitude,
            arguments.longitude,
            45_000.0,
        )?;
        if !terrain.is_empty() {
            selected = Some((root, terrain));
            break;
        }
    }
    let Some((root, terrain)) = selected else {
        return Ok(None);
    };
    let reference = if arguments.scenery_bgl.is_some() {
        airport_elevation
    } else {
        terrain
            .height(arguments.latitude, arguments.longitude)
            .unwrap_or(0.0)
    };
    let mesh = esp_terrain::mesh::Mesh::build(
        &terrain,
        arguments.latitude,
        arguments.longitude,
        30_000.0,
        256,
        reference,
    )?;
    if mesh.indices.is_empty() {
        return Ok(None);
    }
    eprintln!("terrain: loaded elevation from {}", root.display());
    Ok(Some(mesh))
}

pub(super) fn render_mesh(mesh: &esp_terrain::mesh::Mesh) -> esp_render::TerrainMesh {
    esp_render::TerrainMesh {
        vertices: mesh
            .vertices
            .iter()
            .copied()
            .map(DVec3::from_array)
            .collect(),
        normals: mesh
            .normals
            .iter()
            .copied()
            .map(DVec3::from_array)
            .collect(),
        indices: mesh.indices.clone(),
    }
}

pub(super) fn photos(
    arguments: &Arguments,
    terrain: Option<&esp_terrain::mesh::Mesh>,
) -> Result<Vec<esp_render::PhotoTile>> {
    let Some(folder) = arguments.scenery.as_deref() else {
        return Ok(Vec::new());
    };
    let directory = esp_assets::resolve_path(folder, "scenery")
        .ok()
        .filter(|path| path.is_dir())
        .unwrap_or_else(|| folder.to_path_buf());
    let tiles = esp_terrain::photos(&directory, arguments.latitude, arguments.longitude)?;
    let origin = Origin::new(arguments.latitude, arguments.longitude)?;
    let mut result = Vec::with_capacity(tiles.len());
    for tile in tiles {
        let bounds = tile.bounds;
        let point = |latitude, longitude| -> Result<DVec3> {
            let (north, east) = origin.local_position(latitude, longitude)?;
            let height = terrain
                .and_then(|mesh| mesh.height(east, north))
                .unwrap_or(0.0);
            Ok(DVec3::new(east, height, north))
        };
        let nw = point(bounds.north, bounds.west)?;
        let se = point(bounds.south, bounds.east)?;
        let extent = (se.x - nw.x).abs().max((se.z - nw.z).abs());
        let divisions = terrain.map_or(1, |mesh| {
            ((extent / mesh.cell_size() * 2.0).ceil() as usize).clamp(1, 128)
        });
        let mut vertices = Vec::with_capacity((divisions + 1) * (divisions + 1));
        let mut uvs = Vec::with_capacity(vertices.capacity());
        for row in 0..=divisions {
            let v = row as f64 / divisions as f64;
            for col in 0..=divisions {
                let u = col as f64 / divisions as f64;
                vertices.push(point(
                    bounds.north + (bounds.south - bounds.north) * v,
                    bounds.west + (bounds.east - bounds.west) * u,
                )?);
                uvs.push([u as f32, v as f32]);
            }
        }
        let mut indices = Vec::with_capacity(divisions * divisions * 6);
        for row in 0..divisions {
            for col in 0..divisions {
                let a = (row * (divisions + 1) + col) as u32;
                let b = a + 1;
                let c = a + divisions as u32 + 1;
                let d = c + 1;
                indices.extend([a, c, d, a, d, b]);
            }
        }
        result.push(esp_render::PhotoTile {
            vertices,
            uvs,
            indices,
            width: tile.width,
            height: tile.height,
            rgba: tile.rgba,
        });
    }
    eprintln!("scenery: loaded {} photo tiles", result.len());
    Ok(result)
}

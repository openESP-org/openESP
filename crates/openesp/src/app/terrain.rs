use anyhow::Result;
use glam::DVec3;
use std::{
    path::PathBuf,
    sync::{Arc, mpsc},
    time::{Duration, Instant},
};

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

type LoadedTerrain = Result<(
    Arc<esp_terrain::mesh::Mesh>,
    esp_render::TerrainMesh,
    [f64; 2],
    u8,
)>;

pub(super) struct Stream {
    root: PathBuf,
    latitude: f64,
    longitude: f64,
    reference: f64,
    centre: [f64; 2],
    month: u8,
    pending: Option<mpsc::Receiver<LoadedTerrain>>,
    retry_at: Instant,
}

impl Stream {
    pub fn update(
        &mut self,
        east: f64,
        north: f64,
        month: u8,
    ) -> Option<(Arc<esp_terrain::mesh::Mesh>, esp_render::TerrainMesh)> {
        let mut ready = None;
        if let Some(receiver) = &self.pending {
            match receiver.try_recv() {
                Ok(Ok((mesh, render, centre, loaded_month))) => {
                    self.pending = None;
                    if loaded_month == month {
                        self.centre = centre;
                        self.month = month;
                        ready = Some((mesh, render));
                    }
                }
                Ok(Err(error)) => {
                    eprintln!("terrain streaming: {error:#}");
                    self.pending = None;
                    self.retry_at = Instant::now() + Duration::from_secs(10);
                }
                Err(mpsc::TryRecvError::Disconnected) => {
                    eprintln!("terrain streaming: worker disconnected");
                    self.pending = None;
                    self.retry_at = Instant::now() + Duration::from_secs(10);
                }
                Err(mpsc::TryRecvError::Empty) => {}
            }
        }
        if self.pending.is_none()
            && Instant::now() >= self.retry_at
            && (month != self.month
                || (east - self.centre[0])
                    .abs()
                    .max((north - self.centre[1]).abs())
                    > 12_000.0)
            && east.is_finite()
            && north.is_finite()
        {
            // Keep the sampling grid aligned across replacements so overlapping
            // triangles do not change height as the aircraft crosses a boundary.
            let step = 60_000.0 / 256.0;
            let centre = [east, north].map(|coordinate| (coordinate / step).round() * step);
            let root = self.root.clone();
            let (latitude, longitude, reference) = (self.latitude, self.longitude, self.reference);
            let (sender, receiver) = mpsc::channel();
            self.pending = Some(receiver);
            std::thread::spawn(move || {
                let result = (|| -> Result<_> {
                    let origin = Origin::new(latitude, longitude)?;
                    let (lat, lon) = origin.position(centre[1], centre[0])?;
                    let terrain = esp_terrain::elevation::Terrain::load(&root, lat, lon, 45_000.0)?;
                    let mut mesh = esp_terrain::mesh::Mesh::build_at(
                        &terrain, latitude, longitude, centre, 30_000.0, 256, reference,
                    )?;
                    anyhow::ensure!(
                        !mesh.indices.is_empty(),
                        "no elevation data near {lat:.4}, {lon:.4}"
                    );
                    classify(&mut mesh, &root, latitude, longitude, lat, lon, month);
                    let render = render_mesh(&mesh, &root);
                    Ok((Arc::new(mesh), render, centre, month))
                })();
                let _ = sender.send(result);
            });
        }
        ready
    }

    pub fn render_mesh(&self, mesh: &esp_terrain::mesh::Mesh) -> esp_render::TerrainMesh {
        render_mesh(mesh, &self.root)
    }
}

pub(super) fn load(
    arguments: &Arguments,
    airport_elevation: f64,
    month: u8,
) -> Result<Option<(Arc<esp_terrain::mesh::Mesh>, Stream)>> {
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
    let mut mesh = esp_terrain::mesh::Mesh::build(
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
    classify(
        &mut mesh,
        &root,
        arguments.latitude,
        arguments.longitude,
        arguments.latitude,
        arguments.longitude,
        month,
    );
    eprintln!("terrain: loaded elevation from {}", root.display());
    Ok(Some((
        Arc::new(mesh),
        Stream {
            root,
            latitude: arguments.latitude,
            longitude: arguments.longitude,
            reference,
            centre: [0.0, 0.0],
            month,
            pending: None,
            retry_at: Instant::now(),
        },
    )))
}

fn classify(
    mesh: &mut esp_terrain::mesh::Mesh,
    root: &std::path::Path,
    origin_lat: f64,
    origin_lon: f64,
    lat: f64,
    lon: f64,
    month: u8,
) {
    let result = (|| -> Result<()> {
        let land = esp_terrain::landclass::LandClass::load(root, lat, lon, 45_000.0)?;
        let seasons = esp_terrain::landclass::LandClass::seasons(root, lat, lon, 45_000.0, month);
        if let Err(error) = &seasons {
            eprintln!("terrain seasons: {error:#}");
        }
        let textures = (|| -> Result<_> {
            let regions = esp_terrain::landclass::LandClass::regions(root, lat, lon, 45_000.0)?;
            let data = std::fs::read(root.join("Scenery/BASE/Scenery/lclookup.bgl"))?;
            let bgl = esp_formats::bgl::Bgl::parse(&data)?;
            let lookup = esp_formats::bgl::land_lookup::Lookup::from_bgl(&bgl)?;
            Ok((regions, lookup))
        })();
        if let Err(error) = &textures {
            eprintln!("terrain texture lookup: {error:#}");
        }
        mesh.classify(
            &land,
            textures
                .as_ref()
                .ok()
                .map(|(regions, lookup)| (regions, lookup)),
            seasons.as_ref().ok(),
            origin_lat,
            origin_lon,
        )
    })();
    if let Err(error) = result {
        eprintln!("terrain land class: {error:#}");
    }
}

fn render_mesh(mesh: &esp_terrain::mesh::Mesh, root: &std::path::Path) -> esp_render::TerrainMesh {
    let key = |texture: esp_formats::bgl::land_lookup::TextureSet, season: Option<u8>| {
        u32::from(texture.number)
            | (u32::from(texture.region) << 16)
            | (u32::from(season.filter(|value| *value <= 4).unwrap_or(3)) << 24)
    };
    let mut textures = std::collections::HashMap::new();
    let directory = root.join("Scenery/World/texture");
    let mut seen = std::collections::HashSet::new();
    for (texture, season) in mesh.texture_sets.iter().zip(&mesh.seasons) {
        let Some(texture) = texture else { continue };
        let id = key(*texture, *season);
        if !seen.insert(id) || texture.region >= 26 {
            continue;
        }
        let season_name = match (id >> 24) as u8 {
            0 => "wi",
            1 => "hw",
            2 => "sp",
            4 => "fa",
            _ => "su",
        };
        let filename = format!(
            "{:03}{}2{season_name}1.bmp",
            texture.number,
            char::from(b'a' + texture.region)
        );
        let summer = format!(
            "{:03}{}2su1.bmp",
            texture.number,
            char::from(b'a' + texture.region)
        );
        if let Ok(path) = esp_assets::resolve_texture(&directory, &filename)
            .or_else(|_| esp_assets::resolve_texture(&directory, &summer))
            && let Ok(bytes) = std::fs::read(path)
            && let Ok(texture) = esp_assets::texture::Texture::from_bytes(&bytes, true)
        {
            textures.insert(id, texture);
        }
    }
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
        land_classes: mesh.land_classes.clone(),
        texture_sets: mesh
            .texture_sets
            .iter()
            .zip(&mesh.seasons)
            .map(|(texture, season)| texture.map(|texture| key(texture, *season)))
            .collect(),
        textures,
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore = "requires installed Prepar3D root in OPENESP_P3D_ROOT"]
    fn streams_real_elevation_and_preserves_overlapping_heights() {
        let root = PathBuf::from(std::env::var_os("OPENESP_P3D_ROOT").unwrap());
        let (latitude, longitude) = (41.17574, 40.84646);
        let terrain =
            esp_terrain::elevation::Terrain::load(&root, latitude, longitude, 45_000.0).unwrap();
        let reference = terrain.height(latitude, longitude).unwrap();
        let mut initial =
            esp_terrain::mesh::Mesh::build(&terrain, latitude, longitude, 30_000.0, 256, reference)
                .unwrap();
        classify(
            &mut initial,
            &root,
            latitude,
            longitude,
            latitude,
            longitude,
            7,
        );
        assert!(initial.land_classes.iter().any(Option::is_some));
        let initial = Arc::new(initial);
        let mut ground = esp_sim::flight::ground::Ground::new(Arc::clone(&initial));
        ground.add_segment([990.0, 2000.0], [1010.0, 2000.0], 10.0);
        let mut stream = Stream {
            root,
            latitude,
            longitude,
            reference,
            centre: [0.0, 0.0],
            month: 7,
            pending: None,
            retry_at: Instant::now(),
        };
        assert!(stream.update(20_000.0, 0.0, 7).is_none());
        let deadline = Instant::now() + Duration::from_secs(30);
        let (mesh, summer_render) = loop {
            if let Some(loaded) = stream.update(20_000.0, 0.0, 7) {
                break loaded;
            }
            assert!(Instant::now() < deadline, "terrain worker did not finish");
            std::thread::sleep(Duration::from_millis(10));
        };
        assert!(mesh.height(40_000.0, 0.0).is_some());
        assert!(mesh.land_classes.iter().any(Option::is_some));
        assert!(mesh.texture_sets.iter().any(Option::is_some));
        assert!(mesh.seasons.contains(&Some(3)));
        assert!(!summer_render.textures.is_empty());
        assert!(initial.height(40_000.0, 0.0).is_none());
        for point in [[1000.0, 2000.0], [15_000.0, -5000.0], [20_000.0, 1000.0]] {
            let before = initial.height(point[0], point[1]).unwrap();
            let after = mesh.height(point[0], point[1]).unwrap();
            assert!(
                (before - after).abs() < 1e-6,
                "overlapping mesh changed from {before} to {after}"
            );
        }
        ground.set_terrain(Arc::clone(&mesh));
        assert_eq!(ground.surface(2000.0, 1000.0).elevation, 0.0);
        assert_eq!(
            ground.surface(0.0, 40_000.0).elevation,
            mesh.height(40_000.0, 0.0).unwrap()
        );
        assert!(stream.pending.is_none());
        assert!(stream.update(20_000.0, 0.0, 7).is_none());
        assert!(stream.pending.is_none());
        assert!(stream.update(20_000.0, 0.0, 1).is_none());
        let deadline = Instant::now() + Duration::from_secs(30);
        let (winter, winter_render) = loop {
            if let Some(loaded) = stream.update(20_000.0, 0.0, 1) {
                break loaded;
            }
            assert!(Instant::now() < deadline, "season update did not finish");
            std::thread::sleep(Duration::from_millis(10));
        };
        assert!(
            winter
                .seasons
                .iter()
                .any(|season| matches!(season, Some(0 | 1)))
        );
        assert!(winter_render.textures.keys().any(|id| id >> 24 <= 1));
        assert!(
            winter_render.textures.iter().any(|(id, texture)| {
                if id >> 24 > 1 {
                    return false;
                }
                let summer_id = (id & 0x00ff_ffff) | (3 << 24);
                summer_render
                    .textures
                    .get(&summer_id)
                    .is_some_and(|summer| summer.mips[0].data != texture.mips[0].data)
            }),
            "expected distinct summer and winter texture data"
        );
    }
}

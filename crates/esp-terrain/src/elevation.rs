use anyhow::{Context, Result, ensure};
use esp_formats::bgl::{
    Bgl, Directory,
    qmid::{Bounds, Qmid},
    raster::{Elevation, Raster},
};
use std::path::{Path, PathBuf};

struct Tile {
    bounds: Bounds,
    level: u8,
    elevation: Elevation,
}

/// Read-only elevation tiles covering an area in the simulator's base terrain.
pub struct Terrain {
    tiles: Vec<Tile>,
}

impl Terrain {
    pub fn load(root: &Path, latitude: f64, longitude: f64, radius_metres: f64) -> Result<Self> {
        ensure!(
            latitude.is_finite()
                && (-90.0..=90.0).contains(&latitude)
                && longitude.is_finite()
                && (-180.0..=180.0).contains(&longitude)
                && radius_metres.is_finite()
                && (0.0..=100_000.0).contains(&radius_metres),
            "invalid terrain coverage"
        );
        let mut tiles = Vec::new();
        for path in region_paths(root, latitude, longitude, radius_metres) {
            if !path.is_file() {
                continue;
            }
            let bytes =
                std::fs::read(&path).with_context(|| format!("reading {}", path.display()))?;
            let bgl = Bgl::parse(&bytes).with_context(|| format!("parsing {}", path.display()))?;
            for section in bgl.sections.iter().filter(|section| section.kind == 0x67) {
                let Directory::Direct(subsections) = &section.directory else {
                    continue;
                };
                for subsection in subsections {
                    let Some(qmid) = Qmid::from_packed(subsection.qmid) else {
                        continue;
                    };
                    let bounds = qmid.bounds();
                    if qmid.level() > 11 || !intersects(bounds, latitude, longitude, radius_metres)
                    {
                        continue;
                    }
                    let raster = Raster::parse(subsection.records.data)?;
                    let elevation = raster
                        .decode_elevation()
                        .with_context(|| format!("decoding terrain in {}", path.display()))?;
                    tiles.push(Tile {
                        bounds,
                        level: qmid.level(),
                        elevation,
                    });
                }
            }
        }
        tiles.sort_by_key(|tile| std::cmp::Reverse(tile.level));
        Ok(Self { tiles })
    }

    pub fn is_empty(&self) -> bool {
        self.tiles.is_empty()
    }

    /// Bilinear elevation in metres MSL, preferring the most detailed tile.
    pub fn height(&self, latitude: f64, longitude: f64) -> Option<f64> {
        self.tiles.iter().find_map(|tile| {
            let b = tile.bounds;
            if !(b.south..=b.north).contains(&latitude) || !(b.west..=b.east).contains(&longitude) {
                return None;
            }
            let grid = &tile.elevation;
            let x = ((longitude - b.west) / (b.east - b.west) * f64::from(grid.columns - 1))
                .clamp(0.0, f64::from(grid.columns - 1));
            let y = ((b.north - latitude) / (b.north - b.south) * f64::from(grid.rows - 1))
                .clamp(0.0, f64::from(grid.rows - 1));
            let x0 = x.floor() as usize;
            let y0 = y.floor() as usize;
            let x1 = (x0 + 1).min(usize::from(grid.columns - 1));
            let y1 = (y0 + 1).min(usize::from(grid.rows - 1));
            let get = |x, y| f64::from(grid.metres[y * usize::from(grid.columns) + x]);
            let fx = x - x0 as f64;
            let fy = y - y0 as f64;
            let a = get(x0, y0) * (1.0 - fx) + get(x1, y0) * fx;
            let b = get(x0, y1) * (1.0 - fx) + get(x1, y1) * fx;
            Some(a * (1.0 - fy) + b * fy)
        })
    }
}

fn intersects(b: Bounds, lat: f64, lon: f64, radius: f64) -> bool {
    let lat_margin = radius / 110_000.0;
    let lon_margin = lat_margin / lat.to_radians().cos().abs().max(0.01);
    b.north >= lat - lat_margin
        && b.south <= lat + lat_margin
        && [-360.0, 0.0, 360.0].into_iter().any(|offset| {
            b.east + offset >= lon - lon_margin && b.west + offset <= lon + lon_margin
        })
}

fn region_paths(root: &Path, latitude: f64, longitude: f64, radius: f64) -> Vec<PathBuf> {
    let lat_margin = radius / 110_000.0;
    let lon_margin = lat_margin / latitude.to_radians().cos().abs().max(0.01);
    let mut paths = Vec::new();
    for row in 0..8 {
        for column in 0..12 {
            let bounds = Bounds {
                north: 90.0 - row as f64 * 22.5,
                south: 90.0 - (row + 1) as f64 * 22.5,
                west: -180.0 + column as f64 * 30.0,
                east: -180.0 + (column + 1) as f64 * 30.0,
            };
            if bounds.north < latitude - lat_margin || bounds.south > latitude + lat_margin {
                continue;
            }
            if ![-360.0, 0.0, 360.0].into_iter().any(|offset| {
                bounds.east + offset >= longitude - lon_margin
                    && bounds.west + offset <= longitude + lon_margin
            }) {
                continue;
            }
            let id = format!("{column:02}{row:02}");
            paths.push(
                root.join("Scenery")
                    .join(&id)
                    .join("scenery")
                    .join(format!("dem{id}.bgl")),
            );
        }
    }
    paths
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn samples_signed_elevation_and_prefers_detailed_tiles() {
        let bounds = Bounds {
            north: 1.0,
            south: 0.0,
            west: 0.0,
            east: 1.0,
        };
        let make_tile = |level, metres| Tile {
            bounds,
            level,
            elevation: Elevation {
                columns: 2,
                rows: 2,
                metres,
            },
        };
        let terrain = Terrain {
            tiles: vec![
                make_tile(11, vec![-10.0, 10.0, 20.0, 40.0]),
                make_tile(10, vec![999.0; 4]),
            ],
        };
        assert_eq!(terrain.height(1.0, 0.0), Some(-10.0));
        assert_eq!(terrain.height(0.5, 0.5), Some(15.0));
        assert_eq!(terrain.height(2.0, 0.5), None);
    }
    #[test]
    fn locates_regional_dem_and_crosses_dateline() {
        let root = Path::new("sim");
        assert_eq!(
            region_paths(root, 41.17574, 40.84646, 10_000.0),
            [root.join("Scenery/0702/scenery/dem0702.bgl")]
        );
        let paths = region_paths(root, 0.0, 179.99, 10_000.0);
        assert!(paths.iter().any(|p| p.to_string_lossy().contains("1103")));
        assert!(paths.iter().any(|p| p.to_string_lossy().contains("0003")));
    }
    #[test]
    #[ignore = "requires installed Prepar3D root in OPENESP_P3D_ROOT"]
    fn samples_real_elevation() {
        let root = std::env::var_os("OPENESP_P3D_ROOT").unwrap();
        let terrain = Terrain::load(Path::new(&root), 41.17574, 40.84646, 35_000.0).unwrap();
        assert!(!terrain.is_empty());
        let airport = terrain.height(41.17574, 40.84646).unwrap();
        assert!((-20.0..30.0).contains(&airport));
        let inland = terrain.height(41.0, 40.9).unwrap();
        assert!(
            inland > 50.0 && inland < 5000.0,
            "inland elevation: {inland}"
        );
        let mesh =
            crate::mesh::Mesh::build(&terrain, 41.17574, 40.84646, 30_000.0, 128, airport).unwrap();
        assert!(!mesh.indices.is_empty());
        assert!(mesh.height(0.0, 0.0).unwrap().abs() < 1.0);
        assert!(mesh.vertices.iter().any(|vertex| vertex[1] > 100.0));
    }
}

//! Geographic land-class samples from the simulator's base scenery.
use anyhow::{Context, Result, ensure};
use esp_formats::bgl::{
    Bgl, Directory,
    qmid::{Bounds, Qmid},
    raster::Raster,
};
use std::path::Path;

struct Tile {
    bounds: Bounds,
    level: u8,
    columns: usize,
    rows: usize,
    classes: Vec<u8>,
    mask: Option<Vec<u8>>,
}

/// Land classification tiles, ordered from most to least detailed.
pub struct LandClass {
    tiles: Vec<Tile>,
}

impl LandClass {
    pub fn load(root: &Path, latitude: f64, longitude: f64, radius: f64) -> Result<Self> {
        ensure!(
            latitude.is_finite()
                && (-90.0..=90.0).contains(&latitude)
                && longitude.is_finite()
                && (-180.0..=180.0).contains(&longitude)
                && radius.is_finite()
                && (0.0..=100_000.0).contains(&radius),
            "invalid land-class coverage"
        );
        let path = root.join("Scenery/BASE/Scenery/worldlc.bgl");
        let bytes = std::fs::read(&path).with_context(|| format!("reading {}", path.display()))?;
        let bgl = Bgl::parse(&bytes)?;
        let mut tiles = Vec::new();
        for section in bgl.sections.iter().filter(|section| section.kind == 0x68) {
            let Directory::Direct(subsections) = &section.directory else {
                continue;
            };
            for subsection in subsections {
                let Some(qmid) = Qmid::from_packed(subsection.qmid) else {
                    continue;
                };
                let bounds = qmid.bounds();
                if !crate::elevation::intersects(bounds, latitude, longitude, radius) {
                    continue;
                }
                let raster = Raster::parse(subsection.records.data)?;
                ensure!(
                    raster.data_type == 3,
                    "unexpected land-class raster type {}",
                    raster.data_type
                );
                tiles.push(Tile {
                    bounds,
                    level: qmid.level(),
                    columns: usize::from(raster.columns),
                    rows: usize::from(raster.rows),
                    classes: raster.decode_data()?,
                    mask: raster.decode_mask()?,
                });
            }
        }
        tiles.sort_by_key(|tile| std::cmp::Reverse(tile.level));
        Ok(Self { tiles })
    }

    pub fn class(&self, latitude: f64, longitude: f64) -> Option<u8> {
        self.tiles.iter().find_map(|tile| {
            let b = tile.bounds;
            if !(b.south..=b.north).contains(&latitude) || !(b.west..=b.east).contains(&longitude) {
                return None;
            }
            let x = (((longitude - b.west) / (b.east - b.west)) * (tile.columns - 1) as f64).floor()
                as usize;
            let y = (((b.north - latitude) / (b.north - b.south)) * (tile.rows - 1) as f64).floor()
                as usize;
            let index = y * tile.columns + x;
            if tile.mask.as_ref().is_some_and(|mask| mask[index] == 0) {
                return None;
            }
            tile.classes.get(index).copied()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn selects_cells_and_respects_transparent_overlays() {
        let bounds = Bounds {
            north: 1.0,
            south: 0.0,
            west: 0.0,
            east: 1.0,
        };
        let make = |level, classes, mask| Tile {
            bounds,
            level,
            columns: 3,
            rows: 3,
            classes,
            mask,
        };
        let data = LandClass {
            tiles: vec![
                make(
                    8,
                    vec![3; 9],
                    Some(vec![0, 255, 255, 255, 255, 255, 255, 255, 255]),
                ),
                make(7, vec![5; 9], None),
            ],
        };
        assert_eq!(data.class(0.9, 0.1), Some(5));
        assert_eq!(data.class(0.9, 0.6), Some(3));
        assert_eq!(data.class(2.0, 0.5), None);
    }

    #[test]
    #[ignore = "requires installed Prepar3D root in OPENESP_P3D_ROOT"]
    fn reads_real_land_classes() {
        let root = std::env::var_os("OPENESP_P3D_ROOT").unwrap();
        let data = LandClass::load(Path::new(&root), 41.17574, 40.84646, 45_000.0).unwrap();
        let mut classes = std::collections::HashSet::new();
        for i in 0..20 {
            for j in 0..20 {
                classes.insert(
                    data.class(41.0 + f64::from(i) * 0.01, 40.7 + f64::from(j) * 0.01)
                        .unwrap(),
                );
            }
        }
        assert!(
            classes.len() > 1,
            "expected varied terrain, got {classes:?}"
        );
    }
}

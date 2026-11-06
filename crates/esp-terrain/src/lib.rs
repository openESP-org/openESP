//! esp-terrain owns terrain tile selection and decoding.

use anyhow::{Context, Result};
use esp_formats::bgl::{Bgl, qmid::Bounds};
use std::path::Path;

/// A decoded photo tile and its geographic extent, in north-to-south row order.
pub struct PhotoTile {
    pub bounds: Bounds,
    pub level: u8,
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
}

/// Decode daytime photo imagery near the scene origin, coarse layers first.
pub fn photos(directory: &Path, latitude: f64, longitude: f64) -> Result<Vec<PhotoTile>> {
    anyhow::ensure!(
        latitude.is_finite()
            && (-90.0..=90.0).contains(&latitude)
            && longitude.is_finite()
            && (-180.0..=180.0).contains(&longitude),
        "invalid terrain origin"
    );
    let mut paths = std::fs::read_dir(directory)
        .with_context(|| format!("reading terrain directory {}", directory.display()))?
        .map(|entry| entry.map(|entry| entry.path()))
        .collect::<std::io::Result<Vec<_>>>()?
        .into_iter()
        .filter(|path| {
            path.extension()
                .is_some_and(|extension| extension.eq_ignore_ascii_case("bgl"))
        })
        .collect::<Vec<_>>();
    paths.sort();
    let mut result = Vec::new();
    for path in paths {
        let bytes = std::fs::read(&path).with_context(|| format!("reading {}", path.display()))?;
        let bgl = Bgl::parse(&bytes).with_context(|| format!("parsing {}", path.display()))?;
        if !bgl
            .sections
            .iter()
            .any(|section| (0x8c..=0x98).contains(&section.kind))
        {
            continue;
        }
        for (kind, tile) in bgl.raster_tiles()? {
            if !(0x8c..=0x98).contains(&kind)
                || tile.data_type != 1
                || tile.variations & 0x1000 != 0
            {
                continue;
            }
            let Some(qmid) = esp_formats::bgl::qmid::Qmid::from_packed(tile.qmid) else {
                continue;
            };
            let bounds = qmid.bounds();
            if !(12..=16).contains(&qmid.level()) || !near(bounds, latitude, longitude) {
                continue;
            }
            let data = tile
                .decode_data()
                .with_context(|| format!("decoding photo in {}", path.display()))?;
            let mask = tile
                .decode_mask()
                .with_context(|| format!("decoding photo mask in {}", path.display()))?;
            let rgba = rgba(&data, mask.as_deref());
            result.push(PhotoTile {
                bounds,
                level: qmid.level(),
                width: u32::from(tile.columns),
                height: u32::from(tile.rows),
                rgba,
            });
        }
    }
    result.sort_by_key(|tile| tile.level);
    Ok(result)
}

fn near(bounds: Bounds, latitude: f64, longitude: f64) -> bool {
    // Conservative rectangle in degrees; selecting more at high latitudes is preferable to gaps.
    let lat_margin = 5_000.0 / 111_000.0;
    let lon_margin = lat_margin / latitude.to_radians().cos().abs().max(0.01);
    bounds.north >= latitude - lat_margin
        && bounds.south <= latitude + lat_margin
        && [-360.0, 0.0, 360.0].into_iter().any(|offset| {
            bounds.east + offset >= longitude - lon_margin
                && bounds.west + offset <= longitude + lon_margin
        })
}

fn rgba(data: &[u8], mask: Option<&[u8]>) -> Vec<u8> {
    let mut result = Vec::with_capacity(data.len() * 2);
    for (index, pixel) in data.as_chunks::<2>().0.iter().enumerate() {
        let value = u16::from_le_bytes([pixel[0], pixel[1]]);
        let expand = |value: u16| ((value * 255 + 15) / 31) as u8;
        result.extend_from_slice(&[
            expand((value >> 10) & 31),
            expand((value >> 5) & 31),
            expand(value & 31),
            mask.map_or(255, |mask| mask[index]),
        ]);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converts_1555_colors_and_blend_mask() {
        assert_eq!(
            rgba(&[0x00, 0x7c, 0xe0, 0x03, 0x1f, 0x00], Some(&[0, 128, 255])),
            [255, 0, 0, 0, 0, 255, 0, 128, 0, 0, 255, 255]
        );
    }

    #[test]
    fn selects_nearby_bounds_across_the_date_line() {
        let bounds = Bounds {
            north: 1.0,
            south: 0.0,
            west: -180.0,
            east: -179.99,
        };
        assert!(near(bounds, 0.5, 179.99));
        assert!(!near(bounds, 0.5, 170.0));
    }

    #[test]
    #[ignore = "requires LTFO Scenery folder in OPENESP_SCENERY_DIRECTORY"]
    fn loads_real_airport_imagery() {
        let path = std::env::var_os("OPENESP_SCENERY_DIRECTORY").unwrap();
        let tiles = photos(Path::new(&path), 41.1797, 40.8489).unwrap();
        assert!(!tiles.is_empty());
        assert!(tiles.iter().any(|tile| tile.level == 16));
        assert!(
            tiles
                .iter()
                .all(|tile| tile.rgba.len() == (tile.width * tile.height * 4) as usize)
        );
        assert!(tiles.iter().any(|tile| {
            tile.rgba
                .as_chunks::<4>()
                .0
                .iter()
                .any(|pixel| pixel[3] > 0)
        }));
        assert!(
            tiles
                .windows(2)
                .all(|tiles| tiles[0].level <= tiles[1].level)
        );
        eprintln!("loaded {} geographic photo tiles", tiles.len());
    }
}

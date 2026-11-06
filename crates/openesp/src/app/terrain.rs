use anyhow::Result;
use glam::DVec3;

use crate::{cli::Arguments, navigation::Origin};

pub(super) fn photos(arguments: &Arguments) -> Result<Vec<esp_render::PhotoTile>> {
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
            Ok(DVec3::new(east, 0.0, north))
        };
        result.push(esp_render::PhotoTile {
            corners: [
                point(bounds.north, bounds.west)?,
                point(bounds.south, bounds.west)?,
                point(bounds.south, bounds.east)?,
                point(bounds.north, bounds.east)?,
            ],
            width: tile.width,
            height: tile.height,
            rgba: tile.rgba,
        });
    }
    eprintln!("scenery: loaded {} photo tiles", result.len());
    Ok(result)
}

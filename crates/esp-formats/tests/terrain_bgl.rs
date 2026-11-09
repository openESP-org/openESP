use esp_formats::bgl::Bgl;

#[test]
#[ignore = "requires the installed regional DEM in OPENESP_DEM_BGL"]
fn decodes_real_p3d_elevation_near_ltfo() {
    let path = std::env::var_os("OPENESP_DEM_BGL").unwrap();
    let bytes = std::fs::read(path).unwrap();
    let bgl = Bgl::parse(&bytes).unwrap();
    let mut nearby = bgl
        .raster_tiles()
        .unwrap()
        .into_iter()
        .filter_map(|(kind, tile)| {
            let qmid = esp_formats::bgl::qmid::Qmid::from_packed(tile.qmid)?;
            let bounds = qmid.bounds();
            (kind == 0x67
                && bounds.south <= 41.17574
                && bounds.north >= 41.17574
                && bounds.west <= 40.84646
                && bounds.east >= 40.84646)
                .then_some((qmid.level(), tile))
        })
        .collect::<Vec<_>>();
    nearby.sort_by_key(|(level, _)| std::cmp::Reverse(*level));
    assert!(!nearby.is_empty(), "no elevation tiles at LTFO");
    for (_, tile) in nearby.iter().take(3) {
        let elevation = tile.decode_elevation().unwrap();
        let bounds = esp_formats::bgl::qmid::Qmid::from_packed(tile.qmid)
            .unwrap()
            .bounds();
        let x = (((40.84646 - bounds.west) / (bounds.east - bounds.west))
            * f64::from(tile.columns - 1))
        .round() as usize;
        let y = (((bounds.north - 41.17574) / (bounds.north - bounds.south))
            * f64::from(tile.rows - 1))
        .round() as usize;
        let airport_height = elevation.metres[y * usize::from(tile.columns) + x];
        assert!(
            (-20.0..=30.0).contains(&airport_height),
            "LTFO elevation {airport_height} m"
        );
        assert_eq!(
            elevation.metres.len(),
            usize::from(tile.rows) * usize::from(tile.columns)
        );
        let low = elevation
            .metres
            .iter()
            .copied()
            .fold(f32::INFINITY, f32::min);
        let high = elevation
            .metres
            .iter()
            .copied()
            .fold(f32::NEG_INFINITY, f32::max);
        assert!((-1000.0..=9000.0).contains(&low) && (-1000.0..=9000.0).contains(&high));
        assert!(high - low > 100.0, "expected terrain relief around LTFO");
    }
}

#[test]
#[ignore = "requires LTFO photoreal BGL in OPENESP_LTFO_PHOTOREAL_BGL"]
fn locates_ltfo_photo_tiles_on_the_world_grid() {
    let path = std::env::var_os("OPENESP_LTFO_PHOTOREAL_BGL").unwrap();
    let bytes = std::fs::read(path).unwrap();
    let bgl = Bgl::parse(&bytes).unwrap();
    let found = bgl.raster_tiles().unwrap().into_iter().any(|(kind, tile)| {
        let Some(cell) = esp_formats::bgl::qmid::Qmid::from_packed(tile.qmid) else {
            return false;
        };
        let bounds = cell.bounds();
        (0x8c..=0x98).contains(&kind)
            && cell.level() >= 13
            && bounds.south <= 41.17556
            && bounds.north >= 41.17556
            && bounds.west <= 40.845
            && bounds.east >= 40.845
    });
    assert!(found, "no detailed photo tile covers the LTFO terminal");
}

#[test]
#[ignore = "requires photoreal BGL in OPENESP_PHOTOREAL_BGL"]
fn reads_real_photoreal_tiles_and_masks() {
    let path = std::env::var_os("OPENESP_PHOTOREAL_BGL").unwrap();
    let bytes = std::fs::read(path).unwrap();
    let bgl = Bgl::parse(&bytes).unwrap();
    let tiles = bgl.raster_tiles().unwrap();
    assert!(!tiles.is_empty());
    let mut imagery = 0;
    let mut decoded = 0;
    let mut ptc = 0;
    let mut best_coverage = 0;
    let mut masks = 0;
    for (kind, tile) in &tiles {
        if (0x8c..=0x98).contains(kind) {
            assert!(
                esp_formats::bgl::qmid::Qmid::from_packed(tile.qmid).is_some(),
                "invalid photo QMID {:?}",
                tile.qmid
            );
        }
        let mask = tile
            .decode_mask()
            .unwrap_or_else(|error| panic!("mask {:?}: {error}", tile.qmid));
        if let Some(mask) = &mask {
            masks += 1;
            assert_eq!(
                mask.len(),
                usize::from(tile.columns) * usize::from(tile.rows)
            );
        }
        assert!(tile.columns > 0 && tile.rows > 0);
        assert!(!tile.data.is_empty());
        assert!(tile.trailing_data.is_empty());
        if (0x8c..=0x98).contains(kind) {
            assert_eq!(tile.data_type, 1);
            if tile.data_compression == esp_formats::bgl::raster::Compression::Ptc {
                ptc += 1;
                for end in [0, 4, 47, tile.data.len() / 2, tile.data.len() - 1] {
                    assert!(
                        esp_formats::bgl::raster::codec::decode(
                            tile.data_compression,
                            &tile.data[..end],
                            256 * 256 * 2
                        )
                        .is_err()
                    );
                }
            }
            {
                let pixels = tile.decode_data().unwrap_or_else(|error| {
                    panic!("tile {:?} {:?}: {error}", tile.qmid, tile.data_compression)
                });
                assert_eq!(
                    pixels.len(),
                    usize::from(tile.columns) * usize::from(tile.rows) * 2
                );
                let coverage = pixels
                    .as_chunks::<2>()
                    .0
                    .iter()
                    .filter(|pixel| pixel[0] != 0 || pixel[1] != 0)
                    .count();
                if coverage > best_coverage {
                    best_coverage = coverage;
                    let image = image::RgbaImage::from_fn(
                        u32::from(tile.columns),
                        u32::from(tile.rows),
                        |x, y| {
                            let offset = (y as usize * usize::from(tile.columns) + x as usize) * 2;
                            let pixel = u16::from_le_bytes([pixels[offset], pixels[offset + 1]]);
                            let expand = |n: u16| ((n * 255 + 15) / 31) as u8;
                            image::Rgba([
                                expand((pixel >> 10) & 31),
                                expand((pixel >> 5) & 31),
                                expand(pixel & 31),
                                mask.as_ref().map_or(255, |mask| mask[offset / 2]),
                            ])
                        },
                    );
                    image
                        .save(
                            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                                .join("../../target/terrain-ltfo-first.bmp"),
                        )
                        .unwrap();
                }
                decoded += 1;
            }
            imagery += 1;
        }
    }
    assert!(imagery > 0);
    assert!(decoded > 0);
    assert!(masks > 0);
    eprintln!("decoded {masks} tile masks");
    assert_eq!(decoded, imagery);
    eprintln!("decoded {decoded} imagery tiles, including {ptc} PTC tiles");
    eprintln!(
        "{} raster records, {imagery} imagery tiles; first photo {:?}",
        tiles.len(),
        tiles
            .iter()
            .find(|(kind, _)| *kind == 0x8c)
            .map(|(_, tile)| (
                tile.columns,
                tile.rows,
                tile.data_compression,
                tile.mask_compression
            ))
    );
}

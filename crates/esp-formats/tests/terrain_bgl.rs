use esp_formats::bgl::Bgl;

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
            } else {
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
    eprintln!("decoded {decoded} imagery tiles; {ptc} PTC tiles await codec support");
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

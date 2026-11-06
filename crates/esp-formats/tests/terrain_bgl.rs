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
    for (kind, tile) in &tiles {
        assert!(tile.columns > 0 && tile.rows > 0);
        assert!(!tile.data.is_empty());
        assert!(tile.trailing_data.is_empty());
        if (0x8c..=0x98).contains(kind) {
            assert_eq!(tile.data_type, 1);
            imagery += 1;
        }
    }
    assert!(imagery > 0);
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

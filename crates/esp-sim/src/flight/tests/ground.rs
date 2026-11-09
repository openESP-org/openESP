use super::*;
use std::{path::Path, sync::Arc};

#[test]
#[ignore = "requires Prepar3D elevation and original aircraft data"]
fn aircraft_contact_tracks_real_elevated_terrain() {
    let root = std::env::var_os("OPENESP_P3D_ROOT").unwrap();
    let terrain =
        esp_terrain::elevation::Terrain::load(Path::new(&root), 41.17574, 40.84646, 35_000.0)
            .unwrap();
    let reference = terrain.height(41.17574, 40.84646).unwrap();
    let mesh = Arc::new(
        esp_terrain::mesh::Mesh::build(&terrain, 41.17574, 40.84646, 30_000.0, 128, reference)
            .unwrap(),
    );
    let vertex = mesh
        .vertices
        .iter()
        .find(|v| {
            v[1] > 100.0
                && v[0].abs() < 25_000.0
                && v[2].abs() < 25_000.0
                && mesh.height(v[0] + 5.0, v[2] + 5.0).is_some()
        })
        .unwrap();
    let (east, north) = (vertex[0] + 5.0, vertex[2] + 5.0);
    let height = mesh.height(east, north).unwrap();
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let mut flight = Flight::new(&data, 0.0, 0.0, 0.5, true).unwrap();
        let mut variables = controls(&data);
        for _ in 0..600 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        flight.current.position.x = north;
        flight.current.position.y = east;
        flight.current.position.z -= height;
        flight.set_ground(crate::flight::ground::Ground::new(Arc::clone(&mesh)));
        for _ in 0..600 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        assert!(
            flight.on_ground,
            "{path:?}: aircraft did not settle on elevated terrain"
        );
        let surface = flight.surface_at(&flight.current).elevation;
        assert!(surface > 50.0 && -flight.current.position.z > surface);
    }
}

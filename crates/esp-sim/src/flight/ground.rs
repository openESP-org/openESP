use std::sync::Arc;

use esp_physics::ground::Surface;
use esp_terrain::mesh::Mesh;

use super::TEST_SURFACE;

/// Runtime ground heights and flat airport surfaces in local east/north metres.
pub struct Ground {
    terrain: Arc<Mesh>,
    segments: Vec<Segment>,
    triangles: Vec<[[f64; 2]; 3]>,
}

struct Segment {
    start: [f64; 2],
    end: [f64; 2],
    half_width: f64,
}

impl Ground {
    pub fn new(terrain: Arc<Mesh>) -> Self {
        Self {
            terrain,
            segments: Vec::new(),
            triangles: Vec::new(),
        }
    }

    /// Swap terrain while retaining the airport's surface footprints.
    pub fn set_terrain(&mut self, terrain: Arc<Mesh>) {
        self.terrain = terrain;
    }

    pub fn reference_msl(&self) -> f64 {
        self.terrain.reference_msl()
    }

    pub fn add_segment(&mut self, start: [f64; 2], end: [f64; 2], width: f64) {
        if start.iter().chain(&end).all(|value| value.is_finite())
            && width.is_finite()
            && width > 0.0
        {
            self.segments.push(Segment {
                start,
                end,
                half_width: width * 0.5,
            });
        }
    }

    pub fn add_triangle(&mut self, vertices: [[f64; 2]; 3]) {
        if vertices.iter().flatten().all(|value| value.is_finite()) {
            self.triangles.push(vertices);
        }
    }

    pub fn surface(&self, north: f64, east: f64) -> Surface {
        let point = [east, north];
        let airport = self.segments.iter().any(|segment| segment.contains(point))
            || self
                .triangles
                .iter()
                .any(|triangle| contains(*triangle, point));
        Surface {
            elevation: if airport {
                0.0
            } else {
                self.terrain.height(east, north).unwrap_or(0.0)
            },
            ..TEST_SURFACE
        }
    }
}

impl Segment {
    fn contains(&self, p: [f64; 2]) -> bool {
        let dx = self.end[0] - self.start[0];
        let dy = self.end[1] - self.start[1];
        let length = dx * dx + dy * dy;
        if length <= 1e-12 {
            return false;
        }
        let t = ((p[0] - self.start[0]) * dx + (p[1] - self.start[1]) * dy) / length;
        if !(0.0..=1.0).contains(&t) {
            return false;
        }
        let x = p[0] - (self.start[0] + t * dx);
        let y = p[1] - (self.start[1] + t * dy);
        x * x + y * y <= self.half_width * self.half_width
    }
}

fn contains([a, b, c]: [[f64; 2]; 3], p: [f64; 2]) -> bool {
    let cross = |x: [f64; 2], y: [f64; 2], z: [f64; 2]| {
        (y[0] - x[0]) * (z[1] - x[1]) - (y[1] - x[1]) * (z[0] - x[0])
    };
    let area = cross(a, b, c);
    if area.abs() < 1e-9 {
        return false;
    }
    let sides = [cross(a, b, p), cross(b, c, p), cross(c, a, p)];
    sides.iter().all(|side| *side >= -1e-9) || sides.iter().all(|side| *side <= 1e-9)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn airport_surfaces_have_bounded_footprints() {
        let segment = Segment {
            start: [0.0, 0.0],
            end: [0.0, 100.0],
            half_width: 10.0,
        };
        assert!(segment.contains([9.0, 50.0]));
        assert!(!segment.contains([11.0, 50.0]));
        assert!(!segment.contains([0.0, 101.0]));
        let tri = [[0.0, 0.0], [10.0, 0.0], [0.0, 10.0]];
        assert!(contains(tri, [2.0, 2.0]));
        assert!(!contains(tri, [9.0, 9.0]));
        assert!(!contains([[0.0, 0.0]; 3], [0.0, 0.0]));
    }

    #[test]
    #[ignore = "requires installed Prepar3D root in OPENESP_P3D_ROOT"]
    fn real_terrain_height_and_airport_override_share_coordinates() {
        let root = std::env::var_os("OPENESP_P3D_ROOT").unwrap();
        let terrain = esp_terrain::elevation::Terrain::load(
            std::path::Path::new(&root),
            41.17574,
            40.84646,
            35_000.0,
        )
        .unwrap();
        let reference = terrain.height(41.17574, 40.84646).unwrap();
        let mesh =
            Arc::new(Mesh::build(&terrain, 41.17574, 40.84646, 30_000.0, 128, reference).unwrap());
        let vertex = mesh
            .vertices
            .iter()
            .find(|v| v[1] > 100.0 && mesh.height(v[0], v[2]).is_some())
            .unwrap();
        let (east, north) = (vertex[0], vertex[2]);
        let expected = mesh.height(east, north).unwrap();
        let mut ground = Ground::new(Arc::clone(&mesh));
        assert!((ground.surface(north, east).elevation - expected).abs() < 1e-6);
        ground.add_segment([east - 10.0, north], [east + 10.0, north], 10.0);
        assert_eq!(ground.surface(north, east).elevation, 0.0);
        assert!(
            (ground.surface(north + 6.0, east).elevation - mesh.height(east, north + 6.0).unwrap())
                .abs()
                < 1e-6
        );
    }
}

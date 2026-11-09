use anyhow::{Result, ensure};
use esp_nav::navigation::Origin;

use crate::elevation::Terrain;

/// Regular local east/up/north terrain mesh with heights relative to airport MSL.
pub struct Mesh {
    pub vertices: Vec<[f64; 3]>,
    pub normals: Vec<[f64; 3]>,
    pub indices: Vec<u32>,
    valid: Vec<bool>,
    side: usize,
    radius: f64,
    centre: [f64; 2],
}

impl Mesh {
    pub fn cell_size(&self) -> f64 {
        2.0 * self.radius / (self.side - 1) as f64
    }
    pub fn build(
        terrain: &Terrain,
        latitude: f64,
        longitude: f64,
        radius: f64,
        cells: usize,
        reference_msl: f64,
    ) -> Result<Self> {
        Self::build_at(
            terrain,
            latitude,
            longitude,
            [0.0, 0.0],
            radius,
            cells,
            reference_msl,
        )
    }

    /// Build around an east/north offset while retaining the original local frame.
    pub fn build_at(
        terrain: &Terrain,
        latitude: f64,
        longitude: f64,
        centre: [f64; 2],
        radius: f64,
        cells: usize,
        reference_msl: f64,
    ) -> Result<Self> {
        ensure!(
            radius.is_finite()
                && radius > 0.0
                && radius <= 100_000.0
                && (1..=1024).contains(&cells)
                && centre.iter().all(|value| value.is_finite())
                && reference_msl.is_finite(),
            "invalid terrain mesh parameters"
        );
        let origin = Origin::new(latitude, longitude)?;
        let side = cells + 1;
        let step = 2.0 * radius / cells as f64;
        let mut vertices = Vec::with_capacity(side * side);
        let mut valid = Vec::with_capacity(side * side);
        for row in 0..side {
            let north = centre[1] - radius + row as f64 * step;
            for col in 0..side {
                let east = centre[0] - radius + col as f64 * step;
                let (lat, lon) = origin.position(north, east)?;
                let height = terrain.height(lat, lon);
                valid.push(height.is_some());
                vertices.push([east, height.unwrap_or(reference_msl) - reference_msl, north]);
            }
        }
        let mut normals = Vec::with_capacity(vertices.len());
        for row in 0..side {
            for col in 0..side {
                let i = row * side + col;
                let height = vertices[i][1];
                let neighbour = |x: usize, z: usize| {
                    let index = z * side + x;
                    if valid[index] {
                        vertices[index][1]
                    } else {
                        height
                    }
                };
                let dx =
                    neighbour((col + 1).min(cells), row) - neighbour(col.saturating_sub(1), row);
                let dz =
                    neighbour(col, (row + 1).min(cells)) - neighbour(col, row.saturating_sub(1));
                let n = [-dx, 2.0 * step, -dz];
                let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                normals.push(n.map(|value| value / length));
            }
        }
        let mut indices = Vec::with_capacity(cells * cells * 6);
        for row in 0..cells {
            for col in 0..cells {
                let a = row * side + col;
                let b = a + 1;
                let c = a + side;
                let d = c + 1;
                if [a, b, c, d].into_iter().all(|i| valid[i]) {
                    indices.extend([a as u32, c as u32, d as u32, a as u32, d as u32, b as u32]);
                }
            }
        }
        Ok(Self {
            vertices,
            normals,
            indices,
            valid,
            side,
            radius,
            centre,
        })
    }

    /// Height of the generated mesh at an east/north offset, if all samples exist.
    pub fn height(&self, east: f64, north: f64) -> Option<f64> {
        let east = east - self.centre[0];
        let north = north - self.centre[1];
        if !east.is_finite()
            || !north.is_finite()
            || east.abs() > self.radius
            || north.abs() > self.radius
        {
            return None;
        }
        let cells = self.side - 1;
        let x =
            ((east + self.radius) / (2.0 * self.radius) * cells as f64).clamp(0.0, cells as f64);
        let z =
            ((north + self.radius) / (2.0 * self.radius) * cells as f64).clamp(0.0, cells as f64);
        let x0 = (x.floor() as usize).min(cells - 1);
        let z0 = (z.floor() as usize).min(cells - 1);
        let [a, b, c, d] = [
            z0 * self.side + x0,
            z0 * self.side + x0 + 1,
            (z0 + 1) * self.side + x0,
            (z0 + 1) * self.side + x0 + 1,
        ];
        if ![a, b, c, d].into_iter().all(|i| self.valid[i]) {
            return None;
        }
        let fx = x - x0 as f64;
        let fz = z - z0 as f64;
        // Match the two triangles used by the renderer rather than bilinear interpolation.
        let [a, b, c, d] = [a, b, c, d].map(|i| self.vertices[i][1]);
        Some(if fz >= fx {
            a + (d - c) * fx + (c - a) * fz
        } else {
            a + (b - a) * fx + (d - b) * fz
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn samples_the_same_triangles_that_are_rendered() {
        let mut mesh = Mesh {
            vertices: vec![
                [-1.0, 0.0, -1.0],
                [1.0, 10.0, -1.0],
                [-1.0, 20.0, 1.0],
                [1.0, 100.0, 1.0],
            ],
            normals: vec![],
            indices: vec![0, 2, 3, 0, 3, 1],
            valid: vec![true; 4],
            side: 2,
            radius: 1.0,
            centre: [0.0, 0.0],
        };
        assert_eq!(mesh.height(-0.5, 0.5), Some(35.0));
        assert_eq!(mesh.height(0.5, -0.5), Some(30.0));
        assert_eq!(mesh.height(0.0, 0.0), Some(50.0));
        assert_eq!(mesh.height(2.0, 0.0), None);
        mesh.centre = [100.0, -200.0];
        assert_eq!(mesh.height(99.5, -199.5), Some(35.0));
        assert_eq!(mesh.height(0.0, 0.0), None);
        mesh.valid[3] = false;
        assert_eq!(mesh.height(100.0, -200.0), None);
    }
}

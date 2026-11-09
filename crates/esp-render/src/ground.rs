use crate::{TerrainMesh, Vertex};
use anyhow::{Result, ensure};

pub(super) fn mesh(terrain: Option<&TerrainMesh>) -> Result<(Vec<Vertex>, Vec<u32>)> {
    if let Some(terrain) = terrain {
        ensure!(
            terrain.vertices.len() == terrain.normals.len()
                && !terrain.indices.is_empty()
                && terrain.indices.len().is_multiple_of(3)
                && terrain.vertices.iter().all(|vertex| vertex.is_finite())
                && terrain.normals.iter().all(|normal| normal.is_finite())
                && terrain
                    .indices
                    .iter()
                    .all(|index| (*index as usize) < terrain.vertices.len()),
            "invalid terrain mesh"
        );
        let vertices = terrain
            .vertices
            .iter()
            .zip(&terrain.normals)
            .map(|(position, normal)| Vertex {
                position: position.as_vec3().to_array(),
                normal: normal.as_vec3().to_array(),
                uv: [0.0; 2],
                tangent: [0.0; 4],
            })
            .collect();
        return Ok((vertices, terrain.indices.clone()));
    }
    // Keep grid UVs precise near the camera while the surrounding fill reaches the horizon.
    let extent = 40_000.0;
    let pattern = 1_024.0;
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut quad = |points: [(f64, f64); 4]| {
        let base = vertices.len() as u32;
        for (x, z) in points {
            vertices.push(Vertex {
                position: [x as f32, 0.0, z as f32],
                normal: [0.0, 1.0, 0.0],
                uv: [x as f32, z as f32],
                tangent: [0.0; 4],
            });
        }
        indices.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    };
    quad([
        (-pattern, -pattern),
        (-pattern, pattern),
        (pattern, pattern),
        (pattern, -pattern),
    ]);
    quad([
        (-extent, pattern),
        (-extent, extent),
        (extent, extent),
        (extent, pattern),
    ]);
    quad([
        (-extent, -extent),
        (-extent, -pattern),
        (extent, -pattern),
        (extent, -extent),
    ]);
    quad([
        (-extent, -pattern),
        (-extent, pattern),
        (-pattern, pattern),
        (-pattern, -pattern),
    ]);
    quad([
        (pattern, -pattern),
        (pattern, pattern),
        (extent, pattern),
        (extent, -pattern),
    ]);
    Ok((vertices, indices))
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::DVec3;

    #[test]
    fn terrain_mesh_keeps_heights_and_rejects_bad_indices() {
        let mut terrain = TerrainMesh {
            vertices: vec![DVec3::ZERO, DVec3::X, DVec3::new(0.0, 10.0, 1.0)],
            normals: vec![DVec3::Y; 3],
            indices: vec![0, 1, 2],
        };
        let (vertices, indices) = mesh(Some(&terrain)).unwrap();
        assert_eq!(vertices[2].position, [0.0, 10.0, 1.0]);
        assert_eq!(indices, [0, 1, 2]);
        terrain.indices[2] = 3;
        assert!(mesh(Some(&terrain)).is_err());
        assert!(!mesh(None).unwrap().1.is_empty());
    }
}

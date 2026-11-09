use crate::{Renderer, TerrainMesh, Vertex};
use anyhow::{Result, ensure};
use wgpu::util::DeviceExt;

impl Renderer {
    /// Replace the terrain geometry without rebuilding the rest of the scene.
    pub fn update_terrain(&mut self, terrain: &TerrainMesh) -> Result<()> {
        let (vertices, indices) = mesh(Some(terrain))?;
        let count = u32::try_from(indices.len())?;
        self.ground.vertices = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("terrain vertices"),
                contents: bytemuck::cast_slice(&vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });
        self.ground.indices = self
            .device
            .create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("terrain indices"),
                contents: bytemuck::cast_slice(&indices),
                usage: wgpu::BufferUsages::INDEX,
            });
        self.ground.count = count;
        self.ground_tracks_camera = false;
        Ok(())
    }
}

pub(super) fn mesh(terrain: Option<&TerrainMesh>) -> Result<(Vec<Vertex>, Vec<u32>)> {
    if let Some(terrain) = terrain {
        ensure!(
            terrain.vertices.len() == terrain.normals.len()
                && terrain.vertices.len() == terrain.land_classes.len()
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
            .zip(&terrain.land_classes)
            .map(|((position, normal), class)| Vertex {
                position: position.as_vec3().to_array(),
                normal: normal.as_vec3().to_array(),
                uv: [0.0; 2],
                tangent: {
                    let [r, g, b] = land_colour(*class);
                    [r, g, b, 1.0]
                },
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

// Approximate colours until region/season texture lookup and water rendering are available.
// Class meanings are defined by the Prepar3D Terrain SDK's Olson table.
fn land_colour(class: Option<u8>) -> [f32; 3] {
    match class {
        Some(0 | 100 | 123..=127) => [0.07, 0.17, 0.23],
        Some(1 | 101..=121 | 136) => [0.36, 0.35, 0.32],
        Some(
            3..=6 | 13 | 19..=29 | 32..=34 | 48 | 54 | 60..=62 | 72 | 78 | 89 | 90 | 144 | 145,
        ) => [0.16, 0.26, 0.13],
        Some(8 | 11 | 50..=52 | 69 | 99 | 129 | 137 | 146) => [0.48, 0.39, 0.25],
        Some(9 | 42 | 53 | 63 | 130 | 131 | 133 | 138 | 141) => [0.32, 0.31, 0.26],
        Some(122 | 142 | 143) => [0.72, 0.75, 0.74],
        Some(
            2
            | 7
            | 10
            | 12
            | 16
            | 17
            | 30
            | 31
            | 35..=38
            | 40
            | 41
            | 43..=47
            | 55..=59
            | 64
            | 76
            | 91..=98
            | 128
            | 134
            | 135
            | 139
            | 140
            | 147,
        ) => [0.29, 0.35, 0.19],
        _ => [0.32, 0.32, 0.32],
    }
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
            land_classes: vec![Some(0), Some(3), None],
            indices: vec![0, 1, 2],
        };
        let (vertices, indices) = mesh(Some(&terrain)).unwrap();
        assert_eq!(vertices[2].position, [0.0, 10.0, 1.0]);
        assert_eq!(vertices[0].tangent, [0.07, 0.17, 0.23, 1.0]);
        assert_ne!(vertices[0].tangent, vertices[1].tangent);
        assert_eq!(indices, [0, 1, 2]);
        terrain.indices[2] = 3;
        assert!(mesh(Some(&terrain)).is_err());
        assert!(!mesh(None).unwrap().1.is_empty());
    }

    #[test]
    fn terrain_shader_is_valid_wgsl() {
        let shader = naga::front::wgsl::parse_str(include_str!("aircraft.wgsl")).unwrap();
        naga::valid::Validator::new(
            naga::valid::ValidationFlags::all(),
            naga::valid::Capabilities::all(),
        )
        .validate(&shader)
        .unwrap();
    }
}

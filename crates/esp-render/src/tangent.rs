use esp_assets::Mesh;
use glam::{Vec2, Vec3};

/// Preserve authored tangent frames when valid, and derive missing frames from UVs.
pub(super) fn frames(mesh: &Mesh) -> Vec<[f32; 4]> {
    let mut tangent = vec![Vec3::ZERO; mesh.vertices.len()];
    let mut bitangent = tangent.clone();
    for triangle in mesh.indices.as_chunks::<3>().0 {
        let [a, b, c] = triangle.map(|i| i as usize);
        let [v0, v1, v2] = [&mesh.vertices[a], &mesh.vertices[b], &mesh.vertices[c]];
        let e1 = Vec3::from_array(v1.position) - Vec3::from_array(v0.position);
        let e2 = Vec3::from_array(v2.position) - Vec3::from_array(v0.position);
        let uv1 = Vec2::from_array(v1.uv) - Vec2::from_array(v0.uv);
        let uv2 = Vec2::from_array(v2.uv) - Vec2::from_array(v0.uv);
        let det = uv1.x * uv2.y - uv1.y * uv2.x;
        if !det.is_finite() || det.abs() < 1e-10 {
            continue;
        }
        let t = (e1 * uv2.y - e2 * uv1.y) / det;
        let bdir = (e2 * uv1.x - e1 * uv2.x) / det;
        for index in [a, b, c] {
            tangent[index] += t;
            bitangent[index] += bdir;
        }
    }
    mesh.vertices
        .iter()
        .enumerate()
        .map(|(i, vertex)| {
            let n = Vec3::from_array(vertex.normal)
                .try_normalize()
                .unwrap_or(Vec3::Y);
            let authored = mesh.tangents.as_ref().and_then(|values| values.get(i));
            let (t, b) = authored.map_or((tangent[i], bitangent[i]), |basis| {
                (
                    Vec3::from_array(basis.first),
                    Vec3::from_array(basis.second),
                )
            });
            let t = (t - n * n.dot(t))
                .try_normalize()
                .or_else(|| {
                    let fallback = tangent[i] - n * n.dot(tangent[i]);
                    fallback.try_normalize()
                })
                .unwrap_or_else(|| n.any_orthonormal_vector());
            let sign = if n.cross(t).dot(b) < 0.0 { -1.0 } else { 1.0 };
            [t.x, t.y, t.z, sign]
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_formats::mdl::geometry::Vertex;

    fn vertex(position: [f32; 3], uv: [f32; 2]) -> Vertex {
        Vertex {
            position,
            uv,
            normal: [0.0, 0.0, 1.0],
        }
    }

    #[test]
    fn derives_uv_tangents_and_mirrored_handedness() {
        let mut mesh = Mesh {
            vertices: vec![
                vertex([0.0, 0.0, 0.0], [0.0, 0.0]),
                vertex([1.0, 0.0, 0.0], [1.0, 0.0]),
                vertex([0.0, 1.0, 0.0], [0.0, 1.0]),
            ],
            tangents: None,
            indices: vec![0, 1, 2],
            node: 0,
            material: 0,
            mouse_rectangle: -1,
            skin_weights: None,
            bones: None,
        };
        assert_eq!(frames(&mesh)[0], [1.0, 0.0, 0.0, 1.0]);
        mesh.vertices[1].uv[0] = -1.0;
        assert_eq!(frames(&mesh)[0], [-1.0, 0.0, 0.0, -1.0]);
        mesh.tangents = Some(vec![
            esp_formats::mdl::geometry::Tangent {
                first: [0.0, 1.0, 0.0],
                second: [1.0, 0.0, 0.0],
            };
            3
        ]);
        assert_eq!(frames(&mesh)[0], [0.0, 1.0, 0.0, -1.0]);
    }
}

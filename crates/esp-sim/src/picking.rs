use esp_assets::Model;
use glam::{Mat4, Vec3};

use super::animation::ModelState;

pub struct PickResult {
    pub mesh: usize,
    pub uv: [f32; 2],
}

// shortcut: texture-alpha holes are geometric blockers, add alpha-aware picking when needed.
pub fn pick(
    model: &Model,
    state: &ModelState,
    aircraft_to_camera: Vec3,
    ray_origin: Vec3,
    ray_dir: Vec3,
) -> Option<PickResult> {
    let lod = model.lods.first()?;
    let mut closest: Option<(f32, usize, [f32; 2])> = None;

    for (mesh_index, mesh) in lod.meshes.iter().enumerate() {
        // Non-interactive geometry must still occlude controls behind it.
        if !state.visible.get(mesh.node).copied().unwrap_or(false) {
            continue;
        }
        let transform = match state.transforms.get(mesh.node) {
            Some(t) => *t,
            None => continue,
        };
        let model_matrix = Mat4::from_translation(aircraft_to_camera) * transform;
        let double_sided = model
            .materials
            .get(mesh.material)
            .is_some_and(|material| material.source.flags2 & 8 != 0);

        for triangle in mesh.indices.as_chunks::<3>().0 {
            let [i0, i1, i2] = [
                triangle[0] as usize,
                triangle[1] as usize,
                triangle[2] as usize,
            ];
            let (v0, v1, v2) = match (
                mesh.vertices.get(i0),
                mesh.vertices.get(i1),
                mesh.vertices.get(i2),
            ) {
                (Some(a), Some(b), Some(c)) => (a, b, c),
                _ => continue,
            };
            let p0 = model_matrix.transform_point3(Vec3::from_array(v0.position));
            let p1 = model_matrix.transform_point3(Vec3::from_array(v1.position));
            let p2 = model_matrix.transform_point3(Vec3::from_array(v2.position));

            let Some((t, u, v)) = moller_trumbore(ray_origin, ray_dir, p0, p1, p2) else {
                continue;
            };
            if !double_sided && (p1 - p0).cross(p2 - p0).dot(ray_dir) >= 0.0 {
                continue;
            }
            if closest.is_none_or(|(best, _, _)| t <= best) {
                let w = 1.0 - u - v;
                let uv = [
                    w * v0.uv[0] + u * v1.uv[0] + v * v2.uv[0],
                    w * v0.uv[1] + u * v1.uv[1] + v * v2.uv[1],
                ];
                closest = Some((t, mesh_index, uv));
            }
        }
    }

    closest.map(|(_, mesh, uv)| PickResult { mesh, uv })
}

fn moller_trumbore(
    origin: Vec3,
    direction: Vec3,
    v0: Vec3,
    v1: Vec3,
    v2: Vec3,
) -> Option<(f32, f32, f32)> {
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = direction.cross(edge2);
    let a = edge1.dot(h);
    if a.abs() < 1e-7 {
        return None;
    }
    let f = 1.0 / a;
    let s = origin - v0;
    let u = f * s.dot(h);
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let q = s.cross(edge1);
    let v = f * direction.dot(q);
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let t = f * edge2.dot(q);
    if t < 1e-4 {
        return None;
    }
    Some((t, u, v))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn intersects_triangle_and_rejects_misses() {
        let v0 = Vec3::new(-1.0, -1.0, 2.0);
        let v1 = Vec3::new(1.0, -1.0, 2.0);
        let v2 = Vec3::new(0.0, 1.0, 2.0);

        // Forward hit
        let hit = moller_trumbore(Vec3::ZERO, Vec3::Z, v0, v1, v2);
        assert!(hit.is_some());
        let (t, u, v) = hit.unwrap();
        assert!((t - 2.0).abs() < 1e-5);
        assert!(u >= 0.0 && v >= 0.0 && u + v <= 1.0);

        // Miss to the side
        assert!(moller_trumbore(Vec3::new(2.0, 0.0, 0.0), Vec3::Z, v0, v1, v2).is_none());

        // Behind origin
        assert!(moller_trumbore(Vec3::new(0.0, 0.0, 3.0), Vec3::Z, v0, v1, v2).is_none());
        assert!(moller_trumbore(Vec3::ZERO, Vec3::ZERO, v0, v1, v2).is_none());
        assert!(moller_trumbore(Vec3::ZERO, Vec3::Z, v2, v1, v0).is_some());
    }

    fn model() -> Model {
        use esp_assets::{Lod, Mesh, Node};
        let mesh = |node, mouse_rectangle| Mesh {
            vertices: [
                ([-1.0, -1.0, 0.0], [0.0, 0.0]),
                ([1.0, -1.0, 0.0], [1.0, 0.0]),
                ([0.0, 1.0, 0.0], [0.5, 1.0]),
            ]
            .into_iter()
            .map(|(position, uv)| esp_formats::mdl::geometry::Vertex {
                position,
                normal: [0.0, 0.0, -1.0],
                uv,
            })
            .collect(),
            indices: vec![0, 2, 1],
            node,
            material: 0,
            mouse_rectangle,
            skin_weights: None,
            bones: None,
        };
        Model {
            nodes: [2.0, 4.0]
                .into_iter()
                .map(|z| Node {
                    parent: None,
                    transform: Mat4::from_translation(Vec3::Z * z),
                    animation: None,
                    visibility: None,
                })
                .collect(),
            materials: vec![esp_assets::Material {
                source: esp_formats::mdl::material::materials(&[0; 120])
                    .unwrap()
                    .remove(0),
                diffuse_texture: None,
                metallic_texture: None,
                pbr: None,
            }],
            lods: vec![Lod {
                level: 100,
                meshes: vec![mesh(0, -1), mesh(1, 0)],
            }],
            animations: vec![],
            visibility: vec![],
            mouse_rectangles: vec![],
        }
    }

    #[test]
    fn picks_nearest_visible_surface_and_interpolates_uv() {
        let model = model();
        let mut state = ModelState::new(&model).unwrap();
        let hit = pick(&model, &state, Vec3::ZERO, Vec3::ZERO, Vec3::Z).unwrap();
        assert_eq!(hit.mesh, 0);
        assert_eq!(hit.uv, [0.5, 0.5]);
        state.visible[0] = false;
        assert_eq!(
            pick(&model, &state, Vec3::ZERO, Vec3::ZERO, Vec3::Z)
                .unwrap()
                .mesh,
            1
        );
        state.visible[1] = false;
        assert!(pick(&model, &state, Vec3::ZERO, Vec3::ZERO, Vec3::Z).is_none());
    }

    #[test]
    fn follows_runtime_transform_and_camera_offset() {
        let model = model();
        let mut state = ModelState::new(&model).unwrap();
        state.visible[1] = false;
        state.transforms[0] = Mat4::from_translation(Vec3::new(2.0, 0.0, 3.0));
        assert!(pick(&model, &state, Vec3::ZERO, Vec3::ZERO, Vec3::Z).is_none());
        let hit = pick(
            &model,
            &state,
            Vec3::new(-2.0, 0.0, -1.0),
            Vec3::ZERO,
            Vec3::Z,
        )
        .unwrap();
        assert_eq!(hit.mesh, 0);
        assert_eq!(hit.uv, [0.5, 0.5]);
    }

    #[test]
    fn respects_backface_culling_and_double_sided_materials() {
        let mut model = model();
        let mut state = ModelState::new(&model).unwrap();
        state.visible[1] = false;
        model.lods[0].meshes[0].indices = vec![0, 1, 2];
        assert!(pick(&model, &state, Vec3::ZERO, Vec3::ZERO, Vec3::Z).is_none());
        model.materials[0].source.flags2 = 8;
        assert!(pick(&model, &state, Vec3::ZERO, Vec3::ZERO, Vec3::Z).is_some());
        model.materials[0].source.flags2 = 0;
        state.transforms[0] =
            Mat4::from_translation(Vec3::Z * 2.0) * Mat4::from_scale(Vec3::new(-1.0, 1.0, 1.0));
        assert!(pick(&model, &state, Vec3::ZERO, Vec3::ZERO, Vec3::Z).is_some());
    }

    #[test]
    fn starts_at_near_plane_and_selects_later_coplanar_mesh() {
        let model = model();
        let mut state = ModelState::new(&model).unwrap();
        state.transforms[0] = Mat4::from_translation(Vec3::Z * 0.01);
        assert_eq!(
            pick(&model, &state, Vec3::ZERO, Vec3::Z * 0.05, Vec3::Z)
                .unwrap()
                .mesh,
            1
        );
        state.transforms[0] = state.transforms[1];
        assert_eq!(
            pick(&model, &state, Vec3::ZERO, Vec3::ZERO, Vec3::Z)
                .unwrap()
                .mesh,
            1
        );
        state.transforms[1] = Mat4::from_translation(Vec3::Z * 2.0);
        assert_eq!(
            pick(&model, &state, Vec3::ZERO, Vec3::ZERO, Vec3::Z)
                .unwrap()
                .mesh,
            1
        );
    }
}

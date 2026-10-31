use esp_assets::Aircraft;
use esp_script::{Error, Host, Reference, Value};
use esp_sim::animation::ModelState;

#[path = "../../../tests/support/content.rs"]
mod content;

struct PoseHost {
    door: f64,
}
impl Host for PoseHost {
    fn read(&mut self, reference: &Reference) -> Result<Value, Error> {
        Ok(Value::Number(if reference.name == "EXIT OPEN:0" {
            self.door
        } else {
            0.0
        }))
    }
    fn write(&mut self, _: &Reference, _: Value) -> Result<(), Error> {
        Ok(())
    }
}

#[test]
#[ignore = "requires OPENESP_FSX_ROOT or OPENESP_AIRCRAFT_PATHS"]
fn animates_external_aircraft_hierarchies() {
    let mut changed = 0;
    for path in content::aircraft_paths() {
        let aircraft = Aircraft::load(&path, 0).unwrap();
        for model in std::iter::once(&aircraft.exterior).chain(&aircraft.interior) {
            let mut state = ModelState::new(model).unwrap();
            let mut host = PoseHost { door: 0.0 };
            state.update(model, &mut host, 0.0).unwrap();
            let initial = state.transforms.clone();
            assert!(state.transforms.iter().all(|t| t.is_finite()));
            host.door = 100.0;
            state.update(model, &mut host, 1.0).unwrap();
            changed += initial
                .iter()
                .zip(&state.transforms)
                .filter(|(a, b)| !a.abs_diff_eq(**b, 1e-5))
                .count();
            for (index, node) in model.nodes.iter().enumerate() {
                if node.parent.is_some_and(|p| !state.visible[p]) {
                    assert!(!state.visible[index]);
                }
            }
            assert!(state.update(model, &mut host, f64::NAN).is_err());
        }
    }
    assert!(changed > 0, "door parameter did not animate any nodes");
    eprintln!("{changed} node transforms changed with door input");
}

#[test]
#[ignore = "requires OPENESP_FSX_ROOT or OPENESP_AIRCRAFT_PATHS"]
fn picks_external_cockpit_panel_surfaces() {
    use esp_sim::picking::pick;
    use glam::Vec3;
    for path in content::aircraft_paths() {
        let aircraft = Aircraft::load(&path, 0).unwrap();
        let model = aircraft.interior.as_ref().expect("cockpit model");
        let mut state = ModelState::new(model).unwrap();
        state
            .update(model, &mut PoseHost { door: 0.0 }, 0.0)
            .unwrap();
        assert!(state.diagnostics.is_empty(), "{:?}", state.diagnostics);
        let eye = aircraft.eyepoint.expect("cockpit eyepoint");
        let lod = model.lods.first().unwrap();
        let mut hits = std::collections::HashSet::new();
        for mesh in &lod.meshes {
            if !model.materials[mesh.material]
                .diffuse_texture
                .as_ref()
                .is_some_and(|name| name.starts_with('$'))
            {
                continue;
            }
            for indices in mesh.indices.as_chunks::<3>().0 {
                let target = indices
                    .iter()
                    .map(|&index| {
                        state.transforms[mesh.node].transform_point3(Vec3::from_array(
                            mesh.vertices[index as usize].position,
                        ))
                    })
                    .sum::<Vec3>()
                    / 3.0;
                let Some(direction) = (target - eye).try_normalize() else {
                    continue;
                };
                let Some(hit) = pick(model, &state, -eye, Vec3::ZERO, direction) else {
                    continue;
                };
                let texture = model.materials[lod.meshes[hit.mesh].material]
                    .diffuse_texture
                    .as_ref();
                if texture.is_some_and(|name| name.starts_with('$')) {
                    assert!(hit.uv.iter().all(|value| value.is_finite()));
                    hits.insert(hit.mesh);
                    break;
                }
            }
        }
        assert!(
            !hits.is_empty(),
            "no panel surface visible from {} eyepoint",
            aircraft.title
        );
        eprintln!(
            "{}: {} distinct panel surfaces picked",
            aircraft.title,
            hits.len()
        );
    }
}

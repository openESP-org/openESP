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

#[test]
#[ignore = "requires original XML flight director gauges and OPENESP_AIRCRAFT_PATHS"]
fn xml_click_updates_flight_director_artwork_and_animation() {
    use esp_sim::variables::Variables;
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let mut aircraft = Aircraft::load(&path, 0).unwrap();
        let model = aircraft.interior.as_ref().unwrap();
        let mut state = ModelState::new(model).unwrap();
        let mut variables = Variables::default();
        variables.set_number("AUTOPILOT FLIGHT DIRECTOR ACTIVE", "bool", 0.0);
        state.update(model, &mut variables, 0.0).unwrap();
        let initial_transforms = state.transforms.clone();
        for panel in aircraft.panel_images.values_mut() {
            panel.update(&mut variables);
            let control = panel.mouse_areas.iter().find_map(|area| {
                let mut probe = Variables::default();
                area.script.evaluate(&mut probe).ok()?;
                probe
                    .drain_events()
                    .any(|(name, _)| name == "TOGGLE_FLIGHT_DIRECTOR")
                    .then_some((area.x + area.w * 0.5, area.y + area.h * 0.5))
            });
            let Some((x, y)) = control else { continue };
            let before = panel.image.clone();
            assert!(panel.left_click(x, y, &mut variables).unwrap());
            let events = variables.drain_events().collect::<Vec<_>>();
            assert!(
                events
                    .iter()
                    .any(|(name, _)| name == "TOGGLE_FLIGHT_DIRECTOR")
            );
            for (event, value) in events {
                variables.dispatch_event(&event, value).unwrap();
            }
            assert_eq!(
                variables
                    .read(&Reference::new(
                        'A',
                        "AUTOPILOT FLIGHT DIRECTOR ACTIVE",
                        Some("bool")
                    ))
                    .unwrap(),
                Value::Number(1.0)
            );
            assert!(panel.update(&mut variables));
            assert!(
                before
                    .pixels()
                    .zip(panel.image.pixels())
                    .any(|(a, b)| a != b)
            );
            state.update(model, &mut variables, 1.0).unwrap();
            assert!(
                initial_transforms
                    .iter()
                    .zip(&state.transforms)
                    .any(|(a, b)| !a.abs_diff_eq(*b, 1e-5))
            );
            eprintln!(
                "{}: XML flight director click changed state, artwork and animation",
                aircraft.title
            );
            verified += 1;
            break;
        }
    }
    assert!(verified > 0, "no original XML flight director verified");
}

#[test]
#[ignore = "requires original XML clock gauges and OPENESP_AIRCRAFT_PATHS"]
fn original_clock_button_responds_to_press_and_release() {
    use esp_assets::gauge::MouseEvent;
    use esp_sim::variables::Variables;
    let depressed = Reference::new('L', "Clock DAT Depressed", None);
    let function = Reference::new('L', "Clock DAT Function", None);
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let aircraft = Aircraft::load(&path, 0).unwrap();
        for panel in aircraft.panel_images.values() {
            for (index, area) in panel.mouse_areas.iter().enumerate() {
                if !area.events.contains(&MouseEvent::LeftRelease) {
                    continue;
                }
                let mut variables = Variables::default();
                if panel
                    .mouse_event(MouseEvent::LeftSingle, &[index], &mut variables)
                    .is_err()
                {
                    continue;
                }
                if variables.read(&depressed).unwrap() != Value::Number(1.0) {
                    continue;
                }
                let captured = panel.mouse_areas_at(area.x + area.w * 0.5, area.y + area.h * 0.5);
                panel
                    .mouse_event(MouseEvent::LeftRelease, &captured, &mut variables)
                    .unwrap();
                assert_eq!(variables.read(&depressed).unwrap(), Value::Number(0.0));
                assert_eq!(variables.read(&function).unwrap(), Value::Number(1.0));
                assert_eq!(variables.drain_events().count(), 0);
                verified += 1;
            }
        }
    }
    assert!(verified > 0, "no original XML clock button verified");
}

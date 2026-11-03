use esp_assets::Aircraft;
use esp_script::{Error, Host, Reference, Value};
use esp_sim::animation::ModelState;

#[path = "../../../tests/support/content.rs"]
mod content;

struct PoseHost {
    door: f64,
}

#[test]
#[ignore = "requires original MDL cockpit controls and OPENESP_AIRCRAFT_PATHS"]
fn original_model_click_changes_switch_state_and_animation() {
    use esp_assets::gauge::MouseEvent;
    use esp_sim::{interaction::model_mouse, picking::pick, variables::Variables};
    use glam::Vec3;

    let mut verified = std::collections::HashSet::new();
    for path in content::aircraft_paths() {
        let aircraft = Aircraft::load(&path, 0).unwrap();
        let model = aircraft.interior.as_ref().unwrap();
        let eye = aircraft.eyepoint.unwrap();
        for (id, name) in [
            (0x102c1, "ELECTRICAL MASTER BATTERY"),
            (0x102f0, "AUTOPILOT FLIGHT DIRECTOR ACTIVE"),
            (0x100d8, "BRAKE PARKING POSITION"),
        ] {
            let Some(index) = model
                .mouse_rectangles
                .iter()
                .position(|r| r.key_event == Some(id))
            else {
                continue;
            };
            let mut variables = Variables::default();
            variables.set_number(name, "bool", 0.0);
            let mut state = ModelState::new(model).unwrap();
            state.update(model, &mut variables, 0.0).unwrap();
            let before = state.transforms.clone();
            let reachable = model.lods[0]
                .meshes
                .iter()
                .filter(|mesh| mesh.mouse_rectangle == index as i32)
                .any(|mesh| {
                    mesh.indices.as_chunks::<3>().0.iter().any(|triangle| {
                        let target = triangle
                            .iter()
                            .map(|&i| {
                                state.transforms[mesh.node].transform_point3(Vec3::from_array(
                                    mesh.vertices[i as usize].position,
                                ))
                            })
                            .sum::<Vec3>()
                            / 3.0;
                        let Some(direction) = (target - eye).try_normalize() else {
                            return false;
                        };
                        pick(model, &state, -eye, Vec3::ZERO, direction).is_some_and(|hit| {
                            model.lods[0].meshes[hit.mesh].mouse_rectangle == index as i32
                        })
                    })
                });
            assert!(reachable, "{}: {name} is occluded", aircraft.title);
            assert!(model_mouse(model, index, MouseEvent::LeftSingle, &mut variables).unwrap());
            assert_eq!(
                variables
                    .read(&Reference::new('A', name, Some("bool")))
                    .unwrap(),
                Value::Number(1.0)
            );
            state.update(model, &mut variables, 1.0).unwrap();
            assert!(
                before
                    .iter()
                    .zip(&state.transforms)
                    .any(|(a, b)| !a.abs_diff_eq(*b, 1e-5)),
                "{}: {name} did not animate",
                aircraft.title
            );
            assert!(!model_mouse(model, index, MouseEvent::LeftRelease, &mut variables).unwrap());
            assert!(!model_mouse(model, index, MouseEvent::DownRepeat, &mut variables).unwrap());
            eprintln!(
                "{}: picked MDL {name}, updated state and animation",
                aircraft.title
            );
            verified.insert(id);
        }
    }
    assert!(
        verified.contains(&0x102c1),
        "no original battery switch verified"
    );
    assert!(
        verified.contains(&0x102f0),
        "no original flight director verified"
    );
    assert!(
        verified.contains(&0x100d8),
        "no original parking brake verified"
    );
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
                    .mouse_event(
                        MouseEvent::LeftSingle,
                        &[index],
                        [area.x, area.y],
                        &mut variables,
                    )
                    .is_err()
                {
                    continue;
                }
                if variables.read(&depressed).unwrap() != Value::Number(1.0) {
                    continue;
                }
                let captured = panel.mouse_areas_at(area.x + area.w * 0.5, area.y + area.h * 0.5);
                panel
                    .mouse_event(
                        MouseEvent::LeftRelease,
                        &captured,
                        [area.x, area.y],
                        &mut variables,
                    )
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

#[test]
#[ignore = "requires original XML speed brake gauges and OPENESP_AIRCRAFT_PATHS"]
fn original_speed_brake_drag_uses_scaled_gauge_coordinates() {
    use esp_assets::{gauge::MouseEvent, panel::Panel};
    use esp_formats::gauge::Gauge;
    use esp_sim::variables::Variables;
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let aircraft = Aircraft::load(&path, 0).unwrap();
        let Some(panel) = &aircraft.panel else {
            continue;
        };
        let mut seen = std::collections::HashSet::new();
        for gauge in panel
            .layout()
            .unwrap()
            .windows
            .iter()
            .flat_map(|window| &window.gauges)
        {
            if !seen.insert(gauge.reference) {
                continue;
            }
            let Some(xml) = panel
                .read_xml_gauge(gauge.reference, 8 * 1024 * 1024)
                .unwrap()
            else {
                continue;
            };
            let text = std::str::from_utf8(&xml).unwrap();
            if !text.contains("SPOILERS_SET") || !text.contains("LeftDrag") {
                continue;
            }
            let size = Gauge::parse(text).unwrap().size().unwrap().unwrap();
            let root = tempfile::tempdir().unwrap();
            let directory = root.path().join("panel");
            std::fs::create_dir_all(directory.join("sample")).unwrap();
            std::fs::write(directory.join("sample/control.xml"), &xml).unwrap();
            let (w, h) = (size[0], size[1]);
            std::fs::write(directory.join("panel.cfg"), format!("[VCockpit01]\nsize_mm={},{}\npixel_size={},{}\ntexture=$test\ngauge00=sample!control,10,20,{w},{h}", w + 20, h + 40, (w + 20) * 2, (h + 40) * 2)).unwrap();
            let loaded = Panel::load(root.path(), None).unwrap().unwrap();
            let image = esp_assets::gauge::load_panel(&loaded, "$test")
                .unwrap()
                .unwrap();
            for (index, area) in image.mouse_areas.iter().enumerate() {
                if !area.events.contains(&MouseEvent::LeftDrag) {
                    continue;
                }
                let mut variables = Variables::default();
                let position = [area.x + area.w * 0.5, area.y + area.h * 0.25];
                if image
                    .mouse_event(MouseEvent::LeftSingle, &[index], position, &mut variables)
                    .is_err()
                {
                    continue;
                }
                if !variables
                    .drain_events()
                    .any(|(name, _)| name == "SPOILERS_SET")
                {
                    continue;
                }
                for fraction in [0.0, 0.5, 1.0, 1.5] {
                    let position = [area.x + area.w * 0.5, area.y + area.h * fraction];
                    image
                        .mouse_event(MouseEvent::LeftDrag, &[index], position, &mut variables)
                        .unwrap();
                    assert!(variables.dispatch_events().is_empty());
                    let actual = variables
                        .read(&Reference::new(
                            'A',
                            "SPOILERS HANDLE POSITION",
                            Some("percent"),
                        ))
                        .unwrap()
                        .number()
                        .unwrap();
                    let expected = (fraction * 16384.0 / 16383.0 * 100.0).clamp(0.0, 100.0);
                    assert!((actual - expected).abs() < 1e-9);
                }
                verified += 1;
            }
        }
    }
    assert!(verified > 0, "no original speed brake drag verified");
}

#[test]
#[ignore = "requires original XML autopilot gauges and OPENESP_AIRCRAFT_PATHS"]
fn original_heading_knob_repeats_its_script() {
    use esp_assets::gauge::MouseEvent;
    use esp_sim::variables::Variables;
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let aircraft = Aircraft::load(&path, 0).unwrap();
        for panel in aircraft.panel_images.values() {
            for (index, area) in panel.mouse_areas.iter().enumerate() {
                if !area.repeat {
                    continue;
                }
                let mut variables = Variables::default();
                let position = [area.x + area.w * 0.5, area.y + area.h * 0.5];
                if panel
                    .mouse_event(MouseEvent::LeftSingle, &[index], position, &mut variables)
                    .is_err()
                {
                    continue;
                }
                let events = variables.drain_events().collect::<Vec<_>>();
                if !events.iter().any(|(name, _)| name == "HEADING_BUG_INC") {
                    continue;
                }
                for (event, value) in events {
                    variables.dispatch_event(&event, value).unwrap();
                }
                for (expected, heading) in [(20.0, 2.0), (30.0, 3.0), (40.0, 4.0)] {
                    panel
                        .mouse_event(MouseEvent::DownRepeat, &[index], position, &mut variables)
                        .unwrap();
                    let events = variables.drain_events().collect::<Vec<_>>();
                    assert_eq!(events, [("HEADING_BUG_INC".into(), Value::Number(1.0))]);
                    for (event, value) in events {
                        variables.dispatch_event(&event, value).unwrap();
                    }
                    assert_eq!(
                        variables
                            .read(&Reference::new(
                                'A',
                                "AUTOPILOT HEADING LOCK DIR",
                                Some("degrees")
                            ))
                            .unwrap(),
                        Value::Number(heading)
                    );
                    assert_eq!(
                        variables.read(&Reference::new('G', "Var3", None)).unwrap(),
                        Value::Number(expected)
                    );
                }
                verified += 1;
            }
        }
    }
    assert!(verified > 0, "no original repeating heading knob verified");
}

#[test]
#[ignore = "requires original XML EFIS gauges and OPENESP_AIRCRAFT_PATHS"]
fn original_efis_update_initializes_once() {
    use esp_sim::variables::Variables;
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let mut aircraft = Aircraft::load(&path, 0).unwrap();
        let mut variables = Variables::default();
        for panel in aircraft.panel_images.values_mut() {
            panel.advance(0.0, &mut variables).unwrap();
        }
        let init = Reference::new('L', "Init", None);
        if variables.read(&init).unwrap() != Value::Number(1.0) {
            continue;
        }
        for (name, expected) in [
            ("Display Scale", 1.0),
            ("MapItem Shown", 1.0),
            ("EFIS Mode", 2.0),
        ] {
            assert_eq!(
                variables.read(&Reference::new('L', name, None)).unwrap(),
                Value::Number(expected)
            );
        }
        variables.set('L', "EFIS Mode", None, Value::Number(3.0));
        variables.set('L', "Display Scale", None, Value::Number(5.0));
        for panel in aircraft.panel_images.values_mut() {
            panel.advance(1.0, &mut variables).unwrap();
        }
        assert_eq!(
            variables
                .read(&Reference::new('L', "EFIS Mode", None))
                .unwrap(),
            Value::Number(3.0)
        );
        assert_eq!(
            variables
                .read(&Reference::new('L', "Display Scale", None))
                .unwrap(),
            Value::Number(5.0)
        );
        verified += 1;
    }
    assert!(
        verified > 0,
        "no original EFIS initialization update verified"
    );
}

#[test]
#[ignore = "requires original XML EFIS gauges and OPENESP_AIRCRAFT_PATHS"]
fn original_efis_standard_pressure_click_emits_event() {
    use esp_assets::gauge::MouseEvent;
    use esp_sim::variables::Variables;
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let aircraft = Aircraft::load(&path, 0).unwrap();
        for panel in aircraft.panel_images.values() {
            for (index, area) in panel.mouse_areas.iter().enumerate() {
                let mut variables = Variables::default();
                if panel
                    .mouse_event(
                        MouseEvent::LeftSingle,
                        &[index],
                        [area.x, area.y],
                        &mut variables,
                    )
                    .is_err()
                {
                    continue;
                }
                if variables.drain_events().any(|(name, value)| {
                    name == "BAROMETRIC_STD_PRESSURE" && value == Value::Number(1.0)
                }) {
                    verified += 1;
                }
            }
        }
    }
    assert!(
        verified > 0,
        "no original EFIS standard-pressure button verified"
    );
}

#[test]
#[ignore = "requires original XML flap indicators and OPENESP_AIRCRAFT_PATHS"]
fn original_flap_indicator_rotates_with_surface_angles() {
    use esp_sim::variables::Variables;
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let mut aircraft = Aircraft::load(&path, 0).unwrap();
        for panel in aircraft.panel_images.values_mut() {
            let mut variables = Variables::default();
            let set_flaps = |variables: &mut Variables, angle| {
                for surface in [
                    "TRAILING EDGE FLAPS0 LEFT ANGLE",
                    "TRAILING EDGE FLAPS0 RIGHT ANGLE",
                    "TRAILING EDGE FLAPS1 LEFT ANGLE",
                    "TRAILING EDGE FLAPS1 RIGHT ANGLE",
                ] {
                    variables.set_number(surface, "degrees", angle);
                }
            };
            set_flaps(&mut variables, 0.0);
            panel.update(&mut variables);
            let before = panel.image.clone();
            set_flaps(&mut variables, 40.0);
            let started = std::time::Instant::now();
            if !panel.update(&mut variables) {
                continue;
            }
            let update_time = started.elapsed();
            let differences = before
                .pixels()
                .zip(panel.image.pixels())
                .filter(|(a, b)| a != b)
                .count();
            assert!(differences > 30, "flap indicator did not visibly rotate");
            assert!(
                !panel.update(&mut variables),
                "unchanged flap angles rebuilt artwork"
            );
            set_flaps(&mut variables, 50.0);
            assert!(
                !panel.update(&mut variables),
                "flap gauge did not clamp at its configured limit"
            );
            eprintln!(
                "{}: flap needle changed {differences} pixels in {update_time:?}",
                aircraft.title,
            );
            verified += 1;
        }
    }
    assert!(verified > 0, "no original rotating flap indicator verified");
}

#[test]
#[ignore = "requires original aircraft flap definitions and OPENESP_AIRCRAFT_PATHS"]
fn flap_commands_drive_configured_actuators_models_and_gauges() {
    use esp_sim::variables::Variables;
    let mut gauge_changes = 0;
    for path in content::aircraft_paths() {
        let mut aircraft = Aircraft::load(&path, 0).unwrap();
        let primary = aircraft.flaps.iter().find(|set| set.kind == 1).unwrap();
        let time = primary.extending_time.unwrap();
        let first = primary.positions[0].angle;
        let last = primary.positions.last().unwrap().angle;
        let mut variables = Variables::default();
        variables.configure_flaps(&aircraft.flaps).unwrap();
        let detent = Reference::new('A', "FLAPS HANDLE INDEX", Some("number"));
        variables
            .dispatch_key_event(0x1003d, Value::Number(0.0))
            .unwrap();
        assert_eq!(variables.read(&detent).unwrap(), Value::Number(1.0));
        variables.advance(time).unwrap();
        let first_detent = primary
            .positions
            .iter()
            .find(|position| position.index == 1)
            .unwrap();
        let angle = Reference::new('A', "TRAILING EDGE FLAPS0 LEFT ANGLE", Some("degrees"));
        assert!(
            (variables.read(&angle).unwrap().number().unwrap() - first_detent.angle).abs() < 1e-9
        );
        variables
            .dispatch_key_event(0x103e6, Value::Number(-16383.0))
            .unwrap();
        variables.advance(time).unwrap();
        assert_eq!(variables.read(&detent).unwrap(), Value::Number(0.0));
        let mut state = ModelState::new(&aircraft.exterior).unwrap();
        state
            .update(&aircraft.exterior, &mut variables, 0.0)
            .unwrap();
        let initial = state.transforms.clone();
        let images: Vec<_> = aircraft
            .panel_images
            .values_mut()
            .map(|panel| {
                panel.update(&mut variables);
                panel.image.clone()
            })
            .collect();
        variables
            .dispatch_key_event(0x10043, Value::Number(0.0))
            .unwrap();
        let angle = Reference::new('A', "TRAILING EDGE FLAPS0 LEFT ANGLE", Some("degrees"));
        assert_eq!(variables.read(&angle).unwrap(), Value::Number(first));
        let dt = time / 1200.0;
        for _ in 0..600 {
            variables.advance(dt).unwrap();
            state
                .update(&aircraft.exterior, &mut variables, dt)
                .unwrap();
        }
        let half = variables.read(&angle).unwrap().number().unwrap();
        assert!((half - (first + last) * 0.5).abs() < 1e-9);
        assert!(
            initial
                .iter()
                .zip(&state.transforms)
                .any(|(a, b)| !a.abs_diff_eq(*b, 1e-5)),
            "{}: flap actuator did not animate model",
            aircraft.title
        );
        let ratio = variables
            .read(&Reference::new(
                'A',
                "TRAILING EDGE FLAPS LEFT PERCENT",
                Some("percent"),
            ))
            .unwrap()
            .number()
            .unwrap();
        assert!((ratio - 50.0).abs() < 1e-9);
        for (before, panel) in images.iter().zip(aircraft.panel_images.values_mut()) {
            panel.update(&mut variables);
            gauge_changes += before
                .pixels()
                .zip(panel.image.pixels())
                .filter(|(a, b)| a != b)
                .count();
        }
        let mut single_step = Variables::default();
        single_step.configure_flaps(&aircraft.flaps).unwrap();
        single_step
            .dispatch_event("FLAPS_DOWN", Value::Number(0.0))
            .unwrap();
        single_step.advance(time * 0.5).unwrap();
        let single_angle = single_step.read(&angle).unwrap().number().unwrap();
        assert!((single_angle - half).abs() < 1e-9);
        variables.advance(time).unwrap();
        assert!((variables.read(&angle).unwrap().number().unwrap() - last).abs() < 1e-9);
        variables
            .dispatch_event("FLAPS_UP", Value::Number(0.0))
            .unwrap();
        variables.advance(time).unwrap();
        assert!((variables.read(&angle).unwrap().number().unwrap() - first).abs() < 1e-9);
        eprintln!(
            "{}: configured {time}s flap travel verified through events, model transforms and gauge inputs",
            aircraft.title
        );
    }
    assert!(
        gauge_changes > 30,
        "no original flap instrument responded to actuator state"
    );
}

#[test]
#[ignore = "requires original aircraft flap definitions and OPENESP_AIRCRAFT_PATHS"]
fn fixed_ticks_preserve_aircraft_motion_across_render_rates() {
    use esp_sim::{
        time::{Clock, STEP},
        variables::Variables,
    };
    for path in content::aircraft_paths() {
        let aircraft = Aircraft::load(&path, 0).unwrap();
        let primary = aircraft.flaps.iter().find(|s| s.kind == 1).unwrap();
        let first = primary.positions[0].angle;
        let last = primary.positions.last().unwrap().angle;
        let expected =
            first + (last - first) * ((2.0 - STEP) / primary.extending_time.unwrap()).min(1.0);
        let mut samples = Vec::new();
        for fps in [24, 144] {
            let mut clock = Clock::default();
            let mut variables = Variables::default();
            variables.configure_flaps(&aircraft.flaps).unwrap();
            variables
                .dispatch_event("FLAPS_DOWN", Value::Number(0.0))
                .unwrap();
            let mut ticks = 0;
            for _ in 0..fps * 2 {
                let frame = clock.advance(1.0 / f64::from(fps)).unwrap();
                variables.interpolate(1.0).unwrap();
                for _ in 0..frame.steps {
                    variables.advance(STEP).unwrap();
                }
                ticks += frame.steps;
                variables.interpolate(frame.alpha).unwrap();
            }
            assert_eq!(ticks, 120);
            let angle = variables
                .read(&Reference::new(
                    'A',
                    "TRAILING EDGE FLAPS0 LEFT ANGLE",
                    Some("degrees"),
                ))
                .unwrap()
                .number()
                .unwrap();
            assert!(
                (angle - expected).abs() < 1e-8,
                "{}: {fps} fps produced {angle}, expected {expected}",
                aircraft.title
            );
            samples.push(angle);
        }
        assert!((samples[0] - samples[1]).abs() < 1e-9);
        eprintln!("{}: flap state matches at 24 and 144 fps", aircraft.title);
    }
}

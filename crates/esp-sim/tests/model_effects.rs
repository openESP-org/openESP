use esp_sim::{animation::ModelState, variables::Variables};

#[test]
#[ignore = "requires the original B737 in OPENESP_AIRCRAFT_PATHS"]
fn landing_switch_controls_real_model_effect_nodes() {
    let paths = std::env::var_os("OPENESP_AIRCRAFT_PATHS").unwrap();
    let root = std::env::split_paths(&paths).next().unwrap();
    let model = esp_assets::Model::load(&root.join("model/B737_800.mdl")).unwrap();
    let mut state = ModelState::new(&model).unwrap();
    let mut variables = Variables::default();
    variables.set_number("LIGHT LANDING", "bool", 0.0);
    state.update(&model, &mut variables, 0.016).unwrap();
    assert!(
        model
            .effects
            .iter()
            .all(|effect| !state.visible[effect.node])
    );
    variables.set_number("LIGHT LANDING", "bool", 1.0);
    state.update(&model, &mut variables, 0.016).unwrap();
    assert!(
        model
            .effects
            .iter()
            .all(|effect| state.visible[effect.node])
    );
}

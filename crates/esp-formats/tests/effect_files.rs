use esp_formats::cfg::{Config, effect, lights};
use std::path::PathBuf;

#[test]
#[ignore = "requires OPENESP_AIRCRAFT_PATHS and OPENESP_P3D_ROOT"]
fn decodes_real_aircraft_lights_and_installed_effects() {
    let paths = std::env::var_os("OPENESP_AIRCRAFT_PATHS").unwrap();
    let aircraft = std::env::split_paths(&paths).next().unwrap();
    let root = PathBuf::from(std::env::var_os("OPENESP_P3D_ROOT").unwrap());
    let load = |path: PathBuf| {
        Config::from_bytes(&std::fs::read(path).unwrap(), encoding_rs::WINDOWS_1252).unwrap()
    };
    let lights = lights::from_config(&load(aircraft.join("aircraft.cfg"))).unwrap();
    assert!(lights.len() >= 8);
    for light in &lights {
        let path = root.join("Effects").join(format!("{}.fx", light.effect));
        let particles = effect::particles(&load(path)).unwrap();
        assert!(!particles.is_empty(), "{} has no emitters", light.effect);
        assert!(
            particles
                .iter()
                .any(|p| p.kind == 19 && p.is_static && p.texture.is_some())
        );
    }
    assert!(
        lights
            .iter()
            .any(|light| light.kind == 3 && light.position_feet[1] < 0.0)
    );
    assert!(lights.iter().any(|light| light.kind == 2));
    let landing = effect::particles(&load(root.join("Effects/fx_landing.fx"))).unwrap();
    let spotlight = landing.iter().find(|particle| particle.kind == 28).unwrap();
    assert_eq!(spotlight.inner_cone_angle, 50.0);
    assert_eq!(spotlight.outer_cone_angle, 70.0);
    assert_eq!(spotlight.intensity_night, 2000.0);
}

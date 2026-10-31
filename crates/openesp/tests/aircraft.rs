use std::{
    path::Path,
    process::{Command, Stdio},
    time::{Duration, Instant},
};

#[path = "../../../tests/support/content.rs"]
mod content;

fn capture(aircraft: &Path, cockpit: bool, output: &Path) -> image::RgbaImage {
    let mut command = Command::new(env!("CARGO_BIN_EXE_openesp"));
    command
        .arg("--aircraft")
        .arg(aircraft)
        .args(["--frames", "1", "--screenshot"])
        .arg(output);
    if cockpit {
        command.arg("--cockpit");
    }
    let mut child = command.stdout(Stdio::null()).spawn().unwrap();
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        if let Some(status) = child.try_wait().unwrap() {
            assert!(status.success());
            break;
        }
        if Instant::now() >= deadline {
            child.kill().unwrap();
            child.wait().unwrap();
            panic!("render timed out");
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    let image = image::open(output).unwrap().into_rgba8();
    assert!(image.width() >= 640 && image.height() >= 480);
    let colours: std::collections::HashSet<_> = image
        .pixels()
        .map(|p| [p[0] / 16, p[1] / 16, p[2] / 16])
        .collect();
    assert!(colours.len() > 32, "frame has no textured geometry");
    image
}

#[test]
#[ignore = "requires a GPU, desktop and OPENESP_FSX_ROOT or OPENESP_AIRCRAFT_PATHS"]
fn renders_external_aircraft_and_cockpits() {
    let temp = tempfile::tempdir().unwrap();
    for path in content::aircraft_paths() {
        let exterior = capture(&path, false, &temp.path().join("exterior.png"));
        let interior = capture(&path, true, &temp.path().join("interior.png"));
        assert_eq!(exterior.dimensions(), interior.dimensions());
        let difference = exterior
            .pixels()
            .zip(interior.pixels())
            .filter(|(a, b)| a != b)
            .count();
        assert!(
            difference > exterior.len() / 4 / 10,
            "cockpit did not change the scene"
        );
    }
}

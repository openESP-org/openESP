use std::{
    path::Path,
    process::{Command, Stdio},
    time::{Duration, Instant},
};

#[path = "../../../tests/support/content.rs"]
mod content;

fn capture(aircraft: &Path, cockpit: bool, variables: &[&str], output: &Path) -> image::RgbaImage {
    capture_after(aircraft, cockpit, variables, None, 1, output)
}

fn capture_after(
    aircraft: &Path,
    cockpit: bool,
    variables: &[&str],
    event: Option<&str>,
    frames: u64,
    output: &Path,
) -> image::RgbaImage {
    let mut command = Command::new(env!("CARGO_BIN_EXE_openesp"));
    command
        .arg("--aircraft")
        .arg(aircraft)
        .arg("--frames")
        .arg(frames.to_string())
        .arg("--screenshot")
        .arg(output);
    if let Some(event) = event {
        command.args(["--event", event]);
    }
    if cockpit {
        command.arg("--cockpit");
    }
    for variable in variables {
        command.args(["--variable", variable]);
    }
    capture_command(command, output)
}

fn capture_command(mut command: Command, output: &Path) -> image::RgbaImage {
    let mut child = command.stdout(Stdio::null()).spawn().unwrap();
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        if let Some(status) = child.try_wait().unwrap() {
            assert!(
                status.success(),
                "render failed: {}",
                std::fs::read_to_string(output.with_extension("log")).unwrap_or_default()
            );
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
#[ignore = "requires a GPU, desktop and original aircraft AIR data"]
fn renders_airborne_motion_with_opposite_control_inputs() {
    let temp = tempfile::tempdir().unwrap();
    for (index, path) in content::aircraft_paths().iter().enumerate() {
        let mut frames = Vec::new();
        for (direction, percent) in [("left", -25), ("right", 25)] {
            let screenshot = temp.path().join(format!("{index}-{direction}.png"));
            let log_path = screenshot.with_extension("log");
            let mut command = Command::new(env!("CARGO_BIN_EXE_openesp"));
            command
                .arg("--aircraft")
                .arg(path)
                .args([
                    "--glide",
                    "150",
                    "--fuel-percent",
                    "50",
                    "--payload",
                    "--height",
                    "1000",
                    "--frames",
                    "120",
                    "--variable",
                ])
                .arg(format!("A:AILERON LEFT DEFLECTION PCT,percent={percent}"))
                .arg("--screenshot")
                .arg(&screenshot)
                .stderr(std::fs::File::create(&log_path).unwrap());
            frames.push(capture_command(command, &screenshot));
            let log = std::fs::read_to_string(log_path).unwrap();
            let state = log
                .split("flight state: ")
                .nth(1)
                .expect("missing final flight state");
            let north: f64 = state
                .split("position: DVec3(")
                .nth(1)
                .unwrap()
                .split(',')
                .next()
                .unwrap()
                .parse()
                .unwrap();
            let roll: f64 = state
                .split("orientation: DQuat(")
                .nth(1)
                .unwrap()
                .split(',')
                .next()
                .unwrap()
                .parse()
                .unwrap();
            assert!(north > 1.0, "aircraft did not move forward: {state}");
            assert!(
                roll * percent as f64 > 0.0,
                "aircraft banked against the input: {state}"
            );
        }
        assert!(
            frames[0]
                .pixels()
                .zip(frames[1].pixels())
                .filter(|(a, b)| a != b)
                .count()
                > 1000
        );
    }
}

#[test]
#[ignore = "requires a GPU, desktop and OPENESP_FSX_ROOT or OPENESP_AIRCRAFT_PATHS"]
fn renders_external_aircraft_and_cockpits() {
    let temp = tempfile::tempdir().unwrap();
    for path in content::aircraft_paths() {
        let exterior = capture(&path, false, &[], &temp.path().join("exterior.png"));
        let interior = capture(&path, true, &[], &temp.path().join("interior.png"));
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

#[test]
#[ignore = "requires a GPU, desktop and original aircraft with EXIT OPEN:0 animations"]
fn renders_door_animation() {
    let temp = tempfile::tempdir().unwrap();
    let mut differences = 0;
    for path in content::aircraft_paths() {
        let closed = capture(
            &path,
            false,
            &["A:EXIT OPEN:0,percent=0"],
            &temp.path().join("closed.png"),
        );
        let open = capture(
            &path,
            false,
            &["A:EXIT OPEN:0,percent=100"],
            &temp.path().join("open.png"),
        );
        differences += closed
            .pixels()
            .zip(open.pixels())
            .filter(|(a, b)| a != b)
            .count();
    }
    assert!(
        differences > 1000,
        "door input did not visibly change the rendered aircraft"
    );
}

#[test]
#[ignore = "requires a GPU, desktop and an aircraft with XML fire annunciators"]
fn renders_fire_annunciator() {
    let temp = tempfile::tempdir().unwrap();
    let mut differences = 0;
    for path in content::aircraft_paths() {
        let off = capture(
            &path,
            true,
            &["A:Eng On Fire:1,bool=0", "A:Eng On Fire:2,bool=0"],
            &temp.path().join("off.png"),
        );
        let on = capture(
            &path,
            true,
            &["A:Eng On Fire:1,bool=1", "A:Eng On Fire:2,bool=0"],
            &temp.path().join("on.png"),
        );
        differences += off
            .pixels()
            .zip(on.pixels())
            .filter(|(a, b)| a != b)
            .count();
    }
    assert!(
        differences > 50,
        "fire input did not change a cockpit indicator"
    );
}

#[test]
#[ignore = "requires a GPU, desktop and an aircraft with an XML flight director switch"]
fn renders_flight_director_switch() {
    let temp = tempfile::tempdir().unwrap();
    let mut differences = 0;
    for path in content::aircraft_paths() {
        let off = capture(
            &path,
            true,
            &["A:Autopilot flight director active,bool=0"],
            &temp.path().join("off.png"),
        );
        let on = capture(
            &path,
            true,
            &["A:Autopilot flight director active,bool=1"],
            &temp.path().join("on.png"),
        );
        differences += off
            .pixels()
            .zip(on.pixels())
            .filter(|(a, b)| a != b)
            .count();
    }
    assert!(
        differences > 20,
        "flight director input did not change the cockpit switch"
    );
}

#[test]
#[ignore = "requires a GPU, desktop and an aircraft with an XML flap indicator"]
fn renders_flap_indicator() {
    let temp = tempfile::tempdir().unwrap();
    let mut differences = 0;
    for path in content::aircraft_paths() {
        let up = capture(
            &path,
            true,
            &[
                "A:Trailing edge flaps0 left angle,degrees=0",
                "A:Trailing edge flaps0 right angle,degrees=0",
                "A:Trailing edge flaps1 left angle,degrees=0",
                "A:Trailing edge flaps1 right angle,degrees=0",
            ],
            &temp.path().join("up.png"),
        );
        let down = capture(
            &path,
            true,
            &[
                "A:Trailing edge flaps0 left angle,degrees=40",
                "A:Trailing edge flaps0 right angle,degrees=40",
                "A:Trailing edge flaps1 left angle,degrees=40",
                "A:Trailing edge flaps1 right angle,degrees=40",
            ],
            &temp.path().join("down.png"),
        );
        differences += up
            .pixels()
            .zip(down.pixels())
            .filter(|(a, b)| a != b)
            .count();
    }
    assert!(
        differences > 20,
        "flap input did not change a cockpit needle"
    );
}

#[test]
#[ignore = "requires a GPU, desktop and original aircraft with flap animations"]
fn renders_configured_flap_motion_after_startup_command() {
    let temp = tempfile::tempdir().unwrap();
    for path in content::aircraft_paths() {
        let up = capture(&path, false, &[], &temp.path().join("up.png"));
        let moving = capture_after(
            &path,
            false,
            &[],
            Some("FLAPS_DOWN"),
            120,
            &temp.path().join("moving.png"),
        );
        let differences = up
            .pixels()
            .zip(moving.pixels())
            .filter(|(a, b)| a != b)
            .count();
        assert!(
            differences > 100,
            "flap command did not visibly move the original aircraft surfaces"
        );
    }
}

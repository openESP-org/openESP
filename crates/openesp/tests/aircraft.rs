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
    let arguments = format!("{:?}", command.get_args().collect::<Vec<_>>());
    let mut child = command
        .stdout(Stdio::null())
        .stderr(std::fs::File::create(output.with_extension("log")).unwrap())
        .spawn()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        if let Some(status) = child.try_wait().unwrap() {
            assert!(
                status.success(),
                "render failed ({status}) for {arguments}: {}",
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
#[ignore = "requires a GPU, original engine XML gauges and their installed fonts"]
fn renders_live_engine_text_readouts() {
    let temp = tempfile::tempdir().unwrap();
    let mut count = 0;
    for path in content::aircraft_paths() {
        let data = esp_assets::flight::FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(1.0)
        {
            continue;
        }
        let before = capture(
            &path,
            true,
            &[
                "A:CIRCUIT GENERAL PANEL ON,bool=1",
                "A:TURB ENG N1:1,percent=25",
            ],
            &temp.path().join("idle.png"),
        );
        let after = capture(
            &path,
            true,
            &[
                "A:CIRCUIT GENERAL PANEL ON,bool=1",
                "A:TURB ENG N1:1,percent=85",
            ],
            &temp.path().join("power.png"),
        );
        let different = before
            .pixels()
            .zip(after.pixels())
            .filter(|(a, b)| a != b)
            .count();
        assert!(
            different > 10,
            "live N1 text did not reach cockpit pixels: {different}"
        );
        count += 1;
    }
    assert!(count > 0);
}

#[test]
#[ignore = "requires a GPU, desktop and original turbine aircraft"]
fn renders_masked_attitude_horizon() {
    let temp = tempfile::tempdir().unwrap();
    let mut count = 0;
    for path in content::aircraft_paths() {
        let data = esp_assets::flight::FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(1.0)
        {
            continue;
        }
        let mut pictures = Vec::new();
        for (name, variables) in [
            (
                "level",
                [
                    "A:ATTITUDE INDICATOR PITCH DEGREES,degrees=0",
                    "A:ATTITUDE INDICATOR BANK DEGREES,radians=0",
                ],
            ),
            (
                "attitude",
                [
                    "A:ATTITUDE INDICATOR PITCH DEGREES,degrees=-20",
                    "A:ATTITUDE INDICATOR BANK DEGREES,degrees=-30",
                ],
            ),
        ] {
            pictures.push(capture_after(
                &path,
                true,
                &variables,
                Some("TOGGLE_MASTER_BATTERY"),
                1,
                &temp.path().join(format!("horizon-{name}.png")),
            ));
        }
        assert!(
            pictures[0]
                .pixels()
                .zip(pictures[1].pixels())
                .filter(|(a, b)| a != b)
                .count()
                > 100,
            "attitude horizon did not reach cockpit pixels"
        );
        count += 1;
    }
    assert!(count > 0);
}

#[test]
#[ignore = "requires a GPU, desktop and original turbine aircraft"]
fn renders_cockpit_power_from_battery_switch_events() {
    let temp = tempfile::tempdir().unwrap();
    let mut count = 0;
    for path in content::aircraft_paths() {
        let data = esp_assets::flight::FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(1.0)
        {
            continue;
        }
        let mut frames = Vec::new();
        for powered in [false, true] {
            let screenshot = temp.path().join(format!("power-{powered}.png"));
            let mut command = Command::new(env!("CARGO_BIN_EXE_openesp"));
            command
                .arg("--aircraft")
                .arg(&path)
                .args([
                    "--cockpit",
                    "--variable",
                    "A:TURB ENG N1:1,percent=70",
                    "--frames",
                    "3",
                    "--screenshot",
                ])
                .arg(&screenshot);
            if powered {
                command.args(["--event", "TOGGLE_MASTER_BATTERY"]);
            }
            frames.push(capture_command(command, &screenshot));
        }
        let difference = frames[0]
            .pixels()
            .zip(frames[1].pixels())
            .filter(|(a, b)| a != b)
            .count();
        assert!(
            difference > 100,
            "battery switch did not power cockpit artwork: {difference}"
        );
        count += 1;
    }
    assert!(count > 0);
}

#[test]
#[ignore = "requires a GPU, desktop and original turbine aircraft"]
fn renders_powered_flight_with_live_engine_state_and_fuel_use() {
    let temp = tempfile::tempdir().unwrap();
    let mut count = 0;
    for path in content::aircraft_paths() {
        let data = esp_assets::flight::FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(1.0)
        {
            continue;
        }
        let initial_fuel = esp_formats::cfg::loading::tanks(&data.configuration)
            .unwrap()
            .iter()
            .map(|t| t.capacity_gallons)
            .sum::<f64>()
            * 0.5;
        let screenshot = temp.path().join(format!("powered-{count}.png"));
        let log_path = screenshot.with_extension("log");
        let mut command = Command::new(env!("CARGO_BIN_EXE_openesp"));
        command
            .arg("--aircraft")
            .arg(path)
            .args([
                "--airspeed",
                "200",
                "--height",
                "10000",
                "--engines-running",
                "--fuel-percent",
                "50",
                "--payload",
                "--event",
                "THROTTLE_FULL",
                "--frames",
                "180",
                "--cockpit",
                "--screenshot",
            ])
            .arg(&screenshot)
            .stderr(std::fs::File::create(&log_path).unwrap());
        capture_command(command, &screenshot);
        let log = std::fs::read_to_string(log_path).unwrap();
        let remaining = log
            .split("flight fuel: ")
            .nth(1)
            .unwrap()
            .split_whitespace()
            .next()
            .unwrap()
            .parse::<f64>()
            .unwrap();
        assert!(remaining > 0.0 && remaining < initial_fuel);
        for index in [1, 2] {
            let engine = log.split(&format!("engine {index}: N1=")).nth(1).unwrap();
            let n1 = engine
                .split_whitespace()
                .next()
                .unwrap()
                .parse::<f64>()
                .unwrap();
            assert!(n1 > 0.0);
        }
        count += 1;
    }
    assert!(count > 0);
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

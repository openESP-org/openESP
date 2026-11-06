use super::*;

#[test]
#[ignore = "requires original PFD artwork and installed gauge fonts"]
fn original_attitude_horizon_tracks_flight_orientation_within_its_mask() {
    let mut verified = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        let panel = esp_assets::panel::Panel::load(&path, None)
            .unwrap()
            .unwrap();
        let layout = panel.layout().unwrap();
        let Some((window, gauge)) = layout
            .windows
            .iter()
            .filter(|w| w.texture.is_some())
            .find_map(|w| {
                w.gauges
                    .iter()
                    .find(|g| g.reference.to_ascii_lowercase().ends_with("!pfd"))
                    .map(|g| (w, g))
            })
        else {
            continue;
        };
        let mut image = esp_assets::gauge::load_panel(&panel, window.texture.unwrap())
            .unwrap()
            .unwrap();
        let mut variables = controls(&data);
        variables
            .configure_electrical(
                &esp_formats::cfg::electrical::Electrical::from_config(&data.configuration)
                    .unwrap(),
                true,
            )
            .unwrap();
        variables.set('P', "UNITS OF MEASURE", Some("enum"), Value::Number(0.0));
        let flight = Flight::new(&data, 200.0, 10000.0, 0.5, true).unwrap();
        let sx = window.pixel_size.as_ref().unwrap().width / window.size_mm.as_ref().unwrap().width;
        let sy = window.pixel_size.as_ref().unwrap().height.unwrap()
            / window.size_mm.as_ref().unwrap().height.unwrap();
        let gx = gauge.width.unwrap_or(260.0) / 260.0;
        let gy = gauge.height.unwrap_or(260.0) / 260.0;
        let point = |x: f64, y: f64| {
            (
                ((gauge.x + x * gx) * sx).round() as u32,
                ((gauge.y + y * gy) * sy).round() as u32,
            )
        };
        let sample = |image: &image::RgbaImage, x, y| {
            let (x, y) = point(x, y);
            *image.get_pixel(x, y)
        };
        let crop = |image: &image::RgbaImage| {
            let (x, y) = point(47.0, 55.0);
            let (right, bottom) = point(179.0, 194.0);
            image::imageops::crop_imm(image, x, y, right - x, bottom - y).to_image()
        };
        let mut pictures = Vec::new();
        for (name, pitch, bank) in [
            ("level", 0.0_f64, 0.0_f64),
            ("pitch", 20.0, 0.0),
            ("bank", 0.0, 30.0),
        ] {
            let state = State {
                orientation: DQuat::from_euler(
                    EulerRot::ZYX,
                    0.0,
                    pitch.to_radians(),
                    bank.to_radians(),
                ),
                ..flight.current
            };
            Flight::publish(&state, &mut variables).unwrap();
            assert!(image.update(&mut variables));
            if let Some(directory) = std::env::var_os("OPENESP_TEST_CAPTURE") {
                let directory = std::path::PathBuf::from(directory);
                std::fs::create_dir_all(&directory).unwrap();
                let (x, y) = point(0.0, 0.0);
                let (right, bottom) = point(260.0, 260.0);
                image::imageops::crop_imm(&image.image, x, y, right - x, bottom - y)
                    .to_image()
                    .save(directory.join(format!("pfd-{name}.png")))
                    .unwrap();
            }
            pictures.push(crop(&image.image));
            if name == "level" {
                let sky = sample(&image.image, 80.0, 90.0);
                let ground = sample(&image.image, 80.0, 150.0);
                assert!(
                    sky[2] > sky[0] + 20,
                    "sky is outside its expected aperture: {sky:?}"
                );
                assert!(
                    ground[0] > ground[2] + 20,
                    "ground is outside its expected aperture: {ground:?}"
                );
            }
            if name == "pitch" {
                let sky = sample(&image.image, 80.0, 150.0);
                assert!(
                    sky[2] > sky[0] + 20,
                    "nose-up pitch did not move the horizon down: {sky:?}"
                );
            }
            assert!(!image.update(&mut variables));
        }
        for picture in &pictures[1..] {
            assert!(
                pictures[0]
                    .pixels()
                    .zip(picture.pixels())
                    .filter(|(a, b)| a != b)
                    .count()
                    > 1000,
                "flight attitude did not move original horizon artwork"
            );
        }
        verified += 1;
    }
    assert!(verified > 0);
}

#[test]
#[ignore = "requires original EICAS XML artwork and its installed Arial font"]
fn original_engine_text_readouts_follow_air_table_engine_state() {
    let mut count = 0;
    for path in content::aircraft_paths() {
        let data = FlightData::load(&path, 0).unwrap();
        if data
            .configuration
            .number("GeneralEngineData", "engine_type")
            .unwrap()
            != Some(1.0)
        {
            continue;
        }
        let panel = esp_assets::panel::Panel::load(&path, None)
            .unwrap()
            .unwrap();
        let layout = panel.layout().unwrap();
        let Some((window, gauge)) = layout
            .windows
            .iter()
            .filter(|w| w.texture.is_some())
            .find_map(|w| {
                w.gauges
                    .iter()
                    .find(|g| g.reference.to_ascii_lowercase().ends_with("!eicas"))
                    .map(|g| (w, g))
            })
        else {
            continue;
        };
        if window.texture.is_none() {
            continue;
        }
        let mut image = esp_assets::gauge::load_panel(&panel, window.texture.unwrap())
            .unwrap()
            .unwrap();
        let mut flight = Flight::new(&data, 200.0, 10000.0, 0.5, true).unwrap();
        flight.start_running(&data).unwrap();
        let mut variables = controls(&data);
        flight.configure_engines(&mut variables).unwrap();
        Flight::publish(&flight.current, &mut variables).unwrap();
        flight.publish_loading(&mut variables);
        variables
            .configure_electrical(
                &esp_formats::cfg::electrical::Electrical::from_config(&data.configuration)
                    .unwrap(),
                true,
            )
            .unwrap();
        variables.set('P', "UNITS OF MEASURE", Some("enum"), Value::Number(0.0));
        image.update(&mut variables);
        let scale_x =
            window.pixel_size.as_ref().unwrap().width / window.size_mm.as_ref().unwrap().width;
        let scale_y = window.pixel_size.as_ref().unwrap().height.unwrap()
            / window.size_mm.as_ref().unwrap().height.unwrap();
        let gauge_x = gauge.width.unwrap_or(260.0) / 260.0;
        let gauge_y = gauge.height.unwrap_or(260.0) / 260.0;
        let rectangle = (
            ((gauge.x + 44.0 * gauge_x) * scale_x).round() as u32,
            ((gauge.y + 42.0 * gauge_y) * scale_y).round() as u32,
            (42.0 * gauge_x * scale_x).round() as u32,
            (16.0 * gauge_y * scale_y).round() as u32,
        );
        let crop = |image: &image::RgbaImage| {
            image::imageops::crop_imm(image, rectangle.0, rectangle.1, rectangle.2, rectangle.3)
                .to_image()
        };
        let before = crop(&image.image);
        assert!(
            before
                .pixels()
                .filter(|p| p.0[0] > 160 && p.0[1] > 160 && p.0[2] > 160)
                .count()
                > 5,
            "N1 text was not drawn"
        );
        variables
            .dispatch_event("THROTTLE_FULL", Value::Number(0.0))
            .unwrap();
        for _ in 0..600 {
            flight.advance(1.0 / 60.0, &mut variables).unwrap();
        }
        image.update(&mut variables);
        let after = crop(&image.image);
        assert_ne!(
            before, after,
            "AIR-derived N1 did not change original readout pixels"
        );
        assert!(!image.update(&mut variables));
        for event in [
            "TOGGLE_MASTER_BATTERY",
            "TOGGLE_ALTERNATOR1",
            "TOGGLE_ALTERNATOR2",
        ] {
            variables.dispatch_event(event, Value::Number(0.0)).unwrap();
        }
        variables.update_electrical().unwrap();
        assert!(image.update(&mut variables));
        let off = crop(&image.image);
        assert!(
            !off.pixels()
                .any(|p| p.0[0] > 160 && p.0[1] > 160 && p.0[2] > 160),
            "unpowered N1 readout remained lit"
        );
        variables
            .dispatch_event("TOGGLE_MASTER_BATTERY", Value::Number(0.0))
            .unwrap();
        variables.update_electrical().unwrap();
        assert!(image.update(&mut variables));
        assert_eq!(
            crop(&image.image),
            after,
            "battery power did not restore the readout"
        );
        count += 1;
    }
    assert!(count > 0);
}

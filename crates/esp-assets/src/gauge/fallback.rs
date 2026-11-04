use esp_script::{Host, Reference};
use image::{Rgba, RgbaImage};
use imageproc::{
    drawing::{
        draw_filled_circle_mut, draw_filled_rect_mut, draw_hollow_circle_mut, draw_line_segment_mut,
    },
    rect::Rect,
};
use std::f64::consts::PI;

#[derive(Clone, Copy)]
enum Kind {
    Airspeed,
    Altimeter,
    Heading,
    Compass,
    NavGpsSwitch,
    NavGpsAnnunciator,
    VerticalSpeed,
    Attitude,
    Tachometer,
    Fuel,
    FuelFlow,
    Oil,
    Flaps,
    Clock,
    TurnCoordinator,
    Vor1,
    Vor2,
    Radio1,
    Radio2,
    Dme,
}

pub(super) struct Instrument {
    kind: Kind,
    pub image: RgbaImage,
    last: Option<[f64; 5]>,
}

impl Instrument {
    pub fn new(reference: &str) -> Option<Self> {
        let (module, gauge) = reference.split_once('!')?;
        let module = module.to_ascii_uppercase();
        let gauge = gauge.to_ascii_uppercase();
        let kind = match (module.as_str(), gauge.as_str()) {
            ("CESSNA172", "AIRSPEED") => Kind::Airspeed,
            ("CESSNA", "ALTIMETER") => Kind::Altimeter,
            ("CESSNA", "NAV GPS SWITCH") => Kind::NavGpsSwitch,
            ("CESSNA", "NAV_GPS_ANNUNCIATOR") => Kind::NavGpsAnnunciator,
            ("CESSNA", "HEADING_INDICATOR") => Kind::Heading,
            ("CESSNAWALPHA", "WHISKEY_COMPASS_ALPHA")
            | ("CESSNA", "WHISKEY_COMPASS")
            | ("CESSNA172", "WHISKEY_COMPASS") => Kind::Compass,
            ("CESSNA", "VERTICAL_SPEED") => Kind::VerticalSpeed,
            ("BELL_206B", "ATTITUDE") | ("CESSNAWALPHA", "ATTITUDE_ALPHA") => Kind::Attitude,
            ("CESSNA172", "TACHOMETER") => Kind::Tachometer,
            ("CESSNA172", "FUEL") => Kind::Fuel,
            ("CESSNA172", "EGT_FUEL_FLOW") => Kind::FuelFlow,
            ("CESSNA182S", "OIL_PRESS_TEMP") => Kind::Oil,
            ("CESSNA", "FLAPS") => Kind::Flaps,
            ("CESSNA172", "CLOCK") => Kind::Clock,
            ("CESSNA", "TURN_INDICATOR") => Kind::TurnCoordinator,
            ("CESSNA", "VOR1") | ("CESSNAWALPHA", "VOR1_ALPHA") => Kind::Vor1,
            ("CESSNA182S", "VOR2") | ("CESSNAWALPHA", "VOR2_ALPHA") => Kind::Vor2,
            ("BENDIX_KING_RADIO", "BENDIX-KING RADIO NAV-COMM 1") => Kind::Radio1,
            ("BENDIX_KING_RADIO", "BENDIX-KING RADIO NAV-COMM 2") => Kind::Radio2,
            ("BENDIX_KING_RADIO", "BENDIX-KING RADIO DME") => Kind::Dme,
            _ => return None,
        };
        Some(Self {
            kind,
            image: if matches!(
                kind,
                Kind::Radio1 | Kind::Radio2 | Kind::Compass | Kind::Dme
            ) {
                RgbaImage::new(256, 96)
            } else {
                RgbaImage::new(128, 128)
            },
            last: None,
        })
    }

    pub fn obs_index(&self) -> Option<u32> {
        match self.kind {
            Kind::Vor1 => Some(1),
            Kind::Vor2 => Some(2),
            _ => None,
        }
    }

    pub fn radio_index(&self) -> Option<u32> {
        match self.kind {
            Kind::Radio1 => Some(1),
            Kind::Radio2 => Some(2),
            _ => None,
        }
    }

    pub fn size(&self) -> (u32, u32) {
        (self.image.width(), self.image.height())
    }

    pub fn is_nav_gps_switch(&self) -> bool {
        matches!(self.kind, Kind::NavGpsSwitch)
    }

    pub fn is_heading_indicator(&self) -> bool {
        matches!(self.kind, Kind::Heading)
    }

    pub fn is_altimeter(&self) -> bool {
        matches!(self.kind, Kind::Altimeter)
    }

    pub fn is_flaps(&self) -> bool {
        matches!(self.kind, Kind::Flaps)
    }

    pub fn update(&mut self, host: &mut impl Host) -> bool {
        fn read(host: &mut impl Host, name: &str, unit: &str) -> f64 {
            host.read(&Reference::new('A', name, Some(unit)))
                .ok()
                .and_then(|v| v.number().ok())
                .filter(|v| v.is_finite())
                .unwrap_or(f64::MAX)
        }
        let values = match self.kind {
            Kind::Airspeed => [
                read(host, "AIRSPEED INDICATED", "knots"),
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            Kind::Altimeter => [
                read(host, "INDICATED ALTITUDE", "feet"),
                read(host, "KOHLSMAN SETTING HG:1", "inhg"),
                0.0,
                0.0,
                0.0,
            ],
            Kind::Heading => [
                read(host, "HEADING INDICATOR", "degrees"),
                read(host, "AUTOPILOT HEADING LOCK DIR", "degrees"),
                0.0,
                0.0,
                0.0,
            ],
            Kind::Compass => [
                read(host, "MAGNETIC COMPASS", "degrees"),
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            Kind::NavGpsSwitch | Kind::NavGpsAnnunciator => {
                [read(host, "GPS DRIVES NAV1", "bool"), 0.0, 0.0, 0.0, 0.0]
            }
            Kind::VerticalSpeed => [
                read(host, "VERTICAL SPEED", "feet per minute"),
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            Kind::Attitude => [
                read(host, "ATTITUDE INDICATOR PITCH DEGREES", "degrees"),
                read(host, "ATTITUDE INDICATOR BANK DEGREES", "degrees"),
                0.0,
                0.0,
                0.0,
            ],
            Kind::Tachometer => [read(host, "GENERAL ENG RPM:1", "rpm"), 0.0, 0.0, 0.0, 0.0],
            Kind::Fuel => [
                read(host, "FUEL TANK LEFT MAIN QUANTITY", "gallons"),
                read(host, "FUEL TANK RIGHT MAIN QUANTITY", "gallons"),
                read(host, "FUEL TANK LEFT MAIN CAPACITY", "gallons"),
                read(host, "FUEL TANK RIGHT MAIN CAPACITY", "gallons"),
                0.0,
            ],
            Kind::FuelFlow => [
                read(host, "ENG FUEL FLOW PPH:1", "pounds per hour"),
                read(host, "FUEL WEIGHT PER GALLON", "pounds"),
                read(host, "ENG EXHAUST GAS TEMPERATURE:1", "fahrenheit"),
                0.0,
                0.0,
            ],
            Kind::Oil => [
                read(host, "GENERAL ENG OIL PRESSURE:1", "psi"),
                read(host, "GENERAL ENG OIL TEMPERATURE:1", "celsius"),
                0.0,
                0.0,
                0.0,
            ],
            Kind::Flaps => [
                read(host, "TRAILING EDGE FLAPS LEFT ANGLE", "degrees"),
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            Kind::Clock => [
                host.read(&Reference::new('E', "ZULU TIME", Some("seconds")))
                    .ok()
                    .and_then(|value| value.number().ok())
                    .filter(|value| value.is_finite())
                    .map(f64::floor)
                    .unwrap_or(f64::MAX),
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            Kind::TurnCoordinator => [
                read(host, "TURN COORDINATOR RATE", "degrees per second"),
                read(host, "TURN COORDINATOR BALL", "position"),
                0.0,
                0.0,
                0.0,
            ],
            Kind::Radio1 | Kind::Radio2 => {
                let index = if matches!(self.kind, Kind::Radio1) {
                    1
                } else {
                    2
                };
                [
                    read(host, &format!("NAV ACTIVE FREQUENCY:{index}"), "hertz"),
                    read(host, &format!("NAV STANDBY FREQUENCY:{index}"), "hertz"),
                    0.0,
                    0.0,
                    0.0,
                ]
            }
            Kind::Dme => [
                read(host, "NAV HAS DME:1", "bool"),
                read(host, "NAV DME:1", "nautical miles"),
                read(host, "NAV HAS DME:2", "bool"),
                read(host, "NAV DME:2", "nautical miles"),
                0.0,
            ],
            Kind::Vor1 | Kind::Vor2 => {
                let index = if matches!(self.kind, Kind::Vor1) {
                    1
                } else {
                    2
                };
                let gps = index == 1 && read(host, "GPS DRIVES NAV1", "bool") == 1.0;
                let course = if gps {
                    read(host, "GPS WP DESIRED TRACK", "degrees")
                } else {
                    read(host, &format!("NAV OBS:{index}"), "degrees")
                };
                let valid = if index == 1 {
                    read(host, "HSI CDI NEEDLE VALID", "bool")
                } else {
                    read(host, "NAV HAS NAV:2", "bool")
                };
                let (needle, flag, glide, glide_valid) = if valid == 1.0 {
                    if index == 1 {
                        let gv = read(host, "HSI GSI NEEDLE VALID", "bool") == 1.0;
                        (
                            read(host, "HSI CDI NEEDLE", "number"),
                            read(host, "HSI TF FLAGS", "enum"),
                            if gv {
                                read(host, "HSI GSI NEEDLE", "number") * 127.0 / 119.0
                            } else {
                                0.0
                            },
                            gv,
                        )
                    } else {
                        let gv = read(host, "NAV HAS GLIDE SLOPE:2", "bool") == 1.0;
                        (
                            read(host, "NAV CDI:2", "number"),
                            read(host, "NAV TOFROM:2", "enum"),
                            if gv {
                                read(host, "NAV GSI:2", "number")
                            } else {
                                0.0
                            },
                            gv,
                        )
                    }
                } else {
                    (0.0, 0.0, 0.0, false)
                };
                [course, needle, flag, glide, f64::from(glide_valid)]
            }
        };
        let values = std::array::from_fn(|index| {
            let v = values[index];
            if v == f64::MAX {
                v
            } else {
                let scale = if matches!(self.kind, Kind::Altimeter) && index == 1 {
                    100.0
                } else {
                    10.0
                };
                (v.clamp(-1e12, 1e12) * scale).round() / scale
            }
        });
        if self.last == Some(values) {
            return false;
        }
        self.last = Some(values);
        self.image = draw(self.kind, values);
        true
    }
}

fn line(image: &mut RgbaImage, a: (f32, f32), b: (f32, f32), color: Rgba<u8>) {
    draw_line_segment_mut(image, a, b, color);
}

fn point(angle: f64, radius: f64) -> (f32, f32) {
    (
        (64.0 + angle.sin() * radius) as f32,
        (64.0 - angle.cos() * radius) as f32,
    )
}

fn label(image: &mut RgbaImage, x: i32, y: i32, text: &str, color: Rgba<u8>) {
    label_scaled(image, x, y, text, color, 1);
}

fn label_scaled(image: &mut RgbaImage, x: i32, y: i32, text: &str, color: Rgba<u8>, scale: i32) {
    for (i, c) in text.chars().enumerate() {
        let rows: [u8; 5] = match c {
            '0' => [7, 5, 5, 5, 7],
            '1' => [2, 6, 2, 2, 7],
            '2' => [7, 1, 7, 4, 7],
            '3' => [7, 1, 7, 1, 7],
            '4' => [5, 5, 7, 1, 1],
            '5' => [7, 4, 7, 1, 7],
            '6' => [7, 4, 7, 5, 7],
            '7' => [7, 1, 1, 1, 1],
            '8' => [7, 5, 7, 5, 7],
            '9' => [7, 5, 7, 1, 7],
            '-' => [0, 0, 7, 0, 0],
            '+' => [0, 2, 7, 2, 0],
            '.' => [0, 0, 0, 0, 2],
            ':' => [0, 2, 0, 2, 0],
            'A' => [2, 5, 7, 5, 5],
            'B' => [6, 5, 6, 5, 6],
            'C' => [7, 4, 4, 4, 7],
            'D' => [6, 5, 5, 5, 6],
            'E' => [7, 4, 6, 4, 7],
            'F' => [7, 4, 6, 4, 4],
            'G' => [7, 4, 5, 5, 7],
            'H' => [5, 5, 7, 5, 5],
            'I' => [7, 2, 2, 2, 7],
            'K' => [5, 5, 6, 5, 5],
            'L' => [4, 4, 4, 4, 7],
            'M' => [5, 7, 7, 5, 5],
            'N' => [5, 7, 7, 7, 5],
            'O' => [7, 5, 5, 5, 7],
            'P' => [6, 5, 6, 4, 4],
            'R' => [6, 5, 6, 5, 5],
            'S' => [7, 4, 7, 1, 7],
            'T' => [7, 2, 2, 2, 2],
            'U' => [5, 5, 5, 5, 7],
            'V' => [5, 5, 5, 5, 2],
            'W' => [5, 5, 7, 7, 5],
            'Y' => [5, 5, 2, 2, 2],
            _ => [0; 5],
        };
        for (dy, row) in rows.iter().enumerate() {
            for dx in 0..3 {
                if row & (1 << (2 - dx)) != 0 {
                    let px = x + (i as i32 * 4 + dx) * scale;
                    let py = y + dy as i32 * scale;
                    for sy in 0..scale {
                        for sx in 0..scale {
                            let (px, py) = (px + sx, py + sy);
                            if px >= 0
                                && py >= 0
                                && (px as u32) < image.width()
                                && (py as u32) < image.height()
                            {
                                image.put_pixel(px as u32, py as u32, color);
                            }
                        }
                    }
                }
            }
        }
    }
}

fn draw(kind: Kind, values: [f64; 5]) -> RgbaImage {
    if matches!(kind, Kind::NavGpsSwitch | Kind::NavGpsAnnunciator) {
        let mut image = RgbaImage::from_pixel(128, 128, Rgba([25, 25, 23, 255]));
        let dim = Rgba([105, 105, 100, 255]);
        let lit = Rgba([110, 235, 120, 255]);
        let white = Rgba([235, 235, 225, 255]);
        if values[0] == f64::MAX {
            label_scaled(&mut image, 40, 54, "OFF", Rgba([235, 60, 45, 255]), 4);
        } else if matches!(kind, Kind::NavGpsAnnunciator) {
            label_scaled(
                &mut image,
                8,
                45,
                "NAV",
                if values[0] == 0.0 { lit } else { dim },
                3,
            );
            label_scaled(
                &mut image,
                74,
                45,
                "GPS",
                if values[0] != 0.0 { lit } else { dim },
                3,
            );
        } else {
            label_scaled(&mut image, 8, 12, "NAV", white, 2);
            label_scaled(&mut image, 92, 12, "GPS", white, 2);
            draw_filled_rect_mut(&mut image, Rect::at(15, 51).of_size(98, 38), dim);
            let x = if values[0] == 0.0 { 20 } else { 76 };
            draw_filled_rect_mut(&mut image, Rect::at(x, 44).of_size(32, 52), white);
        }
        return image;
    }
    if matches!(kind, Kind::Flaps) {
        let mut image = RgbaImage::from_pixel(128, 128, Rgba([31, 31, 30, 255]));
        draw_filled_rect_mut(
            &mut image,
            Rect::at(14, 8).of_size(100, 112),
            Rgba([8, 9, 10, 255]),
        );
        let white = Rgba([235, 235, 230, 255]);
        let yellow = Rgba([255, 200, 50, 255]);
        label_scaled(&mut image, 44, 12, "FLAPS", white, 2);
        for angle in (0..=40).step_by(10) {
            let y = 35.0 + angle as f32 * 1.8;
            line(&mut image, (37.0, y), (47.0, y), white);
            label(&mut image, 22, y as i32 - 2, &angle.to_string(), white);
        }
        if values[0] == f64::MAX {
            label_scaled(&mut image, 60, 60, "OFF", Rgba([235, 60, 45, 255]), 2);
        } else {
            let y = 35.0 + values[0].clamp(0.0, 40.0) as f32 * 1.8;
            line(&mut image, (48.0, y), (85.0, y), yellow);
            line(&mut image, (48.0, y + 1.0), (85.0, y + 1.0), yellow);
        }
        return image;
    }
    if matches!(kind, Kind::Compass) {
        let mut image = RgbaImage::from_pixel(256, 96, Rgba([40, 40, 38, 255]));
        draw_filled_rect_mut(
            &mut image,
            Rect::at(5, 5).of_size(246, 86),
            Rgba([10, 12, 12, 255]),
        );
        let white = Rgba([235, 235, 225, 255]);
        let yellow = Rgba([255, 200, 50, 255]);
        if values[0] == f64::MAX {
            label_scaled(&mut image, 110, 42, "OFF", Rgba([235, 60, 45, 255]), 2);
        } else {
            let heading = values[0].rem_euclid(360.0);
            for bearing in (0..360).step_by(10) {
                let delta = (bearing as f64 - heading + 180.0).rem_euclid(360.0) - 180.0;
                let x = (128.0 + delta * 2.0).round() as i32;
                if !(18..=238).contains(&x) {
                    continue;
                }
                let major = bearing % 30 == 0;
                line(
                    &mut image,
                    (x as f32, 25.0),
                    (x as f32, if major { 43.0 } else { 35.0 }),
                    white,
                );
                if major {
                    let text = match bearing {
                        0 => "N".to_owned(),
                        90 => "E".to_owned(),
                        180 => "S".to_owned(),
                        270 => "W".to_owned(),
                        _ => (bearing / 10).to_string(),
                    };
                    label_scaled(&mut image, x - text.len() as i32 * 4, 50, &text, white, 2);
                }
            }
            line(&mut image, (128.0, 9.0), (128.0, 71.0), yellow);
        }
        return image;
    }
    if matches!(kind, Kind::Dme) {
        let mut image = RgbaImage::from_pixel(256, 96, Rgba([48, 48, 47, 255]));
        draw_filled_rect_mut(
            &mut image,
            Rect::at(4, 4).of_size(248, 88),
            Rgba([18, 18, 17, 255]),
        );
        let dim = Rgba([170, 170, 160, 255]);
        let display = Rgba([135, 230, 145, 255]);
        label_scaled(&mut image, 10, 10, "DME", dim, 2);
        label_scaled(&mut image, 188, 10, "NM", dim, 2);
        for (x, name, signal, distance) in [
            (10, "NAV 1", values[0], values[1]),
            (130, "NAV 2", values[2], values[3]),
        ] {
            label_scaled(&mut image, x, 31, name, dim, 2);
            let text = if signal == 1.0 && distance != f64::MAX && distance >= 0.0 {
                format!("{:.1}", distance.min(999.9))
            } else {
                "---".to_owned()
            };
            label_scaled(&mut image, x, 58, &text, display, 3);
        }
        return image;
    }
    if matches!(
        kind,
        Kind::Radio1 | Kind::Radio2 | Kind::Compass | Kind::Dme
    ) {
        let mut image = RgbaImage::from_pixel(256, 96, Rgba([48, 48, 47, 255]));
        draw_filled_rect_mut(
            &mut image,
            Rect::at(4, 4).of_size(248, 88),
            Rgba([18, 18, 17, 255]),
        );
        let dim = Rgba([170, 170, 160, 255]);
        let display = Rgba([135, 230, 145, 255]);
        label_scaled(
            &mut image,
            10,
            8,
            if matches!(kind, Kind::Radio1) {
                "NAV 1"
            } else {
                "NAV 2"
            },
            dim,
            2,
        );
        label_scaled(&mut image, 10, 25, "ACT", dim, 2);
        label_scaled(&mut image, 104, 25, "STBY", dim, 2);
        for (x, frequency) in [(10, values[0]), (104, values[1])] {
            let text = if frequency == f64::MAX {
                "OFF".to_owned()
            } else {
                format!("{:.3}", frequency / 1_000_000.0)
            };
            label_scaled(&mut image, x, 47, &text, display, 3);
        }
        label_scaled(&mut image, 191, 38, "SW", dim, 2);
        label_scaled(&mut image, 211, 12, "- +", dim, 2);
        label_scaled(&mut image, 211, 60, "- +", dim, 2);
        return image;
    }
    let mut image = RgbaImage::new(128, 128);
    let white = Rgba([235, 235, 230, 255]);
    let yellow = Rgba([255, 200, 50, 255]);
    draw_filled_circle_mut(&mut image, (64, 64), 62, Rgba([32, 32, 32, 255]));
    draw_filled_circle_mut(&mut image, (64, 64), 57, Rgba([8, 9, 10, 255]));
    if matches!(kind, Kind::Fuel | Kind::FuelFlow | Kind::Oil) {
        let red = Rgba([235, 60, 45, 255]);
        if matches!(kind, Kind::Fuel) {
            label(&mut image, 56, 20, "FUEL", white);
            for (side, x) in [(0, 37_i32), (1, 91_i32)] {
                label(
                    &mut image,
                    x - 2,
                    34,
                    if side == 0 { "L" } else { "R" },
                    white,
                );
                let (quantity, capacity) = (values[side], values[side + 2]);
                if quantity == f64::MAX || capacity == f64::MAX || capacity <= 0.0 {
                    label(&mut image, x - 6, 62, "OFF", red);
                } else {
                    let fraction = (quantity / capacity).clamp(0.0, 1.0);
                    line(
                        &mut image,
                        ((x - 18) as f32, 82.0),
                        ((x + 18) as f32, 82.0),
                        white,
                    );
                    label(&mut image, x - 20, 87, "E", white);
                    label(&mut image, x + 17, 87, "F", white);
                    let needle = x as f32 - 18.0 + 36.0 * fraction as f32;
                    line(
                        &mut image,
                        (needle, 70.0),
                        (needle, 84.0),
                        if fraction < 0.1 { red } else { yellow },
                    );
                    let amount = format!("{quantity:.1}");
                    label(&mut image, x - amount.len() as i32 * 2, 53, &amount, white);
                }
            }
        } else if matches!(kind, Kind::Oil) {
            label(&mut image, 58, 20, "OIL", white);
            for (value, max, title, x) in [
                (values[0], 100.0, "PSI", 37_i32),
                (values[1], 120.0, "C", 91_i32),
            ] {
                label(&mut image, x - title.len() as i32 * 2, 34, title, white);
                if value == f64::MAX {
                    label(&mut image, x - 6, 62, "OFF", red);
                } else {
                    let fraction = (value / max).clamp(0.0, 1.0);
                    line(
                        &mut image,
                        ((x - 18) as f32, 82.0),
                        ((x + 18) as f32, 82.0),
                        white,
                    );
                    let needle = x as f32 - 18.0 + 36.0 * fraction as f32;
                    line(&mut image, (needle, 70.0), (needle, 84.0), yellow);
                    let text = format!("{value:.0}");
                    label(&mut image, x - text.len() as i32 * 2, 53, &text, white);
                }
            }
        } else {
            label(&mut image, 56, 25, "EGT", white);
            let egt = if values[2] == f64::MAX {
                "OFF".to_owned()
            } else {
                format!("{:.0}F", values[2])
            };
            label(
                &mut image,
                64 - egt.len() as i32 * 2,
                39,
                &egt,
                if egt == "OFF" { red } else { white },
            );
            label(&mut image, 56, 65, "GPH", white);
            let text = if values[0] == f64::MAX || values[1] == f64::MAX || values[1] <= 0.0 {
                "OFF".to_owned()
            } else {
                format!("{:.1}", values[0] / values[1])
            };
            label(
                &mut image,
                64 - text.len() as i32 * 2,
                79,
                &text,
                if text == "OFF" { red } else { white },
            );
        }
        draw_hollow_circle_mut(&mut image, (64, 64), 56, white);
        return image;
    }
    if values[0] == f64::MAX
        || (!matches!(kind, Kind::Altimeter | Kind::Heading) && values.contains(&f64::MAX))
    {
        let red = Rgba([235, 60, 45, 255]);
        line(&mut image, (36.0, 36.0), (92.0, 92.0), red);
        line(&mut image, (92.0, 36.0), (36.0, 92.0), red);
        label(&mut image, 58, 93, "OFF", red);
        draw_hollow_circle_mut(&mut image, (64, 64), 56, white);
        return image;
    }
    if matches!(kind, Kind::Attitude) {
        let pitch = values[0].clamp(-30.0, 30.0) * 1.2;
        let bank = values[1].to_radians();
        for y in 10..118 {
            for x in 10..118 {
                let dx = x as f64 - 64.0;
                let dy = y as f64 - 64.0;
                if dx * dx + dy * dy <= 52.0 * 52.0 {
                    let horizon = dx * bank.sin() + dy * bank.cos() - pitch;
                    image.put_pixel(
                        x,
                        y,
                        if horizon < 0.0 {
                            Rgba([65, 135, 190, 255])
                        } else {
                            Rgba([125, 75, 40, 255])
                        },
                    );
                }
            }
        }
        line(&mut image, (32.0, 64.0), (55.0, 64.0), yellow);
        line(&mut image, (73.0, 64.0), (96.0, 64.0), yellow);
        draw_hollow_circle_mut(&mut image, (64, 64), 53, white);
        return image;
    }
    if matches!(kind, Kind::TurnCoordinator) {
        for angle in [-30_f64, 0.0, 30.0] {
            line(
                &mut image,
                point(angle.to_radians(), 46.0),
                point(angle.to_radians(), 53.0),
                white,
            );
        }
        label(&mut image, 26, 57, "L", white);
        label(&mut image, 99, 57, "R", white);
        label(&mut image, 48, 83, "2 MIN", white);
        let bank = (values[0] / 3.0 * 30.0).clamp(-55.0, 55.0).to_radians();
        let wing = (bank.cos() * 31.0, bank.sin() * 31.0);
        line(
            &mut image,
            ((64.0 - wing.0) as f32, (64.0 - wing.1) as f32),
            ((64.0 + wing.0) as f32, (64.0 + wing.1) as f32),
            white,
        );
        draw_filled_circle_mut(&mut image, (64, 64), 4, yellow);
        line(&mut image, (42.0, 101.0), (86.0, 101.0), white);
        line(&mut image, (42.0, 109.0), (86.0, 109.0), white);
        for x in [58.0, 70.0] {
            line(&mut image, (x, 100.0), (x, 110.0), white);
        }
        let ball_x = 64 + (values[1].clamp(-1.0, 1.0) * 18.0).round() as i32;
        draw_filled_circle_mut(&mut image, (ball_x, 105), 3, yellow);
        draw_hollow_circle_mut(&mut image, (64, 64), 56, white);
        return image;
    }
    if matches!(kind, Kind::Clock) {
        for i in 0..12 {
            let angle = i as f64 * PI / 6.0;
            line(
                &mut image,
                point(angle, 46.0),
                point(angle, if i % 3 == 0 { 39.0 } else { 43.0 }),
                white,
            );
            if i % 3 == 0 {
                let text = if i == 0 {
                    "12".to_owned()
                } else {
                    i.to_string()
                };
                let (x, y) = point(angle, 32.0);
                label(
                    &mut image,
                    x as i32 - text.len() as i32 * 2,
                    y as i32 - 2,
                    &text,
                    white,
                );
            }
        }
        let seconds = values[0].rem_euclid(86_400.0);
        let hour = seconds / 3600.0;
        let minute = seconds / 60.0;
        line(
            &mut image,
            (64.0, 64.0),
            point(hour * PI / 6.0, 24.0),
            white,
        );
        line(
            &mut image,
            (64.0, 64.0),
            point(minute * PI / 30.0, 37.0),
            white,
        );
        line(
            &mut image,
            (64.0, 64.0),
            point(seconds * PI / 30.0, 43.0),
            yellow,
        );
        draw_filled_circle_mut(&mut image, (64, 64), 3, yellow);
        label(
            &mut image,
            54,
            87,
            &format!(
                "{:02}:{:02}",
                (seconds / 3600.0) as u32,
                (seconds / 60.0) as u32 % 60
            ),
            white,
        );
        label(&mut image, 58, 96, "UTC", white);
        draw_hollow_circle_mut(&mut image, (64, 64), 56, white);
        return image;
    }
    if matches!(kind, Kind::Vor1 | Kind::Vor2) {
        for i in 0..36 {
            let angle = (i as f64 * 10.0 - values[0]).to_radians();
            line(
                &mut image,
                point(angle, 48.0),
                point(angle, if i % 3 == 0 { 41.0 } else { 45.0 }),
                white,
            );
            if i % 3 == 0 {
                let text = format!("{}", i);
                let (x, y) = point(angle, 34.0);
                label(
                    &mut image,
                    x as i32 - text.len() as i32 * 2,
                    y as i32 - 2,
                    &text,
                    white,
                );
            }
        }
        line(&mut image, (64.0, 9.0), (64.0, 19.0), yellow);
        for x in [44.0, 54.0, 74.0, 84.0] {
            draw_hollow_circle_mut(&mut image, (x as i32, 64), 2, white);
        }
        if values[2] == 0.0 {
            label(&mut image, 58, 55, "OFF", Rgba([235, 60, 45, 255]));
        } else {
            let x = 64.0 + (values[1].clamp(-127.0, 127.0) / 127.0 * 20.0) as f32;
            line(&mut image, (x, 39.0), (x, 89.0), white);
            label(
                &mut image,
                58,
                91,
                if values[2] == 1.0 { "TO" } else { "FROM" },
                white,
            );
        }
        if values[4] == 1.0 {
            let y = 64.0 - (values[3].clamp(-127.0, 127.0) / 127.0 * 20.0) as f32;
            line(&mut image, (92.0, y), (108.0, y), yellow);
        }
        draw_hollow_circle_mut(&mut image, (64, 64), 56, white);
        return image;
    }
    let (title, min, max, ticks) = match kind {
        Kind::Airspeed => ("KTS", 0.0, 200.0, 10),
        Kind::Altimeter => ("FT", 0.0, 1000.0, 10),
        Kind::Heading => ("HDG", 0.0, 360.0, 12),
        Kind::VerticalSpeed => ("FPM", -2000.0, 2000.0, 8),
        Kind::Tachometer => ("RPM", 0.0, 3000.0, 6),
        Kind::Attitude
        | Kind::NavGpsSwitch
        | Kind::NavGpsAnnunciator
        | Kind::Compass
        | Kind::TurnCoordinator
        | Kind::Fuel
        | Kind::FuelFlow
        | Kind::Oil
        | Kind::Flaps
        | Kind::Clock
        | Kind::Vor1
        | Kind::Vor2
        | Kind::Radio1
        | Kind::Radio2
        | Kind::Dme => unreachable!(),
    };
    let heading = matches!(kind, Kind::Heading);
    for i in 0..=ticks {
        let fraction = i as f64 / ticks as f64;
        let angle = if heading {
            fraction * 2.0 * PI - values[0].to_radians()
        } else {
            (-135.0 + 270.0 * fraction).to_radians()
        };
        line(&mut image, point(angle, 45.0), point(angle, 52.0), white);
        let value = min + (max - min) * fraction;
        let number = if heading {
            (value / 10.0).round() as i32 % 36
        } else if matches!(kind, Kind::Altimeter) {
            (value / 100.0).round() as i32 % 10
        } else if matches!(kind, Kind::Tachometer) {
            (value / 100.0).round() as i32
        } else if matches!(kind, Kind::VerticalSpeed) {
            (value / 1000.0).round() as i32
        } else {
            value.round() as i32
        };
        let text = number.to_string();
        let (x, y) = point(angle, 36.0);
        label(
            &mut image,
            x as i32 - text.len() as i32 * 2,
            y as i32 - 2,
            &text,
            white,
        );
    }
    if matches!(kind, Kind::Altimeter) {
        let altitude = format!("{:.0}", values[0]);
        label(
            &mut image,
            64 - altitude.len() as i32 * 2,
            74,
            &altitude,
            white,
        );
        let pressure = if values[1] == f64::MAX {
            "OFF".to_owned()
        } else {
            format!("{:.2}", values[1])
        };
        label(
            &mut image,
            64 - pressure.len() as i32 * 2,
            98,
            &pressure,
            white,
        );
    }
    label(&mut image, 64 - title.len() as i32 * 2, 85, title, white);
    let angle = if heading {
        0.0
    } else {
        let value = if matches!(kind, Kind::Altimeter) {
            values[0].rem_euclid(1000.0)
        } else {
            values[0].clamp(min, max)
        };
        (-135.0 + 270.0 * (value - min) / (max - min)).to_radians()
    };
    if heading {
        // Rotate the card under a fixed heading index.
        let value = format!("{:03}", values[0].round().rem_euclid(360.0) as u32);
        label(&mut image, 58, 57, &value, white);
        line(&mut image, (64.0, 9.0), (64.0, 20.0), yellow);
        label(&mut image, 18, 108, "GYRO", white);
        label(&mut image, 90, 108, "BUG", white);
        if values[1] != f64::MAX {
            let bug_angle = (values[1] - values[0]).to_radians();
            line(
                &mut image,
                point(bug_angle, 44.0),
                point(bug_angle, 55.0),
                yellow,
            );
        }
    } else {
        line(&mut image, (64.0, 64.0), point(angle, 44.0), white);
        line(&mut image, (63.0, 64.0), point(angle, 44.0), white);
        draw_filled_circle_mut(&mut image, (64, 64), 3, yellow);
    }
    draw_hollow_circle_mut(&mut image, (64, 64), 56, white);
    image
}

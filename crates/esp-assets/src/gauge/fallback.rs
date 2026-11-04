use esp_script::{Host, Reference};
use image::{Rgba, RgbaImage};
use imageproc::drawing::{draw_filled_circle_mut, draw_hollow_circle_mut, draw_line_segment_mut};
use std::f64::consts::PI;

#[derive(Clone, Copy)]
enum Kind {
    Airspeed,
    Altimeter,
    Heading,
    VerticalSpeed,
    Attitude,
    Tachometer,
    Vor1,
    Vor2,
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
            ("CESSNA", "HEADING_INDICATOR") => Kind::Heading,
            ("CESSNA", "VERTICAL_SPEED") => Kind::VerticalSpeed,
            ("BELL_206B", "ATTITUDE") | ("CESSNAWALPHA", "ATTITUDE_ALPHA") => Kind::Attitude,
            ("CESSNA172", "TACHOMETER") => Kind::Tachometer,
            ("CESSNA", "VOR1") | ("CESSNAWALPHA", "VOR1_ALPHA") => Kind::Vor1,
            ("CESSNA182S", "VOR2") | ("CESSNAWALPHA", "VOR2_ALPHA") => Kind::Vor2,
            _ => return None,
        };
        Some(Self {
            kind,
            image: RgbaImage::new(128, 128),
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
            Kind::Altimeter => [read(host, "INDICATED ALTITUDE", "feet"), 0.0, 0.0, 0.0, 0.0],
            Kind::Heading => [
                read(host, "HEADING INDICATOR", "degrees"),
                0.0,
                0.0,
                0.0,
                0.0,
            ],
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
        }
        .map(|v| {
            if v == f64::MAX {
                v
            } else {
                (v.clamp(-1e8, 1e8) * 10.0).round() / 10.0
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
            'A' => [2, 5, 7, 5, 5],
            'D' => [6, 5, 5, 5, 6],
            'F' => [7, 4, 6, 4, 4],
            'G' => [7, 4, 5, 5, 7],
            'H' => [5, 5, 7, 5, 5],
            'K' => [5, 5, 6, 5, 5],
            'M' => [5, 7, 7, 5, 5],
            'O' => [7, 5, 5, 5, 7],
            'P' => [6, 5, 6, 4, 4],
            'R' => [6, 5, 6, 5, 5],
            'S' => [7, 4, 7, 1, 7],
            'T' => [7, 2, 2, 2, 2],
            'V' => [5, 5, 5, 5, 2],
            _ => [0; 5],
        };
        for (dy, row) in rows.iter().enumerate() {
            for dx in 0..3 {
                if row & (1 << (2 - dx)) != 0 {
                    let px = x + i as i32 * 4 + dx;
                    let py = y + dy as i32;
                    if (0..128).contains(&px) && (0..128).contains(&py) {
                        image.put_pixel(px as u32, py as u32, color);
                    }
                }
            }
        }
    }
}

fn draw(kind: Kind, values: [f64; 5]) -> RgbaImage {
    let mut image = RgbaImage::new(128, 128);
    let white = Rgba([235, 235, 230, 255]);
    let yellow = Rgba([255, 200, 50, 255]);
    draw_filled_circle_mut(&mut image, (64, 64), 62, Rgba([32, 32, 32, 255]));
    draw_filled_circle_mut(&mut image, (64, 64), 57, Rgba([8, 9, 10, 255]));
    if values.contains(&f64::MAX) {
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
        Kind::Attitude | Kind::Vor1 | Kind::Vor2 => unreachable!(),
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
    } else {
        line(&mut image, (64.0, 64.0), point(angle, 44.0), white);
        line(&mut image, (63.0, 64.0), point(angle, 44.0), white);
        draw_filled_circle_mut(&mut image, (64, 64), 3, yellow);
    }
    draw_hollow_circle_mut(&mut image, (64, 64), 56, white);
    image
}

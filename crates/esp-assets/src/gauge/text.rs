use ab_glyph::{FontArc, FontVec};
use anyhow::{Context, Result, ensure};
use esp_script::{FormattedString, Host, Program, Value};
use image::{Rgba, RgbaImage};
use imageproc::drawing::{draw_text_mut, text_size};
use roxmltree::Node;
use std::{
    collections::HashMap,
    sync::{Mutex, OnceLock},
};

pub(super) struct Text {
    value: FormattedString,
    font: FontArc,
    height: f32,
    horizontal: Align,
    vertical: Align,
    color: Color,
    current_color: Rgba<u8>,
    background: Rgba<u8>,
    cached: Option<(String, Rgba<u8>)>,
    value_reported: bool,
    color_reported: bool,
}

enum Color {
    Fixed(Rgba<u8>),
    Template(FormattedString),
    Script(Program),
}
enum Align {
    Start,
    Center,
    End,
}

struct Fonts {
    database: fontdb::Database,
    faces: HashMap<(String, bool, bool), FontArc>,
}
static FONTS: OnceLock<Mutex<Fonts>> = OnceLock::new();

fn font(face: &str, bold: bool, italic: bool) -> Result<FontArc> {
    let fonts = FONTS.get_or_init(|| {
        let mut database = fontdb::Database::new();
        database.load_system_fonts();
        Mutex::new(Fonts {
            database,
            faces: HashMap::new(),
        })
    });
    let mut fonts = fonts
        .lock()
        .map_err(|_| anyhow::anyhow!("font cache lock was poisoned"))?;
    let key = (face.to_ascii_lowercase(), bold, italic);
    if let Some(font) = fonts.faces.get(&key) {
        return Ok(font.clone());
    }
    let id = fonts
        .database
        .query(&fontdb::Query {
            families: &[fontdb::Family::Name(face)],
            weight: if bold {
                fontdb::Weight::BOLD
            } else {
                fontdb::Weight::NORMAL
            },
            style: if italic {
                fontdb::Style::Italic
            } else {
                fontdb::Style::Normal
            },
            ..fontdb::Query::default()
        })
        .with_context(|| format!("gauge font {face} is unavailable"))?;
    let font = fonts
        .database
        .with_face_data(id, |data, index| {
            FontVec::try_from_vec_and_index(data.to_vec(), index).map(FontArc::from)
        })
        .context("reading gauge font")??;
    fonts.faces.insert(key, font.clone());
    Ok(font)
}

fn text(node: Node<'_, '_>, name: &str) -> Option<String> {
    node.children()
        .find(|n| n.has_tag_name(name))
        .map(esp_formats::gauge::text_content)
}

fn flag(node: Node<'_, '_>, name: &str) -> Result<bool> {
    match text(node, name)
        .as_deref()
        .unwrap_or("False")
        .trim()
        .to_ascii_lowercase()
        .as_str()
    {
        "true" | "yes" | "1" => Ok(true),
        "false" | "no" | "0" => Ok(false),
        _ => anyhow::bail!("invalid gauge text {name}"),
    }
}

fn color(value: &str) -> Result<Rgba<u8>> {
    let value = value.trim().to_ascii_lowercase();
    let rgb = match value.as_str() {
        "white" => [255, 255, 255],
        "black" => [0, 0, 0],
        "red" => [255, 0, 0],
        "green" => [0, 128, 0],
        "lime" => [0, 255, 0],
        "blue" => [0, 0, 255],
        "cyan" | "aqua" => [0, 255, 255],
        "magenta" | "fuchsia" => [255, 0, 255],
        "yellow" => [255, 255, 0],
        "orange" => [255, 165, 0],
        "gray" | "grey" => [128, 128, 128],
        "silver" => [192, 192, 192],
        "navy" => [0, 0, 128],
        "maroon" => [128, 0, 0],
        "teal" => [0, 128, 128],
        "olive" => [128, 128, 0],
        "purple" => [128, 0, 128],
        _ => {
            let number = if let Some(hex) = value.strip_prefix("0x") {
                u32::from_str_radix(hex, 16)?
            } else {
                value.parse::<u32>()?
            };
            ensure!(number <= 0xffffff, "gauge text color is out of range");
            [number as u8, (number >> 8) as u8, (number >> 16) as u8]
        }
    };
    Ok(Rgba([rgb[0], rgb[1], rgb[2], 255]))
}

impl Color {
    fn parse(node: Node<'_, '_>) -> Result<Self> {
        if let Some(source) = text(node, "FontColorScript") {
            return Ok(Self::Script(Program::compile(source.trim())?));
        }
        let source = text(node, "FontColor").unwrap_or_else(|| "white".into());
        if source.trim().starts_with("%(") {
            Ok(Self::Template(FormattedString::compile(source.trim())?))
        } else {
            Ok(Self::Fixed(color(&source)?))
        }
    }

    fn evaluate(&self, host: &mut impl Host) -> Result<Rgba<u8>> {
        match self {
            Self::Fixed(color) => Ok(*color),
            Self::Template(template) => color(&template.evaluate(host)?),
            Self::Script(program) => {
                match program.evaluate(host)?.context("empty font color result")? {
                    Value::Text(text) => color(&text),
                    Value::Number(number) => {
                        ensure!(
                            number.is_finite() && number >= 0.0,
                            "invalid font color result"
                        );
                        color(&format!("{number:.0}"))
                    }
                }
            }
        }
    }
}

impl Text {
    pub fn parse(node: Node<'_, '_>, gauge_size: (u32, u32)) -> Result<(Self, RgbaImage)> {
        ensure!(
            !node.children().any(|n| matches!(
                n.tag_name().name(),
                "FontHeightScript" | "FontWidth" | "AlternateFont"
            )),
            "unsupported dynamic gauge font or character width"
        );
        let size = text(node, "Size")
            .map(|size| -> Result<(u32, u32)> {
                let (w, h) = size.split_once(',').context("invalid gauge text size")?;
                let width = w.trim().parse::<f64>()?;
                let height = h.trim().parse::<f64>()?;
                ensure!(
                    width.is_finite()
                        && height.is_finite()
                        && width > 0.0
                        && height > 0.0
                        && width <= 8192.0
                        && height <= 8192.0,
                    "invalid gauge text size"
                );
                Ok((width.round() as u32, height.round() as u32))
            })
            .transpose()?
            .unwrap_or(gauge_size);
        ensure!(
            u64::from(size.0) * u64::from(size.1) <= 16 * 1024 * 1024,
            "gauge text canvas exceeds size limit"
        );
        let face = text(node, "FontFace").unwrap_or_else(|| "Arial".into());
        let font = font(face.trim(), flag(node, "Bold")?, flag(node, "Italic")?)?;
        let height = text(node, "FontHeight")
            .map(|h| h.trim().parse::<f32>())
            .transpose()?
            .unwrap_or(12.0);
        ensure!(
            height.is_finite() && height > 0.0 && height <= 8192.0,
            "invalid gauge font height"
        );
        let align = |name, default| -> Result<Align> {
            match text(node, name)
                .as_deref()
                .unwrap_or(default)
                .trim()
                .to_ascii_uppercase()
                .as_str()
            {
                "LEFT" | "TOP" => Ok(Align::Start),
                "CENTER" | "MIDDLE" => Ok(Align::Center),
                "RIGHT" | "BOTTOM" => Ok(Align::End),
                _ => anyhow::bail!("invalid gauge text alignment"),
            }
        };
        let value = FormattedString::compile(
            text(node, "GaugeString")
                .context("missing gauge string")?
                .trim(),
        )?;
        let color = Color::parse(node).unwrap_or_else(|error| {
            eprintln!("gauge text color: {error:#}; using default white");
            Color::Fixed(Rgba([255; 4]))
        });
        let background = if flag(node, "Transparent")? {
            Rgba([0; 4])
        } else {
            color_background(node)?
        };
        Ok((
            Self {
                value,
                font,
                height,
                horizontal: align("HorizontalAlign", "LEFT")?,
                vertical: align("VerticalAlign", "TOP")?,
                color,
                current_color: Rgba([255; 4]),
                background,
                cached: None,
                value_reported: false,
                color_reported: false,
            },
            RgbaImage::from_pixel(size.0, size.1, background),
        ))
    }

    pub fn update(&mut self, image: &mut RgbaImage, host: &mut impl Host) -> bool {
        let text = match self.value.evaluate(host).and_then(|text| {
            if text.contains("\\{") {
                Err(esp_script::Error::Unsupported(
                    "inline gauge text style".into(),
                ))
            } else {
                Ok(text)
            }
        }) {
            Ok(text) => text,
            Err(error) => {
                if !self.value_reported {
                    eprintln!("gauge text value: {error}");
                    self.value_reported = true;
                }
                if self.cached.take().is_some() {
                    image.fill(0);
                    return true;
                }
                return false;
            }
        };
        match self.color.evaluate(host) {
            Ok(color) => self.current_color = color,
            Err(error) => {
                if !self.color_reported {
                    eprintln!("gauge text color: {error:#}; retaining last color");
                    self.color_reported = true;
                }
            }
        }
        if self
            .cached
            .as_ref()
            .is_some_and(|(last, color)| *last == text && *color == self.current_color)
        {
            return false;
        }
        for pixel in image.pixels_mut() {
            *pixel = self.background;
        }
        let lines = text.lines().collect::<Vec<_>>();
        let line_height = self.height.ceil() as i32;
        let height = line_height * lines.len() as i32;
        let offset = |align: &Align, available: i32, used: i32| match align {
            Align::Start => 0,
            Align::Center => (available - used) / 2,
            Align::End => available - used,
        };
        let y = offset(&self.vertical, image.height() as i32, height);
        for (index, line) in lines.iter().enumerate() {
            let (width, _) = text_size(self.height, &self.font, line);
            let x = offset(&self.horizontal, image.width() as i32, width as i32);
            draw_text_mut(
                image,
                self.current_color,
                x,
                y + index as i32 * line_height,
                self.height,
                &self.font,
                line,
            );
        }
        self.cached = Some((text, self.current_color));
        true
    }
}

fn color_background(node: Node<'_, '_>) -> Result<Rgba<u8>> {
    color(text(node, "BackgroundColor").as_deref().unwrap_or("black"))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn decodes_sdk_bgr_colors() {
        assert_eq!(color("0x01E00D").unwrap(), Rgba([13, 224, 1, 255]));
        assert_eq!(color("cyan").unwrap(), Rgba([0, 255, 255, 255]));
        assert!(color("0x1FFFFFF").is_err());
    }
}

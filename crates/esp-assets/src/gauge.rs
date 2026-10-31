use anyhow::{Context, Result};
use esp_formats::{cfg::panel::WindowKind, gauge::Gauge};
use esp_script::{Host, Program};
use image::{
    Rgba, RgbaImage,
    imageops::{FilterType, overlay, resize},
};

use crate::{panel::Panel, texture::gauge_bitmap};

pub struct PanelImage {
    pub image: RgbaImage,
    pub omitted_gauges: usize,
    pub omitted_elements: usize,
    gauges: Vec<Canvas>,
    conditions: Vec<Condition>,
    background: Rgba<u8>,
}

struct Canvas {
    size: (u32, u32),
    target: (u32, u32),
    position: (i64, i64),
    layers: Vec<Layer>,
}

struct Layer {
    image: RgbaImage,
    position: (i64, i64),
    conditions: Vec<Requirement>,
    visible: bool,
}

struct Condition {
    program: Program,
    value: Option<f64>,
    minimum: Option<f64>,
    maximum: Option<f64>,
    reported: bool,
}

#[derive(Clone)]
enum Requirement {
    Visible(usize),
    Case(usize, f64),
}

impl PanelImage {
    pub fn update(&mut self, host: &mut impl Host) -> bool {
        for condition in &mut self.conditions {
            let result = condition
                .program
                .evaluate(host)
                .and_then(|value| value.ok_or(esp_script::Error::Stack)?.number());
            match result {
                Ok(mut value) => {
                    if let Some(minimum) = condition.minimum {
                        value = value.max(minimum);
                    }
                    if let Some(maximum) = condition.maximum {
                        value = value.min(maximum);
                    }
                    condition.value = Some(value);
                }
                Err(error) if !condition.reported => {
                    eprintln!("gauge expression: {error}");
                    condition.reported = true;
                }
                Err(_) => {}
            }
        }
        let mut changed = false;
        for gauge in &mut self.gauges {
            for layer in &mut gauge.layers {
                let visible = layer
                    .conditions
                    .iter()
                    .all(|requirement| match requirement {
                        Requirement::Visible(index) => self.conditions[*index]
                            .value
                            .is_some_and(|value| value != 0.0),
                        Requirement::Case(index, expected) => {
                            self.conditions[*index].value == Some(*expected)
                        }
                    });
                changed |= visible != layer.visible;
                layer.visible = visible;
            }
        }
        if changed {
            self.compose();
        }
        changed
    }

    fn compose(&mut self) {
        for pixel in self.image.pixels_mut() {
            *pixel = self.background;
        }
        for gauge in &self.gauges {
            let mut art = RgbaImage::new(gauge.size.0, gauge.size.1);
            for layer in &gauge.layers {
                if layer.visible {
                    overlay(&mut art, &layer.image, layer.position.0, layer.position.1);
                }
            }
            let scaled = resize(&art, gauge.target.0, gauge.target.1, FilterType::Triangle);
            overlay(&mut self.image, &scaled, gauge.position.0, gauge.position.1);
        }
    }
}

pub fn load_panel(panel: &Panel, texture: &str) -> Result<Option<PanelImage>> {
    let layout = panel.layout()?;
    let Some(window) = layout.windows.iter().find(|w| {
        w.kind == WindowKind::VirtualCockpit
            && w.texture
                .is_some_and(|name| name.eq_ignore_ascii_case(texture))
    }) else {
        return Ok(None);
    };
    let pixels = window
        .pixel_size
        .as_ref()
        .context("cockpit panel has no pixel_size")?;
    let dimensions = window
        .size_mm
        .as_ref()
        .context("cockpit panel has no size_mm")?;
    let width = dimension(pixels.width)?;
    let height = dimension(pixels.height.context("cockpit panel has no pixel height")?)?;
    let layout_height = dimensions
        .height
        .context("cockpit panel has no layout height")?;
    anyhow::ensure!(
        dimensions.width > 0.0 && layout_height > 0.0,
        "invalid panel layout size"
    );
    let background = Rgba(panel.background(window.section)?);
    let output = RgbaImage::from_pixel(width, height, background);
    let mut gauges = Vec::new();
    let mut conditions = Vec::new();
    let mut omitted_gauges = 0;
    let mut omitted_elements = 0;
    for gauge in &window.gauges {
        let Some(xml) = panel.read_xml_gauge(gauge.reference, 8 * 1024 * 1024)? else {
            omitted_gauges += 1;
            continue;
        };
        let text = std::str::from_utf8(&xml)?;
        let document = Gauge::parse(text)?;
        let module = gauge
            .reference
            .split_once('!')
            .context("invalid gauge reference")?
            .0;
        let size = match document.size()? {
            Some(size) => Some(size),
            None => {
                let image = document.root().children().find(|n| n.has_tag_name("Image"));
                if let Some(name) = image
                    .filter(|n| n.attribute("ImageSizes").is_none())
                    .and_then(|n| n.attribute("Name"))
                {
                    panel
                        .read_resource(module, name, 64 * 1024 * 1024)?
                        .map(|data| gauge_bitmap(&data, false, false))
                        .transpose()?
                        .map(|image| [image.width(), image.height()])
                } else {
                    None
                }
            }
        };
        let Some([w, h]) = size else {
            omitted_gauges += 1;
            continue;
        };
        if w == 0 || h == 0 || w > 8192 || h > 8192 {
            anyhow::bail!("invalid gauge size {w}x{h}");
        }
        let mut layers = Vec::new();
        let mut stack = vec![(document.root(), 0.0, 0.0, Vec::new())];
        while let Some((node, x, y, mut parents)) = stack.pop() {
            if node.has_tag_name("Element")
                && node.children().any(|child| {
                    child.is_element()
                        && matches!(
                            child.tag_name().name(),
                            "Rotate"
                                | "Rotation"
                                | "Shift"
                                | "MaskImage"
                                | "LeftScript"
                                | "TopScript"
                                | "ClipRect"
                        )
                })
            {
                omitted_elements += 1;
                continue;
            }
            if node.has_tag_name("Element")
                && let Some(visibility) = node.children().find(|c| c.has_tag_name("Visibility"))
            {
                let source = visibility.text().unwrap_or("").trim();
                match Program::compile(source) {
                    Ok(program) if !source.is_empty() => {
                        parents.push(Requirement::Visible(conditions.len()));
                        conditions.push(Condition {
                            program,
                            value: None,
                            minimum: None,
                            maximum: None,
                            reported: false,
                        });
                    }
                    _ => {
                        omitted_elements += 1;
                        continue;
                    }
                }
            }
            if node.has_tag_name("Select") {
                let Some(expression) = node.children().find(|c| c.has_tag_name("Expression"))
                else {
                    omitted_elements += 1;
                    continue;
                };
                let Some(source) = expression
                    .children()
                    .find(|c| c.has_tag_name("Script"))
                    .and_then(|c| c.text())
                    .filter(|source| !source.trim().is_empty())
                else {
                    omitted_elements += 1;
                    continue;
                };
                let Ok(program) = Program::compile(source.trim()) else {
                    omitted_elements += 1;
                    continue;
                };
                let bound = |name| -> Result<Option<f64>> {
                    let value = expression
                        .children()
                        .find(|c| c.has_tag_name(name))
                        .and_then(|c| c.text())
                        .map(|v| v.trim().parse::<f64>())
                        .transpose()?;
                    anyhow::ensure!(value.is_none_or(f64::is_finite), "invalid select bound");
                    Ok(value.filter(|value| *value != -9_999_999.0))
                };
                let minimum = bound("Minimum")?;
                let maximum = bound("Maximum")?;
                anyhow::ensure!(
                    minimum.zip(maximum).is_none_or(|(min, max)| min <= max),
                    "invalid select bounds"
                );
                let index = conditions.len();
                conditions.push(Condition {
                    program,
                    value: None,
                    minimum,
                    maximum,
                    reported: false,
                });
                for case in node.children().filter(|c| c.has_tag_name("Case")).rev() {
                    let Some(value) = case
                        .children()
                        .find(|c| c.has_tag_name("ExpressionResult"))
                        .and_then(|c| c.text())
                        .and_then(|text| text.trim().parse::<f64>().ok())
                        .filter(|value| value.is_finite())
                    else {
                        omitted_elements += 1;
                        continue;
                    };
                    let mut requirements = parents.clone();
                    requirements.push(Requirement::Case(index, value));
                    stack.push((case, x, y, requirements));
                }
                continue;
            }
            let (x, y) = if node.has_tag_name("Element") {
                let (px, py) = position(node)?;
                (x + px, y + py)
            } else {
                (x, y)
            };
            if node.has_tag_name("Image") {
                let name = node.attribute("Name").context("image missing name")?;
                let flag = |key| {
                    node.attribute(key)
                        .or_else(|| {
                            node.children()
                                .find(|c| c.has_tag_name(key))
                                .and_then(|c| c.text())
                        })
                        .is_some_and(|v| {
                            v.eq_ignore_ascii_case("true") || v.eq_ignore_ascii_case("yes")
                        })
                };
                let Some(bytes) = panel.read_resource(module, name, 64 * 1024 * 1024)? else {
                    omitted_elements += 1;
                    continue;
                };
                let bitmap = match gauge_bitmap(&bytes, flag("Alpha"), flag("Transparent")) {
                    Ok(bitmap) => bitmap,
                    Err(error)
                        if matches!(
                            error.downcast_ref::<esp_formats::texture::bmp::Error>(),
                            Some(esp_formats::texture::bmp::Error::AlphaFormat)
                        ) =>
                    {
                        omitted_elements += 1;
                        continue;
                    }
                    Err(error) => {
                        return Err(error).with_context(|| format!("{}: {name}", gauge.reference));
                    }
                };
                layers.push(Layer {
                    image: bitmap,
                    position: (x.round() as i64, y.round() as i64),
                    visible: parents.is_empty(),
                    conditions: parents.clone(),
                });
            }
            for child in node.children().filter(|c| c.is_element()).rev() {
                if child.has_tag_name("Element")
                    || child.has_tag_name("Image")
                    || child.has_tag_name("Select")
                {
                    stack.push((child, x, y, parents.clone()));
                }
            }
        }
        let target_width =
            dimension(gauge.width.unwrap_or(f64::from(w)) * f64::from(width) / dimensions.width)?;
        let target_height =
            dimension(gauge.height.unwrap_or(f64::from(h)) * f64::from(height) / layout_height)?;
        gauges.push(Canvas {
            size: (w, h),
            target: (target_width, target_height),
            position: (
                (gauge.x * f64::from(width) / dimensions.width).round() as i64,
                (gauge.y * f64::from(height) / layout_height).round() as i64,
            ),
            layers,
        });
    }
    let mut result = PanelImage {
        image: output,
        omitted_gauges,
        omitted_elements,
        gauges,
        conditions,
        background,
    };
    result.compose();
    Ok(Some(result))
}

fn dimension(value: f64) -> Result<u32> {
    anyhow::ensure!(
        value.is_finite() && (1.0..=8192.0).contains(&value),
        "invalid panel image dimension {value}"
    );
    Ok(value.round() as u32)
}

fn position(node: roxmltree::Node<'_, '_>) -> Result<(f64, f64)> {
    if let Some(position) = node.children().find(|c| c.has_tag_name("FloatPosition")) {
        let text = position.text().context("empty FloatPosition")?;
        let (x, y) = text.split_once(',').context("invalid FloatPosition")?;
        return finite_position(x.trim().parse()?, y.trim().parse()?);
    }
    if let Some(position) = node.children().find(|c| c.has_tag_name("Position")) {
        return finite_position(
            position.attribute("X").context("missing X")?.parse()?,
            position.attribute("Y").context("missing Y")?.parse()?,
        );
    }
    Ok((0.0, 0.0))
}

fn finite_position(x: f64, y: f64) -> Result<(f64, f64)> {
    anyhow::ensure!(x.is_finite() && y.is_finite(), "invalid gauge position");
    Ok((x, y))
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_script::{Error, Reference, Value};

    struct TestHost(f64);

    impl Host for TestHost {
        fn read(&mut self, _: &Reference) -> Result<Value, Error> {
            Ok(Value::Number(self.0))
        }

        fn write(&mut self, _: &Reference, _: Value) -> Result<(), Error> {
            Err(Error::Type)
        }
    }
    #[test]
    fn composes_images_with_hidden_elements() {
        let root = tempfile::tempdir().unwrap();
        let panel_path = root.path().join("panel");
        let module = panel_path.join("example");
        std::fs::create_dir_all(&module).unwrap();
        std::fs::write(panel_path.join("panel.cfg"),"[VCockpit01]\nsize_mm=8,8\npixel_size=8,8\ntexture=$screen\nbackground_color=10,20,30\ngauge00=Example!Dial,2,2,4,4").unwrap();
        std::fs::write(module.join("dial.xml"),r#"<Gauge Name="Dial"><Size X="4" Y="4"/><Element><Position X="1" Y="1"/><Image Name="red.bmp"/></Element><Element><Visibility>0</Visibility><Image Name="red.bmp"/></Element></Gauge>"#).unwrap();
        let mut data = Vec::new();
        image::codecs::bmp::BmpEncoder::new(&mut data)
            .encode(&[255, 0, 0], 1, 1, image::ExtendedColorType::Rgb8)
            .unwrap();
        std::fs::write(module.join("red.bmp"), data).unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let result = load_panel(&panel, "$SCREEN").unwrap().unwrap();
        assert_eq!(result.omitted_elements, 0);
        assert_eq!(result.omitted_gauges, 0);
        assert_eq!(result.image.get_pixel(3, 3).0, [255, 0, 0, 255]);
        assert_eq!(result.image.get_pixel(2, 2).0, [10, 20, 30, 255]);
    }

    #[test]
    fn uses_background_bitmap_for_unsized_gauge() {
        let root = tempfile::tempdir().unwrap();
        let panel_path = root.path().join("panel");
        let module = panel_path.join("example");
        std::fs::create_dir_all(&module).unwrap();
        std::fs::write(
            panel_path.join("panel.cfg"),
            "[VCockpit01]\nsize_mm=4,4\npixel_size=4,4\ntexture=$screen\ngauge00=example!dial,1,1",
        )
        .unwrap();
        std::fs::write(
            module.join("dial.xml"),
            r#"<Gauge Name="Dial"><Image Name="background.bmp"/></Gauge>"#,
        )
        .unwrap();
        let mut data = Vec::new();
        image::codecs::bmp::BmpEncoder::new(&mut data)
            .encode(
                &[255, 0, 0, 0, 255, 0],
                2,
                1,
                image::ExtendedColorType::Rgb8,
            )
            .unwrap();
        std::fs::write(module.join("background.bmp"), data).unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let result = load_panel(&panel, "$screen").unwrap().unwrap();
        assert_eq!(result.omitted_gauges, 0);
        assert_eq!(result.image.get_pixel(1, 1).0, [255, 0, 0, 255]);
        assert_eq!(result.image.get_pixel(2, 1).0, [0, 255, 0, 255]);
        assert_eq!(result.image.get_pixel(3, 1).0, [0, 0, 0, 255]);
    }

    #[test]
    fn updates_visible_layers_and_restores_background() {
        let root = tempfile::tempdir().unwrap();
        let panel_path = root.path().join("panel");
        let module = panel_path.join("example");
        std::fs::create_dir_all(&module).unwrap();
        std::fs::write(
            panel_path.join("panel.cfg"),
            "[VCockpit01]\nsize_mm=2,2\npixel_size=2,2\ntexture=$screen\ngauge00=example!dial,0,0",
        )
        .unwrap();
        std::fs::write(module.join("dial.xml"), r#"<Gauge><Size X="2" Y="2"/><Element><Visibility>(L:POWER, bool)</Visibility><Element><Image Name="lamp.bmp"/></Element></Element></Gauge>"#).unwrap();
        let mut data = Vec::new();
        image::codecs::bmp::BmpEncoder::new(&mut data)
            .encode(&[255, 0, 0], 1, 1, image::ExtendedColorType::Rgb8)
            .unwrap();
        std::fs::write(module.join("lamp.bmp"), data).unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let mut image = load_panel(&panel, "$screen").unwrap().unwrap();
        let mut host = TestHost(0.0);
        assert!(!image.update(&mut host));
        assert_eq!(image.image.get_pixel(0, 0).0, [0, 0, 0, 255]);
        host.0 = 1.0;
        assert!(image.update(&mut host));
        assert_eq!(image.image.get_pixel(0, 0).0, [255, 0, 0, 255]);
        assert!(!image.update(&mut host));
        host.0 = 0.0;
        assert!(image.update(&mut host));
        assert_eq!(image.image.get_pixel(0, 0).0, [0, 0, 0, 255]);
    }

    #[test]
    fn selects_gauge_images_from_expression() {
        let root = tempfile::tempdir().unwrap();
        let panel_path = root.path().join("panel");
        let module = panel_path.join("example");
        std::fs::create_dir_all(&module).unwrap();
        std::fs::write(
            panel_path.join("panel.cfg"),
            "[VCockpit01]\nsize_mm=2,2\npixel_size=2,2\ntexture=$screen\ngauge00=example!dial,0,0",
        )
        .unwrap();
        std::fs::write(module.join("dial.xml"), r#"<Gauge><Size X="2" Y="2"/><Element><Select><Expression><Minimum>0</Minimum><Maximum>1</Maximum><Script>(L:SWITCH, bool)</Script></Expression><Case><ExpressionResult>0</ExpressionResult><Image Name="off.bmp"/></Case><Case><ExpressionResult>1</ExpressionResult><Image Name="on.bmp"/></Case></Select></Element></Gauge>"#).unwrap();
        for (name, color) in [("off.bmp", [255, 0, 0]), ("on.bmp", [0, 255, 0])] {
            let mut data = Vec::new();
            image::codecs::bmp::BmpEncoder::new(&mut data)
                .encode(&color, 1, 1, image::ExtendedColorType::Rgb8)
                .unwrap();
            std::fs::write(module.join(name), data).unwrap();
        }
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let mut image = load_panel(&panel, "$screen").unwrap().unwrap();
        let mut host = TestHost(0.0);
        assert!(image.update(&mut host));
        assert_eq!(image.image.get_pixel(0, 0).0, [255, 0, 0, 255]);
        host.0 = 1.0;
        assert!(image.update(&mut host));
        assert_eq!(image.image.get_pixel(0, 0).0, [0, 255, 0, 255]);
        host.0 = 2.0;
        assert!(!image.update(&mut host));
        assert_eq!(image.image.get_pixel(0, 0).0, [0, 255, 0, 255]);
    }
}

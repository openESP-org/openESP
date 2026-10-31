use anyhow::{Context, Result};
use esp_formats::{cfg::panel::WindowKind, gauge::Gauge};
use esp_script::{Host, Program};
use image::{
    Rgba, RgbaImage,
    imageops::{FilterType, overlay, resize},
};
use rayon::prelude::*;

use crate::{panel::Panel, texture::gauge_bitmap};

pub struct PanelImage {
    pub image: RgbaImage,
    pub omitted_gauges: usize,
    pub omitted_elements: usize,
    pub mouse_areas: Vec<PanelMouseArea>,
    gauges: Vec<Canvas>,
    conditions: Vec<Condition>,
    background: Rgba<u8>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MouseEvent {
    LeftSingle,
    LeftRelease,
    WheelUp,
    WheelDown,
}

impl MouseEvent {
    fn name(self) -> &'static str {
        match self {
            Self::LeftSingle => "LeftSingle",
            Self::LeftRelease => "LeftRelease",
            Self::WheelUp => "WheelUp",
            Self::WheelDown => "WheelDown",
        }
    }
}

pub struct PanelMouseArea {
    pub x: f64,
    pub y: f64,
    pub w: f64,
    pub h: f64,
    pub script: Program,
    pub events: Vec<MouseEvent>,
}

struct MouseAreaSource {
    bounds: (f64, f64, f64, f64),
    source: String,
    events: Vec<MouseEvent>,
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
    pub fn left_click(&self, x: f64, y: f64, host: &mut impl Host) -> Result<bool> {
        let areas = self.mouse_areas_at(x, y);
        self.mouse_event(MouseEvent::LeftSingle, &areas, host)
    }

    pub fn mouse_areas_at(&self, x: f64, y: f64) -> Vec<usize> {
        self.mouse_areas
            .iter()
            .enumerate()
            .filter_map(|(index, area)| {
                (x >= area.x && y >= area.y && x < area.x + area.w && y < area.y + area.h)
                    .then_some(index)
            })
            .collect()
    }

    pub fn mouse_event(
        &self,
        event: MouseEvent,
        areas: &[usize],
        host: &mut impl Host,
    ) -> Result<bool> {
        struct MouseHost<'a, H>(&'a mut H, MouseEvent);
        impl<H: Host> Host for MouseHost<'_, H> {
            fn read(
                &mut self,
                reference: &esp_script::Reference,
            ) -> std::result::Result<esp_script::Value, esp_script::Error> {
                if reference.category == 'M' && reference.name == "EVENT" {
                    Ok(esp_script::Value::Text(self.1.name().into()))
                } else {
                    self.0.read(reference)
                }
            }
            fn write(
                &mut self,
                reference: &esp_script::Reference,
                value: esp_script::Value,
            ) -> std::result::Result<(), esp_script::Error> {
                self.0.write(reference, value)
            }
        }
        let mut host = MouseHost(host, event);
        let mut handled = false;
        let mut failure = None;
        for &index in areas {
            let area = self
                .mouse_areas
                .get(index)
                .context("invalid mouse area index")?;
            if area.events.contains(&event) {
                if let Err(error) = area.script.evaluate(&mut host) {
                    failure.get_or_insert(error);
                }
                handled = true;
            }
        }
        match failure {
            Some(error) => Err(error.into()),
            None => Ok(handled),
        }
    }

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
    let loaded: Vec<Result<_>> = window
        .gauges
        .par_iter()
        .map(|gauge| {
            let mut conditions = Vec::new();
            let mut omitted_elements = 0;
            let Some(xml) = panel.read_xml_gauge(gauge.reference, 8 * 1024 * 1024)? else {
                return Ok(None);
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
                return Ok(None);
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
                            return Err(error)
                                .with_context(|| format!("{}: {name}", gauge.reference));
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
            let target_width = dimension(
                gauge.width.unwrap_or(f64::from(w)) * f64::from(width) / dimensions.width,
            )?;
            let target_height = dimension(
                gauge.height.unwrap_or(f64::from(h)) * f64::from(height) / layout_height,
            )?;
            let canvas = Canvas {
                size: (w, h),
                target: (target_width, target_height),
                position: (
                    (gauge.x * f64::from(width) / dimensions.width).round() as i64,
                    (gauge.y * f64::from(height) / layout_height).round() as i64,
                ),
                layers,
            };
            let mut raw_areas = Vec::new();
            collect_mouse_areas(document.root(), 0.0, 0.0, &mut raw_areas)?;
            Ok(Some((canvas, conditions, omitted_elements, raw_areas)))
        })
        .collect();
    let mut gauges = Vec::new();
    let mut conditions = Vec::new();
    let mut mouse_areas = Vec::new();
    let mut omitted_gauges = 0;
    let mut omitted_elements = 0;
    for loaded in loaded {
        let Some((mut canvas, mut gauge_conditions, omitted, raw_areas)) = loaded? else {
            omitted_gauges += 1;
            continue;
        };
        for layer in &mut canvas.layers {
            for requirement in &mut layer.conditions {
                match requirement {
                    Requirement::Visible(index) | Requirement::Case(index, _) => {
                        *index += conditions.len()
                    }
                }
            }
        }
        conditions.append(&mut gauge_conditions);
        omitted_elements += omitted;
        let sx = f64::from(canvas.target.0) / f64::from(canvas.size.0);
        let sy = f64::from(canvas.target.1) / f64::from(canvas.size.1);
        let cx = canvas.position.0 as f64;
        let cy = canvas.position.1 as f64;
        for raw in raw_areas {
            let (ax, ay, aw, ah) = raw.bounds;
            match Program::compile(&raw.source) {
                Ok(script) => mouse_areas.push(PanelMouseArea {
                    x: cx + ax * sx,
                    y: cy + ay * sy,
                    w: aw * sx,
                    h: ah * sy,
                    script,
                    events: raw.events,
                }),
                Err(error) => eprintln!("mouse script: {error}"),
            }
        }
        gauges.push(canvas);
    }
    let mut result = PanelImage {
        image: output,
        omitted_gauges,
        omitted_elements,
        mouse_areas,
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

fn area_size(node: roxmltree::Node<'_, '_>) -> Result<(f64, f64)> {
    let Some(size) = node.children().find(|c| c.has_tag_name("Size")) else {
        return Ok((0.0, 0.0));
    };
    let (w, h) = if let Some(w) = size.attribute("X") {
        (
            w.parse()?,
            size.attribute("Y")
                .context("missing mouse height")?
                .parse()?,
        )
    } else {
        let text = esp_formats::gauge::text_content(size);
        let (w, h) = text.split_once(',').context("invalid mouse size")?;
        (w.trim().parse()?, h.trim().parse()?)
    };
    let (w, h) = finite_position(w, h)?;
    anyhow::ensure!(w >= 0.0 && h >= 0.0, "negative mouse size");
    Ok((w, h))
}

fn collect_mouse_areas(
    node: roxmltree::Node<'_, '_>,
    offset_x: f64,
    offset_y: f64,
    out: &mut Vec<MouseAreaSource>,
) -> Result<()> {
    let mut stack = vec![(node, offset_x, offset_y)];
    while let Some((node, offset_x, offset_y)) = stack.pop() {
        for child in node.children().filter(|c| c.has_tag_name("MouseArea")) {
            let bounds = position(child).and_then(|(dx, dy)| {
                let (x, y) = finite_position(offset_x + dx, offset_y + dy)?;
                let (w, h) = area_size(child)?;
                Ok((x, y, w, h))
            });
            let (x, y, w, h) = match bounds {
                Ok(bounds) => bounds,
                Err(error) => {
                    eprintln!("mouse area {:?}: {error}", child.attribute("id"));
                    continue;
                }
            };
            for click in child.children().filter(|c| c.has_tag_name("MouseClick")) {
                let click_type = click
                    .children()
                    .find(|c| c.has_tag_name("ClickType"))
                    .map(esp_formats::gauge::text_content)
                    .unwrap_or_default();
                let mut events = Vec::new();
                if click_type.trim().is_empty() {
                    events.push(MouseEvent::LeftSingle);
                } else {
                    for kind in click_type.split('+').map(str::trim) {
                        match kind.to_ascii_lowercase().as_str() {
                            "leftsingle" => events.push(MouseEvent::LeftSingle),
                            "leftrelease" => events.push(MouseEvent::LeftRelease),
                            "leftall" => {
                                events.extend([MouseEvent::LeftSingle, MouseEvent::LeftRelease])
                            }
                            "wheelup" => events.push(MouseEvent::WheelUp),
                            "wheeldown" => events.push(MouseEvent::WheelDown),
                            "wheel" => events.extend([MouseEvent::WheelUp, MouseEvent::WheelDown]),
                            _ => eprintln!("unsupported mouse event: {kind}"),
                        }
                    }
                }
                if events.is_empty() {
                    continue;
                }
                let source = if let Some(script) =
                    click.children().find(|c| c.has_tag_name("Script"))
                {
                    esp_formats::gauge::text_content(script)
                } else if let Some(key) = click.children().find(|c| c.has_tag_name("KeyEvent")) {
                    let key = esp_formats::gauge::text_content(key);
                    let key = key.trim();
                    anyhow::ensure!(
                        !key.is_empty()
                            && key.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_'),
                        "invalid mouse KeyEvent"
                    );
                    format!("(>K:{key})")
                } else {
                    continue;
                };
                if !source.trim().is_empty() {
                    out.push(MouseAreaSource {
                        bounds: (x, y, w, h),
                        source,
                        events,
                    });
                }
            }
            stack.push((child, x, y));
        }
    }
    Ok(())
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

    #[test]
    fn retains_conditions_for_each_gauge() {
        let root = tempfile::tempdir().unwrap();
        let panel_path = root.path().join("panel");
        let module = panel_path.join("example");
        std::fs::create_dir_all(&module).unwrap();
        std::fs::write(panel_path.join("panel.cfg"), "[VCockpit01]\nsize_mm=2,1\npixel_size=2,1\ntexture=$screen\ngauge00=example!first,0,0\ngauge01=example!second,1,0").unwrap();
        std::fs::write(module.join("first.xml"), r#"<Gauge><Size X="1" Y="1"/><Element><Visibility>1</Visibility><Image Name="lamp.bmp"/></Element></Gauge>"#).unwrap();
        std::fs::write(module.join("second.xml"), r#"<Gauge><Size X="1" Y="1"/><Element><Visibility>(L:POWER, bool)</Visibility><Image Name="lamp.bmp"/></Element></Gauge>"#).unwrap();
        let mut data = Vec::new();
        image::codecs::bmp::BmpEncoder::new(&mut data)
            .encode(&[255, 0, 0], 1, 1, image::ExtendedColorType::Rgb8)
            .unwrap();
        std::fs::write(module.join("lamp.bmp"), data).unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let mut image = load_panel(&panel, "$screen").unwrap().unwrap();
        let mut host = TestHost(0.0);
        assert!(image.update(&mut host));
        assert_eq!(image.image.get_pixel(0, 0).0, [255, 0, 0, 255]);
        assert_eq!(image.image.get_pixel(1, 0).0, [0, 0, 0, 255]);
        host.0 = 1.0;
        assert!(image.update(&mut host));
        assert_eq!(image.image.get_pixel(1, 0).0, [255, 0, 0, 255]);
    }

    #[test]
    fn collects_mouse_areas_with_panel_coordinates() {
        let root = tempfile::tempdir().unwrap();
        let panel_path = root.path().join("panel");
        let module = panel_path.join("example");
        std::fs::create_dir_all(&module).unwrap();
        std::fs::write(
            panel_path.join("panel.cfg"),
            "[VCockpit01]\nsize_mm=4,4\npixel_size=4,4\ntexture=$screen\ngauge00=example!dial,1,1,2,2",
        )
        .unwrap();
        std::fs::write(
            module.join("dial.xml"),
            r#"<SimBase.Document Type="AceXML"><SimGauge.Gauge><Size>2,2</Size>
            <Image Name="bg.bmp"/>
            <MouseArea id="Outer"><FloatPosition>0,0</FloatPosition><Size>2,2</Size>
                <MouseArea id="Inner"><FloatPosition>1,0</FloatPosition><Size>1,1</Size>
                    <MouseClick id="MouseClick"><ClickType>LeftSingle</ClickType>
                    <Script>1 (&gt;K:TEST_EVENT)</Script></MouseClick>
                </MouseArea>
            </MouseArea></SimGauge.Gauge></SimBase.Document>"#,
        )
        .unwrap();
        let mut data = Vec::new();
        image::codecs::bmp::BmpEncoder::new(&mut data)
            .encode(&[0, 0, 0], 1, 1, image::ExtendedColorType::Rgb8)
            .unwrap();
        std::fs::write(module.join("bg.bmp"), data).unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let result = load_panel(&panel, "$screen").unwrap().unwrap();
        // The nested area is scaled into panel pixel coordinates.
        assert_eq!(result.mouse_areas.len(), 1);
        let area = &result.mouse_areas[0];
        assert!((area.x - 2.0).abs() < 1e-9);
        assert!((area.y - 1.0).abs() < 1e-9);
        assert!((area.w - 1.0).abs() < 1e-9);
        assert!((area.h - 1.0).abs() < 1e-9);
    }

    #[test]
    fn keeps_release_handlers_separate_from_presses() {
        let root = tempfile::tempdir().unwrap();
        let panel_path = root.path().join("panel");
        let module = panel_path.join("example");
        std::fs::create_dir_all(&module).unwrap();
        std::fs::write(
            panel_path.join("panel.cfg"),
            "[VCockpit01]\nsize_mm=2,2\npixel_size=2,2\ntexture=$screen\ngauge00=example!dial,0,0,2,2",
        )
        .unwrap();
        std::fs::write(
            module.join("dial.xml"),
            r#"<SimBase.Document Type="AceXML"><SimGauge.Gauge><Size>2,2</Size>
            <Image Name="bg.bmp"/>
            <MouseArea><FloatPosition>0,0</FloatPosition><Size>2,2</Size>
                <MouseClick><ClickType>LeftRelease</ClickType><Script>1 (&gt;K:REL)</Script></MouseClick>
            </MouseArea>
            <MouseArea><FloatPosition>0,0</FloatPosition><Size>2,2</Size>
                <MouseClick><ClickType>LeftSingle</ClickType><Script>1 (&gt;K:PRESS)</Script></MouseClick>
            </MouseArea>
            </SimGauge.Gauge></SimBase.Document>"#,
        )
        .unwrap();
        let mut data = Vec::new();
        image::codecs::bmp::BmpEncoder::new(&mut data)
            .encode(&[0, 0, 0], 1, 1, image::ExtendedColorType::Rgb8)
            .unwrap();
        std::fs::write(module.join("bg.bmp"), data).unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let result = load_panel(&panel, "$screen").unwrap().unwrap();
        assert_eq!(result.mouse_areas.len(), 2);
        assert_eq!(result.mouse_areas[0].events, [MouseEvent::LeftRelease]);
        assert_eq!(result.mouse_areas[1].events, [MouseEvent::LeftSingle]);
        // Script writes to K:, so use a permissive host.
        struct SinkHost;
        impl Host for SinkHost {
            fn read(&mut self, _: &Reference) -> Result<Value, Error> {
                Ok(Value::Number(0.0))
            }
            fn write(&mut self, _: &Reference, _: Value) -> Result<(), Error> {
                Ok(())
            }
        }
        result.mouse_areas[0]
            .script
            .evaluate(&mut SinkHost)
            .unwrap();
    }

    #[test]
    fn key_events_and_scripts_follow_mouse_bounds() {
        let gauge = Gauge::parse(r#"<Gauge><MouseArea><Position X="1" Y="2"/><Size X="4" Y="4"/>
            <MouseClick><KeyEvent>TEST</KeyEvent></MouseClick>
            <MouseArea><Position X="1" Y="1"/><Size X="2" Y="2"/>
            <MouseClick><KeyEvent>IGNORED</KeyEvent><Script>(M:Event) 'LeftSingle' scmp 0 == if{ 2 (&gt;K:OTHER) }<!-- comment --></Script></MouseClick>
            </MouseArea></MouseArea></Gauge>"#).unwrap();
        let mut raw = Vec::new();
        collect_mouse_areas(gauge.root(), 0.0, 0.0, &mut raw).unwrap();
        let mut panel = PanelImage {
            image: RgbaImage::new(8, 8),
            omitted_gauges: 0,
            omitted_elements: 0,
            mouse_areas: raw
                .into_iter()
                .map(|raw| PanelMouseArea {
                    x: raw.bounds.0,
                    y: raw.bounds.1,
                    w: raw.bounds.2,
                    h: raw.bounds.3,
                    script: Program::compile(&raw.source).unwrap(),
                    events: raw.events,
                })
                .collect(),
            gauges: vec![],
            conditions: vec![],
            background: Rgba([0; 4]),
        };
        #[derive(Default)]
        struct Events(Vec<(String, Value)>);
        impl Host for Events {
            fn read(&mut self, _: &Reference) -> Result<Value, Error> {
                Err(Error::Type)
            }
            fn write(&mut self, reference: &Reference, value: Value) -> Result<(), Error> {
                self.0.push((reference.name.clone(), value));
                Ok(())
            }
        }
        let mut host = Events::default();
        assert!(panel.left_click(2.5, 3.5, &mut host).unwrap());
        assert_eq!(
            host.0,
            [
                ("TEST".into(), Value::Number(0.0)),
                ("OTHER".into(), Value::Number(2.0))
            ]
        );
        assert!(!panel.left_click(5.0, 6.0, &mut host).unwrap());
        assert!(!panel.left_click(f64::NAN, 3.0, &mut host).unwrap());
        panel.mouse_areas[0].script = Program::compile("(A:MISSING,bool)").unwrap();
        assert!(panel.left_click(1.5, 2.5, &mut host).is_err());
        host.0.clear();
        assert!(panel.left_click(2.5, 3.5, &mut host).is_err());
        assert_eq!(host.0, [("OTHER".into(), Value::Number(2.0))]);
    }

    #[test]
    fn rejects_invalid_mouse_geometry_without_discarding_valid_controls() {
        for data in [
            "<FloatPosition>NaN,0</FloatPosition>",
            "<FloatPosition>bad</FloatPosition>",
            "<Size>-1,2</Size>",
            "<Size>1,NaN</Size>",
            "<Size X='1'/>",
        ] {
            let source = format!(
                "<Gauge><MouseArea>{data}<MouseClick><KeyEvent>BAD</KeyEvent></MouseClick></MouseArea><MouseArea><Size>1,1</Size><MouseClick><KeyEvent>GOOD</KeyEvent></MouseClick></MouseArea></Gauge>"
            );
            let gauge = Gauge::parse(&source).unwrap();
            let mut raw = Vec::new();
            collect_mouse_areas(gauge.root(), 0.0, 0.0, &mut raw).unwrap();
            assert_eq!(raw.len(), 1, "{data}");
            assert_eq!(raw[0].source, "(>K:GOOD)");
        }
    }

    #[test]
    fn rejects_script_injection_in_key_names() {
        let gauge = Gauge::parse("<Gauge><MouseArea><MouseClick><KeyEvent>TEST) 1 (&gt;L:VALUE</KeyEvent></MouseClick></MouseArea></Gauge>").unwrap();
        assert!(collect_mouse_areas(gauge.root(), 0.0, 0.0, &mut Vec::new()).is_err());
    }

    #[test]
    fn captured_release_and_wheel_use_their_event_context() {
        #[derive(Default)]
        struct Values(std::collections::HashMap<String, Value>);
        impl Host for Values {
            fn read(&mut self, reference: &Reference) -> Result<Value, Error> {
                Ok(self
                    .0
                    .get(&reference.name)
                    .cloned()
                    .unwrap_or(Value::Number(0.0)))
            }
            fn write(&mut self, reference: &Reference, value: Value) -> Result<(), Error> {
                self.0.insert(reference.name.clone(), value);
                Ok(())
            }
        }
        let panel = PanelImage {
            image: RgbaImage::new(8, 8),
            omitted_gauges: 0, omitted_elements: 0,
            mouse_areas: vec![PanelMouseArea {
                x: 1.0, y: 1.0, w: 2.0, h: 2.0,
                script: Program::compile("(M:Event) 'LeftSingle' scmp 0 == if{ 1 (>L:PRESSED) } (M:Event) 'LeftRelease' scmp 0 == if{ 0 (>L:PRESSED) } (M:Event) 'WheelUp' scmp 0 == if{ 1 (>L:WHEEL) } (M:Event) 'WheelDown' scmp 0 == if{ -1 (>L:WHEEL) }").unwrap(),
                events: vec![MouseEvent::LeftSingle, MouseEvent::LeftRelease, MouseEvent::WheelUp, MouseEvent::WheelDown],
            }],
            gauges: vec![], conditions: vec![], background: Rgba([0; 4]),
        };
        let captured = panel.mouse_areas_at(2.0, 2.0);
        let mut host = Values::default();
        panel
            .mouse_event(MouseEvent::LeftSingle, &captured, &mut host)
            .unwrap();
        assert_eq!(host.0["PRESSED"], Value::Number(1.0));
        assert!(panel.mouse_areas_at(5.0, 5.0).is_empty());
        panel
            .mouse_event(MouseEvent::LeftRelease, &captured, &mut host)
            .unwrap();
        assert_eq!(host.0["PRESSED"], Value::Number(0.0));
        panel
            .mouse_event(MouseEvent::WheelUp, &captured, &mut host)
            .unwrap();
        assert_eq!(host.0["WHEEL"], Value::Number(1.0));
        panel
            .mouse_event(MouseEvent::WheelDown, &captured, &mut host)
            .unwrap();
        assert_eq!(host.0["WHEEL"], Value::Number(-1.0));
        assert!(
            panel
                .mouse_event(MouseEvent::WheelUp, &[usize::MAX], &mut host)
                .is_err()
        );
    }
}

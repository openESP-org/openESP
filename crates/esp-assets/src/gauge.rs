use anyhow::{Context, Result};
use esp_formats::{cfg::panel::WindowKind, gauge::Gauge};
use esp_script::{Host, Program};
use glam::{DAffine2, DVec2};
use image::{
    Rgba, RgbaImage,
    imageops::{FilterType, overlay, resize},
};
use rayon::prelude::*;
use std::sync::Arc;

use crate::{panel::Panel, texture::gauge_bitmap};

mod fallback;
mod mask;
mod rotation;
mod text;
use mask::Mask;
use rotation::Motion;

pub struct PanelImage {
    pub image: RgbaImage,
    pub omitted_gauges: usize,
    pub omitted_elements: usize,
    pub mouse_areas: Vec<PanelMouseArea>,
    gauges: Vec<Canvas>,
    conditions: Vec<Condition>,
    updates: Vec<Update>,
    motion_elapsed: f64,
    background: Rgba<u8>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MouseEvent {
    LeftSingle,
    LeftRelease,
    LeftDrag,
    DownRepeat,
    WheelUp,
    WheelDown,
}

impl MouseEvent {
    fn name(self) -> &'static str {
        match self {
            Self::LeftSingle => "LeftSingle",
            Self::LeftRelease => "LeftRelease",
            Self::LeftDrag => "LeftDrag",
            Self::DownRepeat => "DownRepeat",
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
    pub repeat: bool,
    gauge_origin: (f64, f64),
    gauge_scale: (f64, f64),
    visibility: Vec<Arc<Program>>,
}

impl PanelMouseArea {
    fn visible(&self, host: &mut impl Host) -> std::result::Result<bool, esp_script::Error> {
        for program in &self.visibility {
            let value = program
                .evaluate(host)?
                .ok_or(esp_script::Error::Stack)?
                .number()?;
            if value == 0.0 {
                return Ok(false);
            }
        }
        Ok(true)
    }
}
struct MouseAreaSource {
    bounds: (f64, f64, f64, f64),
    source: String,
    events: Vec<MouseEvent>,
    repeat: bool,
    visibility: Vec<Arc<Program>>,
}

struct Canvas {
    size: (u32, u32),
    target: (u32, u32),
    position: (i64, i64),
    layers: Vec<Layer>,
    cached: Option<RgbaImage>,
    fallback: Option<fallback::Instrument>,
}

struct Layer {
    image: RgbaImage,
    text: Option<text::Text>,
    masks: Vec<Mask>,
    motion: Vec<Motion>,
    transform: DAffine2,
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

struct Update {
    program: Program,
    interval: Option<f64>,
    elapsed: f64,
    initialized: bool,
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
        self.mouse_event(MouseEvent::LeftSingle, &areas, [x, y], host)
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

    pub fn visible_mouse_areas_at(
        &self,
        x: f64,
        y: f64,
        host: &mut impl Host,
    ) -> Result<Vec<usize>> {
        let mut visible = Vec::new();
        for index in self.mouse_areas_at(x, y) {
            if self.mouse_areas[index].visible(host)? {
                visible.push(index);
            }
        }
        Ok(visible)
    }
    pub fn mouse_event(
        &self,
        event: MouseEvent,
        areas: &[usize],
        position: [f64; 2],
        host: &mut impl Host,
    ) -> Result<bool> {
        anyhow::ensure!(
            position.iter().all(|v| v.is_finite()),
            "invalid mouse position"
        );
        struct MouseHost<'a, H>(&'a mut H, MouseEvent, [f64; 2]);
        impl<H: Host> Host for MouseHost<'_, H> {
            fn read(
                &mut self,
                reference: &esp_script::Reference,
            ) -> std::result::Result<esp_script::Value, esp_script::Error> {
                if reference.category == 'M' {
                    match reference.name.as_str() {
                        "EVENT" => Ok(esp_script::Value::Text(self.1.name().into())),
                        "X" => Ok(esp_script::Value::Number(self.2[0])),
                        "Y" => Ok(esp_script::Value::Number(self.2[1])),
                        _ => self.0.read(reference),
                    }
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
        let mut handled = false;
        let mut failure = None;
        for &index in areas {
            let area = self
                .mouse_areas
                .get(index)
                .context("invalid mouse area index")?;
            let accepts = area.events.contains(&event)
                || (event == MouseEvent::DownRepeat
                    && area.repeat
                    && area.events.contains(&MouseEvent::LeftSingle));
            if accepts {
                let position = [
                    (position[0] - area.gauge_origin.0) / area.gauge_scale.0,
                    (position[1] - area.gauge_origin.1) / area.gauge_scale.1,
                ];
                let mut host = MouseHost(host, event, position);
                if event != MouseEvent::LeftRelease {
                    match area.visible(&mut host) {
                        Ok(true) => {}
                        Ok(false) => continue,
                        Err(error) => {
                            failure.get_or_insert(error);
                            continue;
                        }
                    }
                }
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

    pub fn advance(&mut self, dt: f64, host: &mut impl Host) -> Result<bool> {
        self.update_scripts(dt, host)?;
        Ok(self.update(host))
    }

    pub fn update_scripts(&mut self, dt: f64, host: &mut impl Host) -> Result<()> {
        anyhow::ensure!(dt.is_finite() && dt >= 0.0, "invalid gauge timestep");
        self.motion_elapsed += dt;
        for update in &mut self.updates {
            let elapsed = update
                .interval
                .map(|interval| update.elapsed + dt % interval);
            let due = !update.initialized
                || update
                    .interval
                    .zip(elapsed)
                    .is_none_or(|(interval, elapsed)| {
                        dt >= interval || elapsed >= interval * (1.0 - 1e-12)
                    });
            if !due {
                update.elapsed = elapsed.unwrap_or(0.0);
                continue;
            }
            update.initialized = true;
            if let Some(interval) = update.interval {
                // Missed updates are coalesced so slow frames cannot trigger an unbounded script burst.
                let remainder = elapsed.unwrap_or(0.0) % interval;
                update.elapsed = if remainder >= interval * (1.0 - 1e-12) {
                    0.0
                } else {
                    remainder
                };
            } else {
                update.elapsed = 0.0;
            }
            if let Err(error) = update.program.evaluate(host)
                && !update.reported
            {
                eprintln!("gauge update: {error}");
                update.reported = true;
            }
        }
        Ok(())
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
        let dt = std::mem::take(&mut self.motion_elapsed);
        let mut changed = false;
        for gauge in &mut self.gauges {
            let mut dirty = false;
            if let Some(instrument) = &mut gauge.fallback {
                dirty |= instrument.update(host);
            }
            for layer in &mut gauge.layers {
                for motion in &mut layer.motion {
                    motion.advance(&self.conditions, dt, host);
                }
                let transform = rotation::transform(&layer.motion, &self.conditions);
                for mask in &mut layer.masks {
                    dirty |= mask.update(&self.conditions, dt, host);
                }
                let visible = transform.is_some()
                    && layer.masks.iter().all(Mask::ready)
                    && layer
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
                dirty |= visible != layer.visible;
                if visible && let Some(text) = &mut layer.text {
                    dirty |= text.update(&mut layer.image, host);
                }
                if let Some(transform) = transform {
                    dirty |= visible && layer.transform != transform;
                    layer.transform = transform;
                }
                layer.visible = visible;
            }
            if dirty {
                gauge.cached = None;
                changed = true;
            }
        }
        if changed {
            self.compose();
        }
        changed
    }

    fn compose(&mut self) {
        let mut dirty: Option<[i64; 4]> = None;
        let width = i64::from(self.image.width());
        let height = i64::from(self.image.height());
        for gauge in &mut self.gauges {
            if gauge.cached.is_some() {
                continue;
            }
            let mut art = gauge.fallback.as_ref().map_or_else(
                || RgbaImage::new(gauge.size.0, gauge.size.1),
                |instrument| instrument.image.clone(),
            );
            for layer in &gauge.layers {
                if layer.visible {
                    mask::draw(&layer.image, layer.transform, &layer.masks, &mut art);
                }
            }
            let scaled = resize(&art, gauge.target.0, gauge.target.1, FilterType::Triangle);
            gauge.cached = Some(scaled);
            let bounds = [
                gauge.position.0.clamp(0, width),
                gauge.position.1.clamp(0, height),
                gauge
                    .position
                    .0
                    .saturating_add(i64::from(gauge.target.0))
                    .clamp(0, width),
                gauge
                    .position
                    .1
                    .saturating_add(i64::from(gauge.target.1))
                    .clamp(0, height),
            ];
            if bounds[0] < bounds[2] && bounds[1] < bounds[3] {
                dirty = Some(match dirty {
                    Some(previous) => [
                        previous[0].min(bounds[0]),
                        previous[1].min(bounds[1]),
                        previous[2].max(bounds[2]),
                        previous[3].max(bounds[3]),
                    ],
                    None => bounds,
                });
            }
        }
        let Some([left, top, right, bottom]) = dirty else {
            return;
        };
        for y in top..bottom {
            for x in left..right {
                self.image.put_pixel(x as u32, y as u32, self.background);
            }
        }
        for gauge in &self.gauges {
            let x = left.max(gauge.position.0);
            let y = top.max(gauge.position.1);
            let end_x = right.min(gauge.position.0.saturating_add(i64::from(gauge.target.0)));
            let end_y = bottom.min(gauge.position.1.saturating_add(i64::from(gauge.target.1)));
            if x >= end_x || y >= end_y {
                continue;
            }
            let image = image::imageops::crop_imm(
                gauge.cached.as_ref().expect("gauge image was composed"),
                (x - gauge.position.0) as u32,
                (y - gauge.position.1) as u32,
                (end_x - x) as u32,
                (end_y - y) as u32,
            );
            overlay(&mut self.image, &*image, x, y);
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
                let Some(instrument) = fallback::Instrument::new(gauge.reference) else {
                    return Ok(None);
                };
                let (native_width, native_height) = instrument.size();
                let target_width = dimension(gauge.width.unwrap_or(f64::from(native_width)) * f64::from(width) / dimensions.width)?;
                let target_height = dimension(gauge.height.unwrap_or(f64::from(native_height)) * f64::from(height) / layout_height)?;
                let mut raw_areas = Vec::new();
                if let Some(index) = instrument.obs_index() {
                    for (bounds, event) in [((0.0, 80.0, 64.0, 48.0), "DEC"), ((64.0, 80.0, 64.0, 48.0), "INC")] {
                        raw_areas.push(MouseAreaSource { bounds, source: format!("0 (>K:VOR{index}_OBI_{event})"), events: vec![MouseEvent::LeftSingle], repeat: true, visibility: Vec::new() });
                    }
                    raw_areas.push(MouseAreaSource {
                        bounds: (0.0, 0.0, 128.0, 128.0),
                        source: format!("(M:Event) 'WheelUp' scmp 0 == if{{ 0 (>K:VOR{index}_OBI_INC) }} (M:Event) 'WheelDown' scmp 0 == if{{ 0 (>K:VOR{index}_OBI_DEC) }}"),
                        events: vec![MouseEvent::WheelUp, MouseEvent::WheelDown], repeat: false, visibility: Vec::new(),
                    });
                }
                if let Some(index) = instrument.radio_index() {
                    for (bounds, event) in [
                        ((188.0, 20.0, 20.0, 56.0), "SWAP"),
                        ((208.0, 0.0, 24.0, 48.0), "WHOLE_DEC"),
                        ((232.0, 0.0, 24.0, 48.0), "WHOLE_INC"),
                        ((208.0, 48.0, 24.0, 48.0), "FRACT_DEC"),
                        ((232.0, 48.0, 24.0, 48.0), "FRACT_INC"),
                    ] {
                        raw_areas.push(MouseAreaSource { bounds, source: format!("0 (>K:NAV{index}_RADIO_{event})"), events: vec![MouseEvent::LeftSingle], repeat: event != "SWAP", visibility: Vec::new() });
                    }
                    for (bounds, part) in [((208.0, 0.0, 48.0, 48.0), "WHOLE"), ((208.0, 48.0, 48.0, 48.0), "FRACT")] {
                        raw_areas.push(MouseAreaSource {
                            bounds,
                            source: format!("(M:Event) 'WheelUp' scmp 0 == if{{ 0 (>K:NAV{index}_RADIO_{part}_INC) }} (M:Event) 'WheelDown' scmp 0 == if{{ 0 (>K:NAV{index}_RADIO_{part}_DEC) }}"),
                            events: vec![MouseEvent::WheelUp, MouseEvent::WheelDown], repeat: false, visibility: Vec::new(),
                        });
                    }
                }
                return Ok(Some((
                    Canvas {
                        size: (native_width, native_height),
                        target: (target_width, target_height),
                        position: (
                            (gauge.x * f64::from(width) / dimensions.width).round() as i64,
                            (gauge.y * f64::from(height) / layout_height).round() as i64,
                        ),
                        layers: Vec::new(),
                        cached: None,
                        fallback: Some(instrument),
                    },
                    conditions,
                    omitted_elements,
                    raw_areas,
                    Vec::new(),
                )));
            };
            let text = std::str::from_utf8(&xml)?;
            let document = Gauge::parse(text)?;
            let mut updates = Vec::new();
            for node in document
                .root()
                .children()
                .filter(|node| node.has_tag_name("Update"))
            {
                let frequency = node.attribute("Frequency").map(str::to_owned).or_else(|| {
                    node.children()
                        .find(|node| node.has_tag_name("Frequency"))
                        .map(esp_formats::gauge::text_content)
                });
                let frequency = frequency
                    .map(|text| text.trim().parse::<u32>())
                    .transpose()?;
                anyhow::ensure!(frequency != Some(0), "zero gauge update frequency");
                let source = node
                    .children()
                    .find(|node| node.has_tag_name("Script"))
                    .map(esp_formats::gauge::text_content)
                    .unwrap_or_else(|| esp_formats::gauge::text_content(node));
                match Program::compile(source.trim()) {
                    Ok(program) if !source.trim().is_empty() => updates.push(Update {
                        program,
                        interval: frequency.map(|frequency| 1.0 / f64::from(frequency)),
                        elapsed: 0.0,
                        initialized: false,
                        reported: false,
                    }),
                    _ => {
                        eprintln!("unsupported gauge update {}: {source}", gauge.reference);
                        omitted_elements += 1;
                    }
                }
            }
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
            let mut stack = vec![(
                document.root(),
                0.0,
                0.0,
                Vec::new(),
                Vec::new(),
                Vec::new(),
            )];
            while let Some((node, x, y, mut parents, mut motion, mut masks)) = stack.pop() {
                if node.has_tag_name("Element")
                    && node.children().any(|child| {
                        child.is_element()
                            && matches!(
                                child.tag_name().name(),
                                "Rotate" | "LeftScript" | "TopScript" | "ClipRect"
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
                        stack.push((case, x, y, requirements, motion.clone(), masks.clone()));
                    }
                    continue;
                }
                let (x, y) = if node.has_tag_name("Element") {
                    let (mut px, mut py) = position(node)?;
                    if let Some(mask) = node.children().find(|c| c.has_tag_name("MaskImage")) {
                        match Mask::parse(
                            panel,
                            module,
                            mask,
                            motion.clone(),
                            DVec2::new(px, py),
                            &conditions,
                        ) {
                            Ok(mask) => {
                                px += mask.anchor().x;
                                py += mask.anchor().y;
                                masks.push(mask);
                            }
                            Err(error) => {
                                eprintln!("gauge mask {}: {error:#}", gauge.reference);
                                omitted_elements += 1;
                                continue;
                            }
                        }
                    }
                    match Motion::element(node, (x + px, y + py), (px, py), &mut conditions) {
                        Ok(transform) => motion.push(transform),
                        Err(error) => {
                            eprintln!("gauge rotation {}: {error}", gauge.reference);
                            omitted_elements += 1;
                            continue;
                        }
                    }
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
                    let mut motion = motion.clone();
                    motion.push(Motion::image(node)?);
                    let transform = rotation::transform(&motion, &conditions);
                    layers.push(Layer {
                        image: bitmap,
                        text: None,
                        masks: masks.clone(),
                        motion,
                        transform: transform.unwrap_or(DAffine2::IDENTITY),
                        visible: parents.is_empty()
                            && transform.is_some()
                            && masks.iter().all(Mask::ready),
                        conditions: parents.clone(),
                    });
                }
                if node.has_tag_name("GaugeText") {
                    match text::Text::parse(node, (w, h)) {
                        Ok((text, image)) => {
                            let transform = rotation::transform(&motion, &conditions);
                            layers.push(Layer {
                                image,
                                text: Some(text),
                                masks: masks.clone(),
                                motion: motion.clone(),
                                transform: transform.unwrap_or(DAffine2::IDENTITY),
                                visible: parents.is_empty()
                                    && transform.is_some()
                                    && masks.iter().all(Mask::ready),
                                conditions: parents.clone(),
                            });
                        }
                        Err(error) => {
                            eprintln!("gauge text {}: {error:#}", gauge.reference);
                            omitted_elements += 1;
                        }
                    }
                }
                for child in node.children().filter(|c| c.is_element()).rev() {
                    if child.has_tag_name("Element")
                        || child.has_tag_name("Image")
                        || child.has_tag_name("Select")
                        || child.has_tag_name("GaugeText")
                    {
                        stack.push((child, x, y, parents.clone(), motion.clone(), masks.clone()));
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
                cached: None,
                fallback: None,
            };
            let mut raw_areas = Vec::new();
            collect_mouse_areas(document.root(), 0.0, 0.0, &mut raw_areas)?;
            Ok(Some((
                canvas,
                conditions,
                omitted_elements,
                raw_areas,
                updates,
            )))
        })
        .collect();
    let mut gauges = Vec::new();
    let mut conditions = Vec::new();
    let mut mouse_areas = Vec::new();
    let mut updates = Vec::new();
    let mut omitted_gauges = 0;
    let mut omitted_elements = 0;
    for loaded in loaded {
        let Some((mut canvas, mut gauge_conditions, omitted, raw_areas, mut gauge_updates)) =
            loaded?
        else {
            omitted_gauges += 1;
            continue;
        };
        for layer in &mut canvas.layers {
            for mask in &mut layer.masks {
                mask.rebase(conditions.len());
            }
            for motion in &mut layer.motion {
                motion.rebase(conditions.len());
            }
            for requirement in &mut layer.conditions {
                match requirement {
                    Requirement::Visible(index) | Requirement::Case(index, _) => {
                        *index += conditions.len()
                    }
                }
            }
        }
        conditions.append(&mut gauge_conditions);
        updates.append(&mut gauge_updates);
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
                    repeat: raw.repeat,
                    gauge_origin: (cx, cy),
                    gauge_scale: (sx, sy),
                    visibility: raw.visibility,
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
        updates,
        motion_elapsed: 0.0,
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
    let mut stack = vec![(node, offset_x, offset_y, Vec::<Arc<Program>>::new())];
    while let Some((node, offset_x, offset_y, parents)) = stack.pop() {
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
            let mut visibility = parents.clone();
            if let Some(node) = child
                .children()
                .find(|node| node.has_tag_name("Visibility"))
            {
                let source = esp_formats::gauge::text_content(node);
                match Program::compile(source.trim()) {
                    Ok(program) if !source.trim().is_empty() => visibility.push(Arc::new(program)),
                    _ => {
                        eprintln!("unsupported mouse visibility: {source}");
                        continue;
                    }
                }
            }
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
                            "leftdrag" => events.push(MouseEvent::LeftDrag),
                            "leftall" => events.extend([
                                MouseEvent::LeftSingle,
                                MouseEvent::LeftRelease,
                                MouseEvent::LeftDrag,
                            ]),
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
                let repeat = click
                    .children()
                    .find(|child| child.has_tag_name("ClickRepeat"))
                    .map(|node| {
                        let text = esp_formats::gauge::text_content(node);
                        match text.trim().to_ascii_lowercase().as_str() {
                            "true" | "yes" => Ok(true),
                            "false" | "no" => Ok(false),
                            _ => anyhow::bail!("invalid ClickRepeat"),
                        }
                    })
                    .transpose()?
                    .unwrap_or(false);
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
                        repeat,
                        visibility: visibility.clone(),
                    });
                }
            }
            stack.push((child, x, y, visibility));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_script::{Error, Reference, Value};

    #[test]
    #[ignore = "requires the gauge's installed Arial font"]
    fn gauge_text_repaints_changed_values_and_restores_hidden_background() {
        let root = tempfile::tempdir().unwrap();
        let directory = root.path().join("panel");
        std::fs::create_dir_all(directory.join("example")).unwrap();
        std::fs::write(directory.join("panel.cfg"),"[VCockpit01]\nsize_mm=100,30\npixel_size=100,30\ntexture=$text\ngauge00=example!text,0,0,100,30\nbackground_color=2,3,4").unwrap();
        std::fs::write(directory.join("example/text.xml"),r#"<Gauge><Size X="100" Y="30"/><Element><Visibility>(L:Visible)</Visibility><GaugeText><Size>100,30</Size><FontFace>Arial</FontFace><FontHeight>22</FontHeight><FontColor>cyan</FontColor><Transparent>True</Transparent><HorizontalAlign>RIGHT</HorizontalAlign><GaugeString>%((L:Value))%!4.1f!</GaugeString></GaugeText></Element></Gauge>"#).unwrap();
        struct TextHost {
            visible: f64,
            value: f64,
        }
        impl Host for TextHost {
            fn read(&mut self, r: &Reference) -> Result<Value, Error> {
                Ok(Value::Number(if r.name == "VISIBLE" {
                    self.visible
                } else {
                    self.value
                }))
            }
            fn write(&mut self, _: &Reference, _: Value) -> Result<(), Error> {
                Err(Error::Type)
            }
        }
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let mut image = load_panel(&panel, "$text").unwrap().unwrap();
        assert_eq!(image.omitted_elements, 0);
        let mut host = TextHost {
            visible: 1.0,
            value: 12.3,
        };
        assert!(image.update(&mut host));
        let before = image.image.clone();
        assert!(before.pixels().any(|p| p.0[1] > 100 && p.0[2] > 100));
        assert!(!image.update(&mut host));
        host.value = 98.7;
        assert!(image.update(&mut host));
        assert_ne!(image.image, before);
        host.visible = 0.0;
        assert!(image.update(&mut host));
        assert!(image.image.pixels().all(|p| p.0 == [2, 3, 4, 255]));
    }

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
    fn missing_flight_gauge_uses_live_fallback_and_installed_xml_takes_precedence() {
        let root = tempfile::tempdir().unwrap();
        let panel_path = root.path().join("panel");
        std::fs::create_dir_all(&panel_path).unwrap();
        std::fs::write(panel_path.join("panel.cfg"), "[VCockpit01]\nsize_mm=128,128\npixel_size=128,128\ntexture=$screen\ngauge00=Cessna172!Airspeed,0,0,128,128").unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let mut image = load_panel(&panel, "$screen").unwrap().unwrap();
        assert_eq!(image.omitted_gauges, 0);
        let mut host = TestHost(50.0);
        assert!(image.update(&mut host));
        let before = image.image.clone();
        assert!(!image.update(&mut host));
        host.0 = 120.0;
        assert!(image.update(&mut host));
        assert_ne!(before, image.image);
        let module = panel_path.join("Cessna172");
        std::fs::create_dir(&module).unwrap();
        std::fs::write(
            module.join("Airspeed.xml"),
            "<Gauge><Size X=\"128\" Y=\"128\"/></Gauge>",
        )
        .unwrap();
        let original = load_panel(&panel, "$screen").unwrap().unwrap();
        assert!(original.gauges[0].fallback.is_none());
    }

    #[test]
    fn fallback_vor_displays_signal_and_dispatches_obs_controls() {
        struct NavHost {
            needle: f64,
            events: Vec<String>,
        }
        impl Host for NavHost {
            fn read(&mut self, reference: &Reference) -> Result<Value, Error> {
                Ok(Value::Number(match reference.name.as_str() {
                    "NAV OBS:1" => 90.0,
                    "HSI CDI NEEDLE VALID" => 1.0,
                    "HSI CDI NEEDLE" => self.needle,
                    "HSI TF FLAGS" => 1.0,
                    _ => 0.0,
                }))
            }
            fn write(&mut self, reference: &Reference, _: Value) -> Result<(), Error> {
                if reference.category == 'K' {
                    self.events.push(reference.name.clone());
                }
                Ok(())
            }
        }
        let root = tempfile::tempdir().unwrap();
        let panel_path = root.path().join("panel");
        std::fs::create_dir_all(&panel_path).unwrap();
        std::fs::write(panel_path.join("panel.cfg"), "[VCockpit01]\nsize_mm=128,128\npixel_size=128,128\ntexture=$screen\ngauge00=Cessna!VOR1,0,0,128,128").unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let mut image = load_panel(&panel, "$screen").unwrap().unwrap();
        assert_eq!(image.omitted_gauges, 0);
        let mut host = NavHost {
            needle: -80.0,
            events: Vec::new(),
        };
        assert!(image.update(&mut host));
        let before = image.image.clone();
        host.needle = 80.0;
        assert!(image.update(&mut host));
        assert_ne!(before, image.image);
        assert!(image.left_click(20.0, 100.0, &mut host).unwrap());
        assert!(image.left_click(100.0, 100.0, &mut host).unwrap());
        let areas = image.mouse_areas_at(50.0, 40.0);
        assert!(
            image
                .mouse_event(MouseEvent::WheelUp, &areas, [50.0, 40.0], &mut host)
                .unwrap()
        );
        assert_eq!(
            host.events,
            ["VOR1_OBI_DEC", "VOR1_OBI_INC", "VOR1_OBI_INC"]
        );
    }

    #[test]
    fn fallback_turn_indicator_moves_in_response_to_turn_rate() {
        let root = tempfile::tempdir().unwrap();
        let panel_path = root.path().join("panel");
        std::fs::create_dir_all(&panel_path).unwrap();
        std::fs::write(panel_path.join("panel.cfg"), "[VCockpit01]\nsize_mm=128,128\npixel_size=128,128\ntexture=$screen\ngauge00=Cessna!Turn_Indicator,0,0,128,128").unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let mut image = load_panel(&panel, "$screen").unwrap().unwrap();
        let mut host = TestHost(0.0);
        assert!(image.update(&mut host));
        let level = image.image.clone();
        host.0 = 3.0;
        assert!(image.update(&mut host));
        let right = image.image.clone();
        assert_ne!(right, level);
        host.0 = -3.0;
        assert!(image.update(&mut host));
        assert_ne!(right, image.image);
    }

    #[test]
    fn fallback_radio_displays_precise_frequencies_and_dispatches_tuning() {
        struct RadioHost {
            standby: f64,
            events: Vec<String>,
        }
        impl Host for RadioHost {
            fn read(&mut self, reference: &Reference) -> Result<Value, Error> {
                Ok(Value::Number(
                    if reference.name == "NAV STANDBY FREQUENCY:1" {
                        self.standby
                    } else {
                        113_000_000.0
                    },
                ))
            }
            fn write(&mut self, reference: &Reference, _: Value) -> Result<(), Error> {
                if reference.category == 'K' {
                    self.events.push(reference.name.clone());
                }
                Ok(())
            }
        }
        let root = tempfile::tempdir().unwrap();
        let panel_path = root.path().join("panel");
        std::fs::create_dir_all(&panel_path).unwrap();
        std::fs::write(panel_path.join("panel.cfg"), "[VCockpit01]\nsize_mm=256,96\npixel_size=256,96\ntexture=$screen\ngauge00=Bendix_King_Radio!Bendix-King Radio Nav-Comm 1,0,0,256,96").unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let mut image = load_panel(&panel, "$screen").unwrap().unwrap();
        assert_eq!(image.omitted_gauges, 0);
        let mut host = RadioHost {
            standby: 110_025_000.0,
            events: Vec::new(),
        };
        assert!(image.update(&mut host));
        let before = image.image.clone();
        host.standby = 110_050_000.0;
        assert!(image.update(&mut host));
        assert_ne!(before, image.image);
        assert!(image.left_click(195.0, 40.0, &mut host).unwrap());
        assert!(image.left_click(240.0, 20.0, &mut host).unwrap());
        assert!(image.left_click(215.0, 70.0, &mut host).unwrap());
        let areas = image.mouse_areas_at(240.0, 70.0);
        assert!(
            image
                .mouse_event(MouseEvent::WheelUp, &areas, [240.0, 70.0], &mut host)
                .unwrap()
        );
        assert_eq!(
            host.events,
            [
                "NAV1_RADIO_SWAP",
                "NAV1_RADIO_WHOLE_INC",
                "NAV1_RADIO_FRACT_DEC",
                "NAV1_RADIO_FRACT_INC"
            ]
        );
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
    fn delayed_gauge_motion_uses_elapsed_simulation_time() {
        let root = tempfile::tempdir().unwrap();
        let directory = root.path().join("panel");
        let module = directory.join("example");
        std::fs::create_dir_all(&module).unwrap();
        std::fs::write(directory.join("panel.cfg"), "[VCockpit01]\nsize_mm=12,1\npixel_size=12,1\ntexture=$screen\ngauge00=example!dial,0,0,12,1").unwrap();
        std::fs::write(module.join("dial.xml"), r#"<SimBase.Document Type="AceXML"><SimGauge.Gauge><Size>12,1</Size><Element><FloatPosition>0,0</FloatPosition><Image Name="dot.bmp"/><Shift><Scale>1,0</Scale><Delay><DelayValue>2</DelayValue><DelayUnits>PIXELS_PER_SECOND</DelayUnits></Delay><Expression><Script>(L:POWER,number)</Script></Expression></Shift></Element></SimGauge.Gauge></SimBase.Document>"#).unwrap();
        let mut bytes = Vec::new();
        image::codecs::bmp::BmpEncoder::new(&mut bytes)
            .encode(&[255, 0, 0], 1, 1, image::ExtendedColorType::Rgb8)
            .unwrap();
        std::fs::write(module.join("dot.bmp"), bytes).unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let mut image = load_panel(&panel, "$screen").unwrap().unwrap();
        let mut host = TestHost(0.0);
        image.advance(0.0, &mut host).unwrap();
        assert_eq!(image.image.get_pixel(0, 0).0, [255, 0, 0, 255]);
        host.0 = 10.0;
        image.update_scripts(0.25, &mut host).unwrap();
        image.update_scripts(0.75, &mut host).unwrap();
        assert!(image.update(&mut host));
        assert_eq!(image.image.get_pixel(2, 0).0, [255, 0, 0, 255]);
        assert!(!image.update(&mut host));
        image.advance(4.0, &mut host).unwrap();
        assert_eq!(image.image.get_pixel(10, 0).0, [255, 0, 0, 255]);
    }

    #[test]
    fn repaint_preserves_overlapping_gauges_and_clips_offscreen_artwork() {
        let root = tempfile::tempdir().unwrap();
        let directory = root.path().join("panel");
        let module = directory.join("example");
        std::fs::create_dir_all(&module).unwrap();
        std::fs::write(directory.join("panel.cfg"), "[VCockpit01]\nsize_mm=4,1\npixel_size=4,1\ntexture=$screen\nbackground_color=25,25,25\ngauge00=example!back,-1,0\ngauge01=example!front,0,0").unwrap();
        std::fs::write(module.join("back.xml"), r#"<Gauge><Size X="3" Y="1"/><Element><Visibility>(L:POWER,bool)</Visibility><Image Name="back.bmp"/></Element></Gauge>"#).unwrap();
        std::fs::write(
            module.join("front.xml"),
            r#"<Gauge><Size X="3" Y="1"/><Image Name="front.bmp" Transparent="True"/></Gauge>"#,
        )
        .unwrap();
        for (name, pixels) in [
            ("back.bmp", [255, 0, 0, 255, 0, 0, 255, 0, 0]),
            ("front.bmp", [0, 0, 0, 0, 0, 255, 0, 0, 0]),
        ] {
            let mut bytes = Vec::new();
            image::codecs::bmp::BmpEncoder::new(&mut bytes)
                .encode(&pixels, 3, 1, image::ExtendedColorType::Rgb8)
                .unwrap();
            std::fs::write(module.join(name), bytes).unwrap();
        }
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let mut image = load_panel(&panel, "$screen").unwrap().unwrap();
        let mut host = TestHost(1.0);
        assert!(image.update(&mut host));
        assert_eq!(image.image.get_pixel(0, 0).0, [255, 0, 0, 255]);
        assert_eq!(image.image.get_pixel(1, 0).0, [0, 0, 255, 255]);
        assert_eq!(image.image.get_pixel(3, 0).0, [25, 25, 25, 255]);
        host.0 = 0.0;
        assert!(image.update(&mut host));
        assert_eq!(image.image.get_pixel(0, 0).0, [25, 25, 25, 255]);
        assert_eq!(image.image.get_pixel(1, 0).0, [0, 0, 255, 255]);
        assert!(!image.update(&mut host));
    }

    #[test]
    fn collects_mouse_areas_with_panel_coordinates() {
        let root = tempfile::tempdir().unwrap();
        let panel_path = root.path().join("panel");
        let module = panel_path.join("example");
        std::fs::create_dir_all(&module).unwrap();
        std::fs::write(
            panel_path.join("panel.cfg"),
            "[VCockpit01]\nsize_mm=4,4\npixel_size=8,8\ntexture=$screen\ngauge00=example!dial,1,1,2,2",
        )
        .unwrap();
        std::fs::write(
            module.join("dial.xml"),
            r#"<SimBase.Document Type="AceXML"><SimGauge.Gauge><Size>2,2</Size>
            <Image Name="bg.bmp"/>
            <MouseArea id="Outer"><FloatPosition>0,0</FloatPosition><Size>2,2</Size>
                <MouseArea id="Inner"><FloatPosition>1,0</FloatPosition><Size>1,1</Size>
                    <MouseClick id="MouseClick"><ClickType>LeftSingle+LeftDrag</ClickType>
                    <Script>(M:X) (&gt;K:X) (M:Y) (&gt;K:Y)</Script></MouseClick>
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
        assert_eq!((area.x, area.y, area.w, area.h), (4.0, 2.0, 2.0, 2.0));
        struct Coordinates(Vec<f64>);
        impl Host for Coordinates {
            fn read(&mut self, _: &Reference) -> Result<Value, Error> {
                Err(Error::Type)
            }
            fn write(&mut self, _: &Reference, value: Value) -> Result<(), Error> {
                self.0.push(value.number()?);
                Ok(())
            }
        }
        let mut host = Coordinates(Vec::new());
        result.left_click(5.0, 3.0, &mut host).unwrap();
        assert_eq!(host.0, [1.5, 0.5]);
        result
            .mouse_event(MouseEvent::LeftDrag, &[0], [7.0, 5.0], &mut host)
            .unwrap();
        assert_eq!(host.0, [1.5, 0.5, 2.5, 1.5]);
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
                    repeat: raw.repeat,
                    gauge_origin: (0.0, 0.0),
                    gauge_scale: (1.0, 1.0),
                    visibility: raw.visibility,
                })
                .collect(),
            gauges: vec![],
            conditions: vec![],
            updates: vec![],
            motion_elapsed: 0.0,
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
        assert!(panel.left_click(f64::NAN, 3.0, &mut host).is_err());
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
                repeat: false,
                gauge_origin: (0.0, 0.0),
                gauge_scale: (1.0, 1.0),
                visibility: vec![],
            }],
            gauges: vec![], conditions: vec![], background: Rgba([0; 4]),
            updates: vec![], motion_elapsed: 0.0,
        };
        let captured = panel.mouse_areas_at(2.0, 2.0);
        let mut host = Values::default();
        panel
            .mouse_event(MouseEvent::LeftSingle, &captured, [2.0, 2.0], &mut host)
            .unwrap();
        assert_eq!(host.0["PRESSED"], Value::Number(1.0));
        assert!(panel.mouse_areas_at(5.0, 5.0).is_empty());
        panel
            .mouse_event(MouseEvent::LeftRelease, &captured, [5.0, 5.0], &mut host)
            .unwrap();
        assert_eq!(host.0["PRESSED"], Value::Number(0.0));
        panel
            .mouse_event(MouseEvent::WheelUp, &captured, [2.0, 2.0], &mut host)
            .unwrap();
        assert_eq!(host.0["WHEEL"], Value::Number(1.0));
        panel
            .mouse_event(MouseEvent::WheelDown, &captured, [2.0, 2.0], &mut host)
            .unwrap();
        assert_eq!(host.0["WHEEL"], Value::Number(-1.0));
        assert!(
            panel
                .mouse_event(MouseEvent::WheelUp, &[usize::MAX], [2.0, 2.0], &mut host)
                .is_err()
        );
    }

    #[test]
    fn only_repeat_enabled_presses_receive_down_repeat() {
        let gauge = Gauge::parse(r#"<Gauge><MouseArea><Size X="2" Y="2"/>
            <MouseClick><ClickRepeat>True</ClickRepeat><Script>(M:Event) (&gt;L:EVENT)</Script></MouseClick>
            <MouseClick><ClickRepeat>False</ClickRepeat><KeyEvent>ONCE</KeyEvent></MouseClick>
            </MouseArea></Gauge>"#).unwrap();
        let mut raw = Vec::new();
        collect_mouse_areas(gauge.root(), 0.0, 0.0, &mut raw).unwrap();
        assert!(raw[0].repeat);
        assert!(!raw[1].repeat);
        struct Writes(Vec<(String, Value)>);
        impl Host for Writes {
            fn read(&mut self, _: &Reference) -> Result<Value, Error> {
                Err(Error::Type)
            }
            fn write(&mut self, reference: &Reference, value: Value) -> Result<(), Error> {
                self.0.push((reference.name.clone(), value));
                Ok(())
            }
        }
        let panel = PanelImage {
            image: RgbaImage::new(2, 2),
            omitted_gauges: 0,
            omitted_elements: 0,
            mouse_areas: raw
                .into_iter()
                .map(|area| PanelMouseArea {
                    x: area.bounds.0,
                    y: area.bounds.1,
                    w: area.bounds.2,
                    h: area.bounds.3,
                    script: Program::compile(&area.source).unwrap(),
                    events: area.events,
                    repeat: area.repeat,
                    gauge_origin: (0.0, 0.0),
                    gauge_scale: (1.0, 1.0),
                    visibility: area.visibility,
                })
                .collect(),
            gauges: vec![],
            conditions: vec![],
            updates: vec![],
            motion_elapsed: 0.0,
            background: Rgba([0; 4]),
        };
        let mut host = Writes(Vec::new());
        panel
            .mouse_event(MouseEvent::DownRepeat, &[0, 1], [1.0, 1.0], &mut host)
            .unwrap();
        assert_eq!(host.0, [("EVENT".into(), Value::Text("DownRepeat".into()))]);
        assert!(
            !panel
                .mouse_event(MouseEvent::DownRepeat, &[1], [1.0, 1.0], &mut host)
                .unwrap()
        );
        assert_eq!(host.0.len(), 1);
    }

    #[test]
    fn hidden_parent_blocks_input_but_captured_release_clears_state() {
        let gauge = Gauge::parse(r#"<Gauge><MouseArea><Size X="2" Y="2"/>
            <Visibility>(L:Visible,bool)</Visibility>
            <MouseArea><Size X="2" Y="2"/><Visibility>1</Visibility>
            <MouseClick><ClickType>LeftSingle+LeftRelease+WheelUp</ClickType>
            <ClickRepeat>True</ClickRepeat><Script>(M:Event) 'LeftRelease' scmp 0 == if{ 0 (&gt;L:Pressed) } els{ 1 (&gt;L:Pressed) }</Script>
            </MouseClick></MouseArea></MouseArea></Gauge>"#).unwrap();
        let mut raw = Vec::new();
        collect_mouse_areas(gauge.root(), 0.0, 0.0, &mut raw).unwrap();
        assert_eq!(raw.len(), 1);
        assert_eq!(raw[0].visibility.len(), 2);
        let panel = PanelImage {
            image: RgbaImage::new(2, 2),
            omitted_gauges: 0,
            omitted_elements: 0,
            mouse_areas: raw
                .into_iter()
                .map(|area| PanelMouseArea {
                    x: area.bounds.0,
                    y: area.bounds.1,
                    w: area.bounds.2,
                    h: area.bounds.3,
                    script: Program::compile(&area.source).unwrap(),
                    events: area.events,
                    repeat: area.repeat,
                    gauge_origin: (0.0, 0.0),
                    gauge_scale: (1.0, 1.0),
                    visibility: area.visibility,
                })
                .collect(),
            gauges: vec![],
            conditions: vec![],
            updates: vec![],
            motion_elapsed: 0.0,
            background: Rgba([0; 4]),
        };
        struct State {
            visible: f64,
            pressed: f64,
        }
        impl Host for State {
            fn read(&mut self, reference: &Reference) -> Result<Value, Error> {
                match reference.name.as_str() {
                    "VISIBLE" => Ok(Value::Number(self.visible)),
                    "PRESSED" => Ok(Value::Number(self.pressed)),
                    _ => Err(Error::Variable(reference.name.clone())),
                }
            }
            fn write(&mut self, reference: &Reference, value: Value) -> Result<(), Error> {
                assert_eq!(reference.name, "PRESSED");
                self.pressed = value.number()?;
                Ok(())
            }
        }
        let mut host = State {
            visible: 0.0,
            pressed: 0.0,
        };
        assert!(
            panel
                .visible_mouse_areas_at(1.0, 1.0, &mut host)
                .unwrap()
                .is_empty()
        );
        for event in [
            MouseEvent::LeftSingle,
            MouseEvent::LeftDrag,
            MouseEvent::WheelUp,
            MouseEvent::DownRepeat,
        ] {
            assert!(
                !panel
                    .mouse_event(event, &[0], [1.0, 1.0], &mut host)
                    .unwrap()
            );
            assert_eq!(host.pressed, 0.0);
        }
        host.visible = 1.0;
        let captured = panel.visible_mouse_areas_at(1.0, 1.0, &mut host).unwrap();
        assert_eq!(captured, [0]);
        assert!(
            panel
                .mouse_event(MouseEvent::LeftSingle, &captured, [1.0, 1.0], &mut host)
                .unwrap()
        );
        assert_eq!(host.pressed, 1.0);
        host.visible = 0.0;
        assert!(
            panel
                .mouse_event(MouseEvent::LeftRelease, &captured, [3.0, 3.0], &mut host)
                .unwrap()
        );
        assert_eq!(host.pressed, 0.0);
        host.visible = f64::NAN;
        assert!(panel.visible_mouse_areas_at(1.0, 1.0, &mut host).is_err());
        assert!(
            panel
                .mouse_event(MouseEvent::WheelUp, &[0], [1.0, 1.0], &mut host)
                .is_err()
        );
        assert_eq!(host.pressed, 0.0);
    }

    #[test]
    fn invalid_parent_visibility_omits_its_controls() {
        let gauge = Gauge::parse(r#"<Gauge><MouseArea><Visibility>unsupported</Visibility>
            <MouseArea><Size X="1" Y="1"/><MouseClick><KeyEvent>BAD</KeyEvent></MouseClick></MouseArea>
            </MouseArea><MouseArea><Size X="1" Y="1"/><MouseClick><KeyEvent>GOOD</KeyEvent></MouseClick></MouseArea></Gauge>"#).unwrap();
        let mut raw = Vec::new();
        collect_mouse_areas(gauge.root(), 0.0, 0.0, &mut raw).unwrap();
        assert_eq!(raw.len(), 1);
        assert_eq!(raw[0].source, "(>K:GOOD)");
    }

    #[test]
    fn runs_xml_updates_before_artwork_and_respects_frequency() {
        let root = tempfile::tempdir().unwrap();
        let directory = root.path().join("panel");
        let module = directory.join("example");
        std::fs::create_dir_all(&module).unwrap();
        std::fs::write(directory.join("panel.cfg"), "[VCockpit01]\nsize_mm=1,1\npixel_size=1,1\ntexture=$test\ngauge00=example!update,0,0,1,1").unwrap();
        std::fs::write(
            module.join("update.xml"),
            r#"<Gauge><Size X="1" Y="1"/>
            <Update Frequency="2">(L:Ticks) ++ (&gt;L:Ticks) 1 (&gt;L:Visible)</Update>
            <Update>(L:Frames) ++ (&gt;L:Frames)</Update>
            <Element><Visibility>(L:Visible)</Visibility><Image Name="lamp.bmp"/></Element>
            </Gauge>"#,
        )
        .unwrap();
        let mut data = Vec::new();
        image::codecs::bmp::BmpEncoder::new(&mut data)
            .encode(&[255, 0, 0], 1, 1, image::ExtendedColorType::Rgb8)
            .unwrap();
        std::fs::write(module.join("lamp.bmp"), data).unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let mut image = load_panel(&panel, "$test").unwrap().unwrap();
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
        let mut host = Values::default();
        assert_eq!(image.image.get_pixel(0, 0).0, [0, 0, 0, 255]);
        assert!(image.advance(0.0, &mut host).unwrap());
        assert_eq!(image.image.get_pixel(0, 0).0, [255, 0, 0, 255]);
        assert_eq!(host.0["TICKS"], Value::Number(1.0));
        image.advance(0.25, &mut host).unwrap();
        assert_eq!(host.0["TICKS"], Value::Number(1.0));
        image.advance(0.25, &mut host).unwrap();
        assert_eq!(host.0["TICKS"], Value::Number(2.0));
        assert_eq!(host.0["FRAMES"], Value::Number(3.0));
        image.advance(10.0, &mut host).unwrap();
        assert_eq!(host.0["TICKS"], Value::Number(3.0));
        for dt in [-1.0, f64::NAN, f64::INFINITY] {
            assert!(image.advance(dt, &mut host).is_err());
            assert_eq!(host.0["FRAMES"], Value::Number(4.0));
        }
        image.advance(f64::MAX, &mut host).unwrap();
        image.advance(f64::MAX, &mut host).unwrap();
        assert!(
            image
                .updates
                .iter()
                .all(|update| update.elapsed.is_finite())
        );
        let mut image = load_panel(&panel, "$test").unwrap().unwrap();
        let mut host = Values::default();
        image.update_scripts(0.0, &mut host).unwrap();
        for _ in 0..60 {
            image.update_scripts(1.0 / 60.0, &mut host).unwrap();
        }
        assert_eq!(host.0["TICKS"], Value::Number(3.0));
        assert_eq!(host.0["FRAMES"], Value::Number(61.0));
        assert_eq!(image.image.get_pixel(0, 0).0, [0, 0, 0, 255]);
        assert!(image.update(&mut host));
        assert_eq!(host.0["FRAMES"], Value::Number(61.0));
        assert_eq!(image.image.get_pixel(0, 0).0, [255, 0, 0, 255]);
    }

    #[test]
    fn failed_updates_do_not_stop_other_scripts() {
        let mut panel = PanelImage {
            image: RgbaImage::new(1, 1),
            omitted_gauges: 0,
            omitted_elements: 0,
            mouse_areas: vec![],
            gauges: vec![],
            conditions: vec![],
            background: Rgba([0; 4]),
            motion_elapsed: 0.0,
            updates: ["(A:MISSING,bool)", "(>K:TEST)"]
                .into_iter()
                .map(|source| Update {
                    program: Program::compile(source).unwrap(),
                    interval: None,
                    elapsed: 0.0,
                    initialized: false,
                    reported: false,
                })
                .collect(),
        };
        struct Events(usize);
        impl Host for Events {
            fn read(&mut self, _: &Reference) -> Result<Value, Error> {
                Err(Error::Type)
            }
            fn write(&mut self, _: &Reference, _: Value) -> Result<(), Error> {
                self.0 += 1;
                Ok(())
            }
        }
        let mut host = Events(0);
        panel.advance(0.0, &mut host).unwrap();
        assert_eq!(host.0, 1);
        assert!(panel.updates[0].reported);
        panel.advance(0.1, &mut host).unwrap();
        assert_eq!(host.0, 2);
    }

    #[test]
    fn reads_ace_update_frequency_and_rejects_invalid_values() {
        let root = tempfile::tempdir().unwrap();
        let directory = root.path().join("panel");
        let module = directory.join("example");
        std::fs::create_dir_all(&module).unwrap();
        std::fs::write(directory.join("panel.cfg"), "[VCockpit01]\nsize_mm=1,1\npixel_size=1,1\ntexture=$test\ngauge00=example!update,0,0,1,1").unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        for frequency in ["0", "-1", "NaN", "1.5", "4294967296"] {
            std::fs::write(module.join("update.xml"), format!("<SimBase.Document Type='AceXML'><SimGauge.Gauge><Size>1,1</Size><Update><Frequency>{frequency}</Frequency><Script>1 (&gt;L:VALUE)</Script></Update></SimGauge.Gauge></SimBase.Document>")).unwrap();
            assert!(load_panel(&panel, "$test").is_err(), "{frequency}");
        }
        std::fs::write(module.join("update.xml"), "<SimBase.Document Type='AceXML'><SimGauge.Gauge><Size>1,1</Size><Update><Frequency>2</Frequency><Script>1 (&gt;L:VALUE)</Script></Update></SimGauge.Gauge></SimBase.Document>").unwrap();
        let image = load_panel(&panel, "$test").unwrap().unwrap();
        assert_eq!(image.updates.len(), 1);
        assert_eq!(image.updates[0].interval, Some(0.5));
    }
    #[test]
    fn rotates_nested_images_about_their_axis_and_skips_unchanged_frames() {
        let root = tempfile::tempdir().unwrap();
        let directory = root.path().join("panel");
        let module = directory.join("example");
        std::fs::create_dir_all(&module).unwrap();
        std::fs::write(
        directory.join("panel.cfg"),
        "[VCockpit01]\nsize_mm=9,9\npixel_size=9,9\ntexture=$screen\ngauge00=example!needle,0,0",
    )
    .unwrap();
        std::fs::write(module.join("needle.xml"), r#"<Gauge><Size X="9" Y="9"/><Element><Position X="1" Y="1"/><Element><Position X="3" Y="3"/><Image Name="needle.bmp"><Transparent>True</Transparent><Axis X="1" Y="3"/></Image><Rotation><PointsTo>NORTH</PointsTo><Expression><Minimum>0</Minimum><Maximum>1.5707963267948966</Maximum><Script>(L:Angle,number)</Script></Expression></Rotation></Element></Element></Gauge>"#).unwrap();
        let mut pixels = vec![0; 3 * 4 * 3];
        for y in 0..4 {
            pixels[(y * 3 + 1) * 3] = 255;
        }
        let mut bytes = Vec::new();
        image::codecs::bmp::BmpEncoder::new(&mut bytes)
            .encode(&pixels, 3, 4, image::ExtendedColorType::Rgb8)
            .unwrap();
        std::fs::write(module.join("needle.bmp"), bytes).unwrap();
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let mut image = load_panel(&panel, "$screen").unwrap().unwrap();
        assert_eq!(image.omitted_elements, 0);
        let mut host = TestHost(0.0);
        assert!(image.update(&mut host));
        assert_eq!(image.image.get_pixel(4, 1).0, [255, 0, 0, 255]);
        assert!(!image.update(&mut host));
        host.0 = std::f64::consts::FRAC_PI_2;
        assert!(image.update(&mut host));
        assert!(image.image.get_pixel(7, 4)[0] > 240);
        assert_eq!(image.image.get_pixel(4, 1).0, [0, 0, 0, 255]);
        assert!(!image.update(&mut host));
        host.0 = 100.0;
        assert!(
            !image.update(&mut host),
            "rotation must clamp at the expression maximum"
        );
    }

    #[test]
    fn shifts_artwork_within_fixed_mask_and_preserves_mask_colors() {
        let root = tempfile::tempdir().unwrap();
        let directory = root.path().join("panel");
        let module = directory.join("example");
        std::fs::create_dir_all(&module).unwrap();
        std::fs::write(directory.join("panel.cfg"), "[VCockpit01]\nsize_mm=8,8\npixel_size=8,8\ntexture=$screen\nbackground_color=10,20,30\ngauge00=example!masked,0,0").unwrap();
        std::fs::write(module.join("masked.xml"), r#"<Gauge><Size X="8" Y="8"/><Element><FloatPosition>1,1</FloatPosition><Element><FloatPosition>3,3</FloatPosition><Image Name="strip.bmp"/><MaskImage Name="mask.bmp"><Axis>1,0</Axis></MaskImage><Shift><Scale>1,0</Scale><Expression><Minimum>-2</Minimum><Maximum>0</Maximum><Script>(L:Value)</Script></Expression><FailureTable><Failure/></FailureTable></Shift></Element></Element></Gauge>"#).unwrap();
        for (name, pixels) in [
            ("strip.bmp", [255, 0, 0, 0, 255, 0, 0, 0, 255]),
            ("mask.bmp", [0, 0, 0, 1, 1, 1, 255, 255, 0]),
        ] {
            let mut bytes = Vec::new();
            image::codecs::bmp::BmpEncoder::new(&mut bytes)
                .encode(&pixels, 3, 1, image::ExtendedColorType::Rgb8)
                .unwrap();
            std::fs::write(module.join(name), bytes).unwrap();
        }
        let panel = Panel::load(root.path(), None).unwrap().unwrap();
        let mut image = load_panel(&panel, "$screen").unwrap().unwrap();
        assert_eq!(image.omitted_elements, 0);
        let mut host = TestHost(0.0);
        assert!(image.update(&mut host));
        assert_eq!(image.image.get_pixel(5, 4).0, [255, 0, 0, 255]);
        assert_eq!(image.image.get_pixel(6, 4).0, [255, 255, 0, 255]);
        assert_eq!(image.image.get_pixel(7, 4).0, [10, 20, 30, 255]);
        host.0 = -1.0;
        assert!(image.update(&mut host));
        assert_eq!(image.image.get_pixel(4, 4).0, [10, 20, 30, 255]);
        assert_eq!(image.image.get_pixel(5, 4).0, [0, 255, 0, 255]);
        assert_eq!(image.image.get_pixel(6, 4).0, [255, 255, 0, 255]);
        assert!(!image.update(&mut host));
        host.0 = -2.0;
        assert!(image.update(&mut host));
        assert_eq!(image.image.get_pixel(5, 4).0, [0, 0, 255, 255]);
        host.0 = -10.0;
        assert!(!image.update(&mut host));
    }
}

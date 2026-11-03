use anyhow::{Context, Result};
use esp_script::{Host, Program, Reference};
use glam::{DAffine2, DMat2, DVec2};
use image::{Rgba, RgbaImage, imageops::overlay};
use imageproc::geometric_transformations::{Border, Interpolation, Projection, warp_into};
use roxmltree::Node;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

use super::Condition;

#[derive(Clone)]
pub(super) struct Motion {
    offset: DVec2,
    rotation: Option<Rotation>,
    shift: Option<Shift>,
}

#[derive(Clone)]
struct Shift {
    expression: usize,
    scale: DVec2,
    origin: DVec2,
    table: Vec<(f64, DVec2)>,
    delay: Option<Delay>,
    delayed_offset: Option<DVec2>,
    failure: FailureState<DVec2>,
}

#[derive(Clone)]
struct Rotation {
    expression: Option<usize>,
    points_to: f64,
    table: Vec<(f64, f64)>,
    delay: Option<Delay>,
    failure: FailureState<f64>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum FailureAction {
    Freeze,
    Zero,
    NoDraw,
}

#[derive(Clone)]
struct FailureRule {
    variable: &'static str,
    inverted: bool,
    action: FailureAction,
}

#[derive(Clone)]
struct FailureState<T> {
    rules: Vec<FailureRule>,
    action: Option<FailureAction>,
    last: Option<T>,
}

impl<T> FailureState<T> {
    fn parse(node: Node<'_, '_>) -> Result<Self> {
        let mut rules = Vec::new();
        for failure in node
            .children()
            .filter(|c| c.has_tag_name("FailureTable"))
            .flat_map(|t| t.children().filter(|c| c.has_tag_name("Failure")))
        {
            let key = child_text(failure, "Fail_Key")
                .unwrap_or("NONE")
                .trim()
                .to_ascii_uppercase();
            if key.is_empty() || key == "NONE" {
                continue;
            }
            let (variable, inverted) = match key.as_str() {
                "SYSTEM_ELECTRICAL_PANELS" => ("CIRCUIT GENERAL PANEL ON", true),
                "SYSTEM_PITOT_STATIC" => ("PARTIAL PANEL PITOT", false),
                "SYSTEM_VACUUM" => ("PARTIAL PANEL VACUUM", false),
                "SYSTEM_ENGINE" => ("PARTIAL PANEL ENGINE", false),
                "GAUGE_ADF" => ("PARTIAL PANEL ADF", false),
                "GAUGE_AIRSPEED" => ("PARTIAL PANEL AIRSPEED", false),
                "GAUGE_ALTIMETER" => ("PARTIAL PANEL ALTIMETER", false),
                "GAUGE_COMMUNICATIONS" => ("PARTIAL PANEL COMM", false),
                "GAUGE_FUEL_INDICATORS" => ("PARTIAL PANEL FUEL INDICATOR", false),
                "GAUGE_GYRO_HEADING" => ("PARTIAL PANEL HEADING", false),
                "GAUGE_MAGNETIC_COMPASS" => ("PARTIAL PANEL COMPASS", false),
                "GAUGE_NAVIGATION_VOR1" | "GAUGE_NAVIGATION_VOR2" => ("PARTIAL PANEL NAV", false),
                "GAUGE_TRANSPONDER" => ("PARTIAL PANEL TRANSPONDER", false),
                "GAUGE_TURN_COORDINATOR" => ("PARTIAL PANEL TURN COORDINATOR", false),
                "GAUGE_VERTICAL_SPEED" => ("PARTIAL PANEL VERTICAL VELOCITY", false),
                "GAUGE_ELECTRICAL_PANEL" => ("PARTIAL PANEL ELECTRICAL", false),
                "GAUGE_ELECTRICAL_AVIONICS" => ("PARTIAL PANEL AVIONICS", false),
                _ => anyhow::bail!("unsupported gauge motion failure {key}"),
            };
            let action = match child_text(failure, "Fail_Action")
                .unwrap_or("None")
                .trim()
                .to_ascii_uppercase()
                .as_str()
            {
                "NONE" | "" => continue,
                "FREEZE" => FailureAction::Freeze,
                "ZERO" => FailureAction::Zero,
                "NODRAW" => FailureAction::NoDraw,
                action => anyhow::bail!("unsupported gauge failure action {action}"),
            };
            rules.push(FailureRule {
                variable,
                inverted,
                action,
            });
        }
        Ok(Self {
            rules,
            action: None,
            last: None,
        })
    }

    fn update(&mut self, host: &mut impl Host) {
        self.action = self.rules.iter().find_map(|rule| {
            let value = host
                .read(&Reference::new(
                    'A',
                    rule.variable,
                    Some(if rule.inverted { "bool" } else { "enum" }),
                ))
                .ok()?
                .number()
                .ok()?;
            (value.is_finite() && ((value == 0.0) == rule.inverted)).then_some(rule.action)
        });
    }
}

#[derive(Clone)]
struct Delay {
    rate: f64,
    units: DelayUnits,
    value: Option<f64>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum DelayUnits {
    Value,
    Angle,
    Pixels,
}

impl Delay {
    fn parse(node: Node<'_, '_>, rotation: bool) -> Result<Option<Self>> {
        let Some(node) = node.children().find(|c| c.has_tag_name("Delay")) else {
            return Ok(None);
        };
        let rate = number(node, "DelayValue")?.context("missing gauge delay value")?;
        anyhow::ensure!(rate.is_finite() && rate > 0.0, "invalid gauge delay value");
        let units = child_text(node, "DelayUnits")
            .context("missing gauge delay units")?
            .trim()
            .to_ascii_uppercase();
        let (units, rate) = match units.as_str() {
            "VALUE_PER_SECOND" => (DelayUnits::Value, rate),
            "DEGREES_PER_SECOND" if rotation => (DelayUnits::Angle, rate.to_radians()),
            "RADIANS_PER_SECOND" if rotation => (DelayUnits::Angle, rate),
            "PIXELS_PER_SECOND" if !rotation => (DelayUnits::Pixels, rate),
            _ => anyhow::bail!("unsupported gauge delay units {units}"),
        };
        Ok(Some(Self {
            rate,
            units,
            value: None,
        }))
    }

    fn advance(&mut self, target: f64, dt: f64) {
        self.value = Some(match self.value {
            Some(value) => value + (target - value).clamp(-self.rate * dt, self.rate * dt),
            None => target,
        });
    }
}

impl Motion {
    pub fn element(
        node: Node<'_, '_>,
        centre: (f64, f64),
        offset: (f64, f64),
        conditions: &mut Vec<Condition>,
    ) -> Result<Self> {
        let rotation = node
            .children()
            .find(|c| c.has_tag_name("Rotation"))
            .map(|node| Rotation::parse(node, DVec2::from(centre), conditions))
            .transpose()?;
        let shift = node
            .children()
            .find(|c| c.has_tag_name("Shift"))
            .map(|node| Shift::parse(node, DVec2::from(offset), conditions))
            .transpose()?;
        Ok(Self {
            offset: DVec2::from(offset),
            rotation,
            shift,
        })
    }

    pub fn image(node: Node<'_, '_>) -> Result<Self> {
        let offset = axis(node)?;
        Ok(Self {
            offset: -offset,
            rotation: None,
            shift: None,
        })
    }

    pub fn rebase(&mut self, offset: usize) {
        if let Some(index) = self.rotation.as_mut().and_then(|r| r.expression.as_mut()) {
            *index += offset;
        }
        if let Some(shift) = &mut self.shift {
            shift.expression += offset;
        }
    }

    pub fn translation(offset: DVec2) -> Self {
        Self {
            offset,
            rotation: None,
            shift: None,
        }
    }

    pub fn advance(&mut self, conditions: &[Condition], dt: f64, host: &mut impl Host) {
        if let Some(rotation) = &mut self.rotation {
            rotation.advance(conditions, dt, host);
        }
        if let Some(shift) = &mut self.shift {
            shift.advance(conditions, dt, host);
        }
    }
}

pub(super) fn axis(node: Node<'_, '_>) -> Result<DVec2> {
    node.children()
        .find(|c| c.has_tag_name("Axis"))
        .map(point)
        .transpose()
        .map(|p| p.unwrap_or(DVec2::ZERO))
}

fn expression(node: Node<'_, '_>, conditions: &mut Vec<Condition>) -> Result<usize> {
    let source = child_text(node, "Script").context("missing motion expression")?;
    anyhow::ensure!(!source.trim().is_empty(), "empty motion expression");
    let program = Program::compile(source.trim())?;
    let minimum = number(node, "Minimum")?.filter(|v| *v != -9_999_999.0);
    let maximum = number(node, "Maximum")?.filter(|v| *v != -9_999_999.0);
    anyhow::ensure!(
        minimum.zip(maximum).is_none_or(|(min, max)| min <= max),
        "invalid motion bounds"
    );
    let index = conditions.len();
    conditions.push(Condition {
        program,
        value: None,
        minimum,
        maximum,
        reported: false,
    });
    Ok(index)
}

impl Shift {
    fn parse(node: Node<'_, '_>, origin: DVec2, conditions: &mut Vec<Condition>) -> Result<Self> {
        let failure = FailureState::parse(node)?;
        let scale = node
            .children()
            .find(|c| c.has_tag_name("Scale"))
            .map(point)
            .transpose()?
            .unwrap_or(DVec2::ZERO);
        let mut table = Vec::new();
        if let Some(entries) = node
            .children()
            .find(|c| c.has_tag_name("NonlinearityTable"))
        {
            for entry in entries
                .children()
                .filter(|c| c.has_tag_name("NonlinearityEntry"))
            {
                let value =
                    number(entry, "ExpressionResult")?.context("missing shift table value")?;
                let position = point(
                    entry
                        .children()
                        .find(|c| c.has_tag_name("FloatPosition"))
                        .context("missing shift table position")?,
                )?;
                table.push((value, position));
            }
            anyhow::ensure!(!table.is_empty(), "empty shift table");
            let increasing = table.last().unwrap().0 > table[0].0;
            anyhow::ensure!(
                table.windows(2).all(|pair| if increasing {
                    pair[1].0 > pair[0].0
                } else {
                    pair[1].0 < pair[0].0
                }),
                "non-monotonic shift table"
            );
        } else {
            anyhow::ensure!(
                node.children().any(|c| c.has_tag_name("Scale")),
                "missing shift scale"
            );
        }
        let expression = expression(
            node.children()
                .find(|c| c.has_tag_name("Expression"))
                .context("missing shift expression")?,
            conditions,
        )?;
        let delay = Delay::parse(node, false)?;
        Ok(Self {
            expression,
            scale,
            origin,
            table,
            delay,
            delayed_offset: None,
            failure,
        })
    }

    fn advance(&mut self, conditions: &[Condition], dt: f64, host: &mut impl Host) {
        self.failure.update(host);
        if self.failure.action == Some(FailureAction::Freeze) {
            if self.failure.last.is_none() {
                self.failure.last = Some(self.offset_for(0.0));
            }
            return;
        }
        if self.failure.action == Some(FailureAction::NoDraw) {
            return;
        }
        if self.failure.action == Some(FailureAction::Zero) {
            self.failure.last = Some(self.offset_for(0.0));
            return;
        }
        let Some(value) = conditions[self.expression].value else {
            return;
        };
        if self
            .delay
            .as_ref()
            .is_some_and(|delay| delay.units == DelayUnits::Pixels)
        {
            let target = self.offset_for(value);
            let rate = self.delay.as_ref().unwrap().rate;
            let current = self.delayed_offset.unwrap_or(target);
            let difference = target - current;
            let distance = difference.length();
            self.delayed_offset = Some(if distance > rate * dt && distance > 0.0 {
                current + difference * (rate * dt / distance)
            } else {
                target
            });
        } else if let Some(delay) = &mut self.delay {
            delay.advance(value, dt);
        }
        self.failure.last = self.offset(conditions);
    }

    fn offset(&self, conditions: &[Condition]) -> Option<DVec2> {
        match self.failure.action {
            Some(FailureAction::Freeze | FailureAction::Zero) => return self.failure.last,
            Some(FailureAction::NoDraw) => return None,
            None => {}
        }
        let value = if let Some(delay) = &self.delay {
            if delay.units == DelayUnits::Pixels {
                return self.delayed_offset;
            }
            delay.value?
        } else {
            conditions[self.expression].value?
        };
        Some(self.offset_for(value))
    }

    fn offset_for(&self, value: f64) -> DVec2 {
        if self.table.is_empty() {
            return self.scale * value;
        }
        let first = self.table[0];
        let last = *self.table.last().unwrap();
        let increasing = last.0 > first.0;
        let position = if (increasing && value <= first.0) || (!increasing && value >= first.0) {
            first.1
        } else if (increasing && value >= last.0) || (!increasing && value <= last.0) {
            last.1
        } else {
            let pair = self
                .table
                .windows(2)
                .find(|pair| {
                    if increasing {
                        value <= pair[1].0
                    } else {
                        value >= pair[1].0
                    }
                })
                .unwrap();
            pair[0]
                .1
                .lerp(pair[1].1, (value - pair[0].0) / (pair[1].0 - pair[0].0))
        };
        if self.scale == DVec2::ZERO {
            position - self.origin
        } else {
            self.scale * position
        }
    }
}

impl Rotation {
    fn parse(node: Node<'_, '_>, centre: DVec2, conditions: &mut Vec<Condition>) -> Result<Self> {
        let failure = FailureState::parse(node)?;
        let points_to = if let Some(direction) = child_text(node, "PointsTo") {
            match direction.trim().to_ascii_uppercase().as_str() {
                "NORTH" => 0.0,
                "EAST" => FRAC_PI_2,
                "SOUTH" => PI,
                "WEST" => -FRAC_PI_2,
                _ => anyhow::bail!("invalid rotation direction"),
            }
        } else {
            number(node, "DegreesPointsTo")?.unwrap_or(0.0).to_radians()
        };
        let mut table: Vec<(f64, f64)> = Vec::new();
        if let Some(entries) = node
            .children()
            .find(|c| c.has_tag_name("NonlinearityTable"))
        {
            for entry in entries
                .children()
                .filter(|c| c.has_tag_name("NonlinearityEntry"))
            {
                let value =
                    number(entry, "ExpressionResult")?.context("missing rotation table value")?;
                let angle = match (
                    number(entry, "Degrees")?.filter(|v| *v != -99_999.0),
                    number(entry, "Radians")?.filter(|v| *v != -99_999.0),
                    entry
                        .children()
                        .find(|c| c.has_tag_name("FloatPosition"))
                        .map(point)
                        .transpose()?
                        .filter(|p| *p != DVec2::ZERO),
                ) {
                    (Some(degrees), None, None) => degrees.to_radians(),
                    (None, Some(radians), None) => radians,
                    (None, None, Some(position)) => {
                        let delta = position - centre;
                        anyhow::ensure!(
                            delta.length_squared() > 0.0,
                            "rotation table point is at pivot"
                        );
                        let mut angle = delta.x.atan2(-delta.y);
                        if let Some((_, previous)) = table.last()
                            && angle < *previous - 1e-9
                        {
                            angle += ((*previous - angle) / TAU).ceil() * TAU;
                        }
                        angle
                    }
                    _ => anyhow::bail!("ambiguous rotation table position"),
                };
                table.push((value, angle));
            }
            anyhow::ensure!(!table.is_empty(), "empty rotation table");
            let increasing = table.last().unwrap().0 > table[0].0;
            anyhow::ensure!(
                table.windows(2).all(|pair| if increasing {
                    pair[1].0 > pair[0].0
                } else {
                    pair[1].0 < pair[0].0
                }),
                "non-monotonic rotation table"
            );
        }
        let expression = node
            .children()
            .find(|c| c.has_tag_name("Expression"))
            .map(|node| expression(node, conditions))
            .transpose()?;
        Ok(Self {
            expression,
            points_to,
            table,
            delay: Delay::parse(node, true)?,
            failure,
        })
    }

    fn angle(&self, conditions: &[Condition]) -> Option<f64> {
        match self.failure.action {
            Some(FailureAction::Freeze | FailureAction::Zero) => return self.failure.last,
            Some(FailureAction::NoDraw) => return None,
            None => {}
        }
        if let Some(delay) = &self.delay
            && delay.units == DelayUnits::Angle
        {
            return delay.value.map(|angle| angle.rem_euclid(TAU));
        }
        let Some(index) = self.expression else {
            return Some(0.0);
        };
        let value = if let Some(delay) = &self.delay {
            delay.value?
        } else {
            conditions[index].value?
        };
        self.angle_for(value).map(|angle| angle.rem_euclid(TAU))
    }

    fn angle_for(&self, value: f64) -> Option<f64> {
        let angle = if self.table.is_empty() {
            value
        } else {
            let first = self.table[0];
            let last = *self.table.last()?;
            let increasing = last.0 > first.0;
            if (increasing && value <= first.0) || (!increasing && value >= first.0) {
                first.1
            } else if (increasing && value >= last.0) || (!increasing && value <= last.0) {
                last.1
            } else {
                let pair = self.table.windows(2).find(|pair| {
                    if increasing {
                        value <= pair[1].0
                    } else {
                        value >= pair[1].0
                    }
                })?;
                let t = (value - pair[0].0) / (pair[1].0 - pair[0].0);
                pair[0].1 + t * (pair[1].1 - pair[0].1)
            }
        };
        Some(angle - self.points_to)
    }

    fn advance(&mut self, conditions: &[Condition], dt: f64, host: &mut impl Host) {
        self.failure.update(host);
        if self.failure.action == Some(FailureAction::Freeze) {
            if self.failure.last.is_none() {
                self.failure.last = self.angle_for(0.0).map(|angle| angle.rem_euclid(TAU));
            }
            return;
        }
        if self.failure.action == Some(FailureAction::NoDraw) {
            return;
        }
        if self.failure.action == Some(FailureAction::Zero) {
            self.failure.last = self.angle_for(0.0).map(|angle| angle.rem_euclid(TAU));
            return;
        }
        let Some(index) = self.expression else { return };
        let Some(value) = conditions[index].value else {
            return;
        };
        let target = match self.delay.as_ref().map(|delay| delay.units) {
            Some(DelayUnits::Angle) => self.angle_for(value),
            Some(DelayUnits::Value) => Some(value),
            _ => None,
        };
        if let (Some(delay), Some(target)) = (&mut self.delay, target) {
            delay.advance(target, dt);
        }
        self.failure.last = self.angle(conditions);
    }
}

pub(super) fn transform(motion: &[Motion], conditions: &[Condition]) -> Option<DAffine2> {
    let mut transform = DAffine2::IDENTITY;
    for step in motion {
        transform *= DAffine2::from_translation(step.offset);
        if let Some(rotation) = &step.rotation {
            transform *= DAffine2::from_angle(rotation.angle(conditions)?);
        }
        if let Some(shift) = &step.shift {
            transform *= DAffine2::from_translation(shift.offset(conditions)?);
        }
    }
    transform.is_finite().then_some(transform)
}

pub(super) fn draw(image: &RgbaImage, transform: DAffine2, output: &mut RgbaImage) {
    draw_sampled(image, transform, output, Interpolation::Bilinear);
}

pub(super) fn draw_sampled(
    image: &RgbaImage,
    transform: DAffine2,
    output: &mut RgbaImage,
    interpolation: Interpolation,
) {
    if transform.matrix2 == DMat2::IDENTITY && transform.translation.fract() == DVec2::ZERO {
        overlay(
            output,
            image,
            transform.translation.x as i64,
            transform.translation.y as i64,
        );
        return;
    }
    let size = DVec2::new(f64::from(image.width()), f64::from(image.height()));
    let mut minimum = DVec2::splat(f64::INFINITY);
    let mut maximum = DVec2::splat(f64::NEG_INFINITY);
    for point in [
        DVec2::ZERO,
        DVec2::new(size.x, 0.0),
        size,
        DVec2::new(0.0, size.y),
    ] {
        let point = transform.transform_point2(point);
        minimum = minimum.min(point);
        maximum = maximum.max(point);
    }
    let limit = DVec2::new(f64::from(output.width()), f64::from(output.height()));
    let minimum = (minimum - DVec2::ONE).floor().clamp(DVec2::ZERO, limit);
    let maximum = (maximum + DVec2::ONE).ceil().clamp(DVec2::ZERO, limit);
    let (width, height) = (
        (maximum.x - minimum.x) as u32,
        (maximum.y - minimum.y) as u32,
    );
    if width == 0 || height == 0 {
        return;
    }
    let transform = DAffine2::from_translation(-minimum) * transform;
    let [a, b, c, d, x, y] = transform.to_cols_array().map(|v| v as f32);
    let projection = Projection::from_matrix([a, c, x, b, d, y, 0.0, 0.0, 1.0])
        .expect("rotation transform is invertible");
    let mut warped = RgbaImage::new(width, height);
    warp_into(
        image,
        projection,
        interpolation,
        Border::Constant(Rgba([0; 4])),
        &mut warped,
    );
    overlay(output, &warped, minimum.x as i64, minimum.y as i64);
}

fn child_text<'a>(node: Node<'a, '_>, name: &str) -> Option<&'a str> {
    node.children()
        .find(|c| c.has_tag_name(name))
        .and_then(|c| c.text())
}

fn number(node: Node<'_, '_>, name: &str) -> Result<Option<f64>> {
    let value = child_text(node, name)
        .map(|v| v.trim().parse::<f64>())
        .transpose()?;
    anyhow::ensure!(
        value.is_none_or(f64::is_finite),
        "non-finite rotation value"
    );
    Ok(value)
}

fn point(node: Node<'_, '_>) -> Result<DVec2> {
    let (x, y) = if let Some(x) = node.attribute("X") {
        (
            x.trim().parse()?,
            node.attribute("Y")
                .context("missing rotation Y")?
                .trim()
                .parse()?,
        )
    } else {
        let (x, y) = node
            .text()
            .context("missing rotation point")?
            .split_once(',')
            .context("invalid rotation point")?;
        (x.trim().parse()?, y.trim().parse()?)
    };
    let point = DVec2::new(x, y);
    anyhow::ensure!(point.is_finite(), "non-finite rotation point");
    Ok(point)
}

#[cfg(test)]
mod tests {
    use super::*;

    struct NoHost;
    impl Host for NoHost {
        fn read(
            &mut self,
            _: &Reference,
        ) -> std::result::Result<esp_script::Value, esp_script::Error> {
            Err(esp_script::Error::Type)
        }
        fn write(
            &mut self,
            _: &Reference,
            _: esp_script::Value,
        ) -> std::result::Result<(), esp_script::Error> {
            Err(esp_script::Error::Type)
        }
    }

    struct PowerHost(f64);
    impl Host for PowerHost {
        fn read(
            &mut self,
            reference: &Reference,
        ) -> std::result::Result<esp_script::Value, esp_script::Error> {
            if reference.name == "CIRCUIT GENERAL PANEL ON" {
                assert_eq!(reference.unit.as_deref(), Some("bool"));
                Ok(esp_script::Value::Number(self.0))
            } else {
                Err(esp_script::Error::Type)
            }
        }
        fn write(
            &mut self,
            _: &Reference,
            _: esp_script::Value,
        ) -> std::result::Result<(), esp_script::Error> {
            Err(esp_script::Error::Type)
        }
    }

    struct PartialPanelHost(f64);
    impl Host for PartialPanelHost {
        fn read(
            &mut self,
            reference: &Reference,
        ) -> std::result::Result<esp_script::Value, esp_script::Error> {
            assert_eq!(reference.name, "PARTIAL PANEL PITOT");
            assert_eq!(reference.unit.as_deref(), Some("enum"));
            Ok(esp_script::Value::Number(self.0))
        }
        fn write(
            &mut self,
            _: &Reference,
            _: esp_script::Value,
        ) -> std::result::Result<(), esp_script::Error> {
            Err(esp_script::Error::Type)
        }
    }

    #[test]
    fn partial_panel_failure_flags_freeze_motion() {
        let (mut dial, mut conditions) = rotation("<Rotation><Expression><Script>0</Script></Expression><FailureTable><Failure><Fail_Key>SYSTEM_PITOT_STATIC</Fail_Key><Fail_Action>Freeze</Fail_Action></Failure></FailureTable></Rotation>").unwrap();
        let mut host = PartialPanelHost(0.0);
        conditions[0].value = Some(0.5);
        dial.advance(&conditions, 0.0, &mut host);
        host.0 = 1.0;
        conditions[0].value = Some(1.0);
        dial.advance(&conditions, 1.0, &mut host);
        assert_eq!(dial.angle(&conditions), Some(0.5));
        host.0 = 0.0;
        dial.advance(&conditions, 1.0, &mut host);
        assert_eq!(dial.angle(&conditions), Some(1.0));
    }

    #[test]
    fn electrical_failures_freeze_zero_hide_and_restore_motion() {
        for (action, expected) in [("Freeze", Some(1.0)), ("Zero", Some(0.0)), ("NoDraw", None)] {
            let text = format!(
                "<Rotation><Expression><Script>0</Script></Expression><FailureTable><Failure><Fail_Key>SYSTEM_ELECTRICAL_PANELS</Fail_Key><Fail_Action>{action}</Fail_Action></Failure></FailureTable></Rotation>"
            );
            let (mut rotation, mut conditions) = self::rotation(&text).unwrap();
            let mut host = PowerHost(1.0);
            conditions[0].value = Some(1.0);
            rotation.advance(&conditions, 0.0, &mut host);
            assert_eq!(rotation.angle(&conditions), Some(1.0));
            host.0 = 0.0;
            conditions[0].value = Some(2.0);
            rotation.advance(&conditions, 1.0, &mut host);
            assert_eq!(rotation.angle(&conditions), expected, "{action}");
            host.0 = 1.0;
            rotation.advance(&conditions, 1.0, &mut host);
            assert_eq!(rotation.angle(&conditions), Some(2.0));
        }
        let doc = roxmltree::Document::parse(r#"<Shift><Scale>2,0</Scale><Expression><Script>0</Script></Expression><FailureTable><Failure><Fail_Key>SYSTEM_ELECTRICAL_PANELS</Fail_Key><Fail_Action>Zero</Fail_Action></Failure></FailureTable></Shift>"#).unwrap();
        let mut conditions = Vec::new();
        let mut shift = Shift::parse(doc.root_element(), DVec2::ZERO, &mut conditions).unwrap();
        let mut host = PowerHost(1.0);
        conditions[0].value = Some(3.0);
        shift.advance(&conditions, 0.0, &mut host);
        assert_eq!(shift.offset(&conditions), Some(DVec2::new(6.0, 0.0)));
        host.0 = 0.0;
        shift.advance(&conditions, 0.0, &mut host);
        assert_eq!(shift.offset(&conditions), Some(DVec2::ZERO));
        host.0 = 1.0;
        shift.advance(&conditions, 0.0, &mut host);
        assert_eq!(shift.offset(&conditions), Some(DVec2::new(6.0, 0.0)));
    }

    fn rotation(text: &str) -> Result<(Rotation, Vec<Condition>)> {
        let document = roxmltree::Document::parse(text).unwrap();
        let mut conditions = Vec::new();
        let rotation = Rotation::parse(document.root_element(), DVec2::ZERO, &mut conditions)?;
        Ok((rotation, conditions))
    }

    #[test]
    fn delay_limits_angular_and_expression_motion() {
        let (mut rotation, mut conditions) = self::rotation(r#"<Rotation><Delay><DelayValue>90</DelayValue><DelayUnits>DEGREES_PER_SECOND</DelayUnits></Delay><Expression><Script>0</Script></Expression></Rotation>"#).unwrap();
        conditions[0].value = Some(0.0);
        rotation.advance(&conditions, 0.0, &mut NoHost);
        conditions[0].value = Some(PI);
        rotation.advance(&conditions, 0.5, &mut NoHost);
        assert!((rotation.angle(&conditions).unwrap() - PI / 4.0).abs() < 1e-12);
        rotation.advance(&conditions, 1.5, &mut NoHost);
        assert!((rotation.angle(&conditions).unwrap() - PI).abs() < 1e-12);
        conditions[0].value = Some(0.0);
        rotation.advance(&conditions, 0.5, &mut NoHost);
        assert!((rotation.angle(&conditions).unwrap() - 3.0 * PI / 4.0).abs() < 1e-12);

        let (mut rotation, mut conditions) = self::rotation(r#"<Rotation><Delay><DelayValue>2</DelayValue><DelayUnits>VALUE_PER_SECOND</DelayUnits></Delay><Expression><Script>0</Script></Expression><NonlinearityTable><NonlinearityEntry><ExpressionResult>0</ExpressionResult><Degrees>0</Degrees></NonlinearityEntry><NonlinearityEntry><ExpressionResult>10</ExpressionResult><Degrees>90</Degrees></NonlinearityEntry></NonlinearityTable></Rotation>"#).unwrap();
        conditions[0].value = Some(0.0);
        rotation.advance(&conditions, 0.0, &mut NoHost);
        conditions[0].value = Some(10.0);
        rotation.advance(&conditions, 2.0, &mut NoHost);
        assert!((rotation.angle(&conditions).unwrap() - 36_f64.to_radians()).abs() < 1e-12);
    }

    #[test]
    fn delay_limits_shift_speed_in_pixels() {
        let doc = roxmltree::Document::parse(r#"<Shift><Scale>3,4</Scale><Delay><DelayValue>10</DelayValue><DelayUnits>PIXELS_PER_SECOND</DelayUnits></Delay><Expression><Script>0</Script></Expression></Shift>"#).unwrap();
        let mut conditions = Vec::new();
        let mut shift = Shift::parse(doc.root_element(), DVec2::ZERO, &mut conditions).unwrap();
        conditions[0].value = Some(0.0);
        shift.advance(&conditions, 0.0, &mut NoHost);
        conditions[0].value = Some(10.0);
        shift.advance(&conditions, 1.0, &mut NoHost);
        assert!(
            shift
                .offset(&conditions)
                .unwrap()
                .abs_diff_eq(DVec2::new(6.0, 8.0), 1e-12)
        );
    }

    #[test]
    fn nonlinear_shift_interpolates_positions_and_limits_pixel_speed() {
        let doc = roxmltree::Document::parse(r#"<Shift><Scale>-1,0</Scale><Delay><DelayValue>10</DelayValue><DelayUnits>PIXELS_PER_SECOND</DelayUnits></Delay><Expression><Script>0</Script></Expression><NonlinearityTable><NonlinearityEntry><ExpressionResult>0</ExpressionResult><FloatPosition>40,0</FloatPosition></NonlinearityEntry><NonlinearityEntry><ExpressionResult>60</ExpressionResult><FloatPosition>100,0</FloatPosition></NonlinearityEntry><NonlinearityEntry><ExpressionResult>100</ExpressionResult><FloatPosition>180,0</FloatPosition></NonlinearityEntry></NonlinearityTable></Shift>"#).unwrap();
        let mut conditions = Vec::new();
        let mut shift = Shift::parse(doc.root_element(), DVec2::ZERO, &mut conditions).unwrap();
        conditions[0].value = Some(30.0);
        shift.advance(&conditions, 0.0, &mut NoHost);
        assert_eq!(shift.offset(&conditions), Some(DVec2::new(-70.0, 0.0)));
        conditions[0].value = Some(80.0);
        shift.advance(&conditions, 1.0, &mut NoHost);
        assert_eq!(shift.offset(&conditions), Some(DVec2::new(-80.0, 0.0)));
        shift.advance(&conditions, 10.0, &mut NoHost);
        assert_eq!(shift.offset(&conditions), Some(DVec2::new(-140.0, 0.0)));

        let doc = roxmltree::Document::parse(r#"<Shift><Scale>0,0</Scale><Expression><Script>0</Script></Expression><NonlinearityTable><NonlinearityEntry><ExpressionResult>100</ExpressionResult><FloatPosition>245,65</FloatPosition></NonlinearityEntry><NonlinearityEntry><ExpressionResult>0</ExpressionResult><FloatPosition>245,123</FloatPosition></NonlinearityEntry></NonlinearityTable></Shift>"#).unwrap();
        let mut conditions = Vec::new();
        let shift = Shift::parse(
            doc.root_element(),
            DVec2::new(245.0, 123.0),
            &mut conditions,
        )
        .unwrap();
        for (value, expected) in [
            (-1.0, 0.0),
            (0.0, 0.0),
            (50.0, -29.0),
            (100.0, -58.0),
            (101.0, -58.0),
        ] {
            conditions[0].value = Some(value);
            assert_eq!(shift.offset(&conditions), Some(DVec2::new(0.0, expected)));
        }
    }

    #[test]
    fn interpolates_clockwise_points_across_north_and_clamps_endpoints() {
        let (rotation, mut conditions) = rotation(r#"<Rotation><PointsTo>WEST</PointsTo><Expression><Script>0</Script></Expression><NonlinearityTable><NonlinearityEntry><ExpressionResult>0</ExpressionResult><FloatPosition>-10,0</FloatPosition></NonlinearityEntry><NonlinearityEntry><ExpressionResult>10</ExpressionResult><FloatPosition>0,-10</FloatPosition></NonlinearityEntry><NonlinearityEntry><ExpressionResult>20</ExpressionResult><FloatPosition>10,0</FloatPosition></NonlinearityEntry></NonlinearityTable></Rotation>"#).unwrap();
        for (value, expected) in [
            (-1.0, 0.0),
            (0.0, 0.0),
            (5.0, PI / 4.0),
            (10.0, FRAC_PI_2),
            (15.0, 3.0 * PI / 4.0),
            (20.0, PI),
            (30.0, PI),
        ] {
            conditions[0].value = Some(value);
            assert!((rotation.angle(&conditions).unwrap() - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn supports_decreasing_scales_and_explicit_angle_entries() {
        let (rotation, mut conditions) = rotation(r#"<Rotation><DegreesPointsTo>90</DegreesPointsTo><Expression><Script>0</Script></Expression><NonlinearityTable><NonlinearityEntry><ExpressionResult>10</ExpressionResult><Degrees>90</Degrees></NonlinearityEntry><NonlinearityEntry><ExpressionResult>0</ExpressionResult><Radians>3.141592653589793</Radians></NonlinearityEntry></NonlinearityTable></Rotation>"#).unwrap();
        conditions[0].value = Some(5.0);
        assert!((rotation.angle(&conditions).unwrap() - PI / 4.0).abs() < 1e-12);
        conditions[0].value = None;
        assert!(rotation.angle(&conditions).is_none());
    }

    #[test]
    fn applies_shift_along_the_rotated_instrument_axes() {
        let document = roxmltree::Document::parse("<Element><Rotation><Expression><Script>0</Script></Expression></Rotation><Shift><Scale>2,-3</Scale><Expression><Script>0</Script></Expression></Shift></Element>").unwrap();
        let mut conditions = Vec::new();
        let motion = Motion::element(
            document.root_element(),
            (10.0, 20.0),
            (10.0, 20.0),
            &mut conditions,
        )
        .unwrap();
        assert!(transform(std::slice::from_ref(&motion), &conditions).is_none());
        conditions[0].value = Some(FRAC_PI_2);
        conditions[1].value = Some(4.0);
        let position = transform(&[motion], &conditions)
            .unwrap()
            .transform_point2(DVec2::ZERO);
        assert!(position.abs_diff_eq(DVec2::new(22.0, 28.0), 1e-12));
    }

    #[test]
    fn empty_failure_entries_do_not_disable_motion() {
        assert!(rotation("<Rotation><FailureTable><Failure/></FailureTable><Expression><Script>0</Script></Expression></Rotation>").is_ok());
        assert!(rotation("<Rotation><FailureTable><Failure><Fail_Key>SYSTEM_PITOT_STATIC</Fail_Key><Fail_Action>Freeze</Fail_Action></Failure></FailureTable></Rotation>").is_ok());
    }

    #[test]
    fn rejects_invalid_or_unsupported_rotation_definitions() {
        for body in [
            "<PointsTo>INVALID</PointsTo>",
            "<DegreesPointsTo>NaN</DegreesPointsTo>",
            "<Delay/>",
            "<Expression><Script> </Script></Expression>",
            "<Expression><Minimum>2</Minimum><Maximum>1</Maximum><Script>0</Script></Expression>",
            "<NonlinearityTable/>",
            "<NonlinearityTable><NonlinearityEntry><ExpressionResult>0</ExpressionResult><Degrees>1</Degrees><Radians>1</Radians></NonlinearityEntry></NonlinearityTable>",
            "<NonlinearityTable><NonlinearityEntry><ExpressionResult>0</ExpressionResult><FloatPosition>0,0</FloatPosition></NonlinearityEntry></NonlinearityTable>",
        ] {
            assert!(
                rotation(&format!("<Rotation>{body}</Rotation>")).is_err(),
                "{body}"
            );
        }
    }
}

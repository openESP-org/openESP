use anyhow::{Context, Result};
use esp_script::Program;
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
    delay: Option<Delay>,
}

#[derive(Clone)]
struct Rotation {
    expression: Option<usize>,
    points_to: f64,
    table: Vec<(f64, f64)>,
    delay: Option<Delay>,
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
            .map(|node| Shift::parse(node, conditions))
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

    pub fn advance(&mut self, conditions: &[Condition], dt: f64) {
        if let Some(rotation) = &mut self.rotation {
            rotation.advance(conditions, dt);
        }
        if let Some(shift) = &mut self.shift {
            shift.advance(conditions, dt);
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

fn supported_effects(node: Node<'_, '_>) -> Result<()> {
    for failure in node
        .children()
        .filter(|c| c.has_tag_name("FailureTable"))
        .flat_map(|t| t.children().filter(|c| c.has_tag_name("Failure")))
    {
        let key = child_text(failure, "Fail_Key").unwrap_or("NONE").trim();
        anyhow::ensure!(
            key.is_empty() || key.eq_ignore_ascii_case("NONE"),
            "unsupported gauge motion failure {key}"
        );
    }
    Ok(())
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
    fn parse(node: Node<'_, '_>, conditions: &mut Vec<Condition>) -> Result<Self> {
        supported_effects(node)?;
        anyhow::ensure!(
            !node.children().any(|c| c.has_tag_name("NonlinearityTable")),
            "unsupported nonlinear gauge shift"
        );
        let scale = point(
            node.children()
                .find(|c| c.has_tag_name("Scale"))
                .context("missing shift scale")?,
        )?;
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
            delay,
        })
    }

    fn advance(&mut self, conditions: &[Condition], dt: f64) {
        let Some(value) = conditions[self.expression].value else {
            return;
        };
        if let Some(delay) = &mut self.delay {
            let target = if delay.units == DelayUnits::Pixels {
                value * self.scale.length()
            } else {
                value
            };
            delay.advance(target, dt);
        }
    }

    fn offset(&self, conditions: &[Condition]) -> Option<DVec2> {
        let value = if let Some(delay) = &self.delay {
            let value = delay.value?;
            if delay.units == DelayUnits::Pixels && self.scale.length() > 0.0 {
                value / self.scale.length()
            } else if delay.units == DelayUnits::Pixels {
                0.0
            } else {
                value
            }
        } else {
            conditions[self.expression].value?
        };
        Some(self.scale * value)
    }
}

impl Rotation {
    fn parse(node: Node<'_, '_>, centre: DVec2, conditions: &mut Vec<Condition>) -> Result<Self> {
        supported_effects(node)?;
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
        })
    }

    fn angle(&self, conditions: &[Condition]) -> Option<f64> {
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

    fn advance(&mut self, conditions: &[Condition], dt: f64) {
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
        rotation.advance(&conditions, 0.0);
        conditions[0].value = Some(PI);
        rotation.advance(&conditions, 0.5);
        assert!((rotation.angle(&conditions).unwrap() - PI / 4.0).abs() < 1e-12);
        rotation.advance(&conditions, 1.5);
        assert!((rotation.angle(&conditions).unwrap() - PI).abs() < 1e-12);
        conditions[0].value = Some(0.0);
        rotation.advance(&conditions, 0.5);
        assert!((rotation.angle(&conditions).unwrap() - 3.0 * PI / 4.0).abs() < 1e-12);

        let (mut rotation, mut conditions) = self::rotation(r#"<Rotation><Delay><DelayValue>2</DelayValue><DelayUnits>VALUE_PER_SECOND</DelayUnits></Delay><Expression><Script>0</Script></Expression><NonlinearityTable><NonlinearityEntry><ExpressionResult>0</ExpressionResult><Degrees>0</Degrees></NonlinearityEntry><NonlinearityEntry><ExpressionResult>10</ExpressionResult><Degrees>90</Degrees></NonlinearityEntry></NonlinearityTable></Rotation>"#).unwrap();
        conditions[0].value = Some(0.0);
        rotation.advance(&conditions, 0.0);
        conditions[0].value = Some(10.0);
        rotation.advance(&conditions, 2.0);
        assert!((rotation.angle(&conditions).unwrap() - 36_f64.to_radians()).abs() < 1e-12);
    }

    #[test]
    fn delay_limits_shift_speed_in_pixels() {
        let doc = roxmltree::Document::parse(r#"<Shift><Scale>3,4</Scale><Delay><DelayValue>10</DelayValue><DelayUnits>PIXELS_PER_SECOND</DelayUnits></Delay><Expression><Script>0</Script></Expression></Shift>"#).unwrap();
        let mut conditions = Vec::new();
        let mut shift = Shift::parse(doc.root_element(), &mut conditions).unwrap();
        conditions[0].value = Some(0.0);
        shift.advance(&conditions, 0.0);
        conditions[0].value = Some(10.0);
        shift.advance(&conditions, 1.0);
        assert!(
            shift
                .offset(&conditions)
                .unwrap()
                .abs_diff_eq(DVec2::new(6.0, 8.0), 1e-12)
        );
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
        assert!(rotation("<Rotation><FailureTable><Failure><Fail_Key>SYSTEM_PITOT_STATIC</Fail_Key><Fail_Action>Freeze</Fail_Action></Failure></FailureTable></Rotation>").is_err());
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

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
}

#[derive(Clone)]
struct Rotation {
    expression: Option<usize>,
    points_to: f64,
    table: Vec<(f64, f64)>,
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
        Ok(Self {
            offset: DVec2::from(offset),
            rotation,
        })
    }

    pub fn image(node: Node<'_, '_>) -> Result<Self> {
        let offset = node
            .children()
            .find(|c| c.has_tag_name("Axis"))
            .map(point)
            .transpose()?
            .unwrap_or(DVec2::ZERO);
        Ok(Self {
            offset: -offset,
            rotation: None,
        })
    }

    pub fn rebase(&mut self, offset: usize) {
        if let Some(index) = self.rotation.as_mut().and_then(|r| r.expression.as_mut()) {
            *index += offset;
        }
    }
}

impl Rotation {
    fn parse(node: Node<'_, '_>, centre: DVec2, conditions: &mut Vec<Condition>) -> Result<Self> {
        anyhow::ensure!(
            !node
                .children()
                .any(|c| c.has_tag_name("Delay") || c.has_tag_name("FailureTable")),
            "unsupported rotation delay or failure table"
        );
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
                    number(entry, "Degrees")?,
                    number(entry, "Radians")?,
                    entry.children().find(|c| c.has_tag_name("FloatPosition")),
                ) {
                    (Some(degrees), None, None) => degrees.to_radians(),
                    (None, Some(radians), None) => radians,
                    (None, None, Some(position)) => {
                        let delta = point(position)? - centre;
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
            .map(|expression| -> Result<usize> {
                let source =
                    child_text(expression, "Script").context("missing rotation expression")?;
                anyhow::ensure!(!source.trim().is_empty(), "empty rotation expression");
                let program = Program::compile(source.trim())?;
                let minimum = number(expression, "Minimum")?.filter(|v| *v != -9_999_999.0);
                let maximum = number(expression, "Maximum")?.filter(|v| *v != -9_999_999.0);
                anyhow::ensure!(
                    minimum.zip(maximum).is_none_or(|(min, max)| min <= max),
                    "invalid rotation bounds"
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
            })
            .transpose()?;
        Ok(Self {
            expression,
            points_to,
            table,
        })
    }

    fn angle(&self, conditions: &[Condition]) -> Option<f64> {
        let Some(index) = self.expression else {
            return Some(0.0);
        };
        let value = conditions[index].value?;
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
        Some((angle - self.points_to).rem_euclid(TAU))
    }
}

pub(super) fn transform(motion: &[Motion], conditions: &[Condition]) -> Option<DAffine2> {
    let mut transform = DAffine2::IDENTITY;
    for step in motion {
        transform *= DAffine2::from_translation(step.offset);
        if let Some(rotation) = &step.rotation {
            transform *= DAffine2::from_angle(rotation.angle(conditions)?);
        }
    }
    transform.is_finite().then_some(transform)
}

pub(super) fn draw(image: &RgbaImage, transform: DAffine2, output: &mut RgbaImage) {
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
        Interpolation::Bilinear,
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

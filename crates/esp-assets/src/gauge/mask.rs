use anyhow::{Context, Result};
use glam::DVec2;
use image::{Rgba, RgbaImage, imageops::overlay};
use imageproc::geometric_transformations::Interpolation;
use roxmltree::Node;
use std::sync::Arc;

use super::{
    Condition,
    rotation::{self, Motion},
};
use crate::{panel::Panel, texture::gauge_bitmap};

#[derive(Clone)]
pub(super) struct Mask {
    image: Arc<RgbaImage>,
    motion: Vec<Motion>,
    transform: Option<glam::DAffine2>,
    anchor: DVec2,
}

impl Mask {
    pub fn parse(
        panel: &Panel,
        module: &str,
        node: Node<'_, '_>,
        mut motion: Vec<Motion>,
        offset: DVec2,
        conditions: &[Condition],
    ) -> Result<Self> {
        let name = node.attribute("Name").context("mask image missing name")?;
        let bytes = panel
            .read_resource(module, name, 64 * 1024 * 1024)?
            .context("mask bitmap is unavailable")?;
        let image = Arc::new(gauge_bitmap(&bytes, false, false)?);
        // A mask defines the element's fixed aperture. The element's own
        // rotation/shift moves its artwork within that aperture; ancestors
        // still transform the complete instrument and its aperture together.
        motion.push(Motion::translation(offset));
        let anchor = rotation::axis(node)?;
        let transform = rotation::transform(&motion, conditions);
        Ok(Self {
            image,
            motion,
            transform,
            anchor,
        })
    }

    pub fn anchor(&self) -> DVec2 {
        self.anchor
    }

    pub fn rebase(&mut self, offset: usize) {
        for motion in &mut self.motion {
            motion.rebase(offset);
        }
    }

    pub fn update(&mut self, conditions: &[Condition]) -> bool {
        let next = rotation::transform(&self.motion, conditions);
        let changed = self.transform != next;
        self.transform = next;
        changed
    }

    pub fn ready(&self) -> bool {
        self.transform.is_some()
    }

    fn apply(&self, image: &mut RgbaImage) {
        let mut mask = RgbaImage::new(image.width(), image.height());
        rotation::draw_sampled(
            &self.image,
            self.transform.unwrap(),
            &mut mask,
            Interpolation::Nearest,
        );
        for (pixel, mask) in image.pixels_mut().zip(mask.pixels()) {
            match mask.0[..3] {
                [0, 0, 0] => *pixel = Rgba([0; 4]),
                [1, 1, 1] => {}
                _ => *pixel = *mask,
            }
        }
    }
}

pub(super) fn draw(
    image: &RgbaImage,
    transform: glam::DAffine2,
    masks: &[Mask],
    output: &mut RgbaImage,
) {
    if masks.is_empty() {
        rotation::draw(image, transform, output);
        return;
    }
    let mut layer = RgbaImage::new(output.width(), output.height());
    rotation::draw(image, transform, &mut layer);
    for mask in masks.iter().rev() {
        mask.apply(&mut layer);
    }
    overlay(output, &layer, 0, 0);
}

use anyhow::{Context, Result};
use esp_formats::{cfg::panel::WindowKind, gauge::Gauge};
use image::{
    Rgba, RgbaImage,
    imageops::{FilterType, overlay, resize},
};

use crate::{panel::Panel, texture::gauge_bitmap};

pub struct PanelImage {
    pub image: RgbaImage,
    pub omitted_gauges: usize,
    pub omitted_elements: usize,
}

pub fn static_panel(panel: &Panel, texture: &str) -> Result<Option<PanelImage>> {
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
    let mut output = RgbaImage::from_pixel(width, height, Rgba(panel.background(window.section)?));
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
        let mut art = RgbaImage::new(w, h);
        let mut stack = vec![(document.root(), 0.0, 0.0)];
        while let Some((node, x, y)) = stack.pop() {
            if node.has_tag_name("Element")
                && node.children().any(|child| {
                    child.is_element()
                        && matches!(
                            child.tag_name().name(),
                            "Visibility" | "Rotate" | "Rotation" | "Shift" | "Select" | "MaskImage"
                        )
                })
            {
                omitted_elements += 1;
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
                overlay(&mut art, &bitmap, x.round() as i64, y.round() as i64);
            }
            for child in node.children().filter(|c| c.is_element()).rev() {
                if child.has_tag_name("Element") || child.has_tag_name("Image") {
                    stack.push((child, x, y));
                }
            }
        }
        let target_width =
            dimension(gauge.width.unwrap_or(f64::from(w)) * f64::from(width) / dimensions.width)?;
        let target_height =
            dimension(gauge.height.unwrap_or(f64::from(h)) * f64::from(height) / layout_height)?;
        let scaled = resize(&art, target_width, target_height, FilterType::Triangle);
        overlay(
            &mut output,
            &scaled,
            (gauge.x * f64::from(width) / dimensions.width).round() as i64,
            (gauge.y * f64::from(height) / layout_height).round() as i64,
        );
    }
    Ok(Some(PanelImage {
        image: output,
        omitted_gauges,
        omitted_elements,
    }))
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
    #[test]
    fn composes_static_images_and_omits_dynamic_elements() {
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
        let result = static_panel(&panel, "$SCREEN").unwrap().unwrap();
        assert_eq!(result.omitted_elements, 1);
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
        let result = static_panel(&panel, "$screen").unwrap().unwrap();
        assert_eq!(result.omitted_gauges, 0);
        assert_eq!(result.image.get_pixel(1, 1).0, [255, 0, 0, 255]);
        assert_eq!(result.image.get_pixel(2, 1).0, [0, 255, 0, 255]);
        assert_eq!(result.image.get_pixel(3, 1).0, [0, 0, 0, 255]);
    }
}

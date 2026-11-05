use super::*;
use esp_script::{Error, Host, Reference, Value};

struct Empty;
impl Host for Empty {
    fn read(&mut self, _: &Reference) -> Result<Value, Error> {
        Err(Error::Type)
    }
    fn write(&mut self, _: &Reference, _: Value) -> Result<(), Error> {
        Err(Error::Type)
    }
}

#[test]
fn empty_select_expression_displays_zero_case() {
    let root = tempfile::tempdir().unwrap();
    let panel_dir = root.path().join("panel");
    let module = panel_dir.join("example");
    std::fs::create_dir_all(&module).unwrap();
    std::fs::write(
        panel_dir.join("panel.cfg"),
        "[VCockpit01]\nsize_mm=1,1\npixel_size=1,1\ntexture=$screen\ngauge00=example!dial,0,0",
    )
    .unwrap();
    std::fs::write(module.join("dial.xml"), r#"<Gauge><Size X="1" Y="1"/><Element><Select><Expression><Minimum>0</Minimum><Maximum>1</Maximum></Expression><Case><ExpressionResult>0</ExpressionResult><Image Name="off.bmp"/></Case><Case><ExpressionResult>1</ExpressionResult><Image Name="on.bmp"/></Case></Select></Element></Gauge>"#).unwrap();
    for (name, color) in [("off.bmp", [255, 0, 0]), ("on.bmp", [0, 255, 0])] {
        let mut data = Vec::new();
        image::codecs::bmp::BmpEncoder::new(&mut data)
            .encode(&color, 1, 1, image::ExtendedColorType::Rgb8)
            .unwrap();
        std::fs::write(module.join(name), data).unwrap();
    }
    let panel = Panel::load(root.path(), None).unwrap().unwrap();
    let mut image = load_panel(&panel, "$screen").unwrap().unwrap();
    assert_eq!(image.omitted_elements, 0);
    assert!(image.update(&mut Empty));
    assert_eq!(image.image.get_pixel(0, 0).0, [255, 0, 0, 255]);
}

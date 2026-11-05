use super::*;
use esp_script::{Error, Reference, Value};

#[derive(Default)]
struct Events(Vec<String>);
impl Host for Events {
    fn read(&mut self, _: &Reference) -> Result<Value, Error> {
        Err(Error::Type)
    }
    fn write(&mut self, reference: &Reference, _: Value) -> Result<(), Error> {
        self.0.push(reference.name.clone());
        Ok(())
    }
}

#[test]
fn recovers_invalid_child_size_from_parent_extent() {
    let xml = r#"<Gauge><MouseArea><Position X="10" Y="20"/><Size X="15" Y="41"/><MouseArea><Position X="0" Y="0"/><Size X="15" Y="20"/><MouseClick><Script>1 (&gt;K:DOWN)</Script></MouseClick></MouseArea><MouseArea><Position X="0" Y="20"/><Size>15.21</Size><MouseClick><Script>1 (&gt;K:UP)</Script></MouseClick></MouseArea></MouseArea></Gauge>"#;
    let document = roxmltree::Document::parse(xml).unwrap();
    let mut areas = Vec::new();
    collect_mouse_areas(document.root_element(), 0.0, 0.0, &mut areas).unwrap();
    assert_eq!(areas.len(), 2);
    assert_eq!(areas[0].bounds, (10.0, 20.0, 15.0, 20.0));
    assert_eq!(areas[1].bounds, (10.0, 40.0, 15.0, 21.0));
    let mut host = Events::default();
    Program::compile(&areas[1].source)
        .unwrap()
        .evaluate(&mut host)
        .unwrap();
    assert_eq!(host.0, ["UP"]);
}

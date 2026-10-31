use esp_formats::mdl::{Model, visibility::Condition};
use esp_script::{Error, Host, Program, Reference, Value};

struct TestHost;
impl Host for TestHost {
    fn read(&mut self, _: &Reference) -> Result<Value, Error> {
        Ok(Value::Number(0.0))
    }
    fn write(&mut self, _: &Reference, _: Value) -> Result<(), Error> {
        Ok(())
    }
}

#[test]
#[ignore = "requires ESP_FORMATS_MDL_FILES containing external aircraft models"]
fn compiles_external_animation_and_visibility_scripts() {
    let paths = std::env::var_os("ESP_FORMATS_MDL_FILES").expect("set ESP_FORMATS_MDL_FILES");
    let mut count = 0;
    for path in std::env::split_paths(&paths) {
        let bytes = std::fs::read(&path).unwrap();
        let model = Model::parse(&bytes).unwrap().decode().unwrap();
        let data = model.exterior.unwrap();
        let scripts = data
            .animations
            .iter()
            .flat_map(|b| &b.animations)
            .flat_map(|a| &a.unknown)
            .filter(|c| &c.id == b"XAPC")
            .map(|c| c.data)
            .chain(data.visibility.iter().filter_map(|c| match c {
                Condition::Code { definition, .. } => Some(*definition),
                _ => None,
            }));
        for script in scripts {
            let script = esp_formats::mdl::parameter::Code::parse(script)
                .unwrap()
                .expression;
            let program = Program::compile(script)
                .unwrap_or_else(|error| panic!("{}: {error}\n{script}", path.display()));
            program
                .evaluate(&mut TestHost)
                .unwrap_or_else(|error| panic!("{}: {error}\n{script}", path.display()));
            count += 1;
        }
    }
    assert!(count > 0);
    eprintln!("{count} compiled model expressions checked");
}

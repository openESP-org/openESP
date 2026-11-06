use crate::hydraulics::*;
use crate::variables::Variables;
use esp_formats::cfg::Config;
use esp_script::Host;
use esp_script::Reference;
use esp_script::Value;

#[test]
fn configured_pumps_and_failure_determine_pressure() {
    let config = Config::parse("[hydraulic_system]\nnormal_pressure=3000\nengine_map=1,0\nelectric_pumps=1\nmin_pct_rpm_for_max_pressure=0.2").unwrap();
    let hydraulics = Hydraulics::from_config(&config).unwrap().unwrap();
    let mut vars = Variables::default();
    vars.set_number("GENERAL ENG PCT MAX RPM:1", "percent", 10.0);
    vars.set_number("GENERAL ENG PCT MAX RPM:2", "percent", 100.0);
    assert_eq!(hydraulics.update(&mut vars).unwrap(), 0.5);
    assert!(
        (vars
            .read(&Reference::new('A', "HYDRAULIC PRESSURE:1", Some("psi")))
            .unwrap()
            .number()
            .unwrap()
            - 1500.0)
            .abs()
            < 1e-9
    );
    vars.set_number("GENERAL ENG PCT MAX RPM:1", "percent", 0.0);
    assert_eq!(hydraulics.update(&mut vars).unwrap(), 0.0);
    vars.set_number("HYDRAULIC SWITCH", "bool", 1.0);
    vars.set_number("CIRCUIT HYDRAULIC PUMP ON", "bool", 1.0);
    assert_eq!(hydraulics.update(&mut vars).unwrap(), 1.0);
    vars.dispatch_key_event(0x10305, Value::Number(0.0))
        .unwrap();
    assert_eq!(hydraulics.update(&mut vars).unwrap(), 0.0);
    vars.dispatch_key_event(0x10305, Value::Number(0.0))
        .unwrap();
    assert_eq!(hydraulics.update(&mut vars).unwrap(), 1.0);
}

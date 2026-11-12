//! Connect aircraft light switches and electrical circuits to effect rendering.
use esp_script::{Host, Reference};
use esp_sim::variables::Variables;

pub(super) fn light_switches(variables: &mut Variables) -> [bool; 14] {
    let mut result = [false; 14];
    for (kind, switch, circuit) in [
        (1, "LIGHT BEACON", "CIRCUIT BEACON LIGHTS ON"),
        (2, "LIGHT STROBE", "CIRCUIT STROBE LIGHTS ON"),
        (3, "LIGHT NAV", "CIRCUIT NAV LIGHTS ON"),
        (4, "LIGHT PANEL", "CIRCUIT PANEL LIGHTS ON"),
        (5, "LIGHT LANDING", "CIRCUIT LANDING LIGHTS ON"),
        (6, "LIGHT TAXI", "CIRCUIT TAXI LIGHTS ON"),
        (7, "LIGHT RECOGNITION", "CIRCUIT RECOGNITION LIGHTS ON"),
        (8, "LIGHT WING", "CIRCUIT WING LIGHTS ON"),
        (9, "LIGHT LOGO", "CIRCUIT LOGO LIGHTS ON"),
        (10, "LIGHT CABIN", "CIRCUIT CABIN LIGHTS ON"),
    ] {
        let read = |variables: &mut Variables, name| {
            variables
                .read(&Reference::new('A', name, Some("bool")))
                .ok()
                .and_then(|value| value.number().ok())
                .is_some_and(|value| value != 0.0)
        };
        result[kind] = read(variables, switch) && read(variables, circuit);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn light_requires_switch_and_powered_circuit() {
        let mut variables = Variables::default();
        variables.set_number("LIGHT NAV", "bool", 1.0);
        variables.set_number("CIRCUIT NAV LIGHTS ON", "bool", 0.0);
        assert!(!light_switches(&mut variables)[3]);
        variables.set_number("CIRCUIT NAV LIGHTS ON", "bool", 1.0);
        assert!(light_switches(&mut variables)[3]);
        variables.set_number("LIGHT NAV", "bool", 0.0);
        assert!(!light_switches(&mut variables)[3]);
    }
}

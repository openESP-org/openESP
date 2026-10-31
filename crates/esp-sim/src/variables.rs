use esp_script::{Error, Host, Reference, Value};
use std::collections::HashMap;

#[derive(Default)]
pub struct Variables {
    values: HashMap<Reference, Value>,
    locals: HashMap<(char, String), Value>,
    events: Vec<(String, Value)>,
}

impl Variables {
    pub fn set(&mut self, category: char, name: &str, unit: Option<&str>, value: Value) {
        let reference = Reference::new(category, name, unit);
        if reference.category == 'L' || reference.category == 'G' {
            self.locals
                .insert((reference.category, reference.name), value);
        } else {
            self.values.insert(reference, value);
        }
    }

    pub fn set_number(&mut self, name: &str, unit: &str, value: f64) {
        self.set('A', name, Some(unit), Value::Number(value));
    }

    pub fn drain_events(&mut self) -> impl Iterator<Item = (String, Value)> + '_ {
        self.events.drain(..)
    }

    pub fn dispatch_events(&mut self) -> Vec<Error> {
        std::mem::take(&mut self.events)
            .into_iter()
            .filter_map(|(name, value)| self.dispatch_event(&name, value).err())
            .collect()
    }

    pub fn dispatch_event(&mut self, name: &str, value: Value) -> Result<(), Error> {
        let value = value.number()?;
        match name {
            "TOGGLE_FLIGHT_DIRECTOR" => {
                let reference =
                    Reference::new('A', "AUTOPILOT FLIGHT DIRECTOR ACTIVE", Some("bool"));
                let active = self
                    .values
                    .get(&reference)
                    .map(Value::number)
                    .transpose()?
                    .unwrap_or(0.0);
                self.values
                    .insert(reference, Value::Number(f64::from(active == 0.0)));
                Ok(())
            }
            "SPOILERS_SET" => {
                self.set_number(
                    "SPOILERS HANDLE POSITION",
                    "percent",
                    value.clamp(0.0, 16383.0) * 100.0 / 16383.0,
                );
                Ok(())
            }
            _ => Err(Error::Unsupported(format!("event {name}"))),
        }
    }
}

impl Host for Variables {
    fn read(&mut self, reference: &Reference) -> Result<Value, Error> {
        if reference.category == 'L' || reference.category == 'G' {
            return Ok(self
                .locals
                .get(&(reference.category, reference.name.clone()))
                .cloned()
                .unwrap_or(Value::Number(0.0)));
        }
        self.values.get(reference).cloned().ok_or_else(|| {
            Error::Variable(format!(
                "{}:{}, {}",
                reference.category,
                reference.name,
                reference.unit.as_deref().unwrap_or("")
            ))
        })
    }
    fn write(&mut self, reference: &Reference, value: Value) -> Result<(), Error> {
        if reference.category == 'L' || reference.category == 'G' {
            self.locals
                .insert((reference.category, reference.name.clone()), value);
            return Ok(());
        }
        if reference.category == 'K' {
            self.events.push((reference.name.clone(), value));
            return Ok(());
        }
        Err(Error::Variable(format!(
            "{}:{} is not writable",
            reference.category, reference.name
        )))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_script::Program;

    #[test]
    fn separates_local_and_gauge_register_namespaces() {
        let mut variables = Variables::default();
        Program::compile("4 (>G:Var1) 8 (>L:Var1)")
            .unwrap()
            .evaluate(&mut variables)
            .unwrap();
        assert_eq!(
            variables
                .read(&Reference::new('G', "Var1", Some("number")))
                .unwrap(),
            Value::Number(4.0)
        );
        assert_eq!(
            variables.read(&Reference::new('L', "Var1", None)).unwrap(),
            Value::Number(8.0)
        );
        assert_eq!(
            variables.read(&Reference::new('G', "Var2", None)).unwrap(),
            Value::Number(0.0)
        );
        assert!(
            variables
                .read(&Reference::new('A', "MISSING", Some("bool")))
                .is_err()
        );
    }

    #[test]
    fn dispatches_queued_flight_director_events() {
        let mut variables = Variables::default();
        Program::compile("(>K:TOGGLE_FLIGHT_DIRECTOR)")
            .unwrap()
            .evaluate(&mut variables)
            .unwrap();
        let events = variables.drain_events().collect::<Vec<_>>();
        assert_eq!(
            events,
            [("TOGGLE_FLIGHT_DIRECTOR".into(), Value::Number(0.0))]
        );
        for (name, value) in events {
            variables.dispatch_event(&name, value).unwrap();
        }
        let reference = Reference::new('A', "AUTOPILOT FLIGHT DIRECTOR ACTIVE", Some("bool"));
        assert_eq!(variables.read(&reference).unwrap(), Value::Number(1.0));
        variables
            .dispatch_event("TOGGLE_FLIGHT_DIRECTOR", Value::Number(0.0))
            .unwrap();
        assert_eq!(variables.read(&reference).unwrap(), Value::Number(0.0));
        assert_eq!(variables.drain_events().count(), 0);
        assert!(
            variables
                .dispatch_event("UNKNOWN", Value::Number(0.0))
                .is_err()
        );
        assert!(
            variables
                .dispatch_event("TOGGLE_FLIGHT_DIRECTOR", Value::Number(f64::NAN))
                .is_err()
        );
        assert!(
            variables
                .dispatch_event("TOGGLE_FLIGHT_DIRECTOR", Value::Text("1".into()))
                .is_err()
        );
        assert!(variables.write(&reference, Value::Number(1.0)).is_err());
    }

    #[test]
    fn processes_events_even_after_script_failure() {
        let mut variables = Variables::default();
        assert!(
            Program::compile("(>K:TOGGLE_FLIGHT_DIRECTOR) (A:MISSING,bool)")
                .unwrap()
                .evaluate(&mut variables)
                .is_err()
        );
        assert!(variables.dispatch_events().is_empty());
        assert_eq!(variables.drain_events().count(), 0);
        assert_eq!(
            variables
                .read(&Reference::new(
                    'A',
                    "AUTOPILOT FLIGHT DIRECTOR ACTIVE",
                    Some("bool")
                ))
                .unwrap(),
            Value::Number(1.0)
        );
        assert!(variables.dispatch_events().is_empty());
    }

    #[test]
    fn spoiler_handle_events_are_bounded_and_finite() {
        let mut variables = Variables::default();
        let reference = Reference::new('A', "SPOILERS HANDLE POSITION", Some("percent"));
        for (input, expected) in [
            (-1.0, 0.0),
            (0.0, 0.0),
            (8191.5, 50.0),
            (16383.0, 100.0),
            (16384.0, 100.0),
        ] {
            variables
                .dispatch_event("SPOILERS_SET", Value::Number(input))
                .unwrap();
            assert_eq!(variables.read(&reference).unwrap(), Value::Number(expected));
        }
        assert!(
            variables
                .dispatch_event("SPOILERS_SET", Value::Number(f64::NAN))
                .is_err()
        );
        assert_eq!(variables.read(&reference).unwrap(), Value::Number(100.0));
    }
}

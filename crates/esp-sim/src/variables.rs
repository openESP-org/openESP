use esp_script::{Error, Host, Reference, Value};
use std::collections::HashMap;

#[derive(Default)]
pub struct Variables {
    values: HashMap<Reference, Value>,
    locals: HashMap<String, Value>,
}

impl Variables {
    pub fn set(&mut self, category: char, name: &str, unit: Option<&str>, value: Value) {
        let reference = Reference::new(category, name, unit);
        if reference.category == 'L' {
            self.locals.insert(reference.name, value);
        } else {
            self.values.insert(reference, value);
        }
    }

    pub fn set_number(&mut self, name: &str, unit: &str, value: f64) {
        self.set('A', name, Some(unit), Value::Number(value));
    }
}

impl Host for Variables {
    fn read(&mut self, reference: &Reference) -> Result<Value, Error> {
        if reference.category == 'L' {
            return Ok(self
                .locals
                .get(&reference.name)
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
        if reference.category != 'L' {
            return Err(Error::Variable(format!(
                "{}:{} is not writable",
                reference.category, reference.name
            )));
        }
        self.locals.insert(reference.name.clone(), value);
        Ok(())
    }
}

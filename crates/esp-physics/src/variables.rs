//! Variable access required by solver adapters, independent of any flight session.
use esp_script::{Error, Host, Value};

pub trait VariableStore: Host {
    fn set(&mut self, category: char, name: &str, unit: Option<&str>, value: Value);
    fn set_number(&mut self, name: &str, unit: &str, value: f64);
    fn dispatch_key_event(&mut self, id: u32, value: Value) -> Result<(), Error>;
}

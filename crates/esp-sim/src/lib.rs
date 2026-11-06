//! esp-sim owns the flight session runtime, state advancement and simulator variable publication.
use esp_nav::navigation;
pub mod autopilot;
pub mod flight;
pub mod speed_limits;
mod units;
pub mod variables;
pub mod weather;
pub use esp_physics::{electrical, flaps};

#[cfg(test)]
mod model_tests;

//! esp-sim owns the flight session, systems, animation, interaction and simulator variables.
use esp_nav::navigation;
pub mod animation;
pub mod autopilot;
pub mod electrical;
pub mod flight;
pub mod hydraulics;
pub mod interaction;
pub mod picking;
pub mod speed_limits;
mod units;
pub mod variables;
pub mod weather;
pub use esp_physics::flaps;

#[cfg(test)]
mod model_tests;

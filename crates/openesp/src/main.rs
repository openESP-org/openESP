use esp_nav::{navigation, vor};
use esp_sim::{flight, speed_limits, weather};
mod app;
mod cli;
#[cfg(test)]
mod nav_tests;
mod scenery;

fn main() -> anyhow::Result<()> {
    app::run()
}

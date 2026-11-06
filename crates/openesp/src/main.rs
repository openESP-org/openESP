use esp_nav::{navigation, vor};
mod app;
mod cli;
mod flight;
#[cfg(test)]
mod nav_tests;
mod scenery;
mod speed_limits;
mod weather;

fn main() -> anyhow::Result<()> {
    app::run()
}

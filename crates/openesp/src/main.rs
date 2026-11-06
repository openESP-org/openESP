mod app;
mod cli;
mod flight;
mod navigation;
mod scenery;
mod speed_limits;
mod vor;
mod weather;

fn main() -> anyhow::Result<()> {
    app::run()
}

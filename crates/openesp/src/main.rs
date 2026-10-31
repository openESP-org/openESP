use anyhow::{Context, Result};
use clap::Parser;
use esp_assets::Aircraft;
use esp_render::{Camera, Renderer};
use esp_script::Value;
use esp_sim::{animation::ModelState, variables::Variables};
use glam::{DVec3, Vec3};
use std::{
    collections::HashSet,
    path::PathBuf,
    sync::{Arc, mpsc},
    time::Instant,
};
use winit::{
    application::ApplicationHandler,
    event::{DeviceEvent, DeviceId, ElementState, MouseButton, WindowEvent},
    event_loop::{ActiveEventLoop, EventLoop},
    keyboard::{KeyCode, PhysicalKey},
    window::{Window, WindowId},
};

#[derive(Parser)]
struct Arguments {
    #[arg(long)]
    aircraft: PathBuf,
    #[arg(long, default_value_t = 0)]
    variant: u32,
    #[arg(long)]
    cockpit: bool,
    #[arg(long, value_parser = clap::value_parser!(u64).range(1..))]
    frames: Option<u64>,
    #[arg(long)]
    screenshot: Option<PathBuf>,
    #[arg(long = "variable", value_parser = parse_variable)]
    variables: Vec<Variable>,
}

#[derive(Clone)]
struct Variable {
    category: char,
    name: String,
    unit: String,
    value: f64,
}

fn parse_variable(text: &str) -> Result<Variable, String> {
    let (reference, value) = text
        .rsplit_once('=')
        .ok_or("expected CATEGORY:NAME,UNIT=VALUE")?;
    let (category, name) = reference
        .split_once(':')
        .ok_or("expected variable category")?;
    let (name, unit) = name.rsplit_once(',').ok_or("expected variable unit")?;
    let mut chars = category.chars();
    let category = chars.next().ok_or("empty variable category")?;
    if chars.next().is_some() {
        return Err("invalid variable category".into());
    }
    let value = value.parse::<f64>().map_err(|e| e.to_string())?;
    if !value.is_finite() {
        return Err("variable must be finite".into());
    }
    Ok(Variable {
        category,
        name: name.into(),
        unit: unit.into(),
        value,
    })
}

struct App {
    arguments: Arguments,
    loaded: mpsc::Receiver<Result<Aircraft>>,
    window: Option<Arc<Window>>,
    renderer: Option<Renderer>,
    aircraft: Option<Aircraft>,
    exterior_state: Option<ModelState>,
    interior_state: Option<ModelState>,
    variables: Variables,
    door_open: bool,
    reported_diagnostics: HashSet<String>,
    camera: Camera,
    keys: HashSet<KeyCode>,
    mouse_look: bool,
    last_frame: Instant,
    frames: u64,
    error: Option<anyhow::Error>,
}

impl App {
    fn reset_camera(&mut self) {
        if let Some(renderer) = &self.renderer {
            self.camera = if self.arguments.cockpit {
                Camera::looking_at(
                    renderer.cockpit_eye.unwrap_or(renderer.aircraft_origin),
                    renderer.cockpit_eye.unwrap_or(renderer.aircraft_origin) + DVec3::Z,
                )
            } else {
                let target = renderer.centre.as_dvec3();
                Camera::looking_at(
                    target + DVec3::new(-0.9, 0.4, 1.4) * f64::from(renderer.radius),
                    target,
                )
            };
        }
    }

    fn frame(&mut self) -> Result<()> {
        if self.renderer.is_none() {
            match self.loaded.try_recv() {
                Ok(aircraft) => {
                    let aircraft = aircraft?;
                    let window = self.window.as_ref().context("window is absent")?;
                    window.set_title(&format!("openESP - {}", aircraft.title));
                    self.renderer = Some(pollster::block_on(Renderer::new(
                        window.clone(),
                        &aircraft,
                    ))?);
                    self.exterior_state = Some(ModelState::new(&aircraft.exterior)?);
                    self.interior_state = aircraft
                        .interior
                        .as_ref()
                        .map(ModelState::new)
                        .transpose()?;
                    self.aircraft = Some(aircraft);
                    self.reset_camera();
                    self.last_frame = Instant::now();
                }
                Err(mpsc::TryRecvError::Empty) => return Ok(()),
                Err(mpsc::TryRecvError::Disconnected) => {
                    anyhow::bail!("aircraft loader disconnected")
                }
            }
        }
        let now = Instant::now();
        let dt = (now - self.last_frame).as_secs_f64().min(0.1);
        self.last_frame = now;
        let axis = |positive, negative| {
            f32::from(self.keys.contains(&positive)) - f32::from(self.keys.contains(&negative))
        };
        let movement = Vec3::new(
            axis(KeyCode::KeyD, KeyCode::KeyA),
            axis(KeyCode::KeyE, KeyCode::KeyQ),
            axis(KeyCode::KeyW, KeyCode::KeyS),
        );
        let speed = if self.keys.contains(&KeyCode::ShiftLeft) {
            20.0
        } else if self.arguments.cockpit {
            1.0
        } else {
            5.0
        };
        self.camera.move_local(movement, dt * speed);
        let roll = axis(KeyCode::ArrowRight, KeyCode::ArrowLeft) as f64;
        let pitch = axis(KeyCode::ArrowDown, KeyCode::ArrowUp) as f64;
        for (name, value) in [
            ("YOKE X POSITION", roll * 100.0),
            ("YOKE Y POSITION", pitch * 100.0),
            ("AILERON LEFT DEFLECTION PCT", roll * 100.0),
            ("AILERON RIGHT DEFLECTION PCT", -roll * 100.0),
            ("ELEVATOR DEFLECTION PCT", pitch * 100.0),
            ("EXIT OPEN:0", if self.door_open { 100.0 } else { 0.0 }),
        ] {
            self.variables.set_number(name, "percent", value);
        }
        for variable in &self.arguments.variables {
            self.variables.set(
                variable.category,
                &variable.name,
                Some(&variable.unit),
                Value::Number(variable.value),
            );
        }
        if let Some(aircraft) = &self.aircraft {
            if let Some(state) = &mut self.exterior_state {
                state.update(&aircraft.exterior, &mut self.variables, dt)?;
            }
            if let (Some(model), Some(state)) = (&aircraft.interior, &mut self.interior_state) {
                state.update(model, &mut self.variables, dt)?;
            }
        }
        for state in [&mut self.exterior_state, &mut self.interior_state]
            .into_iter()
            .flatten()
        {
            for diagnostic in state.diagnostics.drain(..) {
                if !self.reported_diagnostics.contains(&diagnostic) {
                    eprintln!("model: {diagnostic}");
                    self.reported_diagnostics.insert(diagnostic);
                }
            }
        }
        if let Some(renderer) = &mut self.renderer {
            if let Some(state) = &self.exterior_state {
                renderer.update_model(false, &state.transforms, &state.visible)?;
            }
            if let Some(state) = &self.interior_state {
                renderer.update_model(true, &state.transforms, &state.visible)?;
            }
            let screenshot = if self.frames == 0 {
                self.arguments.screenshot.as_deref()
            } else {
                None
            };
            if renderer.render(&self.camera, self.arguments.cockpit, screenshot)? {
                self.frames += 1;
            }
        }
        Ok(())
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        match event_loop.create_window(
            Window::default_attributes()
                .with_title("openESP - loading aircraft")
                .with_inner_size(winit::dpi::LogicalSize::new(1280, 800)),
        ) {
            Ok(window) => {
                self.window = Some(Arc::new(window));
            }
            Err(error) => {
                self.error = Some(error.into());
                event_loop.exit();
            }
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _: WindowId, event: WindowEvent) {
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::Resized(size) => {
                if let Some(renderer) = &mut self.renderer {
                    renderer.resize(size.width, size.height);
                }
            }
            WindowEvent::Focused(false) => {
                self.keys.clear();
                self.mouse_look = false;
                if let Some(window) = &self.window {
                    window.set_cursor_visible(true);
                    let _ = window.set_cursor_grab(winit::window::CursorGrabMode::None);
                }
            }
            WindowEvent::MouseInput {
                state,
                button: MouseButton::Right,
                ..
            } => {
                self.mouse_look = state == ElementState::Pressed;
                if let Some(window) = &self.window {
                    window.set_cursor_visible(!self.mouse_look);
                    let grab = if self.mouse_look {
                        window
                            .set_cursor_grab(winit::window::CursorGrabMode::Locked)
                            .or_else(|_| {
                                window.set_cursor_grab(winit::window::CursorGrabMode::Confined)
                            })
                    } else {
                        window.set_cursor_grab(winit::window::CursorGrabMode::None)
                    };
                    if let Err(error) = grab {
                        eprintln!("mouse capture: {error}");
                    }
                }
            }
            WindowEvent::KeyboardInput { event, .. } => {
                if let PhysicalKey::Code(key) = event.physical_key {
                    if event.state == ElementState::Pressed {
                        self.keys.insert(key);
                        if !event.repeat {
                            match key {
                                KeyCode::Escape => event_loop.exit(),
                                KeyCode::Tab => {
                                    self.arguments.cockpit = !self.arguments.cockpit;
                                    self.reset_camera();
                                }
                                KeyCode::Home => self.reset_camera(),
                                KeyCode::KeyO => self.door_open = !self.door_open,
                                _ => {}
                            }
                        }
                    } else {
                        self.keys.remove(&key);
                    }
                }
            }
            WindowEvent::RedrawRequested => {
                if let Err(error) = self.frame() {
                    self.error = Some(error);
                    event_loop.exit();
                }
                if self
                    .arguments
                    .frames
                    .is_some_and(|limit| self.frames >= limit)
                {
                    event_loop.exit();
                }
            }
            _ => {}
        }
    }

    fn device_event(&mut self, _: &ActiveEventLoop, _: DeviceId, event: DeviceEvent) {
        if self.mouse_look
            && let DeviceEvent::MouseMotion { delta: (x, y) } = event
        {
            self.camera.look(x, y);
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        event_loop.set_control_flow(winit::event_loop::ControlFlow::WaitUntil(
            Instant::now() + std::time::Duration::from_millis(16),
        ));
        if let Some(window) = &self.window {
            window.request_redraw();
        }
    }
}

fn main() -> Result<()> {
    let arguments = Arguments::parse();
    let path = arguments.aircraft.clone();
    let variant = arguments.variant;
    let (sender, loaded) = mpsc::sync_channel(1);
    std::thread::Builder::new()
        .name("aircraft loader".into())
        .spawn(move || {
            let _ = sender.send(Aircraft::load(&path, variant));
        })?;
    let mut app = App {
        arguments,
        loaded,
        window: None,
        renderer: None,
        aircraft: None,
        exterior_state: None,
        interior_state: None,
        variables: Variables::default(),
        door_open: false,
        reported_diagnostics: HashSet::new(),
        camera: Camera::looking_at(DVec3::ZERO, DVec3::Z),
        keys: HashSet::new(),
        mouse_look: false,
        last_frame: Instant::now(),
        frames: 0,
        error: None,
    };
    EventLoop::new()?.run_app(&mut app)?;
    if let Some(error) = app.error {
        return Err(error);
    }
    Ok(())
}

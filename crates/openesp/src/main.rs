use anyhow::{Context, Result};
use clap::Parser;
use esp_assets::{Aircraft, gauge::MouseEvent};
use esp_render::{Camera, Renderer};
use esp_script::Value;
use esp_sim::{
    animation::ModelState,
    interaction, picking,
    time::{Clock, STEP},
    variables::Variables,
};
use glam::{DVec3, Mat4, Vec3};
use std::{
    collections::HashSet,
    path::PathBuf,
    sync::{Arc, mpsc},
    time::Instant,
};
use winit::{
    application::ApplicationHandler,
    event::{DeviceEvent, DeviceId, ElementState, MouseButton, MouseScrollDelta, WindowEvent},
    event_loop::{ActiveEventLoop, EventLoop},
    keyboard::{KeyCode, PhysicalKey},
    window::{Window, WindowId},
};

mod flight;
mod navigation;
mod speed_limits;
mod weather;
use flight::Flight;
use speed_limits::SpeedLimits;

#[derive(Parser)]
struct Arguments {
    #[arg(long)]
    aircraft: PathBuf,
    #[arg(long)]
    fsx_root: Option<PathBuf>,
    #[arg(long, default_value_t = 0)]
    variant: u32,
    #[arg(long)]
    cockpit: bool,
    #[arg(
        long,
        alias = "glide",
        help = "Enable flight physics at this airspeed in knots in the local test scene"
    )]
    airspeed: Option<f64>,
    #[arg(
        long,
        requires = "airspeed",
        help = "Start with the configured engines running at idle"
    )]
    engines_running: bool,
    #[arg(
        long,
        default_value_t = 1000.0,
        help = "Starting height in meters above the test grid; zero starts on the landing gear"
    )]
    height: f64,
    #[arg(
        long,
        default_value_t = 0.0,
        help = "Wind direction in degrees true, measured from where the wind comes"
    )]
    wind_direction: f64,
    #[arg(
        long,
        default_value_t = 0.0,
        allow_negative_numbers = true,
        help = "Latitude of the local test grid origin in degrees"
    )]
    latitude: f64,
    #[arg(
        long,
        default_value_t = 0.0,
        allow_negative_numbers = true,
        help = "Longitude of the local test grid origin in degrees"
    )]
    longitude: f64,
    #[arg(long, default_value_t = 0.0, help = "Wind speed in knots")]
    wind_speed: f64,
    #[arg(
        long,
        default_value_t = 1013.25,
        help = "Sea-level pressure in hectopascals"
    )]
    sea_level_pressure: f64,
    #[arg(
        long,
        default_value_t = 15.0,
        allow_negative_numbers = true,
        help = "Sea-level temperature in degrees Celsius"
    )]
    sea_level_temperature: f64,
    #[arg(
        long,
        default_value_t = 0.0,
        requires = "airspeed",
        help = "Fill usable fuel tanks to this percentage for the airborne test"
    )]
    fuel_percent: f64,
    #[arg(
        long,
        requires = "airspeed",
        help = "Load the aircraft's configured payload stations for the airborne test"
    )]
    payload: bool,
    #[arg(long, value_parser = clap::value_parser!(u64).range(1..))]
    frames: Option<u64>,
    #[arg(
        long,
        help = "Save the final requested frame, or the first frame without --frames"
    )]
    screenshot: Option<PathBuf>,
    #[arg(long = "variable", value_parser = parse_variable)]
    variables: Vec<Variable>,
    #[arg(long = "event", value_parser = parse_event, help = "Send a startup simulator event: NAME or NAME=VALUE")]
    events: Vec<StartupEvent>,
}

#[derive(Clone)]
struct StartupEvent {
    name: String,
    value: f64,
}

fn parse_event(text: &str) -> Result<StartupEvent, String> {
    let (name, value) = text.split_once('=').unwrap_or((text, "0"));
    let name = name.trim();
    if name.is_empty() {
        return Err("event name is empty".into());
    }
    let value = value.trim().parse::<f64>().map_err(|e| e.to_string())?;
    if !value.is_finite() {
        return Err("event value must be finite".into());
    }
    Ok(StartupEvent {
        name: name.to_ascii_uppercase(),
        value,
    })
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

type LoadedAircraft = (Aircraft, Option<(Flight, SpeedLimits)>);

struct App {
    arguments: Arguments,
    loaded: mpsc::Receiver<Result<LoadedAircraft>>,
    window: Option<Arc<Window>>,
    renderer: Option<Renderer>,
    aircraft: Option<Aircraft>,
    flight: Option<Flight>,
    speed_limits: Option<SpeedLimits>,
    exterior_state: Option<ModelState>,
    interior_state: Option<ModelState>,
    variables: Variables,
    clock: Clock,
    door_open: bool,
    reported_diagnostics: HashSet<String>,
    camera: Camera,
    keys: HashSet<KeyCode>,
    keyboard_axes: [f64; 3],
    mouse_look: bool,
    cursor: Option<(f64, f64)>,
    pressed_panel: Option<(String, Vec<usize>, [f64; 2])>,
    pressed_model: Option<usize>,
    repeat_at: Option<Instant>,
    last_frame: Instant,
    frames: u64,
    error: Option<anyhow::Error>,
}

enum CockpitTarget {
    Model(usize),
    Panel(String, f64, f64),
}

impl App {
    fn apply_variable_overrides(&mut self) {
        for variable in &self.arguments.variables {
            self.variables.set(
                variable.category,
                &variable.name,
                Some(&variable.unit),
                Value::Number(variable.value),
            );
        }
    }

    fn reset_camera(&mut self) {
        if let Some(renderer) = &self.renderer {
            let rotation = glam::Quat::from_mat4(&renderer.aircraft_rotation);
            self.camera = if self.arguments.cockpit {
                Camera::looking_at(
                    renderer.cockpit_eye.unwrap_or(renderer.aircraft_origin),
                    renderer.cockpit_eye.unwrap_or(renderer.aircraft_origin) + DVec3::Z,
                )
            } else {
                let target = renderer.centre.as_dvec3();
                let mut camera = Camera::looking_at(
                    target + DVec3::new(-0.9, 0.4, 1.4) * f64::from(renderer.radius),
                    target,
                );
                camera.position =
                    renderer.aircraft_origin + (rotation.as_dquat() * camera.position);
                camera
            };
            self.camera.frame = rotation;
        }
    }

    fn update_flight_pose(&mut self, alpha: f64) -> Result<()> {
        let (Some(flight), Some(renderer)) = (&self.flight, &mut self.renderer) else {
            return Ok(());
        };
        let state = flight.sample(alpha)?;
        let (origin, orientation) = flight.pose(&state);
        let old = glam::Quat::from_mat4(&renderer.aircraft_rotation);
        let local_eye =
            old.as_dquat().conjugate() * (self.camera.position - renderer.aircraft_origin);
        self.camera.position = origin + orientation.as_dquat() * local_eye;
        self.camera.frame = (orientation * old.conjugate() * self.camera.frame).normalize();
        renderer.aircraft_origin = origin;
        renderer.aircraft_rotation = Mat4::from_quat(orientation);
        renderer.cockpit_eye = self
            .aircraft
            .as_ref()
            .and_then(|a| a.eyepoint)
            .map(|eye| origin + orientation.as_dquat() * eye.as_dvec3());
        Ok(())
    }

    fn cockpit_target(&self) -> Option<CockpitTarget> {
        let (cx, cy) = self.cursor?;
        let size = self.window.as_ref()?.inner_size();
        if size.width == 0 || size.height == 0 {
            return None;
        }
        let renderer = self.renderer.as_ref()?;
        let aircraft = self.aircraft.as_ref()?;
        let interior = aircraft.interior.as_ref()?;
        let state = self.interior_state.as_ref()?;
        let aspect = size.width as f32 / size.height as f32;
        let nx = (cx / size.width as f64 * 2.0 - 1.0) as f32;
        let ny = (1.0 - cy / size.height as f64 * 2.0) as f32;
        let (ray_origin, ray_dir) = self.camera.ray(nx, ny, aspect)?;
        let aircraft_to_camera = (renderer.aircraft_origin - self.camera.position).as_vec3();
        let inverse = renderer.aircraft_rotation.inverse();
        let hit = picking::pick(
            interior,
            state,
            inverse.transform_vector3(aircraft_to_camera),
            inverse.transform_vector3(ray_origin),
            inverse.transform_vector3(ray_dir),
        )?;
        let mesh = &interior.lods[0].meshes[hit.mesh];
        if let Ok(index) = usize::try_from(mesh.mouse_rectangle) {
            return Some(CockpitTarget::Model(index));
        }
        let key = interior
            .materials
            .get(mesh.material)?
            .diffuse_texture
            .as_ref()?
            .to_ascii_uppercase();
        let panel = aircraft.panel_images.get(&key)?;
        Some(CockpitTarget::Panel(
            key,
            f64::from(hit.uv[0].rem_euclid(1.0)) * f64::from(panel.image.width()),
            f64::from(hit.uv[1].rem_euclid(1.0)) * f64::from(panel.image.height()),
        ))
    }

    fn cockpit_mouse(&mut self, event: MouseEvent) -> Result<()> {
        let hit = if event == MouseEvent::LeftRelease {
            self.repeat_at = None;
            self.pressed_model.take().map(CockpitTarget::Model)
        } else {
            self.cockpit_target()
        };
        let held_event = matches!(event, MouseEvent::LeftDrag | MouseEvent::DownRepeat);
        if held_event && self.pressed_model.is_some() {
            if let Some(CockpitTarget::Model(index)) = hit
                && self.pressed_model == Some(index)
            {
                return self.model_mouse(index, event);
            }
            return Ok(());
        }
        let panel_hit = match hit {
            Some(CockpitTarget::Model(index)) if !held_event => {
                if event == MouseEvent::LeftSingle {
                    self.pressed_model = Some(index);
                    self.repeat_at = Some(Instant::now() + std::time::Duration::from_millis(400));
                }
                return self.model_mouse(index, event);
            }
            Some(CockpitTarget::Panel(name, x, y)) => Some((name, x, y)),
            _ => None,
        };
        let target = if event == MouseEvent::LeftRelease {
            self.repeat_at = None;
            self.pressed_panel.take()
        } else if matches!(event, MouseEvent::LeftDrag | MouseEvent::DownRepeat) {
            let Some((name, x, y)) = panel_hit else {
                return Ok(());
            };
            let Some((captured, areas, position)) = &mut self.pressed_panel else {
                return Ok(());
            };
            if *captured != name {
                return Ok(());
            }
            *position = [x, y];
            let mut areas = areas.clone();
            if event == MouseEvent::DownRepeat {
                let panel = self
                    .aircraft
                    .as_ref()
                    .context("aircraft is absent")?
                    .panel_images
                    .get(&name)
                    .context("captured panel is absent")?;
                let under_cursor = panel.mouse_areas_at(x, y);
                areas.retain(|index| under_cursor.contains(index));
            }
            Some((name, areas, *position))
        } else {
            if let Some((name, x, y)) = panel_hit {
                let panel = self
                    .aircraft
                    .as_ref()
                    .context("aircraft is absent")?
                    .panel_images
                    .get(&name)
                    .context("cockpit panel is absent")?;
                let areas = panel.visible_mouse_areas_at(x, y, &mut self.variables)?;
                Some((name, areas, [x, y]))
            } else {
                None
            }
        };
        let Some((name, areas, position)) = target else {
            return Ok(());
        };
        if event == MouseEvent::LeftSingle {
            self.pressed_panel = Some((name.clone(), areas.clone(), position));
            // shortcut: repeat timing is 400 ms then 100 ms, calibrate against original runtime behavior.
            self.repeat_at = Some(Instant::now() + std::time::Duration::from_millis(400));
        }
        let panel = self
            .aircraft
            .as_ref()
            .context("aircraft is absent")?
            .panel_images
            .get(&name)
            .context("captured panel is absent")?;
        let result = panel.mouse_event(event, &areas, position, &mut self.variables);
        for error in self.variables.dispatch_events() {
            eprintln!("cockpit event: {error}");
        }
        result?;
        Ok(())
    }

    fn model_mouse(&mut self, index: usize, event: MouseEvent) -> Result<()> {
        let model = self
            .aircraft
            .as_ref()
            .and_then(|aircraft| aircraft.interior.as_ref())
            .context("cockpit model is absent")?;
        interaction::model_mouse(model, index, event, &mut self.variables)?;
        Ok(())
    }

    fn frame(&mut self) -> Result<()> {
        if self.renderer.is_none() {
            match self.loaded.try_recv() {
                Ok(aircraft) => {
                    let (mut aircraft, flight) = aircraft?;
                    if let Some((flight, limits)) = flight {
                        self.flight = Some(flight);
                        self.speed_limits = Some(limits);
                    }
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
                    if let Err(error) = self.variables.configure_flaps(&aircraft.flaps) {
                        if self.flight.is_some() {
                            return Err(error.context("configuring flight control actuators"));
                        }
                        eprintln!("flap system: {error}");
                    }
                    if let Some(flight) = &self.flight {
                        Flight::configure_ground(&mut self.variables);
                        flight.configure_controls(&mut self.variables)?;
                        Flight::publish(&flight.current, &mut self.variables)?;
                        if let Some(limits) = &self.speed_limits {
                            limits.publish(&flight.current, &mut self.variables)?;
                        }
                        flight.publish_loading(&mut self.variables);
                        flight.configure_engines(&mut self.variables)?;
                    }
                    self.variables.configure_electrical(
                        &aircraft.electrical,
                        self.arguments.engines_running,
                    )?;
                    self.apply_variable_overrides();
                    for panel in aircraft.panel_images.values_mut() {
                        panel.update_scripts(0.0, &mut self.variables)?;
                    }
                    for error in self.variables.dispatch_events() {
                        eprintln!("gauge event: {error}");
                    }
                    for event in &self.arguments.events {
                        self.variables
                            .dispatch_event(&event.name, Value::Number(event.value))
                            .with_context(|| format!("startup event {}", event.name))?;
                    }
                    self.aircraft = Some(aircraft);
                    self.update_flight_pose(1.0)?;
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
        let elapsed = (now - self.last_frame).as_secs_f64();
        self.last_frame = now;
        let frame = self.clock.advance(elapsed)?;
        let dt = frame.elapsed;
        if frame.discarded > 0.0
            && self
                .reported_diagnostics
                .insert("simulation catch-up limit".into())
        {
            eprintln!(
                "simulation catch-up capped; discarded {:.3}s after a frame stall",
                frame.discarded
            );
        }
        if self.repeat_at.is_some_and(|deadline| now >= deadline) {
            self.repeat_at = Some(now + std::time::Duration::from_millis(100));
            if let Err(error) = self.cockpit_mouse(MouseEvent::DownRepeat) {
                eprintln!("cockpit repeat: {error}");
            }
        }
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
        self.camera.move_local(movement, dt.min(0.1) * speed);
        let roll = axis(KeyCode::ArrowRight, KeyCode::ArrowLeft) as f64;
        let pitch = axis(KeyCode::ArrowDown, KeyCode::ArrowUp) as f64;
        let rudder = axis(KeyCode::KeyX, KeyCode::KeyZ) as f64;
        for (index, (name, value)) in [
            ("AXIS_AILERONS_SET", roll),
            ("AXIS_ELEVATOR_SET", pitch),
            ("AXIS_RUDDER_SET", rudder),
        ]
        .into_iter()
        .enumerate()
        {
            if self.keyboard_axes[index] != value {
                self.variables
                    .dispatch_event(name, Value::Number(value * 16383.0))?;
                self.keyboard_axes[index] = value;
            }
        }
        self.variables.set_number(
            "EXIT OPEN:0",
            "percent",
            if self.door_open { 100.0 } else { 0.0 },
        );
        if let Some(flight) = &self.flight {
            Flight::publish(&flight.current, &mut self.variables)?;
            if let Some(limits) = &self.speed_limits {
                limits.publish(&flight.current, &mut self.variables)?;
            }
        }
        self.apply_variable_overrides();
        self.variables.interpolate(1.0)?;
        for _ in 0..frame.steps {
            if let Some(aircraft) = &mut self.aircraft {
                for panel in aircraft.panel_images.values_mut() {
                    panel.update_scripts(STEP, &mut self.variables)?;
                }
            }
            for error in self.variables.dispatch_events() {
                eprintln!("gauge event: {error}");
            }
            if let Some(flight) = &mut self.flight {
                self.variables.update_electrical()?;
                flight.advance(STEP, &mut self.variables)?;
                Flight::publish(&flight.current, &mut self.variables)?;
                if let Some(limits) = &self.speed_limits {
                    limits.publish(&flight.current, &mut self.variables)?;
                }
            } else {
                self.variables.advance(STEP)?;
            }
            self.apply_variable_overrides();
        }
        self.variables.update_electrical()?;
        self.variables.interpolate(frame.alpha)?;
        if let Some(flight) = &self.flight {
            let state = flight.sample(frame.alpha)?;
            Flight::publish(&state, &mut self.variables)?;
            if let Some(limits) = &self.speed_limits {
                limits.publish(&state, &mut self.variables)?;
            }
        }
        self.apply_variable_overrides();
        self.update_flight_pose(frame.alpha)?;
        if let Some(aircraft) = &mut self.aircraft {
            for (name, panel) in &mut aircraft.panel_images {
                let changed = panel.update(&mut self.variables);
                if changed && let Some(renderer) = &self.renderer {
                    renderer.update_panel(
                        name,
                        panel.image.width(),
                        panel.image.height(),
                        panel.image.as_raw(),
                    )?;
                }
            }
            for error in self.variables.dispatch_events() {
                eprintln!("gauge event: {error}");
            }
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
            let screenshot = if self
                .arguments
                .frames
                .map_or(self.frames == 0, |limit| self.frames == limit - 1)
            {
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
                if let Err(error) = self.cockpit_mouse(MouseEvent::LeftRelease) {
                    eprintln!("cockpit release: {error}");
                }
                self.cursor = None;
                if self.keys.contains(&KeyCode::KeyB) {
                    for name in ["BRAKE LEFT POSITION", "BRAKE RIGHT POSITION"] {
                        self.variables.set_number(name, "percent over 100", 0.0);
                    }
                }
                self.keys.clear();
                self.mouse_look = false;
                if let Some(window) = &self.window {
                    window.set_cursor_visible(true);
                    let _ = window.set_cursor_grab(winit::window::CursorGrabMode::None);
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                self.cursor = Some((position.x, position.y));
                if self.pressed_panel.is_some()
                    && !self.mouse_look
                    && let Err(error) = self.cockpit_mouse(MouseEvent::LeftDrag)
                {
                    eprintln!("cockpit drag: {error}");
                }
            }
            WindowEvent::CursorLeft { .. } => self.cursor = None,
            WindowEvent::MouseInput {
                state,
                button: MouseButton::Left,
                ..
            } => {
                if state == ElementState::Released {
                    if let Err(error) = self.cockpit_mouse(MouseEvent::LeftRelease) {
                        eprintln!("cockpit release: {error}");
                    }
                } else if self.arguments.cockpit
                    && !self.mouse_look
                    && let Err(error) = self.cockpit_mouse(MouseEvent::LeftSingle)
                {
                    eprintln!("cockpit click: {error}");
                }
            }
            WindowEvent::MouseWheel { delta, .. } if self.arguments.cockpit && !self.mouse_look => {
                let y = match delta {
                    MouseScrollDelta::LineDelta(_, y) => f64::from(y),
                    MouseScrollDelta::PixelDelta(position) => position.y,
                };
                let event = if y > 0.0 {
                    Some(MouseEvent::WheelUp)
                } else if y < 0.0 {
                    Some(MouseEvent::WheelDown)
                } else {
                    None
                };
                if let Some(event) = event
                    && let Err(error) = self.cockpit_mouse(event)
                {
                    eprintln!("cockpit wheel: {error}");
                }
            }
            WindowEvent::MouseInput {
                state,
                button: MouseButton::Right,
                ..
            } => {
                if state == ElementState::Pressed
                    && let Err(error) = self.cockpit_mouse(MouseEvent::LeftRelease)
                {
                    eprintln!("cockpit release: {error}");
                }
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
                    if key == KeyCode::KeyB {
                        for name in ["BRAKE LEFT POSITION", "BRAKE RIGHT POSITION"] {
                            self.variables.set_number(
                                name,
                                "percent over 100",
                                f64::from(event.state == ElementState::Pressed),
                            );
                        }
                    }
                    if event.state == ElementState::Pressed {
                        self.keys.insert(key);
                        if !event.repeat
                            || matches!(
                                key,
                                KeyCode::F2 | KeyCode::F3 | KeyCode::Numpad7 | KeyCode::Numpad1
                            )
                        {
                            match key {
                                KeyCode::Escape => event_loop.exit(),
                                KeyCode::Tab => {
                                    if let Err(error) = self.cockpit_mouse(MouseEvent::LeftRelease)
                                    {
                                        eprintln!("cockpit release: {error}");
                                    }
                                    self.arguments.cockpit = !self.arguments.cockpit;
                                    self.reset_camera();
                                }
                                KeyCode::Home => self.reset_camera(),
                                KeyCode::KeyO => self.door_open = !self.door_open,
                                KeyCode::F5 | KeyCode::F6 | KeyCode::F7 | KeyCode::F8 => {
                                    let name = match key {
                                        KeyCode::F5 => "FLAPS_UP",
                                        KeyCode::F6 => "FLAPS_DECR",
                                        KeyCode::F7 => "FLAPS_INCR",
                                        _ => "FLAPS_DOWN",
                                    };
                                    if let Err(error) =
                                        self.variables.dispatch_event(name, Value::Number(0.0))
                                    {
                                        eprintln!("flap input: {error}");
                                    }
                                }
                                KeyCode::F1 | KeyCode::F2 | KeyCode::F3 | KeyCode::F4 => {
                                    let event = match key {
                                        KeyCode::F1 => "THROTTLE_CUT",
                                        KeyCode::F2 => "THROTTLE_DECR",
                                        KeyCode::F3 => "THROTTLE_INCR",
                                        _ => "THROTTLE_FULL",
                                    };
                                    if let Err(error) =
                                        self.variables.dispatch_event(event, Value::Number(0.0))
                                    {
                                        eprintln!("throttle input: {error}");
                                    }
                                }
                                KeyCode::Numpad7 | KeyCode::Numpad1 => {
                                    let name = if key == KeyCode::Numpad7 {
                                        "ELEV_TRIM_UP"
                                    } else {
                                        "ELEV_TRIM_DN"
                                    };
                                    if let Err(error) =
                                        self.variables.dispatch_event(name, Value::Number(0.0))
                                    {
                                        eprintln!("trim input: {error}");
                                    }
                                }
                                KeyCode::Slash => {
                                    if let Err(error) = self
                                        .variables
                                        .dispatch_event("SPOILERS_TOGGLE", Value::Number(0.0))
                                    {
                                        eprintln!("spoiler input: {error}");
                                    }
                                }
                                KeyCode::KeyG => {
                                    if let Err(error) = self
                                        .variables
                                        .dispatch_event("GEAR_TOGGLE", Value::Number(0.0))
                                    {
                                        eprintln!("gear input: {error}");
                                    }
                                }
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
    let fsx_root = arguments.fsx_root.clone();
    let airspeed = arguments.airspeed;
    let engines_running = arguments.engines_running;
    let height = arguments.height;
    let fuel_fraction = arguments.fuel_percent / 100.0;
    let payload = arguments.payload;
    navigation::Origin::new(arguments.latitude, arguments.longitude)?;
    let starting_wind = weather::vector(arguments.wind_direction, arguments.wind_speed)?;
    let initial_weather = weather::Conditions::new(
        arguments.sea_level_pressure,
        arguments.sea_level_temperature,
    )?;
    initial_weather.at(height)?;
    let (sender, loaded) = mpsc::sync_channel(1);
    std::thread::Builder::new()
        .name("aircraft loader".into())
        .spawn(move || {
            let loaded = (|| {
                let aircraft = Aircraft::load_with_root(&path, variant, fsx_root.as_deref())?;
                let flight = airspeed.map(|knots| {
                    let data = esp_assets::flight::FlightData::load(&path, variant)?;
                    let mut flight=Flight::new(&data, knots, height,fuel_fraction,payload)?;
                    if engines_running {
                        flight.start_running_with_weather(&data, initial_weather)?;
                    } else if let Err(error) = flight.prepare_engines(&data) {
                        eprintln!("cold engine unavailable: {error:#}");
                    }
                    if height > 0.0 { flight.add_initial_wind(starting_wind); }
                    let limits = SpeedLimits::from_data(&data)?;
                    Ok::<_,anyhow::Error>((flight, limits))
                }).transpose()?;
                if flight.is_some() {
                    eprintln!("flight: standard atmosphere; arrows control elevator/ailerons, Z/X rudder and steering, B brakes, F1-F4 throttle");
                }
                Ok((aircraft, flight))
            })();
            let _ = sender.send(loaded);
        })?;
    let mut variables = Variables::default();
    variables.set_number("WORLD ORIGIN LATITUDE", "degrees", arguments.latitude);
    variables.set_number("WORLD ORIGIN LONGITUDE", "degrees", arguments.longitude);
    variables.set_number(
        "AMBIENT WIND DIRECTION",
        "degrees",
        arguments.wind_direction.rem_euclid(360.0),
    );
    variables.set_number("AMBIENT WIND VELOCITY", "knots", arguments.wind_speed);
    variables.set_number(
        "SEA LEVEL PRESSURE",
        "millibars",
        arguments.sea_level_pressure,
    );
    variables.set_number(
        "SEA LEVEL TEMPERATURE",
        "celsius",
        arguments.sea_level_temperature,
    );
    for event in ["AXIS_AILERONS_SET", "AXIS_ELEVATOR_SET", "AXIS_RUDDER_SET"] {
        variables.dispatch_event(event, Value::Number(0.0))?;
    }
    variables.set('P', "UNITS OF MEASURE", Some("enum"), Value::Number(0.0));
    variables.set_number("AUTOPILOT FLIGHT DIRECTOR ACTIVE", "bool", 0.0);
    variables.set_number("AUTOPILOT HEADING LOCK DIR", "degrees", 0.0);
    let mut app = App {
        arguments,
        loaded,
        window: None,
        renderer: None,
        aircraft: None,
        flight: None,
        speed_limits: None,
        exterior_state: None,
        interior_state: None,
        variables,
        clock: Clock::default(),
        door_open: false,
        reported_diagnostics: HashSet::new(),
        camera: Camera::looking_at(DVec3::ZERO, DVec3::Z),
        keys: HashSet::new(),
        keyboard_axes: [0.0; 3],
        mouse_look: false,
        cursor: None,
        pressed_panel: None,
        pressed_model: None,
        repeat_at: None,
        last_frame: Instant::now(),
        frames: 0,
        error: None,
    };
    EventLoop::new()?.run_app(&mut app)?;
    if let Some(flight) = &app.flight {
        eprintln!("flight state: {:?}", flight.current);
        flight.report(
            weather::wind(&mut app.variables)?,
            weather::conditions(&mut app.variables)?,
        )?;
    }
    if let Some(error) = app.error {
        return Err(error);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use esp_script::{Host, Reference};

    #[test]
    fn accepts_negative_geographic_origin() {
        let args = Arguments::try_parse_from([
            "openesp",
            "--aircraft",
            "test",
            "--latitude",
            "-33.9",
            "--longitude",
            "-151.2",
        ])
        .unwrap();
        assert_eq!(args.latitude, -33.9);
        assert_eq!(args.longitude, -151.2);
    }

    #[test]
    fn accepts_negative_sea_level_temperatures() {
        let args = Arguments::try_parse_from([
            "openesp",
            "--aircraft",
            "test",
            "--sea-level-temperature",
            "-20",
        ])
        .unwrap();
        assert_eq!(args.sea_level_temperature, -20.0);
    }

    #[test]
    fn parses_startup_events_with_finite_optional_values() {
        let event = parse_event("flaps_down").unwrap();
        assert_eq!(event.name, "FLAPS_DOWN");
        assert_eq!(event.value, 0.0);
        let event = parse_event(" FLAPS_SET = 8191.5 ").unwrap();
        assert_eq!(event.value, 8191.5);
        for text in ["", "=1", "FLAPS_SET=NaN", "FLAPS_SET=inf", "FLAPS_SET=1=2"] {
            assert!(parse_event(text).is_err(), "{text}");
        }
    }

    #[test]
    fn releases_captured_panel_without_a_cursor() {
        let root = tempfile::tempdir().unwrap();
        let directory = root.path().join("panel");
        std::fs::create_dir_all(directory.join("example")).unwrap();
        std::fs::write(directory.join("panel.cfg"), "[VCockpit01]\nsize_mm=1,1\npixel_size=1,1\ntexture=$test\ngauge00=example!button,0,0,1,1").unwrap();
        std::fs::write(directory.join("example/button.xml"), r#"<Gauge><Size X="1" Y="1"/><MouseArea><Size X="1" Y="1"/><MouseClick><ClickType>LeftSingle+LeftRelease</ClickType><Script>(M:Event) 'LeftSingle' scmp 0 == if{ 1 (&gt;L:Pressed) } (M:Event) 'LeftRelease' scmp 0 == if{ 0 (&gt;L:Pressed) (L:Releases) ++ (&gt;L:Releases) }</Script></MouseClick></MouseArea></Gauge>"#).unwrap();
        let panel = esp_assets::panel::Panel::load(root.path(), None)
            .unwrap()
            .unwrap();
        let image = esp_assets::gauge::load_panel(&panel, "$test")
            .unwrap()
            .unwrap();
        let areas = image.mouse_areas_at(0.5, 0.5);
        let mut variables = Variables::default();
        image
            .mouse_event(MouseEvent::LeftSingle, &areas, [0.5, 0.5], &mut variables)
            .unwrap();
        let empty_model = || esp_assets::Model {
            nodes: vec![],
            materials: vec![],
            lods: vec![],
            animations: vec![],
            visibility: vec![],
            mouse_rectangles: vec![],
        };
        let (_, loaded) = mpsc::channel();
        let mut app = App {
            arguments: Arguments::parse_from(["openesp", "--aircraft", "unused"]),
            loaded,
            window: None,
            renderer: None,
            aircraft: Some(Aircraft {
                title: "test".into(),
                flaps: vec![],
                electrical: esp_formats::cfg::electrical::Electrical::from_config(
                    &esp_formats::cfg::Config::parse("").unwrap(),
                )
                .unwrap(),
                exterior: empty_model(),
                interior: None,
                panel: Some(panel),
                panel_images: [("$TEST".into(), image)].into(),
                eyepoint: None,
                texture_directories: vec![],
            }),
            flight: None,
            speed_limits: None,
            exterior_state: None,
            interior_state: None,
            variables,
            clock: Clock::default(),
            door_open: false,
            reported_diagnostics: HashSet::new(),
            camera: Camera::looking_at(DVec3::ZERO, DVec3::Z),
            keys: HashSet::new(),
            keyboard_axes: [0.0; 3],
            mouse_look: true,
            cursor: None,
            pressed_panel: Some(("$TEST".into(), areas, [0.5, 0.5])),
            pressed_model: None,
            repeat_at: Some(Instant::now()),
            last_frame: Instant::now(),
            frames: 0,
            error: None,
        };
        app.cockpit_mouse(MouseEvent::LeftRelease).unwrap();
        assert!(app.pressed_panel.is_none());
        assert!(app.repeat_at.is_none());
        assert_eq!(
            app.variables
                .read(&Reference::new('L', "Pressed", None))
                .unwrap(),
            Value::Number(0.0)
        );
        assert_eq!(
            app.variables
                .read(&Reference::new('L', "Releases", None))
                .unwrap(),
            Value::Number(1.0)
        );
        app.cockpit_mouse(MouseEvent::LeftRelease).unwrap();
        assert_eq!(
            app.variables
                .read(&Reference::new('L', "Releases", None))
                .unwrap(),
            Value::Number(1.0)
        );
        let mut model = empty_model();
        model
            .mouse_rectangles
            .push(esp_assets::ModelMouseRectangle {
                flags: 1 << 17,
                key_event: Some(0x102f0),
            });
        app.aircraft.as_mut().unwrap().interior = Some(model);
        app.pressed_model = Some(0);
        app.repeat_at = Some(Instant::now());
        app.cockpit_mouse(MouseEvent::LeftRelease).unwrap();
        assert!(app.pressed_model.is_none());
        assert!(app.repeat_at.is_none());
        let reference = Reference::new('A', "AUTOPILOT FLIGHT DIRECTOR ACTIVE", Some("bool"));
        assert_eq!(app.variables.read(&reference).unwrap(), Value::Number(1.0));
        app.cockpit_mouse(MouseEvent::LeftRelease).unwrap();
        assert_eq!(app.variables.read(&reference).unwrap(), Value::Number(1.0));
    }
}

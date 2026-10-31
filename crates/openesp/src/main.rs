use anyhow::{Context, Result};
use clap::Parser;
use esp_assets::{Aircraft, gauge::MouseEvent};
use esp_render::{Camera, Renderer};
use esp_script::Value;
use esp_sim::{animation::ModelState, picking, variables::Variables};
use glam::{DVec3, Vec3};
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
    cursor: Option<(f64, f64)>,
    pressed_panel: Option<(String, Vec<usize>, [f64; 2])>,
    repeat_at: Option<Instant>,
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

    fn cockpit_target(&self) -> Option<(String, f64, f64)> {
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
        let hit = picking::pick(interior, state, aircraft_to_camera, ray_origin, ray_dir)?;
        let mesh = &interior.lods[0].meshes[hit.mesh];
        let key = interior
            .materials
            .get(mesh.material)?
            .diffuse_texture
            .as_ref()?
            .to_ascii_uppercase();
        let panel = aircraft.panel_images.get(&key)?;
        Some((
            key,
            f64::from(hit.uv[0].rem_euclid(1.0)) * f64::from(panel.image.width()),
            f64::from(hit.uv[1].rem_euclid(1.0)) * f64::from(panel.image.height()),
        ))
    }

    fn cockpit_mouse(&mut self, event: MouseEvent) -> Result<()> {
        let target = if event == MouseEvent::LeftRelease {
            self.repeat_at = None;
            self.pressed_panel.take()
        } else if matches!(event, MouseEvent::LeftDrag | MouseEvent::DownRepeat) {
            let Some((name, x, y)) = self.cockpit_target() else {
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
            self.cockpit_target().and_then(|(name, x, y)| {
                let panel = self.aircraft.as_ref()?.panel_images.get(&name)?;
                Some((name, panel.mouse_areas_at(x, y), [x, y]))
            })
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
        if let Some(aircraft) = &mut self.aircraft {
            if let Some(state) = &mut self.exterior_state {
                state.update(&aircraft.exterior, &mut self.variables, dt)?;
            }
            if let (Some(model), Some(state)) = (&aircraft.interior, &mut self.interior_state) {
                state.update(model, &mut self.variables, dt)?;
            }
            if let Some(renderer) = &self.renderer {
                for (name, panel) in &mut aircraft.panel_images {
                    if panel.update(&mut self.variables) {
                        renderer.update_panel(
                            name,
                            panel.image.width(),
                            panel.image.height(),
                            panel.image.as_raw(),
                        )?;
                    }
                }
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
                if let Err(error) = self.cockpit_mouse(MouseEvent::LeftRelease) {
                    eprintln!("cockpit release: {error}");
                }
                self.cursor = None;
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
                    if event.state == ElementState::Pressed {
                        self.keys.insert(key);
                        if !event.repeat {
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
    let (sender, loaded) = mpsc::sync_channel(1);
    std::thread::Builder::new()
        .name("aircraft loader".into())
        .spawn(move || {
            let _ = sender.send(Aircraft::load_with_root(
                &path,
                variant,
                fsx_root.as_deref(),
            ));
        })?;
    let mut variables = Variables::default();
    variables.set_number("AUTOPILOT FLIGHT DIRECTOR ACTIVE", "bool", 0.0);
    let mut app = App {
        arguments,
        loaded,
        window: None,
        renderer: None,
        aircraft: None,
        exterior_state: None,
        interior_state: None,
        variables,
        door_open: false,
        reported_diagnostics: HashSet::new(),
        camera: Camera::looking_at(DVec3::ZERO, DVec3::Z),
        keys: HashSet::new(),
        mouse_look: false,
        cursor: None,
        pressed_panel: None,
        repeat_at: None,
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

#[cfg(test)]
mod tests {
    use super::*;
    use esp_script::{Host, Reference};

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
        };
        let (_, loaded) = mpsc::channel();
        let mut app = App {
            arguments: Arguments::parse_from(["openesp", "--aircraft", "unused"]),
            loaded,
            window: None,
            renderer: None,
            aircraft: Some(Aircraft {
                title: "test".into(),
                exterior: empty_model(),
                interior: None,
                panel: Some(panel),
                panel_images: [("$TEST".into(), image)].into(),
                eyepoint: None,
                texture_directories: vec![],
            }),
            exterior_state: None,
            interior_state: None,
            variables,
            door_open: false,
            reported_diagnostics: HashSet::new(),
            camera: Camera::looking_at(DVec3::ZERO, DVec3::Z),
            keys: HashSet::new(),
            mouse_look: true,
            cursor: None,
            pressed_panel: Some(("$TEST".into(), areas, [0.5, 0.5])),
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
    }
}

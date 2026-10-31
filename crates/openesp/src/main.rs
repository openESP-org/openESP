use anyhow::{Context, Result};
use clap::Parser;
use esp_assets::Aircraft;
use esp_render::{Camera, Renderer};
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
}

struct App {
    arguments: Arguments,
    loaded: mpsc::Receiver<Result<Aircraft>>,
    window: Option<Arc<Window>>,
    renderer: Option<Renderer>,
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
                    target + DVec3::new(0.9, 0.4, 1.4) * f64::from(renderer.radius),
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
        if let Some(renderer) = &mut self.renderer {
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
            }
            WindowEvent::MouseInput {
                state,
                button: MouseButton::Right,
                ..
            } => {
                self.mouse_look = state == ElementState::Pressed;
                if let Some(window) = &self.window {
                    window.set_cursor_visible(!self.mouse_look);
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

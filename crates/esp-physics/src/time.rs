use crate::variables::VariableStore;
use anyhow::{Result, ensure};
use esp_script::Value;

pub const STEP: f64 = 1.0 / 60.0;
const MAX_FRAME: f64 = 0.25;

pub struct SimTime {
    utc_seconds: f64,
    elapsed: f64,
}

impl SimTime {
    pub fn new(utc_seconds: f64) -> Result<Self> {
        ensure!(
            utc_seconds.is_finite() && (0.0..86_400.0).contains(&utc_seconds),
            "UTC time must be within a day"
        );
        Ok(Self {
            utc_seconds,
            elapsed: 0.0,
        })
    }

    pub fn now() -> Result<Self> {
        let seconds = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)?
            .as_secs_f64();
        Self::new(seconds.rem_euclid(86_400.0))
    }

    pub fn advance(&mut self, dt: f64) -> Result<()> {
        ensure!(
            dt.is_finite() && dt >= 0.0 && (self.elapsed + dt).is_finite(),
            "invalid simulation time step"
        );
        self.utc_seconds = (self.utc_seconds + dt.rem_euclid(86_400.0)).rem_euclid(86_400.0);
        self.elapsed += dt;
        Ok(())
    }

    pub fn publish(&self, variables: &mut dyn VariableStore) {
        variables.set(
            'E',
            "ZULU TIME",
            Some("seconds"),
            Value::Number(self.utc_seconds),
        );
        variables.set(
            'E',
            "SIMULATION TIME",
            Some("seconds"),
            Value::Number(self.elapsed),
        );
    }
}

#[derive(Default)]
pub struct Clock {
    remaining: f64,
}

pub struct Frame {
    pub steps: u32,
    pub alpha: f64,
    pub elapsed: f64,
    pub discarded: f64,
}

impl Clock {
    pub fn advance(&mut self, elapsed: f64) -> Result<Frame> {
        ensure!(
            elapsed.is_finite() && elapsed >= 0.0,
            "invalid frame duration"
        );
        let accepted = elapsed.min(MAX_FRAME);
        let available = self.remaining + accepted;
        let steps = (available / STEP + 1e-10).floor() as u32;
        self.remaining = (available - f64::from(steps) * STEP).max(0.0);
        Ok(Frame {
            steps,
            alpha: (self.remaining / STEP).clamp(0.0, 1.0),
            elapsed: accepted,
            discarded: elapsed - accepted,
        })
    }
}

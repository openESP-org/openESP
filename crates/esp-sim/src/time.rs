use anyhow::{Result, ensure};

pub const STEP: f64 = 1.0 / 60.0;
const MAX_FRAME: f64 = 0.25;

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn partitions_render_frames_into_identical_simulation_ticks() {
        for fps in [24, 30, 60, 144, 240] {
            let mut clock = Clock::default();
            let mut total = 0;
            for _ in 0..fps * 10 {
                let frame = clock.advance(1.0 / f64::from(fps)).unwrap();
                assert!((0.0..=1.0).contains(&frame.alpha));
                assert_eq!(frame.discarded, 0.0);
                total += frame.steps;
            }
            assert_eq!(total, 600, "{fps} fps");
            assert!(clock.advance(0.0).unwrap().alpha < 1e-8);
        }
    }

    #[test]
    fn retains_fractional_time_and_bounds_work_after_stalls() {
        let mut clock = Clock::default();
        let frame = clock.advance(STEP * 0.5).unwrap();
        assert_eq!(frame.steps, 0);
        assert!((frame.alpha - 0.5).abs() < 1e-12);
        let frame = clock.advance(1.0).unwrap();
        assert_eq!(frame.steps, 15);
        assert!((frame.alpha - 0.5).abs() < 1e-12);
        assert_eq!(frame.discarded, 0.75);
        assert_eq!(clock.advance(0.0).unwrap().steps, 0);
        for value in [f64::NAN, f64::INFINITY, -1.0] {
            assert!(clock.advance(value).is_err());
        }
        assert!((clock.advance(0.0).unwrap().alpha - 0.5).abs() < 1e-12);
    }
}

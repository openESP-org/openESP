use crate::variables::VariableStore;
use anyhow::{Result, ensure};
use esp_script::Value;

#[cfg(test)]
mod calendar_tests {
    use super::SimTime;

    #[test]
    fn utc_month_handles_year_boundaries_and_leap_days() {
        for (timestamp, month) in [
            (-1.0, 12),
            (0.0, 1),
            (951_782_400.0, 2),
            (951_868_800.0, 3),
            (1_704_067_200.0, 1),
            (1_709_251_200.0, 3),
            (1_735_689_600.0, 1),
        ] {
            let mut time = SimTime::new(0.0).unwrap();
            time.unix_day = (timestamp / 86_400.0_f64).floor();
            assert_eq!(time.utc_month(), month, "timestamp {timestamp}");
        }
    }
}

pub const STEP: f64 = 1.0 / 60.0;
const MAX_FRAME: f64 = 0.25;

pub struct SimTime {
    utc_seconds: f64,
    unix_day: f64,
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
            unix_day: 0.0,
            elapsed: 0.0,
        })
    }

    pub fn now() -> Result<Self> {
        let seconds = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)?
            .as_secs_f64();
        let mut result = Self::new(seconds.rem_euclid(86_400.0))?;
        result.unix_day = (seconds / 86_400.0).floor();
        Ok(result)
    }

    /// Replace time of day while keeping the current simulated UTC date.
    pub fn set_utc(&mut self, utc_seconds: f64) -> Result<()> {
        ensure!(
            utc_seconds.is_finite() && (0.0..86_400.0).contains(&utc_seconds),
            "UTC time must be within a day"
        );
        self.utc_seconds = utc_seconds;
        Ok(())
    }

    pub fn unix_seconds(&self) -> f64 {
        self.unix_day * 86_400.0 + self.utc_seconds
    }

    /// UTC calendar month, numbered January = 1 through December = 12.
    pub fn utc_month(&self) -> u8 {
        let days = (self.unix_day as i64).rem_euclid(146_097) + 719_468;
        let day_of_era = days.rem_euclid(146_097);
        let year_of_era =
            (day_of_era - day_of_era / 1460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
        let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
        let march_month = (5 * day_of_year + 2) / 153;
        (march_month + if march_month < 10 { 3 } else { -9 }) as u8
    }

    pub fn advance(&mut self, dt: f64) -> Result<()> {
        ensure!(
            dt.is_finite() && dt >= 0.0 && (self.elapsed + dt).is_finite(),
            "invalid simulation time step"
        );
        let next = self.utc_seconds + dt;
        let day = self.unix_day + (next / 86_400.0).floor();
        let utc = next.rem_euclid(86_400.0);
        ensure!(
            next.is_finite() && (day * 86_400.0 + utc).is_finite(),
            "invalid simulation time step"
        );
        self.unix_day = day;
        self.utc_seconds = utc;
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

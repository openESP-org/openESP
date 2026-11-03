use anyhow::{Result, ensure};

const RADIUS: f64 = 6_356_766.0;
const GAS_CONSTANT: f64 = 8314.32 / 28.9644;
const GRAVITY: f64 = 9.80665;
const SEA_TEMPERATURE: f64 = 288.15;
const SEA_PRESSURE: f64 = 101_325.0;

#[derive(Debug, Clone, Copy)]
pub struct Atmosphere {
    pub temperature: f64,
    pub pressure: f64,
    pub density: f64,
    pub sound_speed: f64,
}

impl Atmosphere {
    // U.S. Standard Atmosphere 1976, lower atmosphere; SI units.
    pub fn standard(altitude: f64) -> Result<Self> {
        ensure!(
            altitude.is_finite() && (-2000.0..=86_000.0).contains(&altitude),
            "altitude is outside the standard atmosphere model (-2 to 86 km)"
        );
        let height = RADIUS * altitude / (RADIUS + altitude);
        let mut base = 0.0;
        let mut temperature = SEA_TEMPERATURE;
        let mut pressure = SEA_PRESSURE;
        for (ceiling, lapse) in [
            (11_000.0, -0.0065),
            (20_000.0, 0.0),
            (32_000.0, 0.001),
            (47_000.0, 0.0028),
            (51_000.0, 0.0),
            (71_000.0, -0.0028),
            (f64::INFINITY, -0.002),
        ] {
            let thickness = height.min(ceiling) - base;
            let next_temperature = temperature + lapse * thickness;
            pressure *= if lapse == 0.0 {
                (-GRAVITY * thickness / (GAS_CONSTANT * temperature)).exp()
            } else {
                (next_temperature / temperature).powf(-GRAVITY / (GAS_CONSTANT * lapse))
            };
            temperature = next_temperature;
            if height <= ceiling {
                break;
            }
            base = ceiling;
        }
        Ok(Self {
            temperature,
            pressure,
            density: pressure / (GAS_CONSTANT * temperature),
            sound_speed: (1.4 * GAS_CONSTANT * temperature).sqrt(),
        })
    }

    pub fn calibrated_airspeed(&self, true_airspeed: f64) -> Result<f64> {
        Self::calibrated_airspeed_from_impact_pressure(self.impact_pressure(true_airspeed)?)
    }

    pub fn impact_pressure(&self, true_airspeed: f64) -> Result<f64> {
        ensure!(
            true_airspeed.is_finite() && true_airspeed >= 0.0,
            "invalid true airspeed"
        );
        ensure!(
            self.pressure.is_finite()
                && self.pressure > 0.0
                && self.sound_speed.is_finite()
                && self.sound_speed > 0.0,
            "invalid air data atmosphere"
        );
        let pressure = self.pressure * impact_ratio(true_airspeed / self.sound_speed);
        ensure!(
            pressure.is_finite() && pressure >= 0.0,
            "airspeed exceeds the air data model range"
        );
        Ok(pressure)
    }

    pub fn calibrated_airspeed_from_impact_pressure(pressure: f64) -> Result<f64> {
        ensure!(
            pressure.is_finite() && pressure >= 0.0,
            "invalid impact pressure"
        );
        let ratio = pressure / SEA_PRESSURE;
        ensure!(
            ratio.is_finite(),
            "airspeed exceeds the air data model range"
        );
        let sea_mach = if ratio <= impact_ratio(1.0) {
            (5.0 * (ratio.ln_1p() / 3.5).exp_m1()).sqrt()
        } else {
            let mut low = 1.0;
            let mut high = 2.0 * ratio.sqrt() + 1.0;
            for _ in 0..64 {
                let middle = (low + high) * 0.5;
                if impact_ratio(middle) < ratio {
                    low = middle;
                } else {
                    high = middle;
                }
            }
            (low + high) * 0.5
        };
        let calibrated = sea_mach * (1.4 * GAS_CONSTANT * SEA_TEMPERATURE).sqrt();
        ensure!(
            calibrated.is_finite(),
            "airspeed exceeds the air data model range"
        );
        Ok(calibrated)
    }
}

fn impact_ratio(mach: f64) -> f64 {
    let square = mach * mach;
    if mach <= 1.0 {
        (3.5 * (0.2 * square).ln_1p()).exp_m1()
    } else {
        square * 1.2f64.powf(3.5) * (6.0 / (7.0 - 1.0 / square)).powf(2.5) - 1.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_published_layer_boundary_conditions() {
        for (height, temperature, pressure) in [
            (0.0, 288.15, 101_325.0),
            (11_000.0, 216.65, 22632.06),
            (20_000.0, 216.65, 5474.889),
            (32_000.0, 228.65, 868.0187),
            (47_000.0, 270.65, 110.9063),
            (51_000.0, 270.65, 66.93887),
            (71_000.0, 214.65, 3.956420),
            (84_852.0, 186.946, 0.3733836),
        ] {
            let geometric = RADIUS * height / (RADIUS - height);
            let air = Atmosphere::standard(geometric).unwrap();
            assert!((air.temperature - temperature).abs() < 1e-8);
            assert!(
                (air.pressure / pressure - 1.0).abs() < 2e-6,
                "height={height}, pressure={}",
                air.pressure
            );
        }
        let sea = Atmosphere::standard(0.0).unwrap();
        assert!((sea.density - 1.225).abs() < 2e-6);
        assert!((sea.sound_speed - 340.294).abs() < 0.001);
    }

    #[test]
    fn remains_continuous_across_layers_and_density_decreases() {
        let mut previous = Atmosphere::standard(-2000.0).unwrap();
        for altitude in (-1999..=86_000).step_by(100) {
            let air = Atmosphere::standard(altitude as f64).unwrap();
            assert!(air.pressure < previous.pressure && air.density < previous.density);
            previous = air;
        }
        for height in [11_000.0, 20_000.0, 32_000.0, 47_000.0, 51_000.0, 71_000.0] {
            let geometric = RADIUS * height / (RADIUS - height);
            let below = Atmosphere::standard(geometric - 0.001).unwrap();
            let above = Atmosphere::standard(geometric + 0.001).unwrap();
            assert!((below.pressure / above.pressure - 1.0).abs() < 1e-6);
            assert!((below.temperature - above.temperature).abs() < 1e-4);
        }
        assert!(Atmosphere::standard(86_000.0).is_ok());
        for altitude in [f64::NAN, f64::INFINITY, -2001.0, 86_001.0] {
            assert!(Atmosphere::standard(altitude).is_err());
        }
    }

    #[test]
    fn calibrated_airspeed_matches_sea_level_and_accounts_for_altitude() {
        let sea = Atmosphere::standard(0.0).unwrap();
        let high = Atmosphere::standard(10_000.0).unwrap();
        for speed in [0.0, 0.001, 50.0, 200.0, sea.sound_speed, 500.0, 1000.0] {
            assert!((sea.calibrated_airspeed(speed).unwrap() - speed).abs() < 1e-9);
        }
        let calibrated = high.calibrated_airspeed(200.0).unwrap();
        let equivalent = 200.0 * (high.density / sea.density).sqrt();
        assert!(calibrated > equivalent && calibrated < 200.0);
        let below = high
            .calibrated_airspeed(high.sound_speed * (1.0 - 1e-8))
            .unwrap();
        let above = high
            .calibrated_airspeed(high.sound_speed * (1.0 + 1e-8))
            .unwrap();
        assert!((above - below).abs() < 1e-4);
        assert!(high.calibrated_airspeed(-1.0).is_err());
    }
}

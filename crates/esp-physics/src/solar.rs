//! Approximate solar direction from UTC and geographic coordinates.

use anyhow::{Result, ensure};
use glam::DVec3;

/// Direction toward the sun in local north, east, up axes.
///
/// The apparent equatorial coordinates use the US Naval Observatory's
/// approximation, valid to roughly one arcminute near the year 2000.
pub fn direction(latitude: f64, longitude: f64, unix_seconds: f64) -> Result<DVec3> {
    ensure!(
        latitude.is_finite()
            && (-90.0..=90.0).contains(&latitude)
            && longitude.is_finite()
            && (-180.0..=180.0).contains(&longitude)
            && unix_seconds.is_finite(),
        "invalid solar position input"
    );
    let days = unix_seconds / 86_400.0 - 10_957.5;
    let anomaly = (357.529 + 0.98560028 * days).rem_euclid(360.0).to_radians();
    let mean = (280.459 + 0.98564736 * days).rem_euclid(360.0);
    let longitude_sun = (mean + 1.915 * anomaly.sin() + 0.020 * (2.0 * anomaly).sin()).to_radians();
    let obliquity = (23.439 - 0.00000036 * days).to_radians();
    let right_ascension = (obliquity.cos() * longitude_sun.sin()).atan2(longitude_sun.cos());
    let declination = (obliquity.sin() * longitude_sun.sin()).asin();
    let equation = (mean.to_radians() - right_ascension + std::f64::consts::PI)
        .rem_euclid(std::f64::consts::TAU)
        - std::f64::consts::PI;
    let hour_angle =
        (unix_seconds.rem_euclid(86_400.0) / 240.0 + longitude - 180.0).to_radians() + equation;
    let latitude = latitude.to_radians();
    Ok(DVec3::new(
        latitude.cos() * declination.sin() - latitude.sin() * declination.cos() * hour_angle.cos(),
        -declination.cos() * hour_angle.sin(),
        latitude.sin() * declination.sin() + latitude.cos() * declination.cos() * hour_angle.cos(),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sun_tracks_time_longitude_and_seasons() {
        let equinox = 1_710_936_000.0;
        let noon = direction(0.0, 0.0, equinox).unwrap();
        assert!(noon.z > 0.999 && (noon.length() - 1.0).abs() < 1e-12);
        let morning = direction(0.0, 0.0, equinox - 6.0 * 3600.0).unwrap();
        assert!(morning.y > 0.99 && morning.z.abs() < 0.05);
        let shifted = direction(0.0, 90.0, equinox - 6.0 * 3600.0).unwrap();
        assert!(shifted.abs_diff_eq(noon, 0.002));
        let summer = direction(51.5, 0.0, 1_718_971_200.0)
            .unwrap()
            .z
            .asin()
            .to_degrees();
        let winter = direction(51.5, 0.0, 1_734_782_400.0)
            .unwrap()
            .z
            .asin()
            .to_degrees();
        assert!((summer - 61.9).abs() < 0.3);
        assert!((winter - 15.1).abs() < 0.3);
        assert!(direction(f64::NAN, 0.0, equinox).is_err());
    }
}

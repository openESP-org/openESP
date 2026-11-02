use anyhow::{Context, Result, ensure};
use esp_formats::cfg::{self, Config};
use glam::{DMat3, DVec3};

use crate::flight::MassProperties;

const FOOT: f64 = 0.3048;
const POUND: f64 = 0.45359237;

pub struct WeightBalance {
    empty: MassProperties,
    empty_cg: DVec3,
    datum: DVec3,
    stations: Vec<cfg::loading::Station>,
    tanks: Vec<cfg::loading::Tank>,
    fuel_density: f64,
}

pub struct Loading {
    pub mass: MassProperties,
    pub cg: DVec3,
    pub empty_kilograms: f64,
    pub payload_kilograms: f64,
    pub fuel_gallons: f64,
    pub usable_fuel_gallons: f64,
    pub tank_gallons: Vec<f64>,
    pub fuel_capacity_gallons: f64,
    pub fuel_pounds_per_gallon: f64,
}

fn body([x, y, z]: [f64; 3]) -> DVec3 {
    DVec3::new(x, y, -z) * FOOT
}

fn parallel_axis(mass: f64, offset: DVec3) -> DMat3 {
    (DMat3::IDENTITY * offset.length_squared()
        - DMat3::from_cols(offset * offset.x, offset * offset.y, offset * offset.z))
        * mass
}

impl WeightBalance {
    pub fn tanks(&self) -> &[cfg::loading::Tank] {
        &self.tanks
    }

    pub fn from_config(config: &Config) -> Result<Self> {
        let weight = |key| -> Result<f64> {
            config
                .number("weight_and_balance", key)?
                .with_context(|| format!("missing [weight_and_balance] {key}"))
        };
        let mass = weight("empty_weight")? * POUND;
        let unit = 1.3558179483314004;
        let coupled = weight("empty_weight_coupled_MOI")? * unit;
        let inertia = DMat3::from_cols(
            DVec3::new(weight("empty_weight_roll_MOI")? * unit, 0.0, -coupled),
            DVec3::Y * weight("empty_weight_pitch_MOI")? * unit,
            DVec3::new(-coupled, 0.0, weight("empty_weight_yaw_MOI")? * unit),
        );
        let reference = cfg::aircraft::ReferencePoints::from_config(config)?;
        let datum = body(reference.datum_feet.context("missing reference datum")?);
        let empty_cg = datum
            + body(
                reference
                    .empty_cg_feet
                    .context("missing empty aircraft CG")?,
            );
        let tanks = cfg::loading::tanks(config)?;
        let fuel_density = if tanks.is_empty() {
            0.0
        } else {
            match config.number("fuel", "fuel_type")? {
                Some(1.0) => 6.0,
                Some(2.0) => 6.7,
                _ => anyhow::bail!("missing or unsupported aircraft fuel type"),
            }
        };
        Ok(Self {
            empty: MassProperties::new(mass, inertia)?,
            empty_cg,
            datum,
            stations: cfg::loading::stations(config)?,
            tanks,
            fuel_density,
        })
    }

    pub fn load(&self, fuel_fraction: f64, payload: bool) -> Result<Loading> {
        ensure!(
            fuel_fraction.is_finite() && (0.0..=1.0).contains(&fuel_fraction),
            "invalid fuel fraction"
        );
        ensure!(
            fuel_fraction == 0.0 || !self.tanks.is_empty(),
            "aircraft has no fuel tanks"
        );
        let quantities = self
            .tanks
            .iter()
            .map(|tank| (tank.capacity_gallons + tank.unusable_gallons) * fuel_fraction)
            .collect();
        self.load_tanks(quantities, payload)
    }

    pub fn burn(&self, loading: &Loading, gallons: f64, payload: bool) -> Result<Loading> {
        ensure!(
            gallons.is_finite() && gallons >= 0.0,
            "invalid fuel consumption"
        );
        let available = loading.usable_fuel_gallons;
        let fraction = if available > 0.0 {
            (gallons / available).min(1.0)
        } else {
            0.0
        };
        let quantities = loading
            .tank_gallons
            .iter()
            .zip(&self.tanks)
            .map(|(&quantity, tank)| {
                if fraction == 1.0 {
                    quantity.min(tank.unusable_gallons)
                } else {
                    quantity - (quantity - tank.unusable_gallons).max(0.0) * fraction
                }
            })
            .collect();
        self.load_tanks(quantities, payload)
    }

    fn tank_priority(selector: u32, name: &str) -> Option<u8> {
        match (selector, name) {
            (1, _) => Some(0),
            (2, "LeftTip") | (3, "RightTip") => Some(0),
            (2, "LeftAux") | (3, "RightAux") => Some(1),
            (2, "LeftMain") | (3, "RightMain") => Some(2),
            (4, "LeftAux")
            | (5, "RightAux")
            | (6, "Center1")
            | (7, "Center2")
            | (8, "Center3")
            | (9, "External1")
            | (10, "External2")
            | (11, "RightTip")
            | (12, "LeftTip")
            | (19, "LeftMain")
            | (20, "RightMain") => Some(0),
            (16, "LeftMain" | "RightMain") | (17, "External1" | "External2") => Some(0),
            _ => None,
        }
    }

    pub fn selected_fuel(&self, loading: &Loading, selector: u32) -> f64 {
        self.tanks
            .iter()
            .zip(&loading.tank_gallons)
            .filter(|(tank, _)| Self::tank_priority(selector, tank.name).is_some())
            .map(|(tank, &quantity)| (quantity - tank.unusable_gallons).max(0.0))
            .sum()
    }

    pub fn burn_selected(
        &self,
        loading: &Loading,
        selector: u32,
        gallons: f64,
        payload: bool,
    ) -> Result<Loading> {
        ensure!(
            gallons.is_finite() && gallons >= 0.0,
            "invalid fuel consumption"
        );
        let mut quantities = loading.tank_gallons.clone();
        let mut remaining = gallons;
        for priority in 0..=2 {
            let available: f64 = self
                .tanks
                .iter()
                .zip(&quantities)
                .filter(|(tank, _)| Self::tank_priority(selector, tank.name) == Some(priority))
                .map(|(tank, &quantity)| (quantity - tank.unusable_gallons).max(0.0))
                .sum();
            if available == 0.0 {
                continue;
            }
            let fraction = (remaining / available).min(1.0);
            for (tank, quantity) in self.tanks.iter().zip(&mut quantities) {
                if Self::tank_priority(selector, tank.name) == Some(priority) {
                    if fraction == 1.0 {
                        *quantity = quantity.min(tank.unusable_gallons);
                    } else {
                        *quantity -= (*quantity - tank.unusable_gallons).max(0.0) * fraction;
                    }
                }
            }
            remaining = (remaining - available).max(0.0);
        }
        self.load_tanks(quantities, payload)
    }

    fn load_tanks(&self, quantities: Vec<f64>, payload: bool) -> Result<Loading> {
        ensure!(
            quantities.len() == self.tanks.len(),
            "invalid tank quantities"
        );
        let stations = || {
            self.stations
                .iter()
                .filter(|_| payload)
                .map(|s| (s.pounds * POUND, self.datum + body(s.position_feet)))
        };
        let payload_kilograms = stations().map(|(mass, _)| mass).sum();
        let fuel_capacity_gallons = self
            .tanks
            .iter()
            .map(|t| t.capacity_gallons + t.unusable_gallons)
            .sum::<f64>();
        let fuel_gallons = quantities.iter().sum();
        let usable_fuel_gallons = self
            .tanks
            .iter()
            .zip(&quantities)
            .map(|(tank, &quantity)| (quantity - tank.unusable_gallons).max(0.0))
            .sum();
        let points = || {
            stations().chain(self.tanks.iter().zip(&quantities).map(|(t, &quantity)| {
                (
                    quantity * self.fuel_density * POUND,
                    self.datum + body(t.position_feet),
                )
            }))
        };
        let mass = self.empty.kilograms() + points().map(|(mass, _)| mass).sum::<f64>();
        let cg = (self.empty_cg * self.empty.kilograms()
            + points().map(|(mass, pos)| pos * mass).sum::<DVec3>())
            / mass;
        let inertia = self.empty.inertia
            + parallel_axis(self.empty.kilograms(), self.empty_cg - cg)
            + points()
                .map(|(mass, pos)| parallel_axis(mass, pos - cg))
                .sum::<DMat3>();
        Ok(Loading {
            mass: MassProperties::new(mass, inertia)?,
            cg,
            empty_kilograms: self.empty.kilograms(),
            payload_kilograms,
            fuel_gallons,
            usable_fuel_gallons,
            tank_gallons: quantities,
            fuel_capacity_gallons,
            fuel_pounds_per_gallon: self.fuel_density,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> Config {
        Config::parse("[weight_and_balance]\nempty_weight=1000\nempty_weight_roll_MOI=100\nempty_weight_pitch_MOI=200\nempty_weight_yaw_MOI=250\nempty_weight_coupled_MOI=0\nreference_datum_position=2,0,0\nempty_weight_CG_position=-2,0,0\nstation_load.0=200,8,0,0,Pilot\n[fuel]\nfuel_type=1\nleftmain=-2,-5,1,10,0\nrightmain=-2,5,1,10,0").unwrap()
    }

    #[test]
    fn shifts_cg_and_applies_parallel_axis_inertia() {
        let weights = WeightBalance::from_config(&config()).unwrap();
        let empty = weights.load(0.0, false).unwrap();
        assert_eq!(empty.cg, DVec3::ZERO);
        let loaded = weights.load(0.0, true).unwrap();
        let expected_cg = 200.0 * 10.0 / 1200.0 * FOOT;
        assert!(loaded.cg.abs_diff_eq(DVec3::X * expected_cg, 1e-12));
        let extra_pitch = 1000.0 * POUND * expected_cg.powi(2)
            + 200.0 * POUND * (10.0 * FOOT - expected_cg).powi(2);
        assert!(
            (loaded.mass.inertia.y_axis.y - empty.mass.inertia.y_axis.y - extra_pitch).abs() < 1e-9
        );
        assert!((loaded.mass.inertia.x_axis.x - empty.mass.inertia.x_axis.x).abs() < 1e-12);
    }

    #[test]
    fn loads_symmetric_fuel_and_bounds_quantities() {
        let weights = WeightBalance::from_config(&config()).unwrap();
        let empty = weights.load(0.0, false).unwrap();
        let full = weights.load(1.0, false).unwrap();
        assert!((full.mass.kilograms() - (1000.0 + 20.0 * 6.0) * POUND).abs() < 1e-10);
        assert_eq!(full.fuel_gallons, 20.0);
        assert!(full.cg.y.abs() < 1e-12 && full.cg.z < 0.0);
        assert!(full.mass.inertia.x_axis.x > empty.mass.inertia.x_axis.x);
        assert!(weights.load(1.0, true).unwrap().mass.inertia.x_axis.z < 0.0);
        assert!(weights.load(-0.1, false).is_err());
        assert!(weights.load(f64::NAN, false).is_err());
    }

    #[test]
    fn fuel_burn_preserves_unusable_quantities_and_their_mass() {
        let mut weights = WeightBalance::from_config(&config()).unwrap();
        weights.tanks[0].unusable_gallons = 2.0;
        weights.tanks[1].unusable_gallons = 1.0;
        let full = weights.load(1.0, false).unwrap();
        assert_eq!(full.fuel_gallons, 23.0);
        assert_eq!(full.usable_fuel_gallons, 20.0);
        let half = weights.burn(&full, 10.0, false).unwrap();
        assert_eq!(half.tank_gallons, [7.0, 6.0]);
        let exhausted = weights.burn(&half, 100.0, false).unwrap();
        assert_eq!(exhausted.tank_gallons, [2.0, 1.0]);
        assert_eq!(exhausted.usable_fuel_gallons, 0.0);
        assert_eq!(exhausted.fuel_gallons, 3.0);
        assert!(exhausted.cg.y < 0.0);
        assert!((exhausted.mass.kilograms() - (1000.0 + 18.0) * POUND).abs() < 1e-10);
    }

    #[test]
    fn selector_routes_fuel_in_priority_order_and_preserves_other_tanks() {
        let source = "[weight_and_balance]\nempty_weight=1000\nempty_weight_roll_MOI=100\nempty_weight_pitch_MOI=200\nempty_weight_yaw_MOI=250\nempty_weight_coupled_MOI=0\nreference_datum_position=0,0,0\nempty_weight_CG_position=0,0,0\n[fuel]\nfuel_type=1\nlefttip=0,-8,0,1,0.5\nleftaux=0,-5,0,2,0\nleftmain=0,-2,0,3,0\nrightmain=0,2,0,3,0";
        let weights = WeightBalance::from_config(&Config::parse(source).unwrap()).unwrap();
        let full = weights.load(1.0, false).unwrap();
        let left = weights.burn_selected(&full, 2, 2.0, false).unwrap();
        let amount = |name| {
            left.tank_gallons[weights
                .tanks()
                .iter()
                .position(|tank| tank.name == name)
                .unwrap()]
        };
        assert_eq!(amount("LeftTip"), 0.5);
        assert_eq!(amount("LeftAux"), 1.0);
        assert_eq!(amount("LeftMain"), 3.0);
        assert_eq!(amount("RightMain"), 3.0);
        assert_eq!(weights.selected_fuel(&left, 2), 4.0);
        assert_eq!(weights.selected_fuel(&left, 0), 0.0);
        let dry = weights.burn_selected(&left, 2, 100.0, false).unwrap();
        assert_eq!(weights.selected_fuel(&dry, 2), 0.0);
        assert_eq!(weights.selected_fuel(&dry, 3), 3.0);
    }
}

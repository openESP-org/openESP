//! Legacy helicopter AIR records. Values retain the units used by the AIR file.
//! Layout: Prepar3D SDK, TurbineHeloSampleType2.asm, records 0x1400–0x1404.

use super::{Error, Record};

#[derive(Debug, Clone, PartialEq)]
pub struct TailSurface {
    pub area_sq_ft: f64,
    pub aspect_ratio: f64,
    pub angle_rad: f64,
    pub longitudinal_position_in: f64,
    pub vertical_position_in: f64,
    pub lift_coefficient: f64,
    pub drag_coefficient: f64,
    pub rotor_wash_factor: f64,
    pub extensions: Vec<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Rotor {
    pub lateral_position_in: f64,
    pub longitudinal_position_in: f64,
    pub vertical_position_in: f64,
    pub lateral_tilt_rad: f64,
    pub longitudinal_tilt_rad: f64,
    pub radius_ft: f64,
    pub chord_ft: f64,
    pub blade_count: f64,
    pub moment_of_inertia: f64,
    pub hub_loss_factor: f64,
    pub lift_coefficient_per_rad: f64,
    pub drag_coefficient: f64,
    pub solidity: f64,
    pub extensions: Vec<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct MainRotor {
    pub rotor: Rotor,
    pub area_sq_ft: f64,
    pub aspect_ratio: f64,
    pub angular_speed_rad_s: f64,
    pub extensions: Vec<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Fuselage {
    pub frontal_area_sq_ft: f64,
    pub side_area_sq_ft: f64,
    pub top_area_sq_ft: f64,
    pub frontal_drag_coefficient: f64,
    pub side_drag_coefficient: f64,
    pub top_drag_coefficient: f64,
    pub extensions: Vec<f64>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum HelicopterRecord {
    VerticalTail(TailSurface),
    HorizontalTail(TailSurface),
    MainRotor(MainRotor),
    TailRotor(Rotor),
    Fuselage(Fuselage),
}

impl Record<'_> {
    /// Decode a helicopter-specific record, leaving future whole-double fields in `extensions`.
    pub fn helicopter(&self) -> Result<Option<HelicopterRecord>, Error> {
        let count = match self.id {
            0x1400 | 0x1401 => 8,
            0x1402 => 16,
            0x1403 => 13,
            0x1404 => 6,
            _ => return Ok(None),
        };
        let expected = count * 8;
        if self.data.len() < expected || self.data.len() % 8 != 0 {
            return Err(Error::RecordSize {
                id: self.id,
                expected,
                actual: self.data.len(),
            });
        }
        let values: Vec<f64> = self
            .data
            .chunks_exact(8)
            .map(|v| f64::from_le_bytes(v.try_into().unwrap()))
            .collect();
        let rotor = || Rotor {
            lateral_position_in: values[0],
            longitudinal_position_in: values[1],
            vertical_position_in: values[2],
            lateral_tilt_rad: values[3],
            longitudinal_tilt_rad: values[4],
            radius_ft: values[5],
            chord_ft: values[6],
            blade_count: values[7],
            moment_of_inertia: values[8],
            hub_loss_factor: values[9],
            lift_coefficient_per_rad: values[10],
            drag_coefficient: values[11],
            solidity: values[12],
            extensions: Vec::new(),
        };
        Ok(Some(match self.id {
            0x1400 | 0x1401 => {
                let tail = TailSurface {
                    area_sq_ft: values[0],
                    aspect_ratio: values[1],
                    angle_rad: values[2],
                    longitudinal_position_in: values[3],
                    vertical_position_in: values[4],
                    lift_coefficient: values[5],
                    drag_coefficient: values[6],
                    rotor_wash_factor: values[7],
                    extensions: values[8..].to_vec(),
                };
                if self.id == 0x1400 {
                    HelicopterRecord::VerticalTail(tail)
                } else {
                    HelicopterRecord::HorizontalTail(tail)
                }
            }
            0x1402 => HelicopterRecord::MainRotor(MainRotor {
                rotor: rotor(),
                area_sq_ft: values[13],
                aspect_ratio: values[14],
                angular_speed_rad_s: values[15],
                extensions: values[16..].to_vec(),
            }),
            0x1403 => {
                let mut tail = rotor();
                tail.extensions = values[13..].to_vec();
                HelicopterRecord::TailRotor(tail)
            }
            0x1404 => HelicopterRecord::Fuselage(Fuselage {
                frontal_area_sq_ft: values[0],
                side_area_sq_ft: values[1],
                top_area_sq_ft: values[2],
                frontal_drag_coefficient: values[3],
                side_drag_coefficient: values[4],
                top_drag_coefficient: values[5],
                extensions: values[6..].to_vec(),
            }),
            _ => unreachable!(),
        }))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bytes(values: &[f64]) -> Vec<u8> {
        values.iter().flat_map(|v| v.to_le_bytes()).collect()
    }

    #[test]
    fn reads_main_and_tail_rotor_fields_and_extensions() {
        let data = bytes(&[
            0.0, -4.0, 57.6, -0.01745, -0.01745, 16.625, 1.08, 2.0, 267.75, 0.75, 2.5, 0.009,
            0.0413, 868.31, 15.11, 41.36, 0.0, 8.0,
        ]);
        let record = Record {
            id: 0x1402,
            offset: 0,
            data: &data,
        };
        let Some(HelicopterRecord::MainRotor(main)) = record.helicopter().unwrap() else {
            panic!()
        };
        assert_eq!(main.rotor.longitudinal_position_in, -4.0);
        assert_eq!(main.rotor.radius_ft, 16.625);
        assert_eq!(main.angular_speed_rad_s, 41.36);
        assert_eq!(main.extensions, [0.0, 8.0]);
        let record = Record {
            id: 0x1403,
            offset: 0,
            data: &data[..104],
        };
        assert!(matches!(
            record.helicopter().unwrap(),
            Some(HelicopterRecord::TailRotor(_))
        ));
    }

    #[test]
    fn rejects_incomplete_records_but_ignores_other_tokens() {
        for (id, length) in [
            (0x1400, 64),
            (0x1401, 64),
            (0x1402, 128),
            (0x1403, 104),
            (0x1404, 48),
        ] {
            for size in [0, length - 1, length + 1] {
                let data = vec![0; size];
                assert!(
                    Record {
                        id,
                        offset: 0,
                        data: &data
                    }
                    .helicopter()
                    .is_err()
                );
            }
            let data = vec![0; length];
            assert!(
                Record {
                    id,
                    offset: 0,
                    data: &data
                }
                .helicopter()
                .unwrap()
                .is_some()
            );
        }
        assert_eq!(
            Record {
                id: 0x1101,
                offset: 0,
                data: &[]
            }
            .helicopter()
            .unwrap(),
            None
        );
    }
}

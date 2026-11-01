use super::{Error, Record};
use binrw::BinRead;
use std::io::Cursor;

#[derive(Debug, Clone, PartialEq, BinRead)]
#[br(little)]
pub struct Lift {
    pub cl_spoiler: f64,
    pub cl_flaps: f64,
    pub unused0: f64,
    pub cl_ih: f64,
    pub cl_de: f64,
    pub unused1: f64,
}

#[derive(Debug, Clone, PartialEq, BinRead)]
#[br(little)]
pub struct Drag {
    pub cd0: f64,
    pub cd_flaps: f64,
    pub cd_gear: f64,
    pub cd_spoiler: f64,
}

#[derive(Debug, Clone, PartialEq, BinRead)]
#[br(little)]
pub struct Pitch {
    pub cm_ih: f64,
    pub cm_de: f64,
    pub cm_de_propwash: f64,
    pub cl_q: f64,
    pub cl_adot: f64,
    pub cm_adot: f64,
    pub cm_q: f64,
    pub cm_q_propwash: f64,
    pub cm0: f64,
    pub cm_flap: f64,
    pub cm_delta_trim: f64,
    pub cm_gear: f64,
    pub cm_spoiler: f64,
}

#[derive(Debug, Clone, PartialEq, BinRead)]
#[br(little)]
pub struct SideForce {
    pub cy_beta: f64,
    pub cy_p: f64,
    pub cy_r: f64,
    pub cy_delta_rudder: f64,
}

#[derive(Debug, Clone, PartialEq, BinRead)]
#[br(little)]
pub struct Roll {
    pub cl_beta: f64,
    pub cl_p: f64,
    pub cl_r: f64,
    pub cl_delta_spoiler: f64,
    pub cl_delta_aileron: f64,
    pub cl_delta_rudder: f64,
}

#[derive(Debug, Clone, PartialEq, BinRead)]
#[br(little)]
pub struct Yaw {
    pub cn_beta: f64,
    pub cn_p: f64,
    pub cn_r: f64,
    pub cn_r_propwash: f64,
    pub unused: [f64; 2],
    pub cn_delta_aileron: f64,
    pub cn_delta_rudder: f64,
    pub cn_delta_rudder_propwash: f64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Parameters {
    Lift(Lift),
    Drag(Drag),
    Pitch(Pitch),
    SideForce(SideForce),
    Roll(Roll),
    Yaw(Yaw),
}

#[derive(Debug, Clone, PartialEq)]
pub struct LegacyParameters {
    pub lift: Lift,
    pub drag: Drag,
    pub pitch: Pitch,
    pub side: SideForce,
    pub roll: Roll,
    pub yaw: Yaw,
}

impl Record<'_> {
    pub fn legacy_aerodynamics(&self) -> Result<Option<LegacyParameters>, Error> {
        if self.id != 0x1101 {
            return Ok(None);
        }
        if self.data.len() != 250 {
            return Err(Error::RecordSize {
                id: self.id,
                expected: 250,
                actual: self.data.len(),
            });
        }
        let short = |offset| {
            f64::from(i16::from_le_bytes(
                self.data[offset..offset + 2].try_into().unwrap(),
            )) / 2048.0
        };
        let long = |offset| {
            f64::from(i32::from_le_bytes(
                self.data[offset..offset + 4].try_into().unwrap(),
            )) / 2048.0
        };
        let double = |offset| f64::from_le_bytes(self.data[offset..offset + 8].try_into().unwrap());
        Ok(Some(LegacyParameters {
            lift: Lift {
                cl_spoiler: double(0x66),
                cl_flaps: double(0x5e),
                cl_ih: short(0x5c),
                cl_de: short(0x5a),
                unused0: 0.0,
                unused1: 0.0,
            },
            drag: Drag {
                cd0: short(0x30),
                cd_flaps: short(0x32),
                cd_gear: short(0x34),
                cd_spoiler: short(0x36),
            },
            pitch: Pitch {
                cm_ih: long(0x92),
                cm_de: long(0x8e),
                cm_de_propwash: double(0x9a),
                cl_q: long(0x56),
                cl_adot: long(0x52),
                cm_adot: long(0x8a),
                cm_q: long(0x96),
                cm_q_propwash: double(0xa2),
                cm0: long(0x86),
                cm_flap: double(0xb2),
                cm_delta_trim: double(0xaa),
                cm_gear: double(0xba),
                cm_spoiler: double(0xc2),
            },
            side: SideForce {
                cy_beta: short(0xee),
                cy_p: long(0xf2),
                cy_r: long(0xf6),
                cy_delta_rudder: short(0xf0),
            },
            roll: Roll {
                cl_beta: short(0x6e),
                cl_p: long(0x76),
                cl_r: long(0x7a),
                cl_delta_spoiler: double(0x7e),
                cl_delta_aileron: short(0x70),
                cl_delta_rudder: long(0x72),
            },
            yaw: Yaw {
                cn_beta: long(0xca),
                cn_p: long(0xe4),
                cn_r: short(0xe8),
                cn_r_propwash: double(0xdc),
                cn_delta_aileron: short(0xce),
                cn_delta_rudder: long(0xd0),
                cn_delta_rudder_propwash: double(0xd4),
                unused: [0.0; 2],
            },
        }))
    }

    pub fn aerodynamic_parameters(&self) -> Result<Option<Parameters>, Error> {
        let parameters = match self.id {
            0x1539 => Parameters::Lift(read(self, 48)?),
            0x1540 => Parameters::Drag(read(self, 32)?),
            0x1541 => Parameters::Pitch(read(self, 104)?),
            0x1542 => Parameters::SideForce(read(self, 32)?),
            0x1543 => Parameters::Roll(read(self, 48)?),
            0x1544 => Parameters::Yaw(read(self, 72)?),
            _ => return Ok(None),
        };
        Ok(Some(parameters))
    }
}

fn read<T>(record: &Record<'_>, expected: usize) -> Result<T, Error>
where
    T: for<'a> BinRead<Args<'a> = ()>,
{
    let error = || Error::RecordSize {
        id: record.id,
        expected,
        actual: record.data.len(),
    };
    if record.data.len() != expected {
        return Err(error());
    }
    T::read_le(&mut Cursor::new(record.data)).map_err(|_| error())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_legacy_signed_coefficients_and_unscaled_doubles() {
        let mut bytes = [0u8; 250];
        for (offset, value) in [(0x30, 61i16), (0x5a, -763), (0xee, -819), (0xe8, -370)] {
            bytes[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
        }
        for (offset, value) in [(0x86, -81i32), (0x96, -30_000), (0x76, -2100), (0xd0, 112)] {
            bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        }
        bytes[0x5e..0x66].copy_from_slice(&0.57f64.to_le_bytes());
        bytes[0xdc..0xe4].copy_from_slice(&(-12.5f64).to_le_bytes());
        let record = Record {
            id: 0x1101,
            offset: 0,
            data: &bytes,
        };
        let aero = record.legacy_aerodynamics().unwrap().unwrap();
        assert_eq!(aero.drag.cd0, 61.0 / 2048.0);
        assert_eq!(aero.lift.cl_de, -763.0 / 2048.0);
        assert_eq!(aero.lift.cl_flaps, 0.57);
        assert_eq!(aero.pitch.cm0, -81.0 / 2048.0);
        assert_eq!(aero.pitch.cm_q, -30_000.0 / 2048.0);
        assert_eq!(aero.roll.cl_p, -2100.0 / 2048.0);
        assert_eq!(aero.side.cy_beta, -819.0 / 2048.0);
        assert_eq!(aero.yaw.cn_r, -370.0 / 2048.0);
        assert_eq!(aero.yaw.cn_delta_rudder, 112.0 / 2048.0);
        assert_eq!(aero.yaw.cn_r_propwash, -12.5);
    }

    #[test]
    fn legacy_record_requires_its_complete_layout() {
        let bytes = [0u8; 251];
        for length in 0..=251 {
            let record = Record {
                id: 0x1101,
                offset: 0,
                data: &bytes[..length],
            };
            assert_eq!(record.legacy_aerodynamics().is_ok(), length == 250);
        }
        assert_eq!(
            Record {
                id: 0x1539,
                offset: 0,
                data: &bytes
            }
            .legacy_aerodynamics()
            .unwrap(),
            None
        );
    }

    #[test]
    fn reads_aerodynamic_coefficients_and_unused_values() {
        let bytes: Vec<_> = (0..13)
            .flat_map(|value| (value as f64 - 6.5).to_le_bytes())
            .collect();
        let parameters = |id, count: usize| {
            Record {
                id,
                offset: 0,
                data: &bytes[..count * 8],
            }
            .aerodynamic_parameters()
            .unwrap()
            .unwrap()
        };
        let Parameters::Lift(lift) = parameters(0x1539, 6) else {
            panic!()
        };
        assert_eq!(lift.cl_spoiler, -6.5);
        assert_eq!(lift.unused0, -4.5);
        assert_eq!(lift.cl_de, -2.5);
        assert_eq!(lift.unused1, -1.5);
        let Parameters::Drag(drag) = parameters(0x1540, 4) else {
            panic!()
        };
        assert_eq!(drag.cd0, -6.5);
        assert_eq!(drag.cd_spoiler, -3.5);
        let Parameters::Pitch(pitch) = parameters(0x1541, 13) else {
            panic!()
        };
        assert_eq!(pitch.cm_q, -0.5);
        assert_eq!(pitch.cm_delta_trim, 3.5);
        assert_eq!(pitch.cm_spoiler, 5.5);
        let Parameters::SideForce(side) = parameters(0x1542, 4) else {
            panic!()
        };
        assert_eq!(side.cy_p, -5.5);
        assert_eq!(side.cy_delta_rudder, -3.5);
        let Parameters::Roll(roll) = parameters(0x1543, 6) else {
            panic!()
        };
        assert_eq!(roll.cl_delta_spoiler, -3.5);
        assert_eq!(roll.cl_delta_rudder, -1.5);
        let Parameters::Yaw(yaw) = parameters(0x1544, 9) else {
            panic!()
        };
        assert_eq!(yaw.cn_r_propwash, -3.5);
        assert_eq!(yaw.unused, [-2.5, -1.5]);
        assert_eq!(yaw.cn_delta_rudder_propwash, 1.5);
    }

    #[test]
    fn rejects_wrong_sizes_without_guessing_other_record_types() {
        for (id, size) in [
            (0x1539, 48),
            (0x1540, 32),
            (0x1541, 104),
            (0x1542, 32),
            (0x1543, 48),
            (0x1544, 72),
        ] {
            let bytes = vec![0; size + 1];
            for length in 0..=size + 1 {
                let record = Record {
                    id,
                    offset: 0,
                    data: &bytes[..length],
                };
                assert_eq!(record.aerodynamic_parameters().is_ok(), length == size);
            }
        }
        let record = Record {
            id: 0x1101,
            offset: 0,
            data: &[0; 250],
        };
        assert_eq!(record.aerodynamic_parameters().unwrap(), None);
    }
}

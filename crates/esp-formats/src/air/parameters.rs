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

impl Record<'_> {
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

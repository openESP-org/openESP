use super::{Error, Record};
use binrw::BinRead;
use std::io::Cursor;

#[derive(Debug, Clone, Copy, PartialEq, BinRead)]
#[br(little)]
pub struct Point {
    pub input: f64,
    pub output: f64,
}

#[derive(Debug, PartialEq)]
pub struct Curve<'a> {
    pub points: Vec<Point>,
    pub trailing_data: &'a [u8],
}

#[derive(Debug, PartialEq)]
pub enum Table<'a> {
    Curve(Curve<'a>),
    Mach([i16; 17]),
}

impl<'a> Record<'a> {
    pub fn table(&self) -> Result<Option<Table<'a>>, Error> {
        match self.id {
            0x341..=0x343
            | 0x400
            | 0x401
            | 0x404
            | 0x451
            | 0x456
            | 0x460
            | 0x464
            | 0x473
            | 0x1505
            | 0x1524 => {
                let mut reader = Cursor::new(self.data);
                let count = u32::read_le(&mut reader).map_err(|_| self.size_error(4))?;
                let available = self.data.len() - 4;
                if u64::from(count) > (available / 16) as u64 {
                    return Err(Error::TableSize {
                        id: self.id,
                        entries: count.into(),
                        available,
                    });
                }
                let points = (0..count)
                    .map(|_| Point::read(&mut reader))
                    .collect::<Result<Vec<_>, _>>()
                    .map_err(|_| self.size_error(4 + count as usize * 16))?;
                Ok(Some(Table::Curve(Curve {
                    points,
                    trailing_data: &self.data[reader.position() as usize..],
                })))
            }
            0x410..=0x413
            | 0x420..=0x423
            | 0x430
            | 0x433
            | 0x440..=0x443
            | 0x450
            | 0x452..=0x455
            | 0x459
            | 0x461..=0x463
            | 0x465 => {
                if self.data.len() != 34 {
                    return Err(self.size_error(34));
                }
                let values = <[i16; 17]>::read_le(&mut Cursor::new(self.data))
                    .map_err(|_| self.size_error(34))?;
                Ok(Some(Table::Mach(values)))
            }
            _ => Ok(None),
        }
    }

    fn size_error(&self, expected: usize) -> Error {
        Error::RecordSize {
            id: self.id,
            expected,
            actual: self.data.len(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(id: u32, data: &[u8]) -> Record<'_> {
        Record {
            id,
            offset: 0,
            data,
        }
    }

    #[test]
    fn reads_curves_and_preserves_unused_capacity() {
        let data = [
            2u32.to_le_bytes().to_vec(),
            [-1.0f64, 0.25, 3.0, -2.0]
                .into_iter()
                .flat_map(f64::to_le_bytes)
                .collect(),
            vec![0xaa; 16],
        ]
        .concat();
        for id in [
            0x341, 0x342, 0x343, 0x400, 0x401, 0x404, 0x451, 0x456, 0x460, 0x464, 0x473, 0x1505,
            0x1524,
        ] {
            let Some(Table::Curve(curve)) = record(id, &data).table().unwrap() else {
                panic!("missing curve {id:#x}");
            };
            assert_eq!(
                curve.points,
                [
                    Point {
                        input: -1.0,
                        output: 0.25
                    },
                    Point {
                        input: 3.0,
                        output: -2.0
                    }
                ]
            );
            assert_eq!(curve.trailing_data, [0xaa; 16]);
        }
    }

    #[test]
    fn checks_counts_before_reading_points() {
        for size in 0..4 {
            assert!(matches!(
                record(0x404, &vec![0; size]).table(),
                Err(Error::RecordSize { expected: 4, .. })
            ));
        }
        for count in [1, u32::MAX] {
            assert!(matches!(
                record(0x404, &count.to_le_bytes()).table(),
                Err(Error::TableSize { .. })
            ));
        }
        let bytes = [1u32.to_le_bytes().to_vec(), vec![0; 16]].concat();
        for end in 4..bytes.len() {
            assert!(matches!(
                record(0x404, &bytes[..end]).table(),
                Err(Error::TableSize { .. })
            ));
        }
        assert_eq!(
            record(0x404, &[0; 4]).table().unwrap(),
            Some(Table::Curve(Curve {
                points: vec![],
                trailing_data: &[]
            }))
        );
        assert_eq!(record(0xdead, &[]).table().unwrap(), None);
    }

    #[test]
    fn reads_signed_mach_offsets_without_a_count_prefix() {
        let values = std::array::from_fn::<_, 17, _>(|index| (index as i16 - 8) * 2048);
        let data: Vec<_> = values.into_iter().flat_map(i16::to_le_bytes).collect();
        for id in [
            0x410, 0x411, 0x412, 0x413, 0x420, 0x421, 0x422, 0x423, 0x430, 0x433, 0x440, 0x441,
            0x442, 0x443, 0x450, 0x452, 0x453, 0x454, 0x455, 0x459, 0x461, 0x462, 0x463, 0x465,
        ] {
            assert_eq!(
                record(id, &data).table().unwrap(),
                Some(Table::Mach(values))
            );
            for end in 0..34 {
                assert!(matches!(
                    record(id, &data[..end]).table(),
                    Err(Error::RecordSize { expected: 34, .. })
                ));
            }
            assert!(
                record(id, &[data.as_slice(), &[0]].concat())
                    .table()
                    .is_err()
            );
        }
    }
}

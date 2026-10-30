use super::Error;
use binrw::BinRead;
use std::{io::Cursor, str::FromStr};
use uuid::Uuid;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Type {
    Bool,
    Long,
    Ulong,
    Long2,
    Long4,
    Float,
    Float2,
    Float4,
    Double,
    Byte4,
    Guid,
    Pbh,
    Pbh32,
    Enum,
    Lla,
    Filetime,
    Text,
    Mltext,
}

impl FromStr for Type {
    type Err = Error;

    fn from_str(name: &str) -> Result<Self, Self::Err> {
        [
            ("BOOL", Self::Bool),
            ("LONG", Self::Long),
            ("ULONG", Self::Ulong),
            ("LONG2", Self::Long2),
            ("LONG4", Self::Long4),
            ("FLOAT", Self::Float),
            ("FLOAT2", Self::Float2),
            ("FLOAT4", Self::Float4),
            ("DOUBLE", Self::Double),
            ("BYTE4", Self::Byte4),
            ("GUID", Self::Guid),
            ("PBH", Self::Pbh),
            ("PBH32", Self::Pbh32),
            ("ENUM", Self::Enum),
            ("LLA", Self::Lla),
            ("FILETIME", Self::Filetime),
            ("TEXT", Self::Text),
            ("MLTEXT", Self::Mltext),
        ]
        .into_iter()
        .find_map(|(candidate, ty)| candidate.eq_ignore_ascii_case(name).then_some(ty))
        .ok_or_else(|| Error::UnsupportedType(name.into()))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, BinRead)]
#[br(little)]
pub struct Angles {
    pub pitch: u32,
    pub bank: u32,
    pub heading: u32,
    pub padding: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, BinRead)]
#[br(little)]
pub struct Position {
    pub latitude: i64,
    pub longitude: i64,
    pub altitude_fraction: u32,
    pub altitude_meters: i32,
}

#[derive(Debug, PartialEq)]
pub enum Value<'a> {
    Bool(i32),
    Long(i32),
    Ulong(u32),
    Long2([i32; 2]),
    Long4([i32; 4]),
    Float(f32),
    Float2([f32; 2]),
    Float4([f32; 4]),
    Double(f64),
    Byte4([u8; 4]),
    Guid(Uuid),
    Angles(Angles),
    Enum(i32),
    Position(Position),
    Filetime(u64),
    Text(&'a str),
}

pub fn parse(ty: Type, data: &[u8]) -> Result<(Value<'_>, &[u8]), Error> {
    let mut reader = Cursor::new(data);
    let value = match ty {
        Type::Bool => Value::Bool(read(&mut reader)?),
        Type::Long => Value::Long(read(&mut reader)?),
        Type::Ulong => Value::Ulong(read(&mut reader)?),
        Type::Long2 => Value::Long2(read(&mut reader)?),
        Type::Long4 => Value::Long4(read(&mut reader)?),
        Type::Float => Value::Float(read(&mut reader)?),
        Type::Float2 => Value::Float2(read(&mut reader)?),
        Type::Float4 => Value::Float4(read(&mut reader)?),
        Type::Double => Value::Double(read(&mut reader)?),
        Type::Byte4 => Value::Byte4(read(&mut reader)?),
        Type::Guid => Value::Guid(Uuid::from_bytes_le(read(&mut reader)?)),
        Type::Pbh | Type::Pbh32 => Value::Angles(read(&mut reader)?),
        Type::Enum => Value::Enum(read(&mut reader)?),
        Type::Lla => Value::Position(read(&mut reader)?),
        Type::Filetime => Value::Filetime(read(&mut reader)?),
        Type::Text | Type::Mltext => {
            let length: i32 = read(&mut reader)?;
            let length = usize::try_from(length).map_err(|_| Error::TextLength(length))?;
            let bytes = data
                .get(4..4 + length)
                .ok_or(Error::Truncated { offset: 4 })?;
            reader.set_position((4 + length) as u64);
            let text = if bytes.is_empty() {
                ""
            } else {
                let bytes = bytes.strip_suffix(&[0]).ok_or(Error::TextTerminator)?;
                std::str::from_utf8(bytes)?
            };
            Value::Text(text)
        }
    };
    Ok((value, &data[reader.position() as usize..]))
}

fn read<T: for<'a> BinRead<Args<'a> = ()>>(reader: &mut Cursor<&[u8]>) -> Result<T, Error> {
    let offset = reader.position() as usize;
    T::read_le(reader).map_err(|_| Error::Truncated { offset })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_scalar_vectors_and_guid_values_without_normalizing_them() {
        assert_eq!(
            parse(Type::Long, &(-123i32).to_le_bytes()).unwrap().0,
            Value::Long(-123)
        );
        assert_eq!(
            parse(Type::Ulong, &u32::MAX.to_le_bytes()).unwrap().0,
            Value::Ulong(u32::MAX)
        );
        assert_eq!(
            parse(Type::Bool, &2i32.to_le_bytes()).unwrap().0,
            Value::Bool(2)
        );
        let bytes: Vec<_> = [-1i32, 2, -3, 4]
            .into_iter()
            .flat_map(i32::to_le_bytes)
            .collect();
        assert_eq!(
            parse(Type::Long4, &bytes).unwrap().0,
            Value::Long4([-1, 2, -3, 4])
        );
        let bytes: Vec<_> = [-1.25f32, 2.5, 3.0, -4.0]
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect();
        assert_eq!(
            parse(Type::Float4, &bytes).unwrap().0,
            Value::Float4([-1.25, 2.5, 3.0, -4.0])
        );
        let guid = Uuid::from_u128(0x0102030405060708090a0b0c0d0e0f10);
        assert_eq!(
            parse(Type::Guid, &guid.to_bytes_le()).unwrap().0,
            Value::Guid(guid)
        );
        assert_eq!(
            parse(Type::Enum, &(-1i32).to_le_bytes()).unwrap().0,
            Value::Enum(-1)
        );
    }

    #[test]
    fn preserves_packed_coordinates_and_angle_padding() {
        let bytes: Vec<_> = [0x40000000u32, 0x80000000, 0xc0000000, 42]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect();
        for ty in [Type::Pbh, Type::Pbh32] {
            assert_eq!(
                parse(ty, &bytes).unwrap().0,
                Value::Angles(Angles {
                    pitch: 0x40000000,
                    bank: 0x80000000,
                    heading: 0xc0000000,
                    padding: 42
                })
            );
        }
        let bytes = [
            (-123i64).to_le_bytes().as_slice(),
            &456i64.to_le_bytes(),
            &0x80000000u32.to_le_bytes(),
            &(-10i32).to_le_bytes(),
        ]
        .concat();
        assert_eq!(
            parse(Type::Lla, &bytes).unwrap().0,
            Value::Position(Position {
                latitude: -123,
                longitude: 456,
                altitude_fraction: 0x80000000,
                altitude_meters: -10
            })
        );
    }

    #[test]
    fn reads_borrowed_utf8_text_and_checks_lengths_and_terminators() {
        let text = "Café 東京";
        let bytes = [
            (text.len() as i32 + 1).to_le_bytes().to_vec(),
            text.as_bytes().to_vec(),
            vec![0, 42],
        ]
        .concat();
        for ty in [Type::Text, Type::Mltext] {
            let (Value::Text(decoded), remaining) = parse(ty, &bytes).unwrap() else {
                panic!("expected text")
            };
            assert_eq!(decoded, text);
            assert_eq!(remaining, [42]);
            assert!(std::ptr::eq(decoded.as_ptr(), bytes[4..].as_ptr()));
        }
        assert_eq!(parse(Type::Text, &[0; 4]).unwrap().0, Value::Text(""));
        assert!(matches!(
            parse(Type::Text, &(-1i32).to_le_bytes()),
            Err(Error::TextLength(-1))
        ));
        assert!(matches!(
            parse(Type::Text, &[1, 0, 0, 0, 42]),
            Err(Error::TextTerminator)
        ));
        assert!(matches!(
            parse(Type::Text, &[2, 0, 0, 0, 255, 0]),
            Err(Error::TextEncoding(_))
        ));
        assert!(parse(Type::Text, &i32::MAX.to_le_bytes()).is_err());
        for end in 0..bytes.len() - 1 {
            assert!(parse(Type::Text, &bytes[..end]).is_err());
        }
    }

    #[test]
    fn checks_fixed_value_sizes_and_retains_following_data() {
        for (name, size) in [
            ("BOOL", 4),
            ("LONG", 4),
            ("ULONG", 4),
            ("LONG2", 8),
            ("LONG4", 16),
            ("FLOAT", 4),
            ("FLOAT2", 8),
            ("FLOAT4", 16),
            ("DOUBLE", 8),
            ("BYTE4", 4),
            ("GUID", 16),
            ("PBH", 16),
            ("PBH32", 16),
            ("ENUM", 4),
            ("LLA", 24),
            ("FILETIME", 8),
        ] {
            let ty: Type = name.parse().unwrap();
            assert_eq!(name.to_ascii_lowercase().parse::<Type>().unwrap(), ty);
            let bytes = vec![0; size + 1];
            let (_, remaining) = parse(ty, &bytes).unwrap();
            assert!(std::ptr::eq(remaining.as_ptr(), bytes[size..].as_ptr()));
            for end in 0..size {
                assert!(parse(ty, &bytes[..end]).is_err(), "{name}: {end}");
            }
        }
        assert!(matches!(
            "UNKNOWN".parse::<Type>(),
            Err(Error::UnsupportedType(_))
        ));
    }
}

use super::{Chunk, Error, parse_chunks};

#[derive(Debug, PartialEq, Eq)]
pub enum Condition<'a> {
    Sim {
        default_visible: bool,
        definition: &'a [u8],
    },
    Code {
        default_visible: bool,
        definition: &'a [u8],
    },
    Unknown(Chunk<'a>),
}

pub fn conditions(data: &[u8]) -> Result<Vec<Condition<'_>>, Error> {
    Ok(parse_chunks(data)?
        .into_iter()
        .map(|chunk| match &chunk.id {
            b"VISS" => Condition::Sim {
                default_visible: true,
                definition: chunk.data,
            },
            b"VINS" => Condition::Sim {
                default_visible: false,
                definition: chunk.data,
            },
            b"VISC" => Condition::Code {
                default_visible: true,
                definition: chunk.data,
            },
            b"VINC" => Condition::Code {
                default_visible: false,
                definition: chunk.data,
            },
            _ => Condition::Unknown(chunk),
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn chunk(id: &[u8; 4], definition: &[u8]) -> Vec<u8> {
        [
            id.as_slice(),
            &(definition.len() as u32).to_le_bytes(),
            definition,
        ]
        .concat()
    }

    #[test]
    fn keeps_order_and_preserves_code_whitespace() {
        let mut bytes = chunk(b"VISS", b"TEST#bool#0#10");
        bytes.extend(chunk(b"UNKN", &[1]));
        bytes.extend(chunk(b"VINC", b"\r\n (A:TEST, bool) 0 == \r\n"));
        let conditions = conditions(&bytes).unwrap();
        assert_eq!(
            conditions[0],
            Condition::Sim {
                default_visible: true,
                definition: b"TEST#bool#0#10"
            }
        );
        assert_eq!(
            conditions[1],
            Condition::Unknown(Chunk {
                id: *b"UNKN",
                data: &[1]
            })
        );
        assert_eq!(
            conditions[2],
            Condition::Code {
                default_visible: false,
                definition: b"\r\n (A:TEST, bool) 0 == \r\n"
            }
        );
    }

    #[test]
    fn handles_other_modes_and_malformed_sizes() {
        let bytes = [chunk(b"VINS", &[]), chunk(b"VISC", b"0")].concat();
        let result = conditions(&bytes).unwrap();
        assert!(matches!(
            result[0],
            Condition::Sim {
                default_visible: false,
                ..
            }
        ));
        assert!(matches!(
            result[1],
            Condition::Code {
                default_visible: true,
                ..
            }
        ));
        assert!(conditions(b"VISC\xff\xff\xff\xff").is_err());
        assert!(conditions(&[]).unwrap().is_empty());
    }
}

use binrw::BinRead;
use std::io::Cursor;

#[derive(Debug, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    #[error("truncated AIR record header at offset {offset}")]
    Header { offset: usize },
    #[error("AIR record {id:#x} at offset {offset} needs {size} bytes, found {available}")]
    Truncated {
        id: u32,
        offset: usize,
        size: u32,
        available: usize,
    },
}

#[derive(BinRead)]
#[br(little)]
struct Header {
    id: u32,
    size: u32,
}

#[derive(Debug, PartialEq, Eq)]
pub struct Record<'a> {
    pub id: u32,
    pub offset: usize,
    pub data: &'a [u8],
}

pub fn parse(mut bytes: &[u8]) -> Result<Vec<Record<'_>>, Error> {
    let mut records = Vec::new();
    let mut offset = 0;
    while !bytes.is_empty() {
        let mut cursor = Cursor::new(bytes);
        let header = Header::read(&mut cursor).map_err(|_| Error::Header { offset })?;
        let payload = &bytes[8..];
        let (data, remaining) = usize::try_from(header.size)
            .ok()
            .and_then(|size| payload.split_at_checked(size))
            .ok_or(Error::Truncated {
                id: header.id,
                offset,
                size: header.size,
                available: payload.len(),
            })?;
        records.push(Record {
            id: header.id,
            offset,
            data,
        });
        offset += 8 + data.len();
        bytes = remaining;
    }
    Ok(records)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn record(id: u32, data: &[u8]) -> Vec<u8> {
        [
            id.to_le_bytes().as_slice(),
            &(data.len() as u32).to_le_bytes(),
            data,
        ]
        .concat()
    }

    #[test]
    fn preserves_record_order_duplicates_and_unknown_data() {
        let bytes = [
            record(0x105, &2u32.to_le_bytes()),
            record(0xdeadbeef, &[42]),
            record(0x105, &[]),
        ]
        .concat();
        let records = parse(&bytes).unwrap();
        assert_eq!(
            records,
            [
                Record {
                    id: 0x105,
                    offset: 0,
                    data: &[2, 0, 0, 0]
                },
                Record {
                    id: 0xdeadbeef,
                    offset: 12,
                    data: &[42]
                },
                Record {
                    id: 0x105,
                    offset: 21,
                    data: &[]
                },
            ]
        );
        assert!(std::ptr::eq(records[0].data.as_ptr(), bytes[8..].as_ptr()));
        assert!(parse(&[]).unwrap().is_empty());
    }

    #[test]
    fn rejects_incomplete_headers_and_record_lengths() {
        let bytes = record(0x105, &2u32.to_le_bytes());
        for end in 1..8 {
            assert_eq!(parse(&bytes[..end]), Err(Error::Header { offset: 0 }));
        }
        for end in 8..bytes.len() {
            assert!(matches!(
                parse(&bytes[..end]),
                Err(Error::Truncated {
                    id: 0x105,
                    offset: 0,
                    size: 4,
                    ..
                })
            ));
        }
        let bytes = [bytes, vec![0]].concat();
        assert_eq!(parse(&bytes), Err(Error::Header { offset: 12 }));
        let bytes = [0u32.to_le_bytes(), u32::MAX.to_le_bytes()].concat();
        assert!(matches!(
            parse(&bytes),
            Err(Error::Truncated { size: u32::MAX, .. })
        ));
    }
}

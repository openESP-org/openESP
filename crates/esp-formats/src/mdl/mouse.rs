use super::{Error, geometry::records};
use binrw::BinRead;
use std::io::Cursor;

#[derive(Debug, Clone, PartialEq, Eq, BinRead)]
#[br(little)]
pub struct Header {
    pub kind: u32,
    pub bounds: [i32; 4],
    pub cursor: i32,
    pub help_id: u32,
    pub flags: u32,
    pub event_id: u32,
    pub callback: u32,
    pub reserved: u32,
}

#[derive(Debug)]
pub struct Rectangle<'a> {
    pub header: Header,
    pub raw: &'a [u8],
}

impl Rectangle<'_> {
    pub fn key_event(&self) -> Option<u32> {
        // FSX's direct events use the 44-byte, 32-bit SDK MOUSERECT layout.
        (self.raw.len() == 44 && self.header.kind == 2 && self.header.callback == 0)
            .then_some(self.header.event_id)
            .filter(|id| *id != 0)
    }
}

pub fn rectangles<'a>(data: &'a [u8], index: &[u8]) -> Result<Vec<Rectangle<'a>>, Error> {
    let offsets: Vec<u32> = records(index, 4)?;
    if offsets.is_empty() {
        return if data.is_empty() {
            Ok(Vec::new())
        } else {
            Err(Error::InvalidSize)
        };
    }
    if offsets.first() != Some(&0) {
        return Err(Error::InvalidSize);
    }
    let mut result = Vec::with_capacity(offsets.len());
    for (entry, &offset) in offsets.iter().enumerate() {
        let start = usize::try_from(offset).map_err(|_| Error::InvalidSize)?;
        let end = offsets
            .get(entry + 1)
            .map(|offset| usize::try_from(*offset).map_err(|_| Error::InvalidSize))
            .transpose()?
            .unwrap_or(data.len());
        if !start.is_multiple_of(4) || !end.is_multiple_of(4) || end <= start {
            return Err(Error::InvalidSize);
        }
        let raw = data.get(start..end).ok_or(Error::InvalidReference {
            field: "mouse rectangle offset",
            value: i64::from(offset),
            available: data.len(),
        })?;
        let header = Header::read_le(&mut Cursor::new(raw))
            .map_err(|_| Error::Truncated { offset: start })?;
        result.push(Rectangle { header, raw });
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn direct() -> Vec<u8> {
        [2u32, 0, 0, 0, 0, 5, 0, 1 << 29, 0x102f0, 0, 0]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect()
    }

    #[test]
    fn decodes_direct_events_and_preserves_callback_payloads() {
        let mut data = direct();
        let mut callback = direct();
        callback[36..40].copy_from_slice(&1u32.to_le_bytes());
        callback.extend([7; 44]);
        data.extend(callback);
        let index = [0u32, 44]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect::<Vec<_>>();
        let result = rectangles(&data, &index).unwrap();
        assert_eq!(result.len(), 2);
        assert_eq!(result[0].key_event(), Some(0x102f0));
        assert_eq!(result[0].header.flags, 1 << 29);
        assert_eq!(result[0].header.cursor, 5);
        assert_eq!(result[1].key_event(), None);
        assert_eq!(result[1].raw, &data[44..]);
        data[0..4].copy_from_slice(&99u32.to_le_bytes());
        assert!(rectangles(&data, &index).unwrap()[0].key_event().is_none());
    }

    #[test]
    fn rejects_truncated_and_out_of_range_tables() {
        let data = direct();
        for length in 0..44 {
            assert!(rectangles(&data[..length], &0u32.to_le_bytes()).is_err());
        }
        assert!(rectangles(&data, &[0; 3]).is_err());
        assert!(rectangles(&data, &1u32.to_le_bytes()).is_err());
        assert!(rectangles(&data, &u32::MAX.to_le_bytes()).is_err());
        for offsets in [[0u32, 0], [44, 0], [0, 48]] {
            let index = offsets
                .into_iter()
                .flat_map(u32::to_le_bytes)
                .collect::<Vec<_>>();
            assert!(rectangles(&data, &index).is_err());
        }
        assert!(rectangles(&data, &[]).is_err());
        assert!(rectangles(&[], &[]).unwrap().is_empty());
    }
}

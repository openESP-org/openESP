use binrw::BinRead;
use std::io::Cursor;
use uuid::Uuid;

#[derive(Debug, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    #[error("invalid SPB data at offset {offset}")]
    Truncated { offset: usize },
    #[error("invalid SPB signature")]
    Signature,
    #[error("invalid SPB tag count {0}")]
    TagCount(i32),
    #[error("SPB tag table declares {count} entries, but only {available} bytes remain")]
    TagTable { count: u32, available: usize },
    #[error("SPB tag index {index} exceeds {count} entries")]
    TagIndex { index: i32, count: usize },
}

#[derive(BinRead)]
#[br(little)]
struct RawTag {
    guid: [u8; 16],
    metadata: i32,
}

#[derive(Debug, PartialEq, Eq)]
pub struct Tag {
    pub guid: Uuid,
    pub metadata: i32,
}

#[derive(Debug)]
pub struct Bank<'a> {
    pub header: [i32; 10],
    pub tags: Vec<Tag>,
    pub body: &'a [u8],
}

impl<'a> Bank<'a> {
    pub fn parse_fsx(bytes: &'a [u8]) -> Result<Self, Error> {
        let mut reader = Cursor::new(bytes);
        let signature = u16::read_le(&mut reader).map_err(|_| Error::Truncated { offset: 0 })?;
        if signature != 0xebac {
            return Err(Error::Signature);
        }
        let header =
            <[i32; 10]>::read_le(&mut reader).map_err(|_| Error::Truncated { offset: 2 })?;
        if header[4] < 0 {
            return Err(Error::TagCount(header[4]));
        }
        // The count includes tag zero, which denotes an absent element.
        let count = header[4].saturating_sub(1) as u32;
        let available = bytes.len() - reader.position() as usize;
        if u64::from(count) > (available / 20) as u64 {
            return Err(Error::TagTable { count, available });
        }
        let mut tags = Vec::with_capacity(count as usize);
        for _ in 0..count {
            let offset = reader.position() as usize;
            let tag = RawTag::read(&mut reader).map_err(|_| Error::Truncated { offset })?;
            tags.push(Tag {
                guid: Uuid::from_bytes_le(tag.guid),
                metadata: tag.metadata,
            });
        }
        let offset = reader.position() as usize;
        let root = i32::read_le(&mut reader).map_err(|_| Error::Truncated { offset })?;
        let bank = Self {
            header,
            tags,
            body: &bytes[offset..],
        };
        bank.tag(root)?;
        Ok(bank)
    }

    pub fn tag(&self, index: i32) -> Result<Option<&Tag>, Error> {
        if index == 0 {
            return Ok(None);
        }
        usize::try_from(index)
            .ok()
            .and_then(|index| index.checked_sub(1))
            .and_then(|index| self.tags.get(index))
            .map(Some)
            .ok_or(Error::TagIndex {
                index,
                count: self.tags.len(),
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bank() -> Vec<u8> {
        let mut bytes = 0xebacu16.to_le_bytes().to_vec();
        bytes.extend(
            [11i32, 22, 33, 44, 2, 66, 77, 88, 99, 110]
                .into_iter()
                .flat_map(i32::to_le_bytes),
        );
        bytes.extend([4, 3, 2, 1, 6, 5, 8, 7, 9, 10, 11, 12, 13, 14, 15, 16]);
        bytes.extend((-1i32).to_le_bytes());
        bytes.extend(1i32.to_le_bytes());
        bytes
    }

    #[test]
    fn reads_fsx_tag_tables_and_preserves_unknown_header_words_and_body() {
        let bytes = [bank(), vec![42, 17, 99]].concat();
        let bank = Bank::parse_fsx(&bytes).unwrap();
        assert_eq!(bank.header, [11, 22, 33, 44, 2, 66, 77, 88, 99, 110]);
        let tag = bank.tag(1).unwrap().unwrap();
        assert_eq!(
            tag.guid,
            Uuid::from_u128(0x0102030405060708090a0b0c0d0e0f10)
        );
        assert_eq!(tag.metadata, -1);
        assert_eq!(bank.body, [1, 0, 0, 0, 42, 17, 99]);
        assert!(std::ptr::eq(bank.body.as_ptr(), bytes[62..].as_ptr()));
        assert_eq!(bank.tag(0).unwrap(), None);
        for index in [i32::MIN, -1, 2, i32::MAX] {
            assert_eq!(bank.tag(index), Err(Error::TagIndex { index, count: 1 }));
        }
    }

    #[test]
    fn bounds_headers_counts_and_root_indices() {
        let bytes = bank();
        for end in 0..bytes.len() {
            assert!(
                Bank::parse_fsx(&bytes[..end]).is_err(),
                "accepted prefix {end}"
            );
        }
        for count in [-1, i32::MIN, i32::MAX] {
            let mut bytes = bytes.clone();
            bytes[18..22].copy_from_slice(&count.to_le_bytes());
            assert!(Bank::parse_fsx(&bytes).is_err());
        }
        for root in [-1i32, 2, i32::MAX] {
            let mut bytes = bytes.clone();
            bytes[62..66].copy_from_slice(&root.to_le_bytes());
            assert!(matches!(
                Bank::parse_fsx(&bytes),
                Err(Error::TagIndex { .. })
            ));
        }
        let mut bytes = bytes;
        bytes[0] = 0;
        assert!(matches!(Bank::parse_fsx(&bytes), Err(Error::Signature)));
    }

    #[test]
    fn reads_an_absent_root_without_a_tag_table() {
        let bytes = [
            0xebacu16.to_le_bytes().to_vec(),
            [0i32, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
                .into_iter()
                .flat_map(i32::to_le_bytes)
                .collect(),
        ]
        .concat();
        let bank = Bank::parse_fsx(&bytes).unwrap();
        assert!(bank.tags.is_empty());
        assert_eq!(bank.body, [0; 4]);
    }
}

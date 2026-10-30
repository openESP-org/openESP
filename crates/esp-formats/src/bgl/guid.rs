use super::{Directory, Error, Section, range};
use binrw::BinRead;
use std::io::Cursor;
use uuid::Uuid;

#[derive(Debug)]
pub struct GuidRecord<'a> {
    pub guid: Uuid,
    /// Relative to the start of the subsection's data.
    pub offset: u32,
    pub data: &'a [u8],
}

impl<'a> Section<'a> {
    pub fn guid_records(&self) -> Result<Vec<GuidRecord<'a>>, Error> {
        if self.class != 5 {
            return Err(Error::NotGuidIndex);
        }
        let Directory::Direct(subsections) = &self.directory else {
            return Err(Error::NotGuidIndex);
        };
        let mut records = Vec::new();
        for subsection in subsections {
            let count = subsection.record_count.ok_or(Error::DirectorySize)?;
            let bytes = subsection.records.data;
            let table = range(bytes, 0, u64::from(count) * 24)?;
            for entry in table.as_chunks::<24>().0 {
                let mut cursor = Cursor::new(entry);
                let guid = Uuid::from_bytes_le(<[u8; 16]>::read_le(&mut cursor)?);
                let offset = u32::read_le(&mut cursor)?;
                let size = u32::read_le(&mut cursor)?;
                records.push(GuidRecord {
                    guid,
                    offset,
                    data: range(bytes, u64::from(offset), u64::from(size))?,
                });
            }
        }
        Ok(records)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        bgl::{RecordData, Subsection},
        mdl::Model,
    };

    fn section(bytes: &[u8], count: u32) -> Section<'_> {
        Section {
            kind: 0x2b,
            class: 5,
            qmid_high: 0,
            entry_count: 1,
            offset: 0,
            raw_directory: &[],
            directory: Directory::Direct(vec![Subsection {
                qmid: [0, 0],
                record_count: Some(count),
                records: RecordData {
                    offset: 0,
                    data: bytes,
                },
            }]),
        }
    }

    fn pointer(guid: Uuid, offset: u32, size: u32) -> Vec<u8> {
        [
            guid.to_bytes_le().to_vec(),
            offset.to_le_bytes().to_vec(),
            size.to_le_bytes().to_vec(),
        ]
        .concat()
    }

    #[test]
    fn resolves_offsets_from_the_index_base() {
        let guid = Uuid::parse_str("12345678-9abc-def0-1234-56789abcdef0").unwrap();
        let mut bytes = pointer(guid, 48, 12);
        bytes.extend(pointer(Uuid::nil(), 60, 12));
        for _ in 0..2 {
            bytes.extend(b"RIFF\x04\0\0\0MDLX");
        }
        let section = section(&bytes, 2);
        let records = section.guid_records().unwrap();
        assert_eq!(records[0].guid, guid);
        assert_eq!(records[1].guid, Uuid::nil());
        for record in records {
            assert_eq!(record.data.len(), 12);
            assert!(
                Model::parse(record.data)
                    .unwrap()
                    .decode()
                    .unwrap()
                    .unknown
                    .is_empty()
            );
        }
    }

    #[test]
    fn rejects_truncated_indices_and_invalid_model_ranges() {
        let bytes = pointer(Uuid::nil(), 24, 12);
        for end in 0..bytes.len() {
            assert!(section(&bytes[..end], 1).guid_records().is_err());
        }
        assert!(section(&bytes, 1).guid_records().is_err());
        assert!(section(&bytes, u32::MAX).guid_records().is_err());
        let bytes = pointer(Uuid::nil(), u32::MAX, 12);
        assert!(section(&bytes, 1).guid_records().is_err());
        let mut section = section(&[], 0);
        assert!(section.guid_records().unwrap().is_empty());
        section.class = 1;
        assert!(matches!(section.guid_records(), Err(Error::NotGuidIndex)));
    }
}

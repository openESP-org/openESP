use binrw::BinRead;
use std::io::Cursor;

pub mod guid;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("invalid BGL binary data: {0}")]
    Read(#[from] binrw::Error),
    #[error("unsupported BGL header")]
    Header,
    #[error("BGL range at {offset:#x} with size {size} exceeds the file")]
    Range { offset: u64, size: u64 },
    #[error("BGL section directory size does not match its entry count")]
    DirectorySize,
    #[error("BGL indirect record range exceeds its pointer table")]
    IndirectRange,
    #[error("BGL section is not a GUID index")]
    NotGuidIndex,
}

#[derive(Debug, BinRead)]
#[br(little)]
struct Header {
    magic: u32,
    size: u32,
    filetime: u64,
    qmid_magic: u32,
    section_count: u32,
    qmids: [u32; 8],
}

#[derive(Debug, BinRead)]
#[br(little)]
struct SectionHeader {
    kind: u32,
    class: u16,
    qmid_high: u16,
    count: u32,
    offset: u32,
    size: u32,
}

#[derive(Debug)]
pub struct Bgl<'a> {
    pub filetime: u64,
    pub qmids: [u32; 8],
    pub sections: Vec<Section<'a>>,
    pub raw_data: &'a [u8],
}

#[derive(Debug)]
pub struct Section<'a> {
    pub kind: u32,
    pub class: u16,
    pub qmid_high: u16,
    pub entry_count: u32,
    pub offset: u32,
    pub raw_directory: &'a [u8],
    pub directory: Directory<'a>,
}

#[derive(Debug)]
pub enum Directory<'a> {
    Direct(Vec<Subsection<'a>>),
    Indirect {
        subsections: Vec<IndirectSubsection>,
        records: Vec<RecordData<'a>>,
    },
    Unknown,
}

#[derive(Debug)]
pub struct Subsection<'a> {
    pub qmid: [u32; 2],
    pub record_count: Option<u32>,
    pub records: RecordData<'a>,
}

#[derive(Debug)]
pub struct IndirectSubsection {
    pub qmid: [u32; 2],
    pub record_count: u32,
    pub first_record: u32,
    pub size: u32,
}

#[derive(Debug)]
pub struct RecordData<'a> {
    pub offset: u32,
    pub data: &'a [u8],
}

impl<'a> Bgl<'a> {
    pub fn parse(bytes: &'a [u8]) -> Result<Self, Error> {
        let header = Header::read(&mut Cursor::new(bytes))?;
        if header.magic != 0x19920201 || header.size != 56 || header.qmid_magic != 0x08051803 {
            return Err(Error::Header);
        }
        let entries = range(bytes, 56, u64::from(header.section_count) * 20)?;
        let mut sections = Vec::new();
        for entry in entries.as_chunks::<20>().0 {
            let h = SectionHeader::read(&mut Cursor::new(entry))?;
            let raw_directory = range(bytes, u64::from(h.offset), u64::from(h.size))?;
            let directory = parse_directory(bytes, &h, raw_directory)?;
            sections.push(Section {
                kind: h.kind,
                class: h.class,
                qmid_high: h.qmid_high,
                entry_count: h.count,
                offset: h.offset,
                raw_directory,
                directory,
            });
        }
        Ok(Self {
            filetime: header.filetime,
            qmids: header.qmids,
            sections,
            raw_data: bytes,
        })
    }
}

fn range(bytes: &[u8], offset: u64, size: u64) -> Result<&[u8], Error> {
    let error = || Error::Range { offset, size };
    let end = offset.checked_add(size).ok_or_else(error)?;
    let start = usize::try_from(offset).map_err(|_| error())?;
    let end = usize::try_from(end).map_err(|_| error())?;
    bytes.get(start..end).ok_or_else(error)
}

fn parse_directory<'a>(
    bytes: &'a [u8],
    h: &SectionHeader,
    directory: &[u8],
) -> Result<Directory<'a>, Error> {
    if h.qmid_high > 1 || !(1..=7).contains(&h.class) {
        return Ok(Directory::Unknown);
    }
    let entry_size = if h.qmid_high == 1 { 20 } else { 16 };
    let table_size = u64::from(h.count) * entry_size;
    if table_size > directory.len() as u64 || (h.class != 2 && table_size != directory.len() as u64)
    {
        return Err(Error::DirectorySize);
    }
    let table = &directory[..table_size as usize];
    let mut subsections = Vec::new();
    let mut indirect = Vec::new();
    for entry in table.chunks_exact(entry_size as usize) {
        let mut cursor = Cursor::new(entry);
        let low = u32::read_le(&mut cursor)?;
        let high = if h.qmid_high == 1 {
            u32::read_le(&mut cursor)?
        } else {
            0
        };
        let field = u32::read_le(&mut cursor)?;
        let offset = u32::read_le(&mut cursor)?;
        let size = u32::read_le(&mut cursor)?;
        if h.class == 2 {
            indirect.push(IndirectSubsection {
                qmid: [low, high],
                record_count: field,
                first_record: offset,
                size,
            });
        } else {
            // Packed terrain directories use the count slot for QMID high.
            let packed = h.class == 1
                && h.qmid_high == 0
                && matches!(h.kind, 0x65 | 0x67..=0x6a | 0x6c | 0x6e..=0x6f | 0x78..=0x83 | 0x8c..=0x98);
            subsections.push(Subsection {
                qmid: [low, if packed { field } else { high }],
                record_count: if packed { None } else { Some(field) },
                records: RecordData {
                    offset,
                    data: range(bytes, u64::from(offset), u64::from(size))?,
                },
            });
        }
    }
    if h.class != 2 {
        return Ok(Directory::Direct(subsections));
    }
    let remaining = &directory[table_size as usize..];
    if !remaining.len().is_multiple_of(8) {
        return Err(Error::DirectorySize);
    }
    let mut records = Vec::new();
    for entry in remaining.as_chunks::<8>().0 {
        let mut cursor = Cursor::new(entry);
        let offset = u32::read_le(&mut cursor)?;
        let size = u32::read_le(&mut cursor)?;
        records.push(RecordData {
            offset,
            data: range(bytes, u64::from(offset), u64::from(size))?,
        });
    }
    for subsection in &indirect {
        let end = u64::from(subsection.first_record) + u64::from(subsection.record_count);
        if end > records.len() as u64 {
            return Err(Error::IndirectRange);
        }
    }
    Ok(Directory::Indirect {
        subsections: indirect,
        records,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn put(bytes: &mut [u8], offset: usize, value: u32) {
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn file(class: u32, kind: u32, entries: &[u32], payload: &[u8]) -> Vec<u8> {
        let mut bytes = vec![0; 76];
        put(&mut bytes, 0, 0x19920201);
        put(&mut bytes, 4, 56);
        put(&mut bytes, 16, 0x08051803);
        put(&mut bytes, 20, 1);
        put(&mut bytes, 24, 0x207e8);
        put(&mut bytes, 56, kind);
        put(&mut bytes, 60, class);
        put(&mut bytes, 64, 1);
        put(&mut bytes, 68, 76);
        put(&mut bytes, 72, entries.len() as u32 * 4);
        for value in entries {
            bytes.extend(value.to_le_bytes());
        }
        bytes.extend(payload);
        bytes
    }

    #[test]
    fn reads_direct_and_extended_directories() {
        for (class, entries) in [
            (1, vec![0x1234, 2, 92, 3]),
            (0x10001, vec![0x1234, 7, 2, 96, 3]),
        ] {
            let bytes = file(class, 0x25, &entries, &[1, 2, 3]);
            let bgl = Bgl::parse(&bytes).unwrap();
            assert_eq!(bgl.qmids[0], 0x207e8);
            let Directory::Direct(subsections) = &bgl.sections[0].directory else {
                panic!()
            };
            assert_eq!(
                subsections[0].qmid,
                [0x1234, if class == 1 { 0 } else { 7 }]
            );
            assert_eq!(subsections[0].record_count, Some(2));
            assert_eq!(subsections[0].records.data, &[1, 2, 3]);
            assert_eq!(
                subsections[0].records.data.as_ptr(),
                bytes[bytes.len() - 3..].as_ptr()
            );
        }
    }

    #[test]
    fn reads_packed_terrain_qmids() {
        let bytes = file(1, 0x67, &[0x1234, 9, 92, 3], &[1, 2, 3]);
        let bgl = Bgl::parse(&bytes).unwrap();
        let Directory::Direct(subsections) = &bgl.sections[0].directory else {
            panic!()
        };
        assert_eq!(subsections[0].qmid, [0x1234, 9]);
        assert_eq!(subsections[0].record_count, None);
    }

    #[test]
    fn resolves_indirect_pointers_and_checks_indices() {
        let mut bytes = file(2, 0x13, &[0x1234, 2, 0, 0, 108, 2, 110, 1], &[5, 6, 7]);
        let bgl = Bgl::parse(&bytes).unwrap();
        let Directory::Indirect {
            subsections,
            records,
        } = &bgl.sections[0].directory
        else {
            panic!()
        };
        assert_eq!(subsections[0].first_record, 0);
        assert_eq!(subsections[0].record_count, 2);
        assert_eq!(records[0].data, &[5, 6]);
        assert_eq!(records[1].data, &[7]);
        put(&mut bytes, 84, 1);
        assert!(matches!(Bgl::parse(&bytes), Err(Error::IndirectRange)));
    }

    #[test]
    fn retains_unknown_sections_without_guessing_their_layout() {
        let bytes = file(99, 0xffff, &[1, 2, 3], &[]);
        let bgl = Bgl::parse(&bytes).unwrap();
        assert_eq!(bgl.sections[0].kind, 0xffff);
        assert_eq!(bgl.sections[0].raw_directory, &bytes[76..]);
        assert!(matches!(bgl.sections[0].directory, Directory::Unknown));
        assert_eq!(bgl.raw_data, &bytes);
    }

    #[test]
    fn rejects_truncation_and_unbounded_counts() {
        let bytes = file(1, 0x25, &[0x1234, 1, 92, 3], &[1, 2, 3]);
        for end in 0..bytes.len() {
            assert!(Bgl::parse(&bytes[..end]).is_err(), "length {end}");
        }
        for (offset, value) in [
            (0, 0),
            (4, 55),
            (16, 0),
            (20, u32::MAX),
            (64, u32::MAX),
            (68, u32::MAX),
            (72, 15),
            (84, u32::MAX),
            (88, u32::MAX),
        ] {
            let mut invalid = bytes.clone();
            put(&mut invalid, offset, value);
            assert!(Bgl::parse(&invalid).is_err(), "offset {offset}");
        }
    }
}

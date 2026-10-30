pub use ::cabinet::{Cabinet, Compression, Entry, MemberReader};

#[cfg(test)]
mod tests {
    use super::*;
    use ::cabinet::{CabinetBuilder, WriteCompression};
    use std::io::{Cursor, Read};

    fn archive(compression: WriteCompression, files: &[(&str, &[u8])]) -> Vec<u8> {
        let mut builder = CabinetBuilder::new(compression);
        for (name, data) in files {
            builder.add_file(name, data).unwrap();
        }
        let mut output = Cursor::new(Vec::new());
        builder.write(&mut output).unwrap();
        output.into_inner()
    }

    #[test]
    fn reads_compressed_gauge_resources() {
        let bitmap: Vec<_> = (0..90000).map(|index| (index % 251) as u8).collect();
        let files = [
            ("Gauge.xml", b"<Gauge Name=\"test\"/>".as_slice()),
            ("1024\\Background.bmp", bitmap.as_slice()),
            ("empty.xml", b"".as_slice()),
        ];
        for compression in [
            WriteCompression::None,
            WriteCompression::MsZip,
            WriteCompression::Lzx { window_order: 17 },
            WriteCompression::Quantum {
                level: 4,
                window_order: 18,
            },
        ] {
            let bytes = archive(compression, &files);
            let mut cabinet = Cabinet::new(Cursor::new(bytes)).unwrap();
            assert_eq!(cabinet.entries().len(), 3);
            assert_eq!(
                cabinet.entry("1024/background.BMP").unwrap().name,
                "1024\\Background.bmp"
            );
            for (name, expected) in files {
                assert_eq!(
                    cabinet.read_file_bytes(name, expected.len()).unwrap(),
                    expected
                );
            }
            let mut streamed = Vec::new();
            cabinet
                .read_file("1024/background.BMP")
                .unwrap()
                .read_to_end(&mut streamed)
                .unwrap();
            assert_eq!(streamed, bitmap);
            assert!(cabinet.read_file_bytes("GAUGE.XML", 1).is_err());
            assert!(cabinet.read_file_bytes("missing", usize::MAX).is_err());
        }
    }

    #[test]
    fn rejects_ambiguous_windows_names() {
        let mut bytes = archive(
            WriteCompression::None,
            &[("Images\\a.bmp", b"one"), ("images/b.BMP", b"two")],
        );
        let position = bytes
            .windows(12)
            .position(|name| name == b"images/b.BMP")
            .unwrap();
        bytes[position + 7] = b'a';
        assert!(Cabinet::new(Cursor::new(bytes)).is_err());
    }

    #[test]
    fn rejects_truncated_and_damaged_archives() {
        for compression in [WriteCompression::None, WriteCompression::MsZip] {
            let bytes = archive(compression, &[("gauge.xml", b"<Gauge/>")]);
            for end in 0..bytes.len() {
                assert!(Cabinet::new(Cursor::new(&bytes[..end])).is_err());
            }
            let mut damaged = bytes;
            *damaged.last_mut().unwrap() ^= 255;
            let mut cabinet = Cabinet::new(Cursor::new(damaged)).unwrap();
            assert!(cabinet.read_file_bytes("gauge.xml", 1024).is_err());
        }
    }

    #[test]
    fn rejects_out_of_range_member_offset() {
        let mut bytes = archive(WriteCompression::None, &[("gauge.xml", b"<Gauge/>")]);
        let table = u32::from_le_bytes(bytes[16..20].try_into().unwrap()) as usize;
        bytes[table + 4..table + 8].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(Cabinet::new(Cursor::new(bytes)).is_err());
    }

    #[test]
    fn reads_empty_members() {
        let bytes = archive(WriteCompression::None, &[("empty.xml", b"")]);
        let mut cabinet = Cabinet::new(Cursor::new(bytes)).unwrap();
        assert!(cabinet.read_file_bytes("EMPTY.XML", 0).unwrap().is_empty());
    }
}

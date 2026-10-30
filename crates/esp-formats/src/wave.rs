use binrw::BinRead;
use std::io::Cursor;
pub use symphonia::{
    core,
    default::{formats::WavReader, get_codecs},
};

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("invalid WAVE chunk header: {0}")]
    Header(#[from] binrw::Error),
    #[error("expected a RIFF WAVE container")]
    InvalidForm,
    #[error("missing WAVE format chunk")]
    MissingFormat,
    #[error("duplicate WAVE format chunks")]
    DuplicateFormat,
    #[error("invalid WAVE format field {0}")]
    InvalidFormat(&'static str),
    #[error("WAVE chunk at {offset} needs {required} bytes including padding, found {available}")]
    Bounds {
        offset: usize,
        required: u64,
        available: usize,
    },
    #[error(transparent)]
    Audio(#[from] core::errors::Error),
}

#[derive(BinRead)]
#[br(little)]
struct Header {
    id: [u8; 4],
    size: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, BinRead)]
#[br(little)]
pub struct Format {
    pub tag: u16,
    pub channels: u16,
    pub sample_rate: u32,
    pub average_bytes_per_second: u32,
    pub block_align: u16,
    pub bits_per_sample: u16,
}

#[derive(Debug, PartialEq, Eq)]
pub struct Chunk<'a> {
    pub id: [u8; 4],
    pub offset: usize,
    pub data: &'a [u8],
    pub padding: &'a [u8],
}

#[derive(Debug)]
pub struct Wave<'a> {
    chunks: Vec<Chunk<'a>>,
    pub trailing_data: &'a [u8],
    data: &'a [u8],
}

impl<'a> Wave<'a> {
    pub fn parse(data: &'a [u8]) -> Result<Self, Error> {
        let header = Header::read(&mut Cursor::new(data))?;
        if header.id != *b"RIFF" || header.size < 4 {
            return Err(Error::InvalidForm);
        }
        let (body, trailing_data) = usize::try_from(header.size)
            .ok()
            .and_then(|size| data[8..].split_at_checked(size))
            .ok_or(Error::Bounds {
                offset: 0,
                required: u64::from(header.size),
                available: data.len() - 8,
            })?;
        if &body[..4] != b"WAVE" {
            return Err(Error::InvalidForm);
        }
        let end = 8 + body.len();
        let mut reader = Cursor::new(&data[..end]);
        reader.set_position(12);
        let mut chunks = Vec::new();
        while reader.position() < end as u64 {
            let offset = reader.position() as usize;
            let header = Header::read(&mut reader)?;
            let available = end - offset - 8;
            let padded = u64::from(header.size) + u64::from(header.size % 2);
            let count = usize::try_from(padded)
                .ok()
                .filter(|&size| size <= available)
                .ok_or(Error::Bounds {
                    offset,
                    required: padded,
                    available,
                })?;
            let start = offset + 8;
            chunks.push(Chunk {
                id: header.id,
                offset,
                data: &data[start..start + header.size as usize],
                padding: &data[start + header.size as usize..start + count],
            });
            reader.set_position((start + count) as u64);
        }
        Ok(Self {
            chunks,
            trailing_data,
            data: &data[..end],
        })
    }

    pub fn reader(&self) -> Result<WavReader<'a>, Error> {
        use self::core::io::MediaSourceStream;
        let format = self.format()?;
        if matches!(format.tag, 1 | 3)
            && self.chunks.iter().any(|chunk| {
                chunk.id == *b"data"
                    && !chunk
                        .data
                        .len()
                        .is_multiple_of(usize::from(format.block_align))
            })
        {
            return Err(Error::InvalidFormat("PCM data alignment"));
        }
        Ok(WavReader::try_new(
            MediaSourceStream::new(Box::new(Cursor::new(self.data)), Default::default()),
            Default::default(),
        )?)
    }

    pub fn chunks(&self) -> &[Chunk<'a>] {
        &self.chunks
    }

    pub fn format(&self) -> Result<Format, Error> {
        let mut formats = self.chunks.iter().filter(|chunk| chunk.id == *b"fmt ");
        let chunk = formats.next().ok_or(Error::MissingFormat)?;
        if formats.next().is_some() {
            return Err(Error::DuplicateFormat);
        }
        let mut reader = Cursor::new(chunk.data);
        let format = Format::read(&mut reader)?;
        if format.channels == 0 {
            return Err(Error::InvalidFormat("channels"));
        }
        if format.sample_rate == 0 {
            return Err(Error::InvalidFormat("sample rate"));
        }
        if format.block_align == 0 {
            return Err(Error::InvalidFormat("block alignment"));
        }
        if matches!(format.tag, 1 | 3) {
            let block = u32::from(format.channels) * u32::from(format.bits_per_sample).div_ceil(8);
            if block == 0 || u32::from(format.block_align) != block {
                return Err(Error::InvalidFormat("block alignment"));
            }
            if u64::from(format.sample_rate) * u64::from(block)
                != u64::from(format.average_bytes_per_second)
            {
                return Err(Error::InvalidFormat("average byte rate"));
            }
        }
        if format.tag != 1 && chunk.data.len() > 16 {
            let size = u16::read_le(&mut reader)?;
            if usize::from(size) > chunk.data.len() - 18 {
                return Err(Error::InvalidFormat("extension size"));
            }
        }
        Ok(format)
    }
}

#[cfg(test)]
mod tests {
    use super::{Error, WavReader, Wave, core::formats::FormatReader, get_codecs};

    fn chunk(id: &[u8; 4], data: &[u8]) -> Vec<u8> {
        let mut bytes = [id.as_slice(), &(data.len() as u32).to_le_bytes(), data].concat();
        if !data.len().is_multiple_of(2) {
            bytes.push(0);
        }
        bytes
    }

    fn format(tag: u16, channels: u16, bits: u16, block: u16, extra: &[u8]) -> Vec<u8> {
        let mut bytes = [
            tag.to_le_bytes().as_slice(),
            &channels.to_le_bytes(),
            &8000u32.to_le_bytes(),
            &(8000u32 * u32::from(block)).to_le_bytes(),
            &block.to_le_bytes(),
            &bits.to_le_bytes(),
        ]
        .concat();
        if !extra.is_empty() {
            bytes.extend((extra.len() as u16).to_le_bytes());
            bytes.extend(extra);
        }
        bytes
    }

    fn wave(format: &[u8], data: &[u8]) -> Vec<u8> {
        let chunks = [
            b"WAVE".as_slice(),
            &chunk(b"JUNK", &[42]),
            &chunk(b"fmt ", format),
            &chunk(b"data", data),
        ]
        .concat();
        chunk(b"RIFF", &chunks)
    }

    fn reader(data: &[u8]) -> Result<WavReader<'_>, Error> {
        Wave::parse(data)?.reader()
    }

    fn samples(data: &[u8]) -> Result<Vec<f32>, Error> {
        let mut reader = reader(data)?;
        let params = reader.tracks()[0]
            .codec_params
            .as_ref()
            .unwrap()
            .audio()
            .unwrap();
        assert_eq!(params.sample_rate, Some(8000));
        let mut decoder = get_codecs().make_audio_decoder(params, &Default::default())?;
        let mut samples = Vec::new();
        while let Some(packet) = reader.next_packet()? {
            let audio = decoder.decode(&packet)?;
            let mut interleaved = Vec::<f32>::new();
            audio.copy_to_vec_interleaved(&mut interleaved);
            samples.extend(interleaved);
        }
        Ok(samples)
    }

    #[test]
    fn decodes_pcm_and_float_samples_and_skips_padded_unknown_chunks() {
        let data: Vec<_> = [-32768i16, 16384, 0, -16384]
            .into_iter()
            .flat_map(i16::to_le_bytes)
            .collect();
        assert_eq!(
            samples(&wave(&format(1, 2, 16, 4, &[]), &data)).unwrap(),
            [-1.0, 0.5, 0.0, -0.5]
        );
        assert_eq!(
            samples(&wave(&format(1, 1, 8, 1, &[]), &[0, 128, 255])).unwrap(),
            [-1.0, 0.0, 127.0 / 128.0]
        );
        let data: Vec<_> = [-0.75f32, 0.25, 1.0]
            .into_iter()
            .flat_map(f32::to_le_bytes)
            .collect();
        assert_eq!(
            samples(&wave(&format(3, 1, 32, 4, &[]), &data)).unwrap(),
            [-0.75, 0.25, 1.0]
        );
    }

    #[test]
    fn decodes_ima_and_microsoft_adpcm_blocks() {
        let ima = [1000i16.to_le_bytes().as_slice(), &[0; 6]].concat();
        assert_eq!(
            samples(&wave(&format(0x11, 1, 4, 8, &9u16.to_le_bytes()), &ima)).unwrap(),
            vec![1000.0 / 32768.0; 9]
        );
        let extra: Vec<_> = [
            4i16, 7, 256, 0, 512, -256, 0, 0, 192, 64, 240, 0, 460, -208, 392, -232,
        ]
        .into_iter()
        .flat_map(i16::to_le_bytes)
        .collect();
        let ms = [
            &[0],
            16i16.to_le_bytes().as_slice(),
            &1000i16.to_le_bytes(),
            &1000i16.to_le_bytes(),
            &[0],
        ]
        .concat();
        assert_eq!(
            samples(&wave(&format(2, 1, 4, 8, &extra), &ms)).unwrap(),
            vec![1000.0 / 32768.0; 4]
        );
    }

    #[test]
    fn rejects_truncated_headers_data_and_invalid_formats() {
        let bytes = wave(&format(1, 1, 16, 2, &[]), &[0; 4]);
        for end in 0..bytes.len() {
            assert!(samples(&bytes[..end]).is_err(), "accepted prefix {end}");
        }
        for fmt in [
            format(1, 0, 16, 2, &[]),
            format(1, 1, 16, 0, &[]),
            format(0xffff, 1, 16, 2, &[]),
        ] {
            assert!(
                samples(&wave(&fmt, &[0; 4])).is_err(),
                "accepted format {fmt:?}"
            );
        }
        assert!(matches!(
            reader(&wave(&format(1, 1, 16, 2, &[]), &[0; 3])),
            Err(Error::InvalidFormat("PCM data alignment"))
        ));
        let mut bytes = bytes;
        bytes[..4].copy_from_slice(b"MDLX");
        assert!(reader(&bytes).is_err());
    }

    #[test]
    fn retains_unknown_chunks_padding_extensions_and_trailing_bytes() {
        let bytes = [
            wave(&format(1, 1, 8, 1, &[0xaa, 0xbb]), &[128]),
            vec![0xcc; 3],
        ]
        .concat();
        let wave = Wave::parse(&bytes).unwrap();
        assert_eq!(
            wave.chunks()[0],
            super::Chunk {
                id: *b"JUNK",
                offset: 12,
                data: &[42],
                padding: &[0]
            }
        );
        assert_eq!(&wave.chunks()[1].data[16..], [2, 0, 0xaa, 0xbb]);
        assert_eq!(wave.chunks()[2].padding, [0]);
        assert_eq!(wave.trailing_data, [0xcc; 3]);
        assert!(std::ptr::eq(
            wave.chunks()[0].data.as_ptr(),
            bytes[20..].as_ptr()
        ));
        assert_eq!(wave.format().unwrap().bits_per_sample, 8);
    }

    #[test]
    fn rejects_chunk_ranges_that_escape_the_declared_parent() {
        let bytes = wave(&format(1, 1, 8, 1, &[]), &[128]);
        let mut missing_padding = bytes.clone();
        let size = missing_padding.len() as u32 - 9;
        missing_padding[4..8].copy_from_slice(&size.to_le_bytes());
        assert!(matches!(
            Wave::parse(&missing_padding),
            Err(Error::Bounds {
                required: 2,
                available: 1,
                ..
            })
        ));
        let mut huge_chunk = bytes;
        huge_chunk[16..20].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(matches!(
            Wave::parse(&huge_chunk),
            Err(Error::Bounds {
                required: 4294967296,
                ..
            })
        ));
        let format = format(1, 1, 8, 1, &[]);
        let duplicate = chunk(
            b"RIFF",
            &[
                b"WAVE".as_slice(),
                &chunk(b"fmt ", &format),
                &chunk(b"fmt ", &format),
                &chunk(b"data", &[128]),
            ]
            .concat(),
        );
        assert!(matches!(
            Wave::parse(&duplicate).unwrap().reader(),
            Err(Error::DuplicateFormat)
        ));
        let missing = chunk(b"RIFF", b"WAVE");
        assert!(matches!(
            Wave::parse(&missing).unwrap().reader(),
            Err(Error::MissingFormat)
        ));
    }
}

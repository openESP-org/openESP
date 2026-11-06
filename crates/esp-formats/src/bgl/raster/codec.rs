//! Checked Rust implementation of the FSX raster delta and LZ1 algorithms.
//! Algorithm reference: seanisom/flightsimlib, BglDecompressor.cpp (MIT).
//! Its license is retained in third_party/flightsimlib-LICENSE.
use super::Compression;
mod bitpack;
mod ptc;

const LIMIT: usize = 64 * 1024 * 1024;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("truncated raster compression stream")]
    Truncated,
    #[error("invalid raster compression stream")]
    InvalidData,
    #[error("raster output exceeds 64 MiB tile limit")]
    Limit,
    #[error("unsupported raster compression {0:?}")]
    Unsupported(Compression),
}

pub fn decode(compression: Compression, input: &[u8], size: usize) -> Result<Vec<u8>, Error> {
    if size > LIMIT {
        return Err(Error::Limit);
    }
    match compression {
        Compression::None => Ok(input.get(..size).ok_or(Error::Truncated)?.to_vec()),
        Compression::Delta => delta(input, size),
        Compression::Ptc => ptc::photo(input, size),
        Compression::Lz1 => lz1(input, size),
        Compression::Lz2 => lz(input, size, true),
        Compression::DeltaLz1 | Compression::DeltaLz2 => {
            let intermediate_size =
                u32::from_le_bytes(input.get(..4).ok_or(Error::Truncated)?.try_into().unwrap())
                    as usize;
            let intermediate = lz(
                &input[4..],
                intermediate_size,
                compression == Compression::DeltaLz2,
            )?;
            delta(&intermediate, size)
        }
        other => Err(Error::Unsupported(other)),
    }
}

struct Bits<'a> {
    input: &'a [u8],
    position: usize,
}
impl Bits<'_> {
    fn read(&mut self, count: usize) -> Result<usize, Error> {
        if count > 32 {
            return Err(Error::InvalidData);
        }
        let mut value = 0;
        for shift in 0..count {
            let byte = self.input.get(self.position / 8).ok_or(Error::Truncated)?;
            value |= usize::from((byte >> (self.position % 8)) & 1) << shift;
            self.position += 1;
        }
        Ok(value)
    }
}

fn lz1(input: &[u8], size: usize) -> Result<Vec<u8>, Error> {
    lz(input, size, false)
}

fn lz(input: &[u8], size: usize, version2: bool) -> Result<Vec<u8>, Error> {
    if size > LIMIT {
        return Err(Error::Limit);
    }
    let header = input.get(..4).ok_or(Error::Truncated)?;
    if &header[..2] != if version2 { b"JM" } else { b"DS" } {
        return Err(Error::InvalidData);
    }
    let mut bits = Bits {
        input: &input[4..],
        position: 0,
    };
    let mut output = Vec::with_capacity(size);
    while output.len() < size {
        let kind = if version2 {
            if bits.read(1)? == 0 {
                2
            } else if bits.read(1)? != 0 {
                1
            } else if bits.read(1)? != 0 {
                3
            } else {
                0
            }
        } else {
            bits.read(2)?
        };
        if kind == 1 || kind == 2 {
            let byte = bits.read(7)? as u8;
            output.push(byte | if kind == 1 { 0x80 } else { 0 });
            continue;
        }
        let offset = if kind == 3 {
            if bits.read(1)? != 0 {
                bits.read(12)? + 0x140
            } else {
                bits.read(8)? + 0x40
            }
        } else {
            bits.read(6)?
        };
        if offset == 0x113f {
            continue;
        }
        if offset == 0 || offset > output.len() {
            return Err(Error::InvalidData);
        }
        let mut count = 0;
        while bits.read(1)? == 0 {
            count += 1;
            if count > 15 {
                return Err(Error::InvalidData);
            }
        }
        let length = usize::from(version2)
            + if count == 0 {
                2
            } else {
                (1 << count) + bits.read(count)? + 1
            };
        if length > size - output.len() {
            return Err(Error::InvalidData);
        }
        for _ in 0..length {
            output.push(output[output.len() - offset]);
        }
    }
    Ok(output)
}

pub fn decode_mask(
    compression: Compression,
    input: &[u8],
    columns: u16,
    rows: u16,
) -> Result<Vec<u8>, Error> {
    let size = usize::from(columns) * usize::from(rows);
    if size > LIMIT {
        return Err(Error::Limit);
    }
    match compression {
        Compression::BitPack => bitpack::decode(input, columns, rows),
        Compression::BitPackLz1 | Compression::BitPackLz2 => {
            let intermediate_size =
                u32::from_le_bytes(input.get(..4).ok_or(Error::Truncated)?.try_into().unwrap())
                    as usize;
            let intermediate = lz(
                &input[4..],
                intermediate_size,
                compression == Compression::BitPackLz2,
            )?;
            bitpack::decode(&intermediate, columns, rows)
        }
        _ => decode(compression, input, size),
    }
}

fn delta(input: &[u8], size: usize) -> Result<Vec<u8>, Error> {
    let mut output = Vec::with_capacity(size);
    let mut position = 0;
    let take = |position: &mut usize, count| -> Result<&[u8], Error> {
        let bytes = input
            .get(*position..*position + count)
            .ok_or(Error::Truncated)?;
        *position += count;
        Ok(bytes)
    };
    if !size.is_multiple_of(2) {
        output.push(take(&mut position, 1)?[0]);
    }
    if size < 2 {
        return Ok(output);
    }
    let mut previous = i16::from_le_bytes(take(&mut position, 2)?.try_into().unwrap());
    output.extend(previous.to_le_bytes());
    while output.len() < size {
        let code = take(&mut position, 1)?[0];
        previous = match code {
            0x80 => i16::from_le_bytes(take(&mut position, 2)?.try_into().unwrap()),
            0x81 => previous.wrapping_sub(i16::from(take(&mut position, 1)?[0]) + 126),
            0x82 => previous.wrapping_add(i16::from(take(&mut position, 1)?[0]) + 128),
            other => previous.wrapping_add(i16::from(other as i8)),
        };
        output.extend(previous.to_le_bytes());
    }
    Ok(output)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lz2_reads_literals_and_overlapping_dictionary_runs() {
        let mut bits = Vec::new();
        for (value, count) in [(0usize, 1), (65, 7), (1, 1), (0, 1), (0, 1), (1, 6), (1, 1)] {
            for shift in 0..count {
                bits.push((value >> shift) & 1);
            }
        }
        let mut input = b"JM\0\0".to_vec();
        for chunk in bits.chunks(8) {
            input.push(
                chunk
                    .iter()
                    .enumerate()
                    .fold(0u8, |value, (index, bit)| value | ((*bit as u8) << index)),
            );
        }
        assert_eq!(decode(Compression::Lz2, &input, 4).unwrap(), b"AAAA");
        assert!(decode(Compression::Lz2, &input, 3).is_err());
        assert!(decode(Compression::Lz2, &input[..input.len() - 1], 4).is_err());
    }

    #[test]
    fn delta_handles_absolute_values_extended_deltas_and_overflow() {
        let input = [0xff, 0x7f, 1, 0x80, 100, 0, 0x81, 2, 0x82, 3, 0xff];
        let decoded = decode(Compression::Delta, &input, 12).unwrap();
        let values: Vec<_> = decoded
            .as_chunks::<2>()
            .0
            .iter()
            .map(|v| i16::from_le_bytes([v[0], v[1]]))
            .collect();
        assert_eq!(values, [32767, -32768, 100, -28, 103, 102]);
        assert!(decode(Compression::Delta, &input[..input.len() - 1], 12).is_err());
        assert_eq!(
            decode(Compression::Delta, &[7, 1, 0], 3).unwrap(),
            [7, 1, 0]
        );
    }

    #[test]
    fn lz1_reads_literals_and_rejects_truncation_and_bad_references() {
        let mut bits = Vec::new();
        for byte in [0u8, 127, 128, 255] {
            for (value, count) in [
                (if byte >= 128 { 1 } else { 2 }, 2),
                (usize::from(byte & 127), 7),
            ] {
                for index in 0..count {
                    bits.push((value >> index) & 1);
                }
            }
        }
        let mut input = b"DS\0\0".to_vec();
        for chunk in bits.chunks(8) {
            input.push(
                chunk
                    .iter()
                    .enumerate()
                    .fold(0u8, |value, (index, bit)| value | ((*bit as u8) << index)),
            );
        }
        assert_eq!(
            decode(Compression::Lz1, &input, 4).unwrap(),
            [0, 127, 128, 255]
        );
        assert!(decode(Compression::Lz1, &input[..input.len() - 1], 4).is_err());
        assert!(decode(Compression::Lz1, b"DS\0\0\0", 1).is_err());
        assert!(matches!(
            decode(Compression::Lz1, &input, LIMIT + 1),
            Err(Error::Limit)
        ));
    }
}

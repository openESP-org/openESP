//! FSX quadtree bit packing; see the flightsimlib attribution in codec.rs.
use super::{Bits, Error};

struct Image {
    pixels: Vec<u16>,
    width: usize,
    shift: usize,
    max_bits: usize,
}

pub(super) fn decode(input: &[u8], columns: u16, rows: u16) -> Result<Vec<u8>, Error> {
    Ok(decode_values(input, columns, rows)?
        .into_iter()
        .map(|value| value as u8)
        .collect())
}

pub(super) fn decode16(input: &[u8], columns: u16, rows: u16) -> Result<Vec<u8>, Error> {
    Ok(decode_values(input, columns, rows)?
        .into_iter()
        .flat_map(u16::to_le_bytes)
        .collect())
}

fn decode_values(input: &[u8], columns: u16, rows: u16) -> Result<Vec<u16>, Error> {
    if columns == 0 || rows == 0 {
        return Err(Error::InvalidData);
    }
    let mut bits = Bits { input, position: 0 };
    let add_bytes = bits.read(8)?;
    let shift = bits.read(8)?;
    if add_bytes > 4 || shift > 16 {
        return Err(Error::InvalidData);
    }
    let add = bits.read(add_bytes * 8)? as u64;
    let initial_bits = bits.read(4)?;
    let max_bits = match bits.read(4)? {
        0 => 16,
        value => value,
    };
    let width = usize::from(columns);
    let height = usize::from(rows);
    if width
        .checked_mul(height)
        .and_then(|size| size.checked_mul(2))
        .is_none_or(|size| size > super::LIMIT)
    {
        return Err(Error::Limit);
    }
    let mut image = Image {
        pixels: vec![0; width * height],
        width,
        shift,
        max_bits,
    };
    image.children(&mut bits, [0, 0, width, height], add, initial_bits)?;
    Ok(image.pixels)
}

impl Image {
    fn children(
        &mut self,
        bits: &mut Bits<'_>,
        [x, y, width, height]: [usize; 4],
        add: u64,
        count: usize,
    ) -> Result<(), Error> {
        for row in 0..4 {
            let rows = height / 4 + if row == 3 { height % 4 } else { 0 };
            for col in 0..4 {
                let cols = width / 4 + if col == 3 { width % 4 } else { 0 };
                self.slice(
                    bits,
                    [x + col * (width / 4), y + row * (height / 4), cols, rows],
                    add,
                    count,
                )?;
            }
        }
        Ok(())
    }

    fn slice(
        &mut self,
        bits: &mut Bits<'_>,
        region: [usize; 4],
        add: u64,
        count: usize,
    ) -> Result<(), Error> {
        let offset = bits.read(count.min(8))? as u64;
        let next = bits.read(4)?;
        let add = add + (offset << (count.saturating_sub(8) + self.shift));
        let [x, y, width, height] = region;
        if next != 0 && width >= 8 && height >= 8 {
            return self.children(bits, region, add, next);
        }
        for row in y..y + height {
            for col in x..x + width {
                let value = if next == 0 {
                    0
                } else {
                    bits.read(next.min(self.max_bits))? as u64
                };
                self.pixels[row * self.width + col] = (add + (value << self.shift)) as u16;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constant_mask_covers_uneven_partitions_and_rejects_truncation() {
        let mut input = vec![1, 0, 37, 0];
        input.extend([0; 8]);
        assert_eq!(decode(&input, 9, 10).unwrap(), vec![37; 90]);
        for end in 0..input.len() {
            assert!(decode(&input[..end], 9, 10).is_err());
        }
        input[0] = 255;
        assert!(decode(&input, 9, 10).is_err());
    }

    #[test]
    fn retains_high_bits_for_elevation_samples() {
        let mut input = vec![2, 0, 0x34, 0x12, 0];
        input.extend([0; 8]);
        let output = decode16(&input, 9, 10).unwrap();
        assert_eq!(output.len(), 180);
        assert!(
            output
                .as_chunks::<2>()
                .0
                .iter()
                .all(|sample| *sample == [0x34, 0x12])
        );
    }
}

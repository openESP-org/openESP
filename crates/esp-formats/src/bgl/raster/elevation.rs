use super::{Compression, Raster, codec};

/// Raster elevation samples in metres above mean sea level, row-major.
pub struct Elevation {
    pub columns: u16,
    pub rows: u16,
    pub metres: Vec<f32>,
}

impl Raster<'_> {
    pub fn decode_elevation(&self) -> Result<Elevation, codec::Error> {
        if self.data_type != 2 {
            return Err(codec::Error::InvalidData);
        }
        let (scale, base, data) = if self.data.starts_with(b"RCS1") {
            let header = self.data.get(..12).ok_or(codec::Error::Truncated)?;
            let scale = f32::from_le_bytes(header[4..8].try_into().unwrap());
            let base = f32::from_le_bytes(header[8..12].try_into().unwrap());
            if !scale.is_finite() || scale <= 0.0 || !base.is_finite() {
                return Err(codec::Error::InvalidData);
            }
            (scale, base, &self.data[12..])
        } else {
            (1.0, 0.0, self.data)
        };
        let decoded = if self.data_compression == Compression::Ptc {
            expand_ptc(data, self.columns, self.rows)?
        } else {
            codec::decode_raster16(self.data_compression, data, self.columns, self.rows)?
        };
        let metres: Vec<_> = decoded
            .as_chunks::<2>()
            .0
            .iter()
            .map(|bytes| f32::from(i16::from_le_bytes(*bytes)) * scale + base)
            .collect();
        if metres.iter().any(|value| !value.is_finite()) {
            return Err(codec::Error::InvalidData);
        }
        Ok(Elevation {
            columns: self.columns,
            rows: self.rows,
            metres,
        })
    }
}

fn expand_ptc(data: &[u8], columns: u16, rows: u16) -> Result<Vec<u8>, codec::Error> {
    let prefix = data.get(..4).ok_or(codec::Error::Truncated)?;
    let skip_row = usize::from(prefix[1] & 1);
    let skip_col = usize::from((prefix[1] >> 1) & 1);
    if prefix[0] != 5
        || usize::from(columns) != 256 + skip_col
        || usize::from(rows) != 256 + skip_row
    {
        return Err(codec::Error::InvalidData);
    }
    let bias = i16::from_le_bytes([prefix[2], prefix[3]]);
    let source = codec::decode_ptc_elevation(data)?;
    let width = usize::from(columns);
    let mut output = vec![0u8; width * usize::from(rows) * 2];
    for y in 0..256 {
        for x in 0..width {
            let source_x = x.saturating_sub(skip_col);
            let offset = (y * 256 + source_x) * 2;
            let value = i16::from_le_bytes([source[offset], source[offset + 1]]).wrapping_sub(bias);
            let target = ((y + skip_row) * width + x) * 2;
            output[target..target + 2].copy_from_slice(&value.to_le_bytes());
        }
    }
    if skip_row != 0 {
        output.copy_within(width * 2..width * 4, 0);
    }
    Ok(output)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bgl::raster::Raster;

    #[test]
    fn applies_elevation_scale_and_rejects_truncated_scaling() {
        let mut data = b"RCS1".to_vec();
        data.extend_from_slice(&0.5f32.to_le_bytes());
        data.extend_from_slice(&100.0f32.to_le_bytes());
        data.extend_from_slice(&(-20i16).to_le_bytes());
        data.extend_from_slice(&(40i16).to_le_bytes());
        let mut bytes = vec![0u8; 40];
        bytes[..4].copy_from_slice(b"TRQ1");
        bytes[4..8].copy_from_slice(&40u32.to_le_bytes());
        bytes[8..10].copy_from_slice(&2u16.to_le_bytes());
        bytes[24..26].copy_from_slice(&2u16.to_le_bytes());
        bytes[28..30].copy_from_slice(&1u16.to_le_bytes());
        bytes[32..36].copy_from_slice(&(data.len() as u32).to_le_bytes());
        bytes.extend(data);
        let tile = Raster::parse(&bytes).unwrap();
        assert_eq!(tile.decode_elevation().unwrap().metres, [90.0, 120.0]);
        assert!(expand_ptc(&[5, 3, 0, 0], 256, 257).is_err());
        assert!(expand_ptc(&[5, 3, 0], 257, 257).is_err());
    }
}

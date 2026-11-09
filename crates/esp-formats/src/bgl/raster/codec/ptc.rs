use super::Error;

unsafe extern "C" {
    fn openesp_ptc_photo(source: *const u8, length: i32, output: *mut u8, output_size: i32) -> i32;
    fn openesp_ptc_elevation(
        source: *const u8,
        length: i32,
        output: *mut u8,
        output_size: i32,
    ) -> i32;
}

pub(super) fn elevation(input: &[u8]) -> Result<Vec<u8>, Error> {
    const SIZE: usize = 256 * 256 * 2;
    if input.len() > super::LIMIT {
        return Err(Error::Limit);
    }
    let mut output = vec![0u8; SIZE];
    // The bridge validates the fixed monochrome frame and tile bounds before decoding.
    let result = unsafe {
        openesp_ptc_elevation(
            input.as_ptr(),
            input.len() as i32,
            output.as_mut_ptr(),
            SIZE as i32,
        )
    };
    if result != SIZE as i32 {
        return Err(Error::InvalidData);
    }
    Ok(output)
}

pub(super) fn photo(input: &[u8], size: usize) -> Result<Vec<u8>, Error> {
    if size != 256 * 256 * 2 || input.len() > super::LIMIT {
        return Err(Error::InvalidData);
    }
    let mut output = vec![0u8; size];
    // The bridge validates fixed dimensions and format before invoking the
    // native decoder; both buffers remain allocated throughout the call.
    let result = unsafe {
        openesp_ptc_photo(
            input.as_ptr(),
            input.len() as i32,
            output.as_mut_ptr(),
            size as i32,
        )
    };
    if result != size as i32 {
        return Err(Error::InvalidData);
    }
    Ok(output)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_incomplete_headers_and_unsupported_output_sizes() {
        for length in 0..128 {
            assert!(photo(&vec![0; length], 256 * 256 * 2).is_err());
        }
        assert!(photo(&[], 1024).is_err());
        let mut header = vec![0; 256];
        header[0] = 1;
        header[4..8].copy_from_slice(b"PTC\0");
        header[9] = 4;
        header[12..16].copy_from_slice(&256u32.to_le_bytes());
        header[16..20].copy_from_slice(&256u32.to_le_bytes());
        header[32..36].copy_from_slice(&44u32.to_le_bytes());
        assert!(photo(&header, 256 * 256 * 2).is_err());
    }
}

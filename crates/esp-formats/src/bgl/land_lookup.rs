//! FSX/P3D land-class to terrain texture-set lookup records.
use super::{Bgl, Directory};

#[derive(Debug, thiserror::Error)]
#[error("invalid or unsupported terrain texture lookup")]
pub struct Error;

/// The texture number selected for each land class in each region.
pub struct Lookup {
    texture_sets: Vec<TextureSet>,
    classes: Vec<Vec<Option<usize>>>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TextureSet {
    pub number: u16,
    pub region: u8,
}

impl Lookup {
    pub fn from_bgl(bgl: &Bgl<'_>) -> Result<Self, Error> {
        let section = bgl
            .sections
            .iter()
            .find(|section| section.kind == 0x6f)
            .ok_or(Error)?;
        let Directory::Direct(subsections) = &section.directory else {
            return Err(Error);
        };
        let [subsection] = subsections.as_slice() else {
            return Err(Error);
        };
        Self::parse(subsection.records.data)
    }

    pub fn parse(data: &[u8]) -> Result<Self, Error> {
        let u32_at = |offset: usize| -> Result<u32, Error> {
            Ok(u32::from_le_bytes(
                data.get(offset..offset.checked_add(4).ok_or(Error)?)
                    .ok_or(Error)?
                    .try_into()
                    .map_err(|_| Error)?,
            ))
        };
        if u32_at(0)? != 1 {
            return Err(Error);
        }
        let sets = u32_at(4)? as usize;
        let regions = u32_at(8)? as usize;
        let land_classes = u32_at(12)? as usize;
        let sets_offset = u32_at(20)? as usize;
        let regions_offset = u32_at(24)? as usize;
        if sets > 65_536 || regions > 256 || land_classes > 65_536 {
            return Err(Error);
        }
        let set_bytes = data
            .get(
                sets_offset
                    ..sets_offset
                        .checked_add(sets.checked_mul(24).ok_or(Error)?)
                        .ok_or(Error)?,
            )
            .ok_or(Error)?;
        let texture_sets = set_bytes
            .as_chunks::<24>()
            .0
            .iter()
            .map(|entry| TextureSet {
                number: u16::from_le_bytes([entry[0], entry[1]]),
                region: entry[2],
            })
            .collect();
        let mut classes = Vec::with_capacity(regions);
        for region in 0..regions {
            let entry_offset = regions_offset
                .checked_add(region.checked_mul(8).ok_or(Error)?)
                .ok_or(Error)?;
            let class_offset = u32_at(entry_offset)? as usize;
            let bytes = data
                .get(
                    class_offset
                        ..class_offset
                            .checked_add(land_classes.checked_mul(4).ok_or(Error)?)
                            .ok_or(Error)?,
                )
                .ok_or(Error)?;
            let mut mapping = Vec::with_capacity(land_classes);
            for entry in bytes.as_chunks::<4>().0 {
                let index = u32::from_le_bytes(*entry);
                if index != u32::MAX && index as usize >= sets {
                    return Err(Error);
                }
                mapping.push((index != u32::MAX).then_some(index as usize));
            }
            classes.push(mapping);
        }
        Ok(Self {
            texture_sets,
            classes,
        })
    }

    pub fn texture_set(&self, region: u8, class: u8) -> Option<TextureSet> {
        let index = self
            .classes
            .get(usize::from(region))?
            .get(usize::from(class))?
            .as_ref()?;
        self.texture_sets.get(*index).copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_invalid_indices_and_truncated_tables() {
        let mut data = vec![0; 100];
        let put = |data: &mut [u8], offset, value: u32| {
            data[offset..offset + 4].copy_from_slice(&value.to_le_bytes())
        };
        for (offset, value) in [
            (0, 1),
            (4, 1),
            (8, 1),
            (12, 2),
            (20, 36),
            (24, 60),
            (60, 68),
            (68, 0),
            (72, u32::MAX),
        ] {
            put(&mut data, offset, value);
        }
        data[36..38].copy_from_slice(&42u16.to_le_bytes());
        let lookup = Lookup::parse(&data).unwrap();
        assert_eq!(
            lookup.texture_set(0, 0),
            Some(TextureSet {
                number: 42,
                region: 0
            })
        );
        assert_eq!(lookup.texture_set(0, 1), None);
        assert_eq!(lookup.texture_set(1, 0), None);
        for end in 0..76 {
            assert!(Lookup::parse(&data[..end]).is_err());
        }
        put(&mut data, 68, 1);
        assert!(Lookup::parse(&data).is_err());
    }
}

//! Geographic bounds of the packed terrain and scenery cell identifiers used by BGL.

/// A Quad Mesh Identifier in the simulator's geographic grid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Qmid {
    level: u8,
    u: u32,
    v: u32,
}

/// Latitude and longitude bounds in degrees, using the WGS84 grid.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    pub north: f64,
    pub south: f64,
    pub west: f64,
    pub east: f64,
}

impl Qmid {
    pub fn level(self) -> u8 {
        self.level
    }

    pub fn u(self) -> u32 {
        self.u
    }

    pub fn v(self) -> u32 {
        self.v
    }

    /// Decode the two little-endian DWORDs stored in a BGL record.
    pub fn from_packed(words: [u32; 2]) -> Option<Self> {
        let packed = u64::from(words[0]) | (u64::from(words[1]) << 32);
        if packed == 0 {
            return None;
        }
        let marker = 63 - packed.leading_zeros();
        if marker % 2 != 1 || !(5..=59).contains(&marker) {
            return None;
        }
        let level = ((marker - 1) / 2) as u8;
        let mut u = 0;
        let mut v = 0;
        for bit in 0..level {
            u |= (((packed >> (2 * bit)) & 1) as u32) << bit;
            v |= (((packed >> (2 * bit + 1)) & 1) as u32) << bit;
        }
        let columns = 3u32 << (level - 2);
        let rows = 2u32 << (level - 2);
        (u < columns && v < rows).then_some(Self { level, u, v })
    }

    pub fn bounds(self) -> Bounds {
        let scale = f64::from(1u32 << (self.level - 2));
        let width = 120.0 / scale;
        let height = 90.0 / scale;
        Bounds {
            north: 90.0 - f64::from(self.v) * height,
            south: 90.0 - (f64::from(self.v) + 1.0) * height,
            west: -180.0 + f64::from(self.u) * width,
            east: -180.0 + (f64::from(self.u) + 1.0) * width,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_documented_cells_and_bounds() {
        let cell = Qmid::from_packed([0x207e8, 0]).unwrap();
        assert_eq!(
            cell,
            Qmid {
                level: 8,
                u: 56,
                v: 30
            }
        );
        assert_eq!(
            cell.bounds(),
            Bounds {
                north: 47.8125,
                south: 46.40625,
                west: -75.0,
                east: -73.125
            }
        );
        assert_eq!(
            Qmid::from_packed([0x81fab65, 0]),
            Some(Qmid {
                level: 13,
                u: 1819,
                v: 1012
            })
        );
        assert!(Qmid::from_packed([0, 0]).is_none());
        assert!(Qmid::from_packed([1, 0]).is_none());
    }
}

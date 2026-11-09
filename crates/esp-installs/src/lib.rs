//! esp-installs discovers installed flight simulators without modifying them.

use std::path::PathBuf;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Simulator {
    Prepar3d6,
    Prepar3d5,
    Prepar3d4,
    FsxSteam,
    Fsx,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Install {
    pub simulator: Simulator,
    pub root: PathBuf,
}

/// Discover supported installations in preferred order. Missing entries are normal.
#[cfg(windows)]
pub fn discover() -> Vec<Install> {
    use winreg::{
        RegKey,
        enums::{HKEY_LOCAL_MACHINE, KEY_READ, KEY_WOW64_32KEY, KEY_WOW64_64KEY},
    };
    let machine = RegKey::predef(HKEY_LOCAL_MACHINE);
    let candidates = [
        (
            Simulator::Prepar3d6,
            r"SOFTWARE\Lockheed Martin\Prepar3D v6",
            KEY_WOW64_64KEY,
        ),
        (
            Simulator::Prepar3d5,
            r"SOFTWARE\Lockheed Martin\Prepar3D v5",
            KEY_WOW64_64KEY,
        ),
        (
            Simulator::Prepar3d4,
            r"SOFTWARE\Lockheed Martin\Prepar3D v4",
            KEY_WOW64_64KEY,
        ),
        (
            Simulator::FsxSteam,
            r"SOFTWARE\Microsoft\Microsoft Games\Flight Simulator - Steam Edition\10.0",
            KEY_WOW64_32KEY,
        ),
        (
            Simulator::Fsx,
            r"SOFTWARE\Microsoft\Microsoft Games\Flight Simulator\10.0",
            KEY_WOW64_32KEY,
        ),
    ];
    candidates
        .into_iter()
        .filter_map(|(simulator, key, view)| {
            let value: String = machine
                .open_subkey_with_flags(key, KEY_READ | view)
                .ok()?
                .get_value("SetupPath")
                .ok()?;
            let root = PathBuf::from(value);
            root.is_dir().then_some(Install { simulator, root })
        })
        .collect()
}

#[cfg(not(windows))]
pub fn discover() -> Vec<Install> {
    Vec::new()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[ignore = "requires a registered Prepar3D v5 installation"]
    fn discovers_real_p3d_install() {
        let install = discover()
            .into_iter()
            .find(|install| install.simulator == Simulator::Prepar3d5)
            .unwrap();
        assert!(install.root.join("Scenery").is_dir());
    }
}

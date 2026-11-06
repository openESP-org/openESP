//! Session-owned aircraft state, backed by a solver motion snapshot.
use esp_physics::flight::Motion;
use std::ops::{Deref, DerefMut};

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SessionState(Motion);

impl From<Motion> for SessionState {
    fn from(snapshot: Motion) -> Self {
        Self(snapshot)
    }
}

impl Deref for SessionState {
    type Target = Motion;

    fn deref(&self) -> &Motion {
        &self.0
    }
}

impl DerefMut for SessionState {
    fn deref_mut(&mut self) -> &mut Motion {
        &mut self.0
    }
}

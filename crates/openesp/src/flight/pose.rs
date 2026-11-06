use super::*;

impl Flight {
    pub fn sample(&self, alpha: f64) -> Result<State> {
        let mut previous = self.previous;
        previous.rebase(self.loading.cg - self.previous_cg)?;
        previous.interpolate(&self.current, alpha)
    }

    pub fn pose(&self, state: &State) -> (DVec3, Quat) {
        let basis = DMat3::from_cols(DVec3::Z, DVec3::X, -DVec3::Y);
        let orientation =
            DQuat::from_mat3(&(basis * DMat3::from_quat(state.orientation) * basis.transpose()))
                .as_quat();
        let origin = basis * (state.position - state.orientation * self.loading.cg);
        (origin, orientation)
    }
}

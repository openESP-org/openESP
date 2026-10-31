use glam::{DVec3, Mat4, Vec3};

pub struct Camera {
    pub position: DVec3,
    pub yaw: f32,
    pub pitch: f32,
}

impl Camera {
    pub fn looking_at(position: DVec3, target: DVec3) -> Self {
        let direction = (target - position).normalize().as_vec3();
        Self {
            position,
            yaw: direction.x.atan2(direction.z),
            pitch: direction.y.asin(),
        }
    }

    pub fn forward(&self) -> Vec3 {
        Vec3::new(
            self.yaw.sin() * self.pitch.cos(),
            self.pitch.sin(),
            self.yaw.cos() * self.pitch.cos(),
        )
    }

    pub fn look(&mut self, x: f64, y: f64) {
        self.yaw += x as f32 * 0.003;
        self.pitch = (self.pitch - y as f32 * 0.003).clamp(-1.55, 1.55);
    }

    pub fn move_local(&mut self, movement: Vec3, distance: f64) {
        let forward = self.forward();
        let right = Vec3::Y.cross(forward).normalize_or_zero();
        let direction =
            (right * movement.x + Vec3::Y * movement.y + forward * movement.z).normalize_or_zero();
        self.position += direction.as_dvec3() * distance;
    }

    pub fn view_projection(&self, aspect: f32) -> Mat4 {
        Mat4::perspective_infinite_reverse_lh(60f32.to_radians(), aspect, 0.05)
            * Mat4::look_to_lh(Vec3::ZERO, self.forward(), Vec3::Y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn preserves_precision_and_normalizes_movement() {
        let origin = DVec3::splat(6_378_137.0);
        let mut camera = Camera::looking_at(origin, origin + DVec3::Z);
        camera.move_local(Vec3::new(1.0, 0.0, 1.0), 0.001);
        assert!(((camera.position - origin).length() - 0.001).abs() < 1e-8);
        camera.look(0.0, 1e6);
        assert!(camera.view_projection(1.0).is_finite());
    }
}

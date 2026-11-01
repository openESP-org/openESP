use glam::{DVec3, Mat4, Quat, Vec3};

pub struct Camera {
    pub position: DVec3,
    pub frame: Quat,
    pub yaw: f32,
    pub pitch: f32,
}

impl Camera {
    pub fn looking_at(position: DVec3, target: DVec3) -> Self {
        let direction = (target - position).normalize().as_vec3();
        Self {
            position,
            frame: Quat::IDENTITY,
            yaw: direction.x.atan2(direction.z),
            pitch: direction.y.asin(),
        }
    }

    pub fn forward(&self) -> Vec3 {
        self.frame
            * Vec3::new(
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
        let up = self.frame * Vec3::Y;
        let right = up.cross(forward).normalize_or_zero();
        let direction =
            (right * movement.x + up * movement.y + forward * movement.z).normalize_or_zero();
        self.position += direction.as_dvec3() * distance;
    }

    pub fn ray(&self, x: f32, y: f32, aspect: f32) -> Option<(Vec3, Vec3)> {
        if !x.is_finite() || !y.is_finite() || !aspect.is_finite() || aspect <= 0.0 {
            return None;
        }
        let point = self
            .view_projection(aspect)
            .inverse()
            .project_point3(Vec3::new(x, y, 1.0));
        Some((point, point.try_normalize()?))
    }

    pub fn view_projection(&self, aspect: f32) -> Mat4 {
        Mat4::perspective_infinite_reverse_lh(60f32.to_radians(), aspect, 0.05)
            * Mat4::look_to_lh(Vec3::ZERO, self.forward(), self.frame * Vec3::Y)
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

    #[test]
    fn rays_match_projection_after_camera_rotation() {
        for (yaw, pitch, aspect) in [(0.0, 0.0, 1.6), (0.6, -0.3, 0.8)] {
            let camera = Camera {
                position: DVec3::splat(6_378_137.0),
                frame: Quat::IDENTITY,
                yaw,
                pitch,
            };
            assert!(
                camera
                    .ray(0.0, 0.0, aspect)
                    .unwrap()
                    .1
                    .abs_diff_eq(camera.forward(), 1e-6)
            );
            for (x, y) in [(-0.8, 0.6), (0.7, -0.4)] {
                let (origin, ray) = camera.ray(x, y, aspect).unwrap();
                let near = camera.view_projection(aspect).project_point3(origin);
                assert!((near.z - 1.0).abs() < 1e-6);
                assert!((origin.dot(camera.forward()) - 0.05).abs() < 1e-6);
                let projected = camera.view_projection(aspect).project_point3(ray * 2.0);
                assert!((projected.x - x).abs() < 1e-6);
                assert!((projected.y - y).abs() < 1e-6);
            }
            assert!(camera.ray(f32::NAN, 0.0, aspect).is_none());
            assert!(camera.ray(0.0, 0.0, 0.0).is_none());
        }
    }

    #[test]
    fn banked_camera_rotates_its_view_and_movement_frame() {
        let mut camera = Camera::looking_at(DVec3::ZERO, DVec3::Z);
        camera.frame = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
        camera.move_local(Vec3::X, 1.0);
        assert!(camera.position.abs_diff_eq(DVec3::Y, 1e-6));
        let (origin, ray) = camera.ray(0.0, 0.5, 1.0).unwrap();
        assert!(origin.x < 0.0 && ray.x < 0.0);
        assert!(
            camera
                .ray(0.0, 0.0, 1.0)
                .unwrap()
                .1
                .abs_diff_eq(Vec3::Z, 1e-6)
        );
    }
}

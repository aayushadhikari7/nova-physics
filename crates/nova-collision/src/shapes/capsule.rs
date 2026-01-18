//! Capsule collision shape

use nova_math::{capsule_inertia, Aabb, Isometry, Mat3, Vec3};

use super::SupportMap;

/// A capsule collision shape (cylinder with hemispherical caps).
/// Aligned along the Y axis in local space.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CapsuleShape {
    /// Radius of the capsule
    pub radius: f32,
    /// Half-height of the cylindrical portion (total height = 2 * half_height + 2 * radius)
    pub half_height: f32,
}

impl CapsuleShape {
    /// Create a new capsule
    pub fn new(radius: f32, half_height: f32) -> Self {
        assert!(radius > 0.0, "Capsule radius must be positive");
        assert!(half_height >= 0.0, "Capsule half_height must be non-negative");
        Self { radius, half_height }
    }

    /// Create a capsule from total height and radius
    pub fn from_height(radius: f32, height: f32) -> Self {
        let half_height = (height - 2.0 * radius).max(0.0) / 2.0;
        Self::new(radius, half_height)
    }

    /// Get the total height of the capsule
    pub fn total_height(&self) -> f32 {
        2.0 * (self.half_height + self.radius)
    }

    /// Get the segment endpoints in local space
    pub fn segment(&self) -> (Vec3, Vec3) {
        (
            Vec3::new(0.0, -self.half_height, 0.0),
            Vec3::new(0.0, self.half_height, 0.0),
        )
    }

    /// Compute the AABB
    pub fn compute_aabb(&self, transform: &Isometry) -> Aabb {
        let (a, b) = self.segment();
        let world_a = transform.transform_point(a);
        let world_b = transform.transform_point(b);

        let min = world_a.min(world_b) - Vec3::splat(self.radius);
        let max = world_a.max(world_b) + Vec3::splat(self.radius);

        Aabb::new(min, max)
    }

    /// Compute the inertia tensor
    pub fn compute_inertia(&self, mass: f32) -> Mat3 {
        capsule_inertia(self.radius, self.half_height, mass)
    }

    /// Get the volume
    pub fn volume(&self) -> f32 {
        let sphere_vol = (4.0 / 3.0) * std::f32::consts::PI * self.radius.powi(3);
        let cylinder_vol = std::f32::consts::PI * self.radius * self.radius * 2.0 * self.half_height;
        sphere_vol + cylinder_vol
    }

    /// Get the closest point on the capsule axis to a point
    pub fn closest_point_on_axis(&self, point: Vec3) -> Vec3 {
        let t = (point.y / self.half_height).clamp(-1.0, 1.0);
        Vec3::new(0.0, t * self.half_height, 0.0)
    }
}

impl SupportMap for CapsuleShape {
    fn support_point(&self, transform: &Isometry, direction: Vec3) -> Vec3 {
        let local_dir = transform.inverse_transform_vector(direction);
        let local_support = self.local_support_point(local_dir);
        transform.transform_point(local_support)
    }

    fn local_support_point(&self, direction: Vec3) -> Vec3 {
        let dir_len = direction.length();
        if dir_len < nova_math::EPSILON {
            return Vec3::ZERO;
        }

        let normalized = direction / dir_len;

        // Support point is on one of the hemisphere caps
        let sign = if direction.y >= 0.0 { 1.0 } else { -1.0 };
        let center = Vec3::new(0.0, sign * self.half_height, 0.0);

        center + normalized * self.radius
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_capsule_aabb() {
        let capsule = CapsuleShape::new(0.5, 1.0);
        let transform = Isometry::IDENTITY;
        let aabb = capsule.compute_aabb(&transform);

        assert!((aabb.min - Vec3::new(-0.5, -1.5, -0.5)).length() < 1e-5);
        assert!((aabb.max - Vec3::new(0.5, 1.5, 0.5)).length() < 1e-5);
    }

    #[test]
    fn test_capsule_support() {
        let capsule = CapsuleShape::new(1.0, 2.0);
        let transform = Isometry::IDENTITY;

        let support_up = capsule.support_point(&transform, Vec3::Y);
        assert!((support_up - Vec3::new(0.0, 3.0, 0.0)).length() < 1e-5);

        let support_right = capsule.support_point(&transform, Vec3::X);
        // Should be at radius in x, somewhere on the axis in y
        assert!((support_right.x - 1.0).abs() < 1e-5);
    }
}

//! Sphere collision shape

use nova_math::{sphere_inertia, Aabb, Isometry, Mat3, Vec3};

use super::SupportMap;

/// A sphere collision shape defined by its radius
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SphereShape {
    pub radius: f32,
}

impl SphereShape {
    /// Create a new sphere with the given radius
    pub fn new(radius: f32) -> Self {
        assert!(radius > 0.0, "Sphere radius must be positive");
        Self { radius }
    }

    /// Compute the AABB of the sphere at the given transform
    pub fn compute_aabb(&self, transform: &Isometry) -> Aabb {
        let center = transform.translation;
        Aabb::from_center_half_extents(center, Vec3::splat(self.radius))
    }

    /// Compute the inertia tensor for the given mass
    pub fn compute_inertia(&self, mass: f32) -> Mat3 {
        sphere_inertia(self.radius, mass)
    }

    /// Get the volume of the sphere
    pub fn volume(&self) -> f32 {
        (4.0 / 3.0) * std::f32::consts::PI * self.radius.powi(3)
    }

    /// Check if a point is inside the sphere
    pub fn contains_point(&self, transform: &Isometry, point: Vec3) -> bool {
        let local_point = transform.inverse_transform_point(point);
        local_point.length_squared() <= self.radius * self.radius
    }

    /// Get the closest point on the sphere surface to a point
    pub fn closest_point(&self, transform: &Isometry, point: Vec3) -> Vec3 {
        let local_point = transform.inverse_transform_point(point);
        let dist = local_point.length();
        if dist < nova_math::EPSILON {
            // Point is at center, return arbitrary surface point
            transform.transform_point(Vec3::new(self.radius, 0.0, 0.0))
        } else {
            let local_closest = local_point * (self.radius / dist);
            transform.transform_point(local_closest)
        }
    }
}

impl SupportMap for SphereShape {
    fn support_point(&self, transform: &Isometry, direction: Vec3) -> Vec3 {
        let local_support = self.local_support_point(transform.inverse_transform_vector(direction));
        transform.transform_point(local_support)
    }

    fn local_support_point(&self, direction: Vec3) -> Vec3 {
        let len = direction.length();
        if len < nova_math::EPSILON {
            Vec3::ZERO
        } else {
            direction * (self.radius / len)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Quat;

    #[test]
    fn test_sphere_aabb() {
        let sphere = SphereShape::new(1.0);
        let transform = Isometry::from_translation(Vec3::new(5.0, 0.0, 0.0));
        let aabb = sphere.compute_aabb(&transform);

        assert_eq!(aabb.min, Vec3::new(4.0, -1.0, -1.0));
        assert_eq!(aabb.max, Vec3::new(6.0, 1.0, 1.0));
    }

    #[test]
    fn test_sphere_support() {
        let sphere = SphereShape::new(2.0);
        let transform = Isometry::IDENTITY;

        let support = sphere.support_point(&transform, Vec3::X);
        assert!((support - Vec3::new(2.0, 0.0, 0.0)).length() < 1e-5);
    }

    #[test]
    fn test_sphere_contains() {
        let sphere = SphereShape::new(1.0);
        let transform = Isometry::IDENTITY;

        assert!(sphere.contains_point(&transform, Vec3::ZERO));
        assert!(sphere.contains_point(&transform, Vec3::new(0.5, 0.0, 0.0)));
        assert!(!sphere.contains_point(&transform, Vec3::new(1.5, 0.0, 0.0)));
    }
}

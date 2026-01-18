//! Box collision shape

use nova_math::{box_inertia, Aabb, Isometry, Mat3, Vec3};

use super::SupportMap;

/// A box collision shape defined by its half-extents
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoxShape {
    /// Half-extents on each axis
    pub half_extents: Vec3,
}

impl BoxShape {
    /// Create a new box with the given half-extents
    pub fn new(half_extents: Vec3) -> Self {
        assert!(
            half_extents.x > 0.0 && half_extents.y > 0.0 && half_extents.z > 0.0,
            "Box half-extents must be positive"
        );
        Self { half_extents }
    }

    /// Create a cube with the given half-extent
    pub fn cube(half_extent: f32) -> Self {
        Self::new(Vec3::splat(half_extent))
    }

    /// Create a box from full extents (size)
    pub fn from_size(size: Vec3) -> Self {
        Self::new(size * 0.5)
    }

    /// Get the full size of the box
    pub fn size(&self) -> Vec3 {
        self.half_extents * 2.0
    }

    /// Get the 8 corners of the box in local space
    pub fn corners(&self) -> [Vec3; 8] {
        let h = self.half_extents;
        [
            Vec3::new(-h.x, -h.y, -h.z),
            Vec3::new(h.x, -h.y, -h.z),
            Vec3::new(h.x, h.y, -h.z),
            Vec3::new(-h.x, h.y, -h.z),
            Vec3::new(-h.x, -h.y, h.z),
            Vec3::new(h.x, -h.y, h.z),
            Vec3::new(h.x, h.y, h.z),
            Vec3::new(-h.x, h.y, h.z),
        ]
    }

    /// Compute the AABB
    pub fn compute_aabb(&self, transform: &Isometry) -> Aabb {
        // Transform all corners and find min/max
        let corners = self.corners();
        let mut aabb = Aabb::EMPTY;

        for corner in &corners {
            let world_corner = transform.transform_point(*corner);
            aabb = aabb.expand_to_include_point(world_corner);
        }

        aabb
    }

    /// Compute the inertia tensor
    pub fn compute_inertia(&self, mass: f32) -> Mat3 {
        box_inertia(self.half_extents, mass)
    }

    /// Get the volume
    pub fn volume(&self) -> f32 {
        8.0 * self.half_extents.x * self.half_extents.y * self.half_extents.z
    }

    /// Check if a point (in local space) is inside the box
    pub fn contains_local_point(&self, point: Vec3) -> bool {
        point.x.abs() <= self.half_extents.x
            && point.y.abs() <= self.half_extents.y
            && point.z.abs() <= self.half_extents.z
    }

    /// Get the closest point on the box surface to a local point
    pub fn closest_local_point(&self, point: Vec3) -> Vec3 {
        point.clamp(-self.half_extents, self.half_extents)
    }

    /// Get face normals (6 faces)
    pub fn face_normals() -> [Vec3; 6] {
        [
            Vec3::X,
            Vec3::NEG_X,
            Vec3::Y,
            Vec3::NEG_Y,
            Vec3::Z,
            Vec3::NEG_Z,
        ]
    }

    /// Get edge directions (3 unique directions)
    pub fn edge_directions() -> [Vec3; 3] {
        [Vec3::X, Vec3::Y, Vec3::Z]
    }
}

impl SupportMap for BoxShape {
    fn support_point(&self, transform: &Isometry, direction: Vec3) -> Vec3 {
        let local_dir = transform.inverse_transform_vector(direction);
        let local_support = self.local_support_point(local_dir);
        transform.transform_point(local_support)
    }

    fn local_support_point(&self, direction: Vec3) -> Vec3 {
        Vec3::new(
            if direction.x >= 0.0 {
                self.half_extents.x
            } else {
                -self.half_extents.x
            },
            if direction.y >= 0.0 {
                self.half_extents.y
            } else {
                -self.half_extents.y
            },
            if direction.z >= 0.0 {
                self.half_extents.z
            } else {
                -self.half_extents.z
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_box_aabb_identity() {
        let box_shape = BoxShape::new(Vec3::ONE);
        let transform = Isometry::IDENTITY;
        let aabb = box_shape.compute_aabb(&transform);

        assert_eq!(aabb.min, Vec3::NEG_ONE);
        assert_eq!(aabb.max, Vec3::ONE);
    }

    #[test]
    fn test_box_aabb_rotated() {
        let box_shape = BoxShape::new(Vec3::new(1.0, 0.0, 0.0) + Vec3::splat(0.001)); // Near-degenerate
        let transform = Isometry::from_rotation(glam::Quat::from_rotation_z(
            std::f32::consts::FRAC_PI_4,
        ));
        let aabb = box_shape.compute_aabb(&transform);

        // Rotated 45 degrees, the extent in both x and y should be sqrt(2)/2
        assert!(aabb.max.x > 0.7);
        assert!(aabb.max.y > 0.7);
    }

    #[test]
    fn test_box_support() {
        let box_shape = BoxShape::new(Vec3::new(2.0, 1.0, 0.5));
        let transform = Isometry::IDENTITY;

        let support = box_shape.support_point(&transform, Vec3::X);
        assert_eq!(support, Vec3::new(2.0, 1.0, 0.5));

        let support_neg = box_shape.support_point(&transform, Vec3::NEG_X);
        assert_eq!(support_neg, Vec3::new(-2.0, 1.0, 0.5));
    }
}

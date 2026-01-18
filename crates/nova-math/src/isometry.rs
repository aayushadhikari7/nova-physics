//! Isometry - Position and rotation transform

use glam::{Mat4, Quat, Vec3};

/// An isometry represents a rigid body transformation (position + rotation).
/// No scaling is applied.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Isometry {
    /// Translation component
    pub translation: Vec3,
    /// Rotation component (unit quaternion)
    pub rotation: Quat,
}

impl Default for Isometry {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Isometry {
    /// The identity isometry (no transformation)
    pub const IDENTITY: Self = Self {
        translation: Vec3::ZERO,
        rotation: Quat::IDENTITY,
    };

    /// Create a new isometry from translation and rotation
    #[inline]
    pub fn new(translation: Vec3, rotation: Quat) -> Self {
        Self {
            translation,
            rotation: rotation.normalize(),
        }
    }

    /// Create an isometry from just a translation
    #[inline]
    pub fn from_translation(translation: Vec3) -> Self {
        Self {
            translation,
            rotation: Quat::IDENTITY,
        }
    }

    /// Create an isometry from just a rotation
    #[inline]
    pub fn from_rotation(rotation: Quat) -> Self {
        Self {
            translation: Vec3::ZERO,
            rotation: rotation.normalize(),
        }
    }

    /// Transform a point by this isometry
    #[inline]
    pub fn transform_point(&self, point: Vec3) -> Vec3 {
        self.rotation * point + self.translation
    }

    /// Transform a vector (direction) by this isometry (rotation only)
    #[inline]
    pub fn transform_vector(&self, vector: Vec3) -> Vec3 {
        self.rotation * vector
    }

    /// Inverse transform a point
    #[inline]
    pub fn inverse_transform_point(&self, point: Vec3) -> Vec3 {
        self.rotation.inverse() * (point - self.translation)
    }

    /// Inverse transform a vector
    #[inline]
    pub fn inverse_transform_vector(&self, vector: Vec3) -> Vec3 {
        self.rotation.inverse() * vector
    }

    /// Compute the inverse of this isometry
    #[inline]
    pub fn inverse(&self) -> Self {
        let inv_rot = self.rotation.inverse();
        Self {
            translation: inv_rot * (-self.translation),
            rotation: inv_rot,
        }
    }

    /// Multiply two isometries (compose transformations)
    #[inline]
    pub fn mul(&self, other: &Self) -> Self {
        Self {
            translation: self.transform_point(other.translation),
            rotation: (self.rotation * other.rotation).normalize(),
        }
    }

    /// Convert to a 4x4 transformation matrix
    #[inline]
    pub fn to_mat4(&self) -> Mat4 {
        Mat4::from_rotation_translation(self.rotation, self.translation)
    }

    /// Create from a 4x4 transformation matrix (assumes no scaling)
    #[inline]
    pub fn from_mat4(mat: Mat4) -> Self {
        let (_, rotation, translation) = mat.to_scale_rotation_translation();
        Self {
            translation,
            rotation: rotation.normalize(),
        }
    }

    /// Linearly interpolate between two isometries
    #[inline]
    pub fn lerp(&self, other: &Self, t: f32) -> Self {
        Self {
            translation: self.translation.lerp(other.translation, t),
            rotation: self.rotation.slerp(other.rotation, t),
        }
    }

    /// Get the forward direction (-Z in local space)
    #[inline]
    pub fn forward(&self) -> Vec3 {
        self.rotation * Vec3::NEG_Z
    }

    /// Get the right direction (+X in local space)
    #[inline]
    pub fn right(&self) -> Vec3 {
        self.rotation * Vec3::X
    }

    /// Get the up direction (+Y in local space)
    #[inline]
    pub fn up(&self) -> Vec3 {
        self.rotation * Vec3::Y
    }
}

impl std::ops::Mul for Isometry {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            translation: self.transform_point(rhs.translation),
            rotation: (self.rotation * rhs.rotation).normalize(),
        }
    }
}

impl std::ops::Mul<Vec3> for Isometry {
    type Output = Vec3;

    #[inline]
    fn mul(self, rhs: Vec3) -> Self::Output {
        self.transform_point(rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity() {
        let iso = Isometry::IDENTITY;
        let point = Vec3::new(1.0, 2.0, 3.0);
        assert_eq!(iso.transform_point(point), point);
    }

    #[test]
    fn test_translation() {
        let iso = Isometry::from_translation(Vec3::new(1.0, 0.0, 0.0));
        let point = Vec3::ZERO;
        assert_eq!(iso.transform_point(point), Vec3::new(1.0, 0.0, 0.0));
    }

    #[test]
    fn test_inverse() {
        let iso = Isometry::new(
            Vec3::new(1.0, 2.0, 3.0),
            Quat::from_rotation_y(std::f32::consts::FRAC_PI_2),
        );
        let point = Vec3::new(5.0, 6.0, 7.0);
        let transformed = iso.transform_point(point);
        let back = iso.inverse().transform_point(transformed);
        assert!((point - back).length() < 1e-5);
    }
}

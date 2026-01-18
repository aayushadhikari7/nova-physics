//! Fixed joint - 0 DOF (rigid connection)

use nova_math::{Quat, Vec3};

/// A fixed joint that rigidly connects two bodies.
/// No relative movement is allowed.
#[derive(Debug, Clone)]
pub struct FixedJoint {
    /// Local rotation from body A to body B when joint was created
    pub local_rotation: Quat,
    /// Cached linear impulse for warm starting
    pub linear_impulse: Vec3,
    /// Cached angular impulse for warm starting
    pub angular_impulse: Vec3,
}

impl Default for FixedJoint {
    fn default() -> Self {
        Self::new()
    }
}

impl FixedJoint {
    pub fn new() -> Self {
        Self {
            local_rotation: Quat::IDENTITY,
            linear_impulse: Vec3::ZERO,
            angular_impulse: Vec3::ZERO,
        }
    }

    /// Create a fixed joint with a specific relative rotation
    pub fn with_rotation(mut self, rotation: Quat) -> Self {
        self.local_rotation = rotation;
        self
    }

    /// Calculate the rotation error
    pub fn rotation_error(&self, rotation_a: Quat, rotation_b: Quat) -> Vec3 {
        let target_rotation = rotation_a * self.local_rotation;
        let error_quat = rotation_b * target_rotation.inverse();

        // Convert quaternion to axis-angle representation
        let (axis, angle) = if error_quat.w < 0.0 {
            (
                Vec3::new(-error_quat.x, -error_quat.y, -error_quat.z),
                2.0 * (-error_quat.w).acos(),
            )
        } else {
            (
                Vec3::new(error_quat.x, error_quat.y, error_quat.z),
                2.0 * error_quat.w.acos(),
            )
        };

        let axis_len = axis.length();
        if axis_len > nova_math::EPSILON {
            axis * (angle / axis_len)
        } else {
            Vec3::ZERO
        }
    }
}

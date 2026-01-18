//! Ball joint (spherical joint) - 3 DOF rotation

use nova_math::Vec3;

use super::JointLimits;

/// A ball joint that allows free rotation around a point.
/// Also known as a spherical joint.
#[derive(Debug, Clone)]
pub struct BallJoint {
    /// Swing limit (cone angle in radians)
    pub swing_limit: JointLimits,
    /// Twist limit (rotation around the axis connecting anchors)
    pub twist_limit: JointLimits,
    /// Cached impulse for warm starting
    pub impulse: Vec3,
}

impl Default for BallJoint {
    fn default() -> Self {
        Self::new()
    }
}

impl BallJoint {
    pub fn new() -> Self {
        Self {
            swing_limit: JointLimits::default(),
            twist_limit: JointLimits::default(),
            impulse: Vec3::ZERO,
        }
    }

    /// Set swing limit (cone angle)
    pub fn with_swing_limit(mut self, max_angle: f32) -> Self {
        self.swing_limit = JointLimits::new(-max_angle, max_angle);
        self
    }

    /// Set twist limit
    pub fn with_twist_limit(mut self, min: f32, max: f32) -> Self {
        self.twist_limit = JointLimits::new(min, max);
        self
    }
}

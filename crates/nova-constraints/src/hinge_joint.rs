//! Hinge joint (revolute joint) - 1 DOF rotation

use nova_math::Vec3;

use super::{JointLimits, JointMotor};

/// A hinge joint that allows rotation around a single axis.
/// Also known as a revolute joint.
#[derive(Debug, Clone)]
pub struct HingeJoint {
    /// Local axis on body A
    pub local_axis_a: Vec3,
    /// Local axis on body B
    pub local_axis_b: Vec3,
    /// Angular limits
    pub limits: JointLimits,
    /// Motor
    pub motor: JointMotor,
    /// Reference angle (angle when joint was created)
    pub reference_angle: f32,
    /// Cached impulses for warm starting
    pub impulse: Vec3,
    pub motor_impulse: f32,
}

impl Default for HingeJoint {
    fn default() -> Self {
        Self::new(Vec3::X)
    }
}

impl HingeJoint {
    /// Create a new hinge joint with the given axis (in local space of body A)
    pub fn new(axis: Vec3) -> Self {
        let axis = axis.normalize();
        Self {
            local_axis_a: axis,
            local_axis_b: axis,
            limits: JointLimits::default(),
            motor: JointMotor::default(),
            reference_angle: 0.0,
            impulse: Vec3::ZERO,
            motor_impulse: 0.0,
        }
    }

    /// Set angular limits
    pub fn with_limits(mut self, min: f32, max: f32) -> Self {
        self.limits = JointLimits::new(min, max);
        self
    }

    /// Set motor with velocity target
    pub fn with_motor_velocity(mut self, velocity: f32, max_torque: f32) -> Self {
        self.motor = JointMotor::velocity(velocity, max_torque);
        self
    }

    /// Set motor with position target
    pub fn with_motor_position(mut self, target: f32, stiffness: f32, damping: f32) -> Self {
        self.motor = JointMotor::position(target, stiffness, damping);
        self
    }

    /// Calculate current angle between bodies
    pub fn calculate_angle(&self, rotation_a: nova_math::Quat, rotation_b: nova_math::Quat) -> f32 {
        let world_axis_a = rotation_a * self.local_axis_a;
        let world_axis_b = rotation_b * self.local_axis_b;

        // Project onto plane perpendicular to axis
        let perp_a = (rotation_a * Vec3::Y).reject_from(world_axis_a).normalize_or(Vec3::Y);
        let perp_b = (rotation_b * Vec3::Y).reject_from(world_axis_b).normalize_or(Vec3::Y);

        let dot = perp_a.dot(perp_b).clamp(-1.0, 1.0);
        let cross = perp_a.cross(perp_b);
        let sign = if cross.dot(world_axis_a) >= 0.0 { 1.0 } else { -1.0 };

        sign * dot.acos() - self.reference_angle
    }
}

trait Vec3Ext {
    fn reject_from(self, axis: Vec3) -> Vec3;
}

impl Vec3Ext for Vec3 {
    fn reject_from(self, axis: Vec3) -> Vec3 {
        self - axis * self.dot(axis)
    }
}

//! Prismatic joint (slider joint) - 1 DOF translation

use nova_math::Vec3;

use super::{JointLimits, JointMotor};

/// A prismatic joint that allows translation along a single axis.
/// Also known as a slider joint.
#[derive(Debug, Clone)]
pub struct PrismaticJoint {
    /// Local axis on body A (direction of allowed translation)
    pub local_axis_a: Vec3,
    /// Local axis on body B
    pub local_axis_b: Vec3,
    /// Translation limits
    pub limits: JointLimits,
    /// Motor
    pub motor: JointMotor,
    /// Reference translation (distance when joint was created)
    pub reference_translation: f32,
    /// Cached impulses for warm starting
    pub impulse: Vec3,
    pub motor_impulse: f32,
}

impl Default for PrismaticJoint {
    fn default() -> Self {
        Self::new(Vec3::X)
    }
}

impl PrismaticJoint {
    /// Create a new prismatic joint with the given axis
    pub fn new(axis: Vec3) -> Self {
        let axis = axis.normalize();
        Self {
            local_axis_a: axis,
            local_axis_b: axis,
            limits: JointLimits::default(),
            motor: JointMotor::default(),
            reference_translation: 0.0,
            impulse: Vec3::ZERO,
            motor_impulse: 0.0,
        }
    }

    /// Set translation limits
    pub fn with_limits(mut self, min: f32, max: f32) -> Self {
        self.limits = JointLimits::new(min, max);
        self
    }

    /// Set motor with velocity target
    pub fn with_motor_velocity(mut self, velocity: f32, max_force: f32) -> Self {
        self.motor = JointMotor::velocity(velocity, max_force);
        self
    }

    /// Set motor with position target
    pub fn with_motor_position(mut self, target: f32, stiffness: f32, damping: f32) -> Self {
        self.motor = JointMotor::position(target, stiffness, damping);
        self
    }

    /// Calculate current translation along the axis
    pub fn calculate_translation(
        &self,
        position_a: Vec3,
        rotation_a: nova_math::Quat,
        anchor_a: Vec3,
        position_b: Vec3,
        rotation_b: nova_math::Quat,
        anchor_b: Vec3,
    ) -> f32 {
        let world_anchor_a = position_a + rotation_a * anchor_a;
        let world_anchor_b = position_b + rotation_b * anchor_b;
        let world_axis = rotation_a * self.local_axis_a;

        (world_anchor_b - world_anchor_a).dot(world_axis) - self.reference_translation
    }
}

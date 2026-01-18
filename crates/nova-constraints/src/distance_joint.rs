//! Distance joint (spring) - maintains distance between anchors

use nova_math::Vec3;

use super::JointLimits;

/// A distance joint that maintains a specific distance between two anchor points.
/// Can be configured as a spring with stiffness and damping.
#[derive(Debug, Clone)]
pub struct DistanceJoint {
    /// Rest length of the joint
    pub rest_length: f32,
    /// Minimum/maximum length limits
    pub limits: JointLimits,
    /// Spring stiffness (0 = rigid constraint)
    pub stiffness: f32,
    /// Spring damping
    pub damping: f32,
    /// Cached impulse for warm starting
    pub impulse: f32,
}

impl Default for DistanceJoint {
    fn default() -> Self {
        Self::new(1.0)
    }
}

impl DistanceJoint {
    /// Create a new distance joint with the given rest length
    pub fn new(rest_length: f32) -> Self {
        Self {
            rest_length,
            limits: JointLimits::default(),
            stiffness: 0.0,
            damping: 0.0,
            impulse: 0.0,
        }
    }

    /// Configure as a spring with given stiffness and damping
    pub fn as_spring(mut self, stiffness: f32, damping: f32) -> Self {
        self.stiffness = stiffness;
        self.damping = damping;
        self
    }

    /// Set distance limits
    pub fn with_limits(mut self, min: f32, max: f32) -> Self {
        self.limits = JointLimits::new(min, max);
        self
    }

    /// Calculate current distance error
    pub fn calculate_error(&self, world_anchor_a: Vec3, world_anchor_b: Vec3) -> f32 {
        let current_length = (world_anchor_b - world_anchor_a).length();
        current_length - self.rest_length
    }

    /// Check if this is a spring (has stiffness)
    pub fn is_spring(&self) -> bool {
        self.stiffness > 0.0
    }
}

//! Common joint types and traits

use nova_core::{JointHandle, RigidBodyHandle};
use nova_math::{Isometry, Vec3};

/// Motor control model
#[derive(Debug, Clone, Copy, Default)]
pub enum MotorModel {
    /// No motor
    #[default]
    None,
    /// Position-based motor (target position)
    Position,
    /// Velocity-based motor (target velocity)
    Velocity,
}

/// Joint motor configuration
#[derive(Debug, Clone, Copy)]
pub struct JointMotor {
    /// Motor model type
    pub model: MotorModel,
    /// Target position (for position motor)
    pub target_position: f32,
    /// Target velocity (for velocity motor)
    pub target_velocity: f32,
    /// Maximum impulse the motor can apply
    pub max_impulse: f32,
    /// Motor stiffness (for position motor)
    pub stiffness: f32,
    /// Motor damping
    pub damping: f32,
}

impl Default for JointMotor {
    fn default() -> Self {
        Self {
            model: MotorModel::None,
            target_position: 0.0,
            target_velocity: 0.0,
            max_impulse: f32::MAX,
            stiffness: 0.0,
            damping: 0.0,
        }
    }
}

impl JointMotor {
    pub fn is_enabled(&self) -> bool {
        !matches!(self.model, MotorModel::None)
    }

    pub fn position(target: f32, stiffness: f32, damping: f32) -> Self {
        Self {
            model: MotorModel::Position,
            target_position: target,
            stiffness,
            damping,
            ..Default::default()
        }
    }

    pub fn velocity(target: f32, max_impulse: f32) -> Self {
        Self {
            model: MotorModel::Velocity,
            target_velocity: target,
            max_impulse,
            ..Default::default()
        }
    }
}

/// Joint limits configuration
#[derive(Debug, Clone, Copy)]
pub struct JointLimits {
    /// Is the limit enabled?
    pub enabled: bool,
    /// Minimum value
    pub min: f32,
    /// Maximum value
    pub max: f32,
    /// Stiffness of the limit (for soft limits)
    pub stiffness: f32,
    /// Damping of the limit
    pub damping: f32,
}

impl Default for JointLimits {
    fn default() -> Self {
        Self {
            enabled: false,
            min: f32::NEG_INFINITY,
            max: f32::INFINITY,
            stiffness: 0.0,
            damping: 0.0,
        }
    }
}

impl JointLimits {
    pub fn new(min: f32, max: f32) -> Self {
        Self {
            enabled: true,
            min,
            max,
            ..Default::default()
        }
    }

    pub fn with_stiffness(mut self, stiffness: f32, damping: f32) -> Self {
        self.stiffness = stiffness;
        self.damping = damping;
        self
    }

    /// Check if a value is within limits
    pub fn is_within(&self, value: f32) -> bool {
        !self.enabled || (value >= self.min && value <= self.max)
    }

    /// Clamp a value to the limits
    pub fn clamp(&self, value: f32) -> f32 {
        if self.enabled {
            value.clamp(self.min, self.max)
        } else {
            value
        }
    }
}

/// Enumeration of joint types
#[derive(Debug, Clone)]
pub enum JointData {
    Ball(super::BallJoint),
    Hinge(super::HingeJoint),
    Prismatic(super::PrismaticJoint),
    Fixed(super::FixedJoint),
    Distance(super::DistanceJoint),
}

/// A joint connecting two rigid bodies
#[derive(Debug, Clone)]
pub struct Joint {
    /// First body (anchor body)
    pub body_a: RigidBodyHandle,
    /// Second body (attached body)
    pub body_b: RigidBodyHandle,
    /// Local anchor point on body A
    pub local_anchor_a: Vec3,
    /// Local anchor point on body B
    pub local_anchor_b: Vec3,
    /// Joint-specific data
    pub data: JointData,
    /// Is this joint enabled?
    pub enabled: bool,
    /// Should connected bodies collide?
    pub collide_connected: bool,
    /// Cached impulse for warm starting
    pub cached_impulse: Vec3,
    /// User data
    pub user_data: u64,
}

impl Joint {
    pub fn new(body_a: RigidBodyHandle, body_b: RigidBodyHandle, data: JointData) -> Self {
        Self {
            body_a,
            body_b,
            local_anchor_a: Vec3::ZERO,
            local_anchor_b: Vec3::ZERO,
            data,
            enabled: true,
            collide_connected: false,
            cached_impulse: Vec3::ZERO,
            user_data: 0,
        }
    }

    pub fn with_anchors(mut self, anchor_a: Vec3, anchor_b: Vec3) -> Self {
        self.local_anchor_a = anchor_a;
        self.local_anchor_b = anchor_b;
        self
    }

    pub fn with_collide_connected(mut self, collide: bool) -> Self {
        self.collide_connected = collide;
        self
    }

    /// Get world-space anchor positions given body transforms
    pub fn world_anchors(&self, transform_a: &Isometry, transform_b: &Isometry) -> (Vec3, Vec3) {
        let world_a = transform_a.transform_point(self.local_anchor_a);
        let world_b = transform_b.transform_point(self.local_anchor_b);
        (world_a, world_b)
    }
}

//! Fluent builders for physics objects

use nova_collision::{
    Collider, CollisionGroups, CollisionShape, PhysicsMaterial,
};
use nova_core::RigidBodyHandle;
use nova_dynamics::body::{RigidBody, RigidBodyType};
use nova_math::{Isometry, Quat, Vec3};

/// Builder for creating rigid bodies
pub struct RigidBodyBuilder {
    body: RigidBody,
}

impl Default for RigidBodyBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl RigidBodyBuilder {
    pub fn new() -> Self {
        Self {
            body: RigidBody::new(),
        }
    }

    /// Create a dynamic body builder
    pub fn dynamic() -> Self {
        Self::new().body_type(RigidBodyType::Dynamic)
    }

    /// Create a kinematic body builder
    pub fn kinematic() -> Self {
        Self::new().body_type(RigidBodyType::Kinematic)
    }

    /// Create a static body builder
    pub fn fixed() -> Self {
        Self::new().body_type(RigidBodyType::Static)
    }

    /// Set position
    pub fn position(mut self, position: Vec3) -> Self {
        self.body.position = position;
        self
    }

    /// Set rotation
    pub fn rotation(mut self, rotation: Quat) -> Self {
        self.body.rotation = rotation.normalize();
        self
    }

    /// Set linear velocity
    pub fn linear_velocity(mut self, velocity: Vec3) -> Self {
        self.body.linear_velocity = velocity;
        self
    }

    /// Set angular velocity
    pub fn angular_velocity(mut self, velocity: Vec3) -> Self {
        self.body.angular_velocity = velocity;
        self
    }

    /// Set body type
    pub fn body_type(mut self, body_type: RigidBodyType) -> Self {
        self.body.set_body_type(body_type);
        self
    }

    /// Set mass
    pub fn mass(mut self, mass: f32) -> Self {
        self.body.set_mass(mass);
        self
    }

    /// Set linear damping
    pub fn linear_damping(mut self, damping: f32) -> Self {
        self.body.linear_damping = damping.max(0.0);
        self
    }

    /// Set angular damping
    pub fn angular_damping(mut self, damping: f32) -> Self {
        self.body.angular_damping = damping.max(0.0);
        self
    }

    /// Set gravity scale
    pub fn gravity_scale(mut self, scale: f32) -> Self {
        self.body.gravity_scale = scale;
        self
    }

    /// Set whether body can sleep
    pub fn can_sleep(mut self, can_sleep: bool) -> Self {
        self.body.can_sleep = can_sleep;
        self
    }

    /// Enable CCD
    pub fn ccd_enabled(mut self, enabled: bool) -> Self {
        self.body.ccd_enabled = enabled;
        self
    }

    /// Set user data
    pub fn user_data(mut self, data: u64) -> Self {
        self.body.user_data = data;
        self
    }

    /// Build the rigid body
    pub fn build(self) -> RigidBody {
        self.body
    }
}

/// Builder for creating colliders
pub struct ColliderBuilder {
    parent: RigidBodyHandle,
    shape: CollisionShape,
    local_transform: Isometry,
    material: PhysicsMaterial,
    membership: CollisionGroups,
    filter: CollisionGroups,
    is_sensor: bool,
    user_data: u64,
}

impl ColliderBuilder {
    pub fn new(parent: RigidBodyHandle, shape: CollisionShape) -> Self {
        Self {
            parent,
            shape,
            local_transform: Isometry::IDENTITY,
            material: PhysicsMaterial::default(),
            membership: CollisionGroups::DEFAULT,
            filter: CollisionGroups::ALL,
            is_sensor: false,
            user_data: 0,
        }
    }

    /// Set local position
    pub fn position(mut self, position: Vec3) -> Self {
        self.local_transform.translation = position;
        self
    }

    /// Set local rotation
    pub fn rotation(mut self, rotation: Quat) -> Self {
        self.local_transform.rotation = rotation;
        self
    }

    /// Set friction
    pub fn friction(mut self, friction: f32) -> Self {
        self.material.friction = friction.clamp(0.0, 1.0);
        self
    }

    /// Set restitution (bounciness)
    pub fn restitution(mut self, restitution: f32) -> Self {
        self.material.restitution = restitution.clamp(0.0, 1.0);
        self
    }

    /// Set collision group membership
    pub fn membership(mut self, groups: CollisionGroups) -> Self {
        self.membership = groups;
        self
    }

    /// Set collision filter
    pub fn filter(mut self, groups: CollisionGroups) -> Self {
        self.filter = groups;
        self
    }

    /// Make this a sensor (trigger)
    pub fn sensor(mut self, is_sensor: bool) -> Self {
        self.is_sensor = is_sensor;
        self
    }

    /// Set user data
    pub fn user_data(mut self, data: u64) -> Self {
        self.user_data = data;
        self
    }

    /// Build the collider
    pub fn build(self) -> Collider {
        let mut collider = Collider::new(self.parent, self.shape);
        collider.local_transform = self.local_transform;
        collider.material = self.material;
        collider.membership = self.membership;
        collider.filter = self.filter;
        collider.is_sensor = self.is_sensor;
        collider.user_data = self.user_data;
        collider
    }
}

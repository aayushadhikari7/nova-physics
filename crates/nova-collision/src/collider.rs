//! Collider definitions and properties

use bitflags::bitflags;
use nova_core::{ColliderHandle, RigidBodyHandle};
use nova_math::{Aabb, Isometry, Vec3};

use crate::shapes::CollisionShape;

bitflags! {
    /// Collision groups for filtering collisions
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct CollisionGroups: u32 {
        const GROUP_1 = 1 << 0;
        const GROUP_2 = 1 << 1;
        const GROUP_3 = 1 << 2;
        const GROUP_4 = 1 << 3;
        const GROUP_5 = 1 << 4;
        const GROUP_6 = 1 << 5;
        const GROUP_7 = 1 << 6;
        const GROUP_8 = 1 << 7;
        const GROUP_9 = 1 << 8;
        const GROUP_10 = 1 << 9;
        const GROUP_11 = 1 << 10;
        const GROUP_12 = 1 << 11;
        const GROUP_13 = 1 << 12;
        const GROUP_14 = 1 << 13;
        const GROUP_15 = 1 << 14;
        const GROUP_16 = 1 << 15;

        const ALL = u32::MAX;
        const NONE = 0;
        const DEFAULT = Self::GROUP_1.bits();
    }
}

impl Default for CollisionGroups {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Material properties for collision response
#[derive(Debug, Clone, Copy)]
pub struct PhysicsMaterial {
    /// Friction coefficient (0-1)
    pub friction: f32,
    /// Restitution/bounciness (0-1)
    pub restitution: f32,
    /// How friction is combined between two colliders
    pub friction_combine: CombineRule,
    /// How restitution is combined between two colliders
    pub restitution_combine: CombineRule,
}

impl Default for PhysicsMaterial {
    fn default() -> Self {
        Self {
            friction: 0.5,
            restitution: 0.0,
            friction_combine: CombineRule::Average,
            restitution_combine: CombineRule::Average,
        }
    }
}

impl PhysicsMaterial {
    pub fn new(friction: f32, restitution: f32) -> Self {
        Self {
            friction: friction.clamp(0.0, 1.0),
            restitution: restitution.clamp(0.0, 1.0),
            ..Default::default()
        }
    }

    /// Combine two materials into effective collision properties
    pub fn combine(&self, other: &Self) -> (f32, f32) {
        let friction = self.friction_combine.combine(self.friction, other.friction);
        let restitution = self.restitution_combine.combine(self.restitution, other.restitution);
        (friction, restitution)
    }
}

/// How to combine material properties
#[derive(Debug, Clone, Copy, Default)]
pub enum CombineRule {
    #[default]
    Average,
    Min,
    Max,
    Multiply,
}

impl CombineRule {
    pub fn combine(&self, a: f32, b: f32) -> f32 {
        match self {
            CombineRule::Average => (a + b) * 0.5,
            CombineRule::Min => a.min(b),
            CombineRule::Max => a.max(b),
            CombineRule::Multiply => a * b,
        }
    }
}

/// A collider attached to a rigid body
#[derive(Debug, Clone)]
pub struct Collider {
    /// Handle to the parent rigid body
    pub parent: RigidBodyHandle,
    /// Local transform relative to the body
    pub local_transform: Isometry,
    /// The collision shape
    pub shape: CollisionShape,
    /// Material properties
    pub material: PhysicsMaterial,
    /// Collision groups this collider belongs to
    pub membership: CollisionGroups,
    /// Collision groups this collider can collide with
    pub filter: CollisionGroups,
    /// Is this a sensor/trigger (no physical response)?
    pub is_sensor: bool,
    /// User data
    pub user_data: u64,
    /// Cached world AABB (updated each frame)
    pub(crate) cached_aabb: Aabb,
}

impl Collider {
    /// Create a new collider
    pub fn new(parent: RigidBodyHandle, shape: CollisionShape) -> Self {
        Self {
            parent,
            local_transform: Isometry::IDENTITY,
            shape,
            material: PhysicsMaterial::default(),
            membership: CollisionGroups::DEFAULT,
            filter: CollisionGroups::ALL,
            is_sensor: false,
            user_data: 0,
            cached_aabb: Aabb::EMPTY,
        }
    }

    /// Get the world-space AABB
    pub fn compute_world_aabb(&self, body_transform: &Isometry) -> Aabb {
        let world_transform = body_transform.mul(&self.local_transform);
        self.shape.compute_aabb(&world_transform)
    }

    /// Get the world transform
    pub fn world_transform(&self, body_transform: &Isometry) -> Isometry {
        body_transform.mul(&self.local_transform)
    }

    /// Check if this collider can collide with another
    pub fn can_collide_with(&self, other: &Collider) -> bool {
        // Check group filters
        self.membership.intersects(other.filter) && other.membership.intersects(self.filter)
    }

    /// Set the cached AABB (called during broad phase update)
    pub fn set_cached_aabb(&mut self, aabb: Aabb) {
        self.cached_aabb = aabb;
    }

    /// Get the cached AABB
    pub fn cached_aabb(&self) -> &Aabb {
        &self.cached_aabb
    }
}

/// Builder for creating colliders with a fluent API
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

    /// Set the local position offset
    pub fn position(mut self, position: Vec3) -> Self {
        self.local_transform.translation = position;
        self
    }

    /// Set the local rotation
    pub fn rotation(mut self, rotation: glam::Quat) -> Self {
        self.local_transform.rotation = rotation;
        self
    }

    /// Set the local transform
    pub fn local_transform(mut self, transform: Isometry) -> Self {
        self.local_transform = transform;
        self
    }

    /// Set friction coefficient
    pub fn friction(mut self, friction: f32) -> Self {
        self.material.friction = friction.clamp(0.0, 1.0);
        self
    }

    /// Set restitution (bounciness)
    pub fn restitution(mut self, restitution: f32) -> Self {
        self.material.restitution = restitution.clamp(0.0, 1.0);
        self
    }

    /// Set the physics material
    pub fn material(mut self, material: PhysicsMaterial) -> Self {
        self.material = material;
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

    /// Make this a sensor (trigger volume)
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
        Collider {
            parent: self.parent,
            local_transform: self.local_transform,
            shape: self.shape,
            material: self.material,
            membership: self.membership,
            filter: self.filter,
            is_sensor: self.is_sensor,
            user_data: self.user_data,
            cached_aabb: Aabb::EMPTY,
        }
    }
}

//! Handle types for physics entities

use slotmap::{new_key_type, Key};

new_key_type! {
    /// Handle to a rigid body in the physics world
    pub struct RigidBodyHandle;

    /// Handle to a collider in the physics world
    pub struct ColliderHandle;

    /// Handle to a joint/constraint in the physics world
    pub struct JointHandle;

    /// Handle to an island (group of connected bodies)
    pub struct IslandHandle;
}

/// Trait for handle types
pub trait Handle: Copy + Eq + std::hash::Hash {
    /// Check if this handle is valid (not null)
    fn is_valid(&self) -> bool;
}

impl Handle for RigidBodyHandle {
    fn is_valid(&self) -> bool {
        !Key::is_null(self)
    }
}

impl Handle for ColliderHandle {
    fn is_valid(&self) -> bool {
        !Key::is_null(self)
    }
}

impl Handle for JointHandle {
    fn is_valid(&self) -> bool {
        !Key::is_null(self)
    }
}

impl Handle for IslandHandle {
    fn is_valid(&self) -> bool {
        !Key::is_null(self)
    }
}

/// A pair of collider handles (for collision pairs)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ColliderPair {
    pub first: ColliderHandle,
    pub second: ColliderHandle,
}

impl ColliderPair {
    /// Create a new collider pair, ensuring consistent ordering
    pub fn new(a: ColliderHandle, b: ColliderHandle) -> Self {
        // Use KeyData for ordering since handles don't implement Ord
        let a_data = Key::data(&a);
        let b_data = Key::data(&b);

        if a_data.as_ffi() <= b_data.as_ffi() {
            Self { first: a, second: b }
        } else {
            Self { first: b, second: a }
        }
    }
}

/// A pair of rigid body handles
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BodyPair {
    pub first: RigidBodyHandle,
    pub second: RigidBodyHandle,
}

impl BodyPair {
    /// Create a new body pair, ensuring consistent ordering
    pub fn new(a: RigidBodyHandle, b: RigidBodyHandle) -> Self {
        let a_data = Key::data(&a);
        let b_data = Key::data(&b);

        if a_data.as_ffi() <= b_data.as_ffi() {
            Self { first: a, second: b }
        } else {
            Self { first: b, second: a }
        }
    }
}

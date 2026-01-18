//! Broad phase collision detection using Dynamic BVH

mod bvh;

pub use bvh::{BroadPhase, BvhNode};

use nova_core::ColliderHandle;
use slotmap::Key;

/// A potential collision pair from the broad phase
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BroadPhasePair {
    pub first: ColliderHandle,
    pub second: ColliderHandle,
}

impl BroadPhasePair {
    pub fn new(a: ColliderHandle, b: ColliderHandle) -> Self {
        // Ensure consistent ordering
        let a_data = Key::data(&a);
        let b_data = Key::data(&b);

        if a_data.as_ffi() <= b_data.as_ffi() {
            Self { first: a, second: b }
        } else {
            Self { first: b, second: a }
        }
    }
}

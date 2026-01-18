//! Arena-based storage for physics entities

use slotmap::{SecondaryMap, SlotMap};

use crate::handles::{ColliderHandle, JointHandle, RigidBodyHandle};

/// Arena for storing rigid bodies with O(1) lookup
pub type RigidBodyArena<T> = SlotMap<RigidBodyHandle, T>;

/// Arena for storing colliders with O(1) lookup
pub type ColliderArena<T> = SlotMap<ColliderHandle, T>;

/// Arena for storing joints with O(1) lookup
pub type JointArena<T> = SlotMap<JointHandle, T>;

/// Secondary map for rigid body data
pub type RigidBodySecondaryMap<T> = SecondaryMap<RigidBodyHandle, T>;

/// Secondary map for collider data
pub type ColliderSecondaryMap<T> = SecondaryMap<ColliderHandle, T>;

/// A generic component storage that can store data associated with handles
#[derive(Debug, Clone)]
pub struct ComponentStorage<H: slotmap::Key, T> {
    data: SecondaryMap<H, T>,
}

impl<H: slotmap::Key, T> Default for ComponentStorage<H, T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<H: slotmap::Key, T> ComponentStorage<H, T> {
    /// Create a new empty component storage
    pub fn new() -> Self {
        Self {
            data: SecondaryMap::new(),
        }
    }

    /// Insert a component for a handle
    pub fn insert(&mut self, handle: H, value: T) -> Option<T> {
        self.data.insert(handle, value)
    }

    /// Remove a component
    pub fn remove(&mut self, handle: H) -> Option<T> {
        self.data.remove(handle)
    }

    /// Get a component reference
    pub fn get(&self, handle: H) -> Option<&T> {
        self.data.get(handle)
    }

    /// Get a mutable component reference
    pub fn get_mut(&mut self, handle: H) -> Option<&mut T> {
        self.data.get_mut(handle)
    }

    /// Check if a component exists
    pub fn contains(&self, handle: H) -> bool {
        self.data.contains_key(handle)
    }

    /// Iterate over all components
    pub fn iter(&self) -> impl Iterator<Item = (H, &T)> {
        self.data.iter()
    }

    /// Iterate mutably over all components
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (H, &mut T)> {
        self.data.iter_mut()
    }

    /// Get the number of components
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// Clear all components
    pub fn clear(&mut self) {
        self.data.clear();
    }
}

/// Index for quick spatial lookups
#[derive(Debug, Clone)]
pub struct SpatialIndex<H> {
    /// Handles organized by some spatial criterion
    handles: Vec<H>,
}

impl<H: Copy> Default for SpatialIndex<H> {
    fn default() -> Self {
        Self::new()
    }
}

impl<H: Copy> SpatialIndex<H> {
    /// Create a new spatial index
    pub fn new() -> Self {
        Self {
            handles: Vec::new(),
        }
    }

    /// Clear the index
    pub fn clear(&mut self) {
        self.handles.clear();
    }

    /// Add a handle to the index
    pub fn push(&mut self, handle: H) {
        self.handles.push(handle);
    }

    /// Get all handles
    pub fn handles(&self) -> &[H] {
        &self.handles
    }

    /// Get the number of handles
    pub fn len(&self) -> usize {
        self.handles.len()
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.handles.is_empty()
    }
}

//! Event system for physics notifications

use crate::handles::{ColliderHandle, ColliderPair, RigidBodyHandle};
use nova_math::Vec3;
use parking_lot::Mutex;
use smallvec::SmallVec;

/// Type of collision event
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollisionEventType {
    /// Collision has just started
    Started,
    /// Collision is ongoing (objects still touching)
    Persisted,
    /// Collision has ended (objects separated)
    Ended,
}

/// A collision event between two colliders
#[derive(Debug, Clone)]
pub struct CollisionEvent {
    /// The pair of colliders involved
    pub pair: ColliderPair,
    /// Type of collision event
    pub event_type: CollisionEventType,
    /// Contact points (empty for Ended events)
    pub contacts: SmallVec<[ContactPoint; 4]>,
}

/// A single contact point
#[derive(Debug, Clone, Copy)]
pub struct ContactPoint {
    /// World-space position on first collider
    pub point_a: Vec3,
    /// World-space position on second collider
    pub point_b: Vec3,
    /// Contact normal (from A to B)
    pub normal: Vec3,
    /// Penetration depth (positive = penetrating)
    pub depth: f32,
}

/// Event indicating a body has woken up or gone to sleep
#[derive(Debug, Clone, Copy)]
pub struct SleepEvent {
    pub body: RigidBodyHandle,
    pub is_sleeping: bool,
}

/// Event indicating a body entered or left a trigger volume
#[derive(Debug, Clone, Copy)]
pub struct TriggerEvent {
    pub trigger: ColliderHandle,
    pub other: ColliderHandle,
    pub entered: bool,
}

/// Thread-safe event channel for a specific event type
#[derive(Debug)]
pub struct EventChannel<T> {
    events: Mutex<Vec<T>>,
}

impl<T> Default for EventChannel<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> EventChannel<T> {
    /// Create a new event channel
    pub fn new() -> Self {
        Self {
            events: Mutex::new(Vec::new()),
        }
    }

    /// Push an event to the channel
    pub fn push(&self, event: T) {
        self.events.lock().push(event);
    }

    /// Drain all events from the channel
    pub fn drain(&self) -> Vec<T> {
        std::mem::take(&mut *self.events.lock())
    }

    /// Check if there are any pending events
    pub fn is_empty(&self) -> bool {
        self.events.lock().is_empty()
    }

    /// Get the number of pending events
    pub fn len(&self) -> usize {
        self.events.lock().len()
    }

    /// Clear all events
    pub fn clear(&self) {
        self.events.lock().clear();
    }
}

impl<T: Clone> EventChannel<T> {
    /// Read all events without draining
    pub fn read(&self) -> Vec<T> {
        self.events.lock().clone()
    }
}

/// Aggregates all physics events
#[derive(Debug, Default)]
pub struct PhysicsEvents {
    /// Collision events
    pub collisions: EventChannel<CollisionEvent>,
    /// Sleep state change events
    pub sleep: EventChannel<SleepEvent>,
    /// Trigger events
    pub triggers: EventChannel<TriggerEvent>,
}

impl PhysicsEvents {
    /// Create a new event aggregator
    pub fn new() -> Self {
        Self::default()
    }

    /// Clear all event channels
    pub fn clear(&self) {
        self.collisions.clear();
        self.sleep.clear();
        self.triggers.clear();
    }
}

/// A callback-based event handler
pub struct EventHandler {
    collision_callbacks: Vec<Box<dyn Fn(&CollisionEvent) + Send + Sync>>,
    sleep_callbacks: Vec<Box<dyn Fn(&SleepEvent) + Send + Sync>>,
}

impl Default for EventHandler {
    fn default() -> Self {
        Self::new()
    }
}

impl EventHandler {
    pub fn new() -> Self {
        Self {
            collision_callbacks: Vec::new(),
            sleep_callbacks: Vec::new(),
        }
    }

    /// Register a collision event callback
    pub fn on_collision<F>(&mut self, callback: F)
    where
        F: Fn(&CollisionEvent) + Send + Sync + 'static,
    {
        self.collision_callbacks.push(Box::new(callback));
    }

    /// Register a sleep event callback
    pub fn on_sleep<F>(&mut self, callback: F)
    where
        F: Fn(&SleepEvent) + Send + Sync + 'static,
    {
        self.sleep_callbacks.push(Box::new(callback));
    }

    /// Process collision events
    pub fn process_collisions(&self, events: &[CollisionEvent]) {
        for event in events {
            for callback in &self.collision_callbacks {
                callback(event);
            }
        }
    }

    /// Process sleep events
    pub fn process_sleep(&self, events: &[SleepEvent]) {
        for event in events {
            for callback in &self.sleep_callbacks {
                callback(event);
            }
        }
    }
}

impl std::fmt::Debug for EventHandler {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("EventHandler")
            .field("collision_callbacks", &self.collision_callbacks.len())
            .field("sleep_callbacks", &self.sleep_callbacks.len())
            .finish()
    }
}

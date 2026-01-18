//! Sleeping system for performance optimization

use nova_core::RigidBodyHandle;
use nova_math::Vec3;

use crate::body::RigidBody;

/// Configuration for the sleeping system
#[derive(Debug, Clone)]
pub struct SleepingConfig {
    /// Linear velocity threshold below which a body can sleep
    pub linear_threshold: f32,
    /// Angular velocity threshold below which a body can sleep
    pub angular_threshold: f32,
    /// Time a body must be below threshold before sleeping
    pub time_until_sleep: f32,
}

impl Default for SleepingConfig {
    fn default() -> Self {
        Self {
            linear_threshold: 0.1,
            angular_threshold: 0.1,
            time_until_sleep: 0.5,
        }
    }
}

/// Tracks sleeping state for bodies
#[derive(Debug, Default)]
pub struct SleepingManager {
    config: SleepingConfig,
    /// Time each body has been below the sleep threshold
    sleep_timers: hashbrown::HashMap<RigidBodyHandle, f32>,
}

impl SleepingManager {
    pub fn new(config: SleepingConfig) -> Self {
        Self {
            config,
            sleep_timers: hashbrown::HashMap::new(),
        }
    }

    /// Get the sleeping configuration
    pub fn config(&self) -> &SleepingConfig {
        &self.config
    }

    /// Set the sleeping configuration
    pub fn set_config(&mut self, config: SleepingConfig) {
        self.config = config;
    }

    /// Clear all sleep timers
    pub fn clear(&mut self) {
        self.sleep_timers.clear();
    }

    /// Remove a body from tracking
    pub fn remove_body(&mut self, handle: RigidBodyHandle) {
        self.sleep_timers.remove(&handle);
    }

    /// Update sleeping state for a single body
    pub fn update_body(&mut self, handle: RigidBodyHandle, body: &mut RigidBody, dt: f32) {
        if !body.can_sleep || !body.body_type.is_dynamic() {
            return;
        }

        let lin_speed = body.linear_velocity.length();
        let ang_speed = body.angular_velocity.length();

        let below_threshold = lin_speed < self.config.linear_threshold
            && ang_speed < self.config.angular_threshold;

        if below_threshold {
            let timer = self.sleep_timers.entry(handle).or_insert(0.0);
            *timer += dt;

            if *timer >= self.config.time_until_sleep {
                body.sleep();
            }
        } else {
            self.sleep_timers.remove(&handle);
            if body.is_sleeping {
                body.wake_up();
            }
        }
    }

    /// Check if a body should be woken due to external influence
    pub fn check_wake_condition(&self, body: &RigidBody, force: Vec3, torque: Vec3) -> bool {
        if !body.is_sleeping {
            return false;
        }

        // Wake if significant force or torque is applied
        let force_threshold = self.config.linear_threshold * body.mass * 10.0;
        let torque_threshold = self.config.angular_threshold * 10.0;

        force.length() > force_threshold || torque.length() > torque_threshold
    }

    /// Reset sleep timer for a body (call when waking)
    pub fn reset_timer(&mut self, handle: RigidBodyHandle) {
        self.sleep_timers.remove(&handle);
    }

    /// Get the sleep timer for a body
    pub fn get_timer(&self, handle: RigidBodyHandle) -> f32 {
        self.sleep_timers.get(&handle).copied().unwrap_or(0.0)
    }
}

/// Wake up bodies that are in contact with a moving body
pub fn propagate_wake<F, G>(
    body: RigidBodyHandle,
    get_contacts: F,
    wake_body: G,
) where
    F: Fn(RigidBodyHandle) -> Vec<RigidBodyHandle>,
    G: Fn(RigidBodyHandle),
{
    let contacts = get_contacts(body);
    for contact_body in contacts {
        wake_body(contact_body);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use slotmap::SlotMap;

    #[test]
    fn test_sleeping_after_threshold() {
        let config = SleepingConfig {
            linear_threshold: 0.1,
            angular_threshold: 0.1,
            time_until_sleep: 0.5,
        };

        let mut manager = SleepingManager::new(config);
        let mut bodies: SlotMap<RigidBodyHandle, RigidBody> = SlotMap::with_key();

        let handle = bodies.insert(RigidBody::new());
        let body = &mut bodies[handle];
        body.linear_velocity = Vec3::ZERO;
        body.angular_velocity = Vec3::ZERO;

        // Update for less than sleep time
        for _ in 0..25 {
            manager.update_body(handle, body, 1.0 / 60.0);
        }
        assert!(!body.is_sleeping);

        // Continue updating until sleep time is reached
        for _ in 0..10 {
            manager.update_body(handle, body, 1.0 / 60.0);
        }
        assert!(body.is_sleeping);
    }

    #[test]
    fn test_wake_on_movement() {
        let config = SleepingConfig::default();
        let mut manager = SleepingManager::new(config);
        let mut bodies: SlotMap<RigidBodyHandle, RigidBody> = SlotMap::with_key();

        let handle = bodies.insert(RigidBody::new());
        let body = &mut bodies[handle];
        body.sleep();

        // Add velocity
        body.linear_velocity = Vec3::new(1.0, 0.0, 0.0);

        manager.update_body(handle, body, 1.0 / 60.0);
        assert!(!body.is_sleeping);
    }
}

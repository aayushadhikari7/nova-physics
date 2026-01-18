//! Rigid body definition and properties

use nova_core::RigidBodyHandle;
use nova_math::{
    Isometry, Mat3, Quat, Vec3, DEFAULT_ANGULAR_DAMPING, DEFAULT_LINEAR_DAMPING,
    MAX_ANGULAR_VELOCITY, MAX_LINEAR_VELOCITY,
};

/// Type of rigid body
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RigidBodyType {
    /// Fully simulated body affected by forces and collisions
    #[default]
    Dynamic,
    /// Body controlled by user, affects dynamic bodies but isn't affected by them
    Kinematic,
    /// Immovable body (infinite mass)
    Static,
}

impl RigidBodyType {
    pub fn is_dynamic(&self) -> bool {
        matches!(self, RigidBodyType::Dynamic)
    }

    pub fn is_kinematic(&self) -> bool {
        matches!(self, RigidBodyType::Kinematic)
    }

    pub fn is_static(&self) -> bool {
        matches!(self, RigidBodyType::Static)
    }

    pub fn can_move(&self) -> bool {
        !matches!(self, RigidBodyType::Static)
    }
}

/// A rigid body in the physics simulation
#[derive(Debug, Clone)]
pub struct RigidBody {
    /// World-space position
    pub position: Vec3,
    /// World-space rotation
    pub rotation: Quat,
    /// Linear velocity (m/s)
    pub linear_velocity: Vec3,
    /// Angular velocity (rad/s)
    pub angular_velocity: Vec3,
    /// Accumulated force (cleared each step)
    pub force: Vec3,
    /// Accumulated torque (cleared each step)
    pub torque: Vec3,
    /// Mass (kg)
    pub mass: f32,
    /// Inverse mass (0 for static bodies)
    pub inv_mass: f32,
    /// Local-space inertia tensor
    pub local_inertia: Mat3,
    /// World-space inverse inertia tensor
    pub inv_inertia_world: Mat3,
    /// Body type
    pub body_type: RigidBodyType,
    /// Linear damping factor
    pub linear_damping: f32,
    /// Angular damping factor
    pub angular_damping: f32,
    /// Gravity scale (1.0 = normal gravity)
    pub gravity_scale: f32,
    /// Is the body sleeping?
    pub is_sleeping: bool,
    /// User data
    pub user_data: u64,
    /// Can this body be put to sleep?
    pub can_sleep: bool,
    /// Is CCD (continuous collision detection) enabled?
    pub ccd_enabled: bool,
}

impl Default for RigidBody {
    fn default() -> Self {
        Self::new()
    }
}

impl RigidBody {
    pub fn new() -> Self {
        Self {
            position: Vec3::ZERO,
            rotation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            force: Vec3::ZERO,
            torque: Vec3::ZERO,
            mass: 1.0,
            inv_mass: 1.0,
            local_inertia: Mat3::IDENTITY,
            inv_inertia_world: Mat3::IDENTITY,
            body_type: RigidBodyType::Dynamic,
            linear_damping: DEFAULT_LINEAR_DAMPING,
            angular_damping: DEFAULT_ANGULAR_DAMPING,
            gravity_scale: 1.0,
            is_sleeping: false,
            user_data: 0,
            can_sleep: true,
            ccd_enabled: false,
        }
    }

    /// Create a static body (immovable)
    pub fn new_static() -> Self {
        let mut body = Self::new();
        body.set_body_type(RigidBodyType::Static);
        body
    }

    /// Create a kinematic body (user-controlled)
    pub fn new_kinematic() -> Self {
        let mut body = Self::new();
        body.set_body_type(RigidBodyType::Kinematic);
        body
    }

    /// Get the isometry (position + rotation) of the body
    pub fn isometry(&self) -> Isometry {
        Isometry::new(self.position, self.rotation)
    }

    /// Set the body type and update mass properties
    pub fn set_body_type(&mut self, body_type: RigidBodyType) {
        self.body_type = body_type;
        match body_type {
            RigidBodyType::Dynamic => {
                if self.mass > 0.0 {
                    self.inv_mass = 1.0 / self.mass;
                }
                self.update_world_inertia();
            }
            RigidBodyType::Kinematic | RigidBodyType::Static => {
                self.inv_mass = 0.0;
                self.inv_inertia_world = Mat3::ZERO;
            }
        }
    }

    /// Set mass and update inverse mass
    pub fn set_mass(&mut self, mass: f32) {
        self.mass = mass.max(0.0);
        if self.body_type.is_dynamic() && self.mass > 0.0 {
            self.inv_mass = 1.0 / self.mass;
        } else {
            self.inv_mass = 0.0;
        }
    }

    /// Set the local inertia tensor
    pub fn set_inertia(&mut self, inertia: Mat3) {
        self.local_inertia = inertia;
        self.update_world_inertia();
    }

    /// Update the world-space inverse inertia tensor based on current rotation
    pub fn update_world_inertia(&mut self) {
        if !self.body_type.is_dynamic() {
            self.inv_inertia_world = Mat3::ZERO;
            return;
        }

        let local_inv_inertia = nova_math::inverse_inertia(self.local_inertia);
        self.inv_inertia_world = nova_math::rotate_inertia(local_inv_inertia, self.rotation);
    }

    /// Apply a force at the center of mass
    pub fn apply_force(&mut self, force: Vec3) {
        if self.body_type.is_dynamic() {
            self.force += force;
            self.wake_up();
        }
    }

    /// Apply a force at a world-space point
    pub fn apply_force_at_point(&mut self, force: Vec3, point: Vec3) {
        if self.body_type.is_dynamic() {
            self.force += force;
            self.torque += (point - self.position).cross(force);
            self.wake_up();
        }
    }

    /// Apply an impulse at the center of mass
    pub fn apply_impulse(&mut self, impulse: Vec3) {
        if self.body_type.is_dynamic() {
            self.linear_velocity += impulse * self.inv_mass;
            self.wake_up();
        }
    }

    /// Apply an impulse at a world-space point
    pub fn apply_impulse_at_point(&mut self, impulse: Vec3, point: Vec3) {
        if self.body_type.is_dynamic() {
            self.linear_velocity += impulse * self.inv_mass;
            self.angular_velocity += self.inv_inertia_world * (point - self.position).cross(impulse);
            self.wake_up();
        }
    }

    /// Apply a torque
    pub fn apply_torque(&mut self, torque: Vec3) {
        if self.body_type.is_dynamic() {
            self.torque += torque;
            self.wake_up();
        }
    }

    /// Apply an angular impulse
    pub fn apply_angular_impulse(&mut self, impulse: Vec3) {
        if self.body_type.is_dynamic() {
            self.angular_velocity += self.inv_inertia_world * impulse;
            self.wake_up();
        }
    }

    /// Clear accumulated forces and torques
    pub fn clear_forces(&mut self) {
        self.force = Vec3::ZERO;
        self.torque = Vec3::ZERO;
    }

    /// Clamp velocities to maximum values
    pub fn clamp_velocities(&mut self) {
        let lin_speed = self.linear_velocity.length();
        if lin_speed > MAX_LINEAR_VELOCITY {
            self.linear_velocity *= MAX_LINEAR_VELOCITY / lin_speed;
        }

        let ang_speed = self.angular_velocity.length();
        if ang_speed > MAX_ANGULAR_VELOCITY {
            self.angular_velocity *= MAX_ANGULAR_VELOCITY / ang_speed;
        }
    }

    /// Get kinetic energy
    pub fn kinetic_energy(&self) -> f32 {
        let linear = 0.5 * self.mass * self.linear_velocity.length_squared();
        let angular = 0.5 * self.angular_velocity.dot(self.local_inertia * self.angular_velocity);
        linear + angular
    }

    /// Get the velocity at a world-space point
    pub fn velocity_at_point(&self, point: Vec3) -> Vec3 {
        self.linear_velocity + self.angular_velocity.cross(point - self.position)
    }

    /// Wake up the body
    pub fn wake_up(&mut self) {
        self.is_sleeping = false;
    }

    /// Put the body to sleep
    pub fn sleep(&mut self) {
        if self.can_sleep {
            self.is_sleeping = true;
            self.linear_velocity = Vec3::ZERO;
            self.angular_velocity = Vec3::ZERO;
        }
    }

    /// Check if the body is awake
    pub fn is_awake(&self) -> bool {
        !self.is_sleeping
    }
}

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

    pub fn dynamic() -> Self {
        Self::new().body_type(RigidBodyType::Dynamic)
    }

    pub fn kinematic() -> Self {
        Self::new().body_type(RigidBodyType::Kinematic)
    }

    pub fn fixed() -> Self {
        Self::new().body_type(RigidBodyType::Static)
    }

    pub fn position(mut self, position: Vec3) -> Self {
        self.body.position = position;
        self
    }

    pub fn rotation(mut self, rotation: Quat) -> Self {
        self.body.rotation = rotation.normalize();
        self
    }

    pub fn linear_velocity(mut self, velocity: Vec3) -> Self {
        self.body.linear_velocity = velocity;
        self
    }

    pub fn angular_velocity(mut self, velocity: Vec3) -> Self {
        self.body.angular_velocity = velocity;
        self
    }

    pub fn body_type(mut self, body_type: RigidBodyType) -> Self {
        self.body.set_body_type(body_type);
        self
    }

    pub fn mass(mut self, mass: f32) -> Self {
        self.body.set_mass(mass);
        self
    }

    pub fn linear_damping(mut self, damping: f32) -> Self {
        self.body.linear_damping = damping.max(0.0);
        self
    }

    pub fn angular_damping(mut self, damping: f32) -> Self {
        self.body.angular_damping = damping.max(0.0);
        self
    }

    pub fn gravity_scale(mut self, scale: f32) -> Self {
        self.body.gravity_scale = scale;
        self
    }

    pub fn can_sleep(mut self, can_sleep: bool) -> Self {
        self.body.can_sleep = can_sleep;
        self
    }

    pub fn ccd_enabled(mut self, enabled: bool) -> Self {
        self.body.ccd_enabled = enabled;
        self
    }

    pub fn user_data(mut self, data: u64) -> Self {
        self.body.user_data = data;
        self
    }

    pub fn build(self) -> RigidBody {
        self.body
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rigid_body_default() {
        let body = RigidBody::new();
        assert!(body.body_type.is_dynamic());
        assert!((body.inv_mass - 1.0).abs() < 1e-5);
    }

    #[test]
    fn test_rigid_body_static() {
        let body = RigidBody::new_static();
        assert!(body.body_type.is_static());
        assert_eq!(body.inv_mass, 0.0);
    }

    #[test]
    fn test_apply_impulse() {
        let mut body = RigidBody::new();
        body.set_mass(2.0);
        body.apply_impulse(Vec3::X);

        assert!((body.linear_velocity.x - 0.5).abs() < 1e-5);
    }

    #[test]
    fn test_builder() {
        let body = RigidBodyBuilder::dynamic()
            .position(Vec3::new(1.0, 2.0, 3.0))
            .mass(5.0)
            .build();

        assert_eq!(body.position, Vec3::new(1.0, 2.0, 3.0));
        assert!((body.mass - 5.0).abs() < 1e-5);
        assert!((body.inv_mass - 0.2).abs() < 1e-5);
    }
}

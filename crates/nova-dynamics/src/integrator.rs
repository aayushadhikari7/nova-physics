//! Physics integrators

use nova_math::{Quat, Vec3};

use crate::body::RigidBody;

/// Type of integrator
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum IntegratorType {
    /// Semi-implicit Euler (default, stable)
    #[default]
    SemiImplicitEuler,
    /// Velocity Verlet (higher accuracy)
    VelocityVerlet,
}

/// Physics integrator for rigid body simulation
#[derive(Debug, Clone, Default)]
pub struct Integrator {
    pub integrator_type: IntegratorType,
}

impl Integrator {
    pub fn new(integrator_type: IntegratorType) -> Self {
        Self { integrator_type }
    }

    /// Integrate velocities (apply forces)
    pub fn integrate_velocities(&self, body: &mut RigidBody, gravity: Vec3, dt: f32) {
        if !body.body_type.is_dynamic() || body.is_sleeping {
            return;
        }

        match self.integrator_type {
            IntegratorType::SemiImplicitEuler => {
                integrate_velocities_semi_implicit(body, gravity, dt);
            }
            IntegratorType::VelocityVerlet => {
                integrate_velocities_verlet(body, gravity, dt);
            }
        }
    }

    /// Integrate positions (apply velocities)
    pub fn integrate_positions(&self, body: &mut RigidBody, dt: f32) {
        if !body.body_type.can_move() || body.is_sleeping {
            return;
        }

        match self.integrator_type {
            IntegratorType::SemiImplicitEuler | IntegratorType::VelocityVerlet => {
                integrate_positions_euler(body, dt);
            }
        }

        // Update world-space inertia tensor after rotation change
        body.update_world_inertia();
    }
}

/// Semi-implicit Euler velocity integration
fn integrate_velocities_semi_implicit(body: &mut RigidBody, gravity: Vec3, dt: f32) {
    // Apply gravity
    let gravity_force = gravity * body.gravity_scale * body.mass;
    body.force += gravity_force;

    // Linear velocity: v += (F/m) * dt
    body.linear_velocity += body.force * body.inv_mass * dt;

    // Angular velocity: ω += I⁻¹ * τ * dt
    body.angular_velocity += body.inv_inertia_world * body.torque * dt;

    // Apply damping
    body.linear_velocity *= 1.0 / (1.0 + body.linear_damping * dt);
    body.angular_velocity *= 1.0 / (1.0 + body.angular_damping * dt);

    // Clamp velocities
    body.clamp_velocities();

    // Clear forces
    body.clear_forces();
}

/// Velocity Verlet integration (velocity step)
fn integrate_velocities_verlet(body: &mut RigidBody, gravity: Vec3, dt: f32) {
    // Apply gravity
    let gravity_force = gravity * body.gravity_scale * body.mass;
    body.force += gravity_force;

    // Verlet: v += 0.5 * (a_old + a_new) * dt
    // Simplified: v += a * dt (same as semi-implicit for constant force)
    body.linear_velocity += body.force * body.inv_mass * dt;
    body.angular_velocity += body.inv_inertia_world * body.torque * dt;

    // Apply damping
    body.linear_velocity *= 1.0 / (1.0 + body.linear_damping * dt);
    body.angular_velocity *= 1.0 / (1.0 + body.angular_damping * dt);

    // Clamp velocities
    body.clamp_velocities();

    // Clear forces
    body.clear_forces();
}

/// Position integration (same for both integrators)
fn integrate_positions_euler(body: &mut RigidBody, dt: f32) {
    // Linear: x += v * dt
    body.position += body.linear_velocity * dt;

    // Angular: q += 0.5 * ω * q * dt
    let angular_vel_quat = Quat::from_xyzw(
        body.angular_velocity.x,
        body.angular_velocity.y,
        body.angular_velocity.z,
        0.0,
    );
    let spin = angular_vel_quat * body.rotation * 0.5;
    body.rotation = Quat::from_xyzw(
        body.rotation.x + spin.x * dt,
        body.rotation.y + spin.y * dt,
        body.rotation.z + spin.z * dt,
        body.rotation.w + spin.w * dt,
    )
    .normalize();
}

/// Apply an impulse to a body at a point
pub fn apply_impulse_at_point(
    body: &mut RigidBody,
    impulse: Vec3,
    point_relative_to_center: Vec3,
) {
    if !body.body_type.is_dynamic() {
        return;
    }

    body.linear_velocity += impulse * body.inv_mass;
    body.angular_velocity += body.inv_inertia_world * point_relative_to_center.cross(impulse);
}

/// Apply a position correction to a body
pub fn apply_position_correction(
    body: &mut RigidBody,
    correction: Vec3,
    point_relative_to_center: Vec3,
    inv_mass: f32,
    inv_inertia: nova_math::Mat3,
) {
    if !body.body_type.is_dynamic() {
        return;
    }

    body.position += correction * inv_mass;

    // Angular correction
    let angular_correction = inv_inertia * point_relative_to_center.cross(correction);
    let angle = angular_correction.length();
    if angle > nova_math::EPSILON {
        let axis = angular_correction / angle;
        let delta_q = Quat::from_axis_angle(axis, angle);
        body.rotation = (delta_q * body.rotation).normalize();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gravity_integration() {
        let mut body = RigidBody::new();
        body.set_mass(1.0);

        let integrator = Integrator::new(IntegratorType::SemiImplicitEuler);
        let gravity = Vec3::new(0.0, -9.81, 0.0);
        let dt = 1.0 / 60.0;

        integrator.integrate_velocities(&mut body, gravity, dt);

        // After one frame, velocity should be approximately gravity * dt
        assert!((body.linear_velocity.y - (-9.81 * dt)).abs() < 0.01);
    }

    #[test]
    fn test_position_integration() {
        let mut body = RigidBody::new();
        body.linear_velocity = Vec3::new(1.0, 0.0, 0.0);

        let integrator = Integrator::new(IntegratorType::SemiImplicitEuler);
        let dt = 1.0;

        integrator.integrate_positions(&mut body, dt);

        // After 1 second at 1 m/s, position should be 1 meter
        assert!((body.position.x - 1.0).abs() < 0.01);
    }
}

//! Joint constraint solver

use nova_core::RigidBodyHandle;
use nova_dynamics::body::RigidBody;
use nova_math::{Mat3, Vec3, EPSILON};
use slotmap::SlotMap;

use super::{Joint, JointData, MotorModel};

/// Configuration for joint solver
#[derive(Debug, Clone)]
pub struct JointSolverConfig {
    /// Number of solver iterations
    pub iterations: u32,
    /// Warm starting factor
    pub warm_start_factor: f32,
}

impl Default for JointSolverConfig {
    fn default() -> Self {
        Self {
            iterations: 12,  // Increased for more stable joints
            warm_start_factor: 0.9,  // Higher warm start for better convergence
        }
    }
}

/// Joint constraint solver
#[derive(Debug, Default)]
pub struct JointSolver {
    config: JointSolverConfig,
}

impl JointSolver {
    pub fn new(config: JointSolverConfig) -> Self {
        Self { config }
    }

    /// Solve joint constraints
    pub fn solve<F, G>(
        &self,
        joints: &mut [&mut Joint],
        mut get_body: F,
        mut apply_impulse: G,
        dt: f32,
    ) where
        F: FnMut(RigidBodyHandle) -> Option<RigidBody>,
        G: FnMut(RigidBodyHandle, Vec3, Vec3, Vec3), // body, linear_impulse, angular_impulse, r
    {
        for _ in 0..self.config.iterations {
            for joint in joints.iter_mut() {
                if !joint.enabled {
                    continue;
                }

                let Some(body_a) = get_body(joint.body_a) else {
                    continue;
                };
                let Some(body_b) = get_body(joint.body_b) else {
                    continue;
                };

                // Extract anchor and body info before mutable borrow
                let body_a_handle = joint.body_a;
                let body_b_handle = joint.body_b;
                let local_anchor_a = joint.local_anchor_a;
                let local_anchor_b = joint.local_anchor_b;

                match &mut joint.data {
                    JointData::Ball(_ball) => {
                        solve_ball_joint(
                            body_a_handle, body_b_handle,
                            local_anchor_a, local_anchor_b,
                            &body_a, &body_b, &mut apply_impulse,
                        );
                    }
                    JointData::Hinge(hinge) => {
                        solve_hinge_joint(
                            body_a_handle, body_b_handle,
                            local_anchor_a, local_anchor_b,
                            hinge, &body_a, &body_b, &mut apply_impulse, dt,
                        );
                    }
                    JointData::Prismatic(prismatic) => {
                        solve_prismatic_joint(
                            body_a_handle, body_b_handle,
                            local_anchor_a, local_anchor_b,
                            prismatic, &body_a, &body_b, &mut apply_impulse, dt,
                        );
                    }
                    JointData::Fixed(fixed) => {
                        solve_fixed_joint(
                            body_a_handle, body_b_handle,
                            local_anchor_a, local_anchor_b,
                            fixed, &body_a, &body_b, &mut apply_impulse,
                        );
                    }
                    JointData::Distance(distance) => {
                        solve_distance_joint(
                            body_a_handle, body_b_handle,
                            local_anchor_a, local_anchor_b,
                            distance, &body_a, &body_b, &mut apply_impulse, dt,
                        );
                    }
                }
            }
        }
    }

    /// Solve joint constraints with direct body access
    pub fn solve_direct(
        &self,
        joints: &mut SlotMap<nova_core::JointHandle, Joint>,
        bodies: &mut SlotMap<RigidBodyHandle, RigidBody>,
        dt: f32,
    ) {
        for _ in 0..self.config.iterations {
            for (_, joint) in joints.iter_mut() {
                if !joint.enabled {
                    continue;
                }

                let Some(body_a) = bodies.get(joint.body_a) else { continue; };
                let Some(body_b) = bodies.get(joint.body_b) else { continue; };

                // Copy body data for calculation
                let body_a_data: RigidBody = body_a.clone();
                let body_b_data: RigidBody = body_b.clone();

                let body_a_handle = joint.body_a;
                let body_b_handle = joint.body_b;
                let local_anchor_a = joint.local_anchor_a;
                let local_anchor_b = joint.local_anchor_b;

                // Calculate impulses based on joint type
                let impulses = match &mut joint.data {
                    JointData::Ball(_ball) => {
                        calc_ball_joint_impulses(
                            local_anchor_a, local_anchor_b,
                            &body_a_data, &body_b_data,
                        )
                    }
                    JointData::Hinge(hinge) => {
                        calc_hinge_joint_impulses(
                            local_anchor_a, local_anchor_b,
                            hinge, &body_a_data, &body_b_data, dt,
                        )
                    }
                    JointData::Prismatic(prismatic) => {
                        calc_prismatic_joint_impulses(
                            local_anchor_a, local_anchor_b,
                            prismatic, &body_a_data, &body_b_data,
                        )
                    }
                    JointData::Fixed(fixed) => {
                        calc_fixed_joint_impulses(
                            local_anchor_a, local_anchor_b,
                            fixed, &body_a_data, &body_b_data,
                        )
                    }
                    JointData::Distance(distance) => {
                        calc_distance_joint_impulses(
                            local_anchor_a, local_anchor_b,
                            distance, &body_a_data, &body_b_data, dt,
                        )
                    }
                };

                // Apply impulses
                for (lin, ang, r, is_a) in impulses {
                    let handle = if is_a { body_a_handle } else { body_b_handle };
                    if let Some(body) = bodies.get_mut(handle) {
                        if body.body_type.is_dynamic() {
                            body.linear_velocity += lin * body.inv_mass;
                            body.angular_velocity += body.inv_inertia_world * (r.cross(lin) + ang);
                        }
                    }
                }
            }
        }
    }
}

// Helper to calculate ball joint impulses
fn calc_ball_joint_impulses(
    local_anchor_a: Vec3,
    local_anchor_b: Vec3,
    body_a: &RigidBody,
    body_b: &RigidBody,
) -> Vec<(Vec3, Vec3, Vec3, bool)> {
    let world_anchor_a = body_a.position + body_a.rotation * local_anchor_a;
    let world_anchor_b = body_b.position + body_b.rotation * local_anchor_b;

    let r_a = world_anchor_a - body_a.position;
    let r_b = world_anchor_b - body_b.position;

    let error = world_anchor_b - world_anchor_a;
    let k = compute_point_mass(body_a, body_b, r_a, r_b);
    let k_inv = k.inverse();
    let impulse = k_inv * (-error);

    // A gets -impulse (move towards B), B gets +impulse (move towards A)
    vec![
        (-impulse, Vec3::ZERO, r_a, true),
        (impulse, Vec3::ZERO, r_b, false),
    ]
}

// Helper to calculate hinge joint impulses
fn calc_hinge_joint_impulses(
    local_anchor_a: Vec3,
    local_anchor_b: Vec3,
    hinge: &mut super::HingeJoint,
    body_a: &RigidBody,
    body_b: &RigidBody,
    _dt: f32,
) -> Vec<(Vec3, Vec3, Vec3, bool)> {
    let mut impulses = Vec::new();

    let world_anchor_a = body_a.position + body_a.rotation * local_anchor_a;
    let world_anchor_b = body_b.position + body_b.rotation * local_anchor_b;

    let r_a = world_anchor_a - body_a.position;
    let r_b = world_anchor_b - body_b.position;

    let error = world_anchor_b - world_anchor_a;
    let k = compute_point_mass(body_a, body_b, r_a, r_b);

    if let Some(k_inv) = try_inverse_mat3(k) {
        let impulse = k_inv * (-error);
        // A gets -impulse, B gets +impulse
        impulses.push((-impulse, Vec3::ZERO, r_a, true));
        impulses.push((impulse, Vec3::ZERO, r_b, false));
    }

    let world_axis_a = body_a.rotation * hinge.local_axis_a;
    let world_axis_b = body_b.rotation * hinge.local_axis_b;

    let axis_error = world_axis_a.cross(world_axis_b);
    if axis_error.length_squared() > EPSILON {
        let k_angular = body_a.inv_inertia_world + body_b.inv_inertia_world;
        if let Some(k_inv) = try_inverse_mat3(k_angular) {
            let angular_impulse = k_inv * (-axis_error);
            // A gets -angular_impulse, B gets +angular_impulse
            impulses.push((Vec3::ZERO, -angular_impulse, Vec3::ZERO, true));
            impulses.push((Vec3::ZERO, angular_impulse, Vec3::ZERO, false));
        }
    }

    if hinge.motor.is_enabled() {
        let current_angle = hinge.calculate_angle(body_a.rotation, body_b.rotation);
        let rel_angular_vel = (body_b.angular_velocity - body_a.angular_velocity).dot(world_axis_a);

        let target_vel = match hinge.motor.model {
            MotorModel::Velocity => hinge.motor.target_velocity,
            MotorModel::Position => {
                let err = hinge.motor.target_position - current_angle;
                err * hinge.motor.stiffness - rel_angular_vel * hinge.motor.damping
            }
            MotorModel::None => 0.0,
        };

        let effective_mass = 1.0 / (world_axis_a.dot(body_a.inv_inertia_world * world_axis_a)
            + world_axis_a.dot(body_b.inv_inertia_world * world_axis_a));

        let motor_impulse = ((target_vel - rel_angular_vel) * effective_mass)
            .clamp(-hinge.motor.max_impulse, hinge.motor.max_impulse);

        let angular_impulse = world_axis_a * motor_impulse;
        // A gets -angular_impulse, B gets +angular_impulse
        impulses.push((Vec3::ZERO, -angular_impulse, Vec3::ZERO, true));
        impulses.push((Vec3::ZERO, angular_impulse, Vec3::ZERO, false));
    }

    impulses
}

// Helper to calculate prismatic joint impulses
fn calc_prismatic_joint_impulses(
    local_anchor_a: Vec3,
    local_anchor_b: Vec3,
    prismatic: &mut super::PrismaticJoint,
    body_a: &RigidBody,
    body_b: &RigidBody,
) -> Vec<(Vec3, Vec3, Vec3, bool)> {
    let mut impulses = Vec::new();

    let world_axis = body_a.rotation * prismatic.local_axis_a;
    let world_anchor_a = body_a.position + body_a.rotation * local_anchor_a;
    let world_anchor_b = body_b.position + body_b.rotation * local_anchor_b;

    let diff = world_anchor_b - world_anchor_a;
    let perp_error = diff - world_axis * diff.dot(world_axis);

    let r_a = world_anchor_a - body_a.position;
    let r_b = world_anchor_b - body_b.position;

    if perp_error.length_squared() > EPSILON {
        let k = compute_point_mass(body_a, body_b, r_a, r_b);
        if let Some(k_inv) = try_inverse_mat3(k) {
            let impulse = k_inv * (-perp_error);
            // A gets -impulse, B gets +impulse
            impulses.push((-impulse, Vec3::ZERO, r_a, true));
            impulses.push((impulse, Vec3::ZERO, r_b, false));
        }
    }

    let axis_error = (body_a.rotation * prismatic.local_axis_a)
        .cross(body_b.rotation * prismatic.local_axis_b);
    if axis_error.length_squared() > EPSILON {
        let k_angular = body_a.inv_inertia_world + body_b.inv_inertia_world;
        if let Some(k_inv) = try_inverse_mat3(k_angular) {
            let angular_impulse = k_inv * (-axis_error);
            // A gets -angular_impulse, B gets +angular_impulse
            impulses.push((Vec3::ZERO, -angular_impulse, Vec3::ZERO, true));
            impulses.push((Vec3::ZERO, angular_impulse, Vec3::ZERO, false));
        }
    }

    impulses
}

// Helper to calculate fixed joint impulses
fn calc_fixed_joint_impulses(
    local_anchor_a: Vec3,
    local_anchor_b: Vec3,
    fixed: &mut super::FixedJoint,
    body_a: &RigidBody,
    body_b: &RigidBody,
) -> Vec<(Vec3, Vec3, Vec3, bool)> {
    let mut impulses = Vec::new();

    let world_anchor_a = body_a.position + body_a.rotation * local_anchor_a;
    let world_anchor_b = body_b.position + body_b.rotation * local_anchor_b;

    let r_a = world_anchor_a - body_a.position;
    let r_b = world_anchor_b - body_b.position;

    let error = world_anchor_b - world_anchor_a;
    let k = compute_point_mass(body_a, body_b, r_a, r_b);

    if let Some(k_inv) = try_inverse_mat3(k) {
        let impulse = k_inv * (-error);
        // A gets -impulse, B gets +impulse
        impulses.push((-impulse, Vec3::ZERO, r_a, true));
        impulses.push((impulse, Vec3::ZERO, r_b, false));
    }

    let rotation_error = fixed.rotation_error(body_a.rotation, body_b.rotation);
    if rotation_error.length_squared() > EPSILON {
        let k_angular = body_a.inv_inertia_world + body_b.inv_inertia_world;
        if let Some(k_inv) = try_inverse_mat3(k_angular) {
            let angular_impulse = k_inv * (-rotation_error);
            // A gets -angular_impulse, B gets +angular_impulse
            impulses.push((Vec3::ZERO, -angular_impulse, Vec3::ZERO, true));
            impulses.push((Vec3::ZERO, angular_impulse, Vec3::ZERO, false));
        }
    }

    impulses
}

// Helper to calculate distance joint impulses
fn calc_distance_joint_impulses(
    local_anchor_a: Vec3,
    local_anchor_b: Vec3,
    distance: &mut super::DistanceJoint,
    body_a: &RigidBody,
    body_b: &RigidBody,
    dt: f32,
) -> Vec<(Vec3, Vec3, Vec3, bool)> {
    let mut impulses = Vec::new();

    let world_anchor_a = body_a.position + body_a.rotation * local_anchor_a;
    let world_anchor_b = body_b.position + body_b.rotation * local_anchor_b;

    let r_a = world_anchor_a - body_a.position;
    let r_b = world_anchor_b - body_b.position;

    let diff = world_anchor_b - world_anchor_a;
    let current_length = diff.length();

    if current_length < EPSILON {
        return impulses;
    }

    let direction = diff / current_length;
    let error = current_length - distance.rest_length;

    let rn_a = r_a.cross(direction);
    let rn_b = r_b.cross(direction);

    let k = body_a.inv_mass
        + body_b.inv_mass
        + rn_a.dot(body_a.inv_inertia_world * rn_a)
        + rn_b.dot(body_b.inv_inertia_world * rn_b);

    if k < EPSILON {
        return impulses;
    }

    let effective_mass = 1.0 / k;

    let impulse_magnitude = if distance.is_spring() {
        let rel_vel = (body_b.linear_velocity + body_b.angular_velocity.cross(r_b))
            - (body_a.linear_velocity + body_a.angular_velocity.cross(r_a));
        let vel_along = rel_vel.dot(direction);
        let spring_force = -distance.stiffness * error - distance.damping * vel_along;
        spring_force * dt
    } else {
        -error * effective_mass
    };

    let impulse = direction * impulse_magnitude;

    // A gets -impulse, B gets +impulse
    impulses.push((-impulse, Vec3::ZERO, r_a, true));
    impulses.push((impulse, Vec3::ZERO, r_b, false));

    impulses
}

fn solve_ball_joint<F>(
    body_a_handle: RigidBodyHandle,
    body_b_handle: RigidBodyHandle,
    local_anchor_a: Vec3,
    local_anchor_b: Vec3,
    body_a: &RigidBody,
    body_b: &RigidBody,
    apply_impulse: &mut F,
) where
    F: FnMut(RigidBodyHandle, Vec3, Vec3, Vec3),
{
    let world_anchor_a = body_a.position + body_a.rotation * local_anchor_a;
    let world_anchor_b = body_b.position + body_b.rotation * local_anchor_b;

    let r_a = world_anchor_a - body_a.position;
    let r_b = world_anchor_b - body_b.position;

    // Position error
    let error = world_anchor_b - world_anchor_a;

    // Compute effective mass
    let k = compute_point_mass(body_a, body_b, r_a, r_b);
    let k_inv = k.inverse();

    // Compute impulse
    let impulse = k_inv * (-error);

    // A gets -impulse (move towards B), B gets +impulse (move towards A)
    apply_impulse(body_a_handle, -impulse, Vec3::ZERO, r_a);
    apply_impulse(body_b_handle, impulse, Vec3::ZERO, r_b);
}

fn solve_hinge_joint<F>(
    body_a_handle: RigidBodyHandle,
    body_b_handle: RigidBodyHandle,
    local_anchor_a: Vec3,
    local_anchor_b: Vec3,
    hinge: &mut super::HingeJoint,
    body_a: &RigidBody,
    body_b: &RigidBody,
    apply_impulse: &mut F,
    _dt: f32,
) where
    F: FnMut(RigidBodyHandle, Vec3, Vec3, Vec3),
{
    // Solve point constraint (like ball joint)
    let world_anchor_a = body_a.position + body_a.rotation * local_anchor_a;
    let world_anchor_b = body_b.position + body_b.rotation * local_anchor_b;

    let r_a = world_anchor_a - body_a.position;
    let r_b = world_anchor_b - body_b.position;

    let error = world_anchor_b - world_anchor_a;
    let k = compute_point_mass(body_a, body_b, r_a, r_b);

    if let Some(k_inv) = try_inverse_mat3(k) {
        let impulse = k_inv * (-error);
        // A gets -impulse, B gets +impulse
        apply_impulse(body_a_handle, -impulse, Vec3::ZERO, r_a);
        apply_impulse(body_b_handle, impulse, Vec3::ZERO, r_b);
    }

    // Solve angular constraint (align axes)
    let world_axis_a = body_a.rotation * hinge.local_axis_a;
    let world_axis_b = body_b.rotation * hinge.local_axis_b;

    let axis_error = world_axis_a.cross(world_axis_b);
    if axis_error.length_squared() > EPSILON {
        let k_angular = body_a.inv_inertia_world + body_b.inv_inertia_world;
        if let Some(k_inv) = try_inverse_mat3(k_angular) {
            let angular_impulse = k_inv * (-axis_error);
            // A gets -angular_impulse, B gets +angular_impulse
            apply_impulse(body_a_handle, Vec3::ZERO, -angular_impulse, Vec3::ZERO);
            apply_impulse(body_b_handle, Vec3::ZERO, angular_impulse, Vec3::ZERO);
        }
    }

    // Motor
    if hinge.motor.is_enabled() {
        let current_angle = hinge.calculate_angle(body_a.rotation, body_b.rotation);
        let rel_angular_vel = (body_b.angular_velocity - body_a.angular_velocity).dot(world_axis_a);

        let target_vel = match hinge.motor.model {
            MotorModel::Velocity => hinge.motor.target_velocity,
            MotorModel::Position => {
                let error = hinge.motor.target_position - current_angle;
                error * hinge.motor.stiffness - rel_angular_vel * hinge.motor.damping
            }
            MotorModel::None => 0.0,
        };

        let effective_mass = 1.0 / (world_axis_a.dot(body_a.inv_inertia_world * world_axis_a)
            + world_axis_a.dot(body_b.inv_inertia_world * world_axis_a));

        let motor_impulse = ((target_vel - rel_angular_vel) * effective_mass)
            .clamp(-hinge.motor.max_impulse, hinge.motor.max_impulse);

        let angular_impulse = world_axis_a * motor_impulse;
        // A gets -angular_impulse, B gets +angular_impulse
        apply_impulse(body_a_handle, Vec3::ZERO, -angular_impulse, Vec3::ZERO);
        apply_impulse(body_b_handle, Vec3::ZERO, angular_impulse, Vec3::ZERO);
    }
}

fn solve_prismatic_joint<F>(
    body_a_handle: RigidBodyHandle,
    body_b_handle: RigidBodyHandle,
    local_anchor_a: Vec3,
    local_anchor_b: Vec3,
    prismatic: &mut super::PrismaticJoint,
    body_a: &RigidBody,
    body_b: &RigidBody,
    apply_impulse: &mut F,
    _dt: f32,
) where
    F: FnMut(RigidBodyHandle, Vec3, Vec3, Vec3),
{
    let world_axis = body_a.rotation * prismatic.local_axis_a;

    // Constrain perpendicular movement
    let world_anchor_a = body_a.position + body_a.rotation * local_anchor_a;
    let world_anchor_b = body_b.position + body_b.rotation * local_anchor_b;

    let diff = world_anchor_b - world_anchor_a;
    let perp_error = diff - world_axis * diff.dot(world_axis);

    let r_a = world_anchor_a - body_a.position;
    let r_b = world_anchor_b - body_b.position;

    if perp_error.length_squared() > EPSILON {
        let k = compute_point_mass(body_a, body_b, r_a, r_b);
        if let Some(k_inv) = try_inverse_mat3(k) {
            let impulse = k_inv * (-perp_error);
            // A gets -impulse, B gets +impulse
            apply_impulse(body_a_handle, -impulse, Vec3::ZERO, r_a);
            apply_impulse(body_b_handle, impulse, Vec3::ZERO, r_b);
        }
    }

    // Constrain rotation
    let axis_error = (body_a.rotation * prismatic.local_axis_a)
        .cross(body_b.rotation * prismatic.local_axis_b);
    if axis_error.length_squared() > EPSILON {
        let k_angular = body_a.inv_inertia_world + body_b.inv_inertia_world;
        if let Some(k_inv) = try_inverse_mat3(k_angular) {
            let angular_impulse = k_inv * (-axis_error);
            // A gets -angular_impulse, B gets +angular_impulse
            apply_impulse(body_a_handle, Vec3::ZERO, -angular_impulse, Vec3::ZERO);
            apply_impulse(body_b_handle, Vec3::ZERO, angular_impulse, Vec3::ZERO);
        }
    }
}

fn solve_fixed_joint<F>(
    body_a_handle: RigidBodyHandle,
    body_b_handle: RigidBodyHandle,
    local_anchor_a: Vec3,
    local_anchor_b: Vec3,
    fixed: &mut super::FixedJoint,
    body_a: &RigidBody,
    body_b: &RigidBody,
    apply_impulse: &mut F,
) where
    F: FnMut(RigidBodyHandle, Vec3, Vec3, Vec3),
{
    // Solve point constraint
    let world_anchor_a = body_a.position + body_a.rotation * local_anchor_a;
    let world_anchor_b = body_b.position + body_b.rotation * local_anchor_b;

    let r_a = world_anchor_a - body_a.position;
    let r_b = world_anchor_b - body_b.position;

    let error = world_anchor_b - world_anchor_a;
    let k = compute_point_mass(body_a, body_b, r_a, r_b);

    if let Some(k_inv) = try_inverse_mat3(k) {
        let impulse = k_inv * (-error);
        // A gets -impulse, B gets +impulse
        apply_impulse(body_a_handle, -impulse, Vec3::ZERO, r_a);
        apply_impulse(body_b_handle, impulse, Vec3::ZERO, r_b);
    }

    // Solve rotation constraint
    let rotation_error = fixed.rotation_error(body_a.rotation, body_b.rotation);
    if rotation_error.length_squared() > EPSILON {
        let k_angular = body_a.inv_inertia_world + body_b.inv_inertia_world;
        if let Some(k_inv) = try_inverse_mat3(k_angular) {
            let angular_impulse = k_inv * (-rotation_error);
            // A gets -angular_impulse, B gets +angular_impulse
            apply_impulse(body_a_handle, Vec3::ZERO, -angular_impulse, Vec3::ZERO);
            apply_impulse(body_b_handle, Vec3::ZERO, angular_impulse, Vec3::ZERO);
        }
    }
}

fn solve_distance_joint<F>(
    body_a_handle: RigidBodyHandle,
    body_b_handle: RigidBodyHandle,
    local_anchor_a: Vec3,
    local_anchor_b: Vec3,
    distance: &mut super::DistanceJoint,
    body_a: &RigidBody,
    body_b: &RigidBody,
    apply_impulse: &mut F,
    dt: f32,
) where
    F: FnMut(RigidBodyHandle, Vec3, Vec3, Vec3),
{
    let world_anchor_a = body_a.position + body_a.rotation * local_anchor_a;
    let world_anchor_b = body_b.position + body_b.rotation * local_anchor_b;

    let r_a = world_anchor_a - body_a.position;
    let r_b = world_anchor_b - body_b.position;

    let diff = world_anchor_b - world_anchor_a;
    let current_length = diff.length();

    if current_length < EPSILON {
        return;
    }

    let direction = diff / current_length;
    let error = current_length - distance.rest_length;

    // Compute effective mass along direction
    let rn_a = r_a.cross(direction);
    let rn_b = r_b.cross(direction);

    let k = body_a.inv_mass
        + body_b.inv_mass
        + rn_a.dot(body_a.inv_inertia_world * rn_a)
        + rn_b.dot(body_b.inv_inertia_world * rn_b);

    if k < EPSILON {
        return;
    }

    let effective_mass = 1.0 / k;

    let impulse_magnitude = if distance.is_spring() {
        // Spring behavior
        let rel_vel = (body_b.linear_velocity + body_b.angular_velocity.cross(r_b))
            - (body_a.linear_velocity + body_a.angular_velocity.cross(r_a));
        let vel_along = rel_vel.dot(direction);

        let spring_force = -distance.stiffness * error - distance.damping * vel_along;
        spring_force * dt
    } else {
        // Rigid constraint
        -error * effective_mass
    };

    let impulse = direction * impulse_magnitude;

    // A gets -impulse, B gets +impulse
    apply_impulse(body_a_handle, -impulse, Vec3::ZERO, r_a);
    apply_impulse(body_b_handle, impulse, Vec3::ZERO, r_b);
}

fn compute_point_mass(body_a: &RigidBody, body_b: &RigidBody, r_a: Vec3, r_b: Vec3) -> Mat3 {
    let mut k = Mat3::from_diagonal(Vec3::splat(body_a.inv_mass + body_b.inv_mass));

    // Add angular contribution from body A
    let r_a_skew = skew_symmetric(r_a);
    k -= r_a_skew * body_a.inv_inertia_world * r_a_skew;

    // Add angular contribution from body B
    let r_b_skew = skew_symmetric(r_b);
    k -= r_b_skew * body_b.inv_inertia_world * r_b_skew;

    k
}

fn skew_symmetric(v: Vec3) -> Mat3 {
    Mat3::from_cols(
        Vec3::new(0.0, v.z, -v.y),
        Vec3::new(-v.z, 0.0, v.x),
        Vec3::new(v.y, -v.x, 0.0),
    )
}

fn try_inverse_mat3(m: Mat3) -> Option<Mat3> {
    let det = m.determinant();
    if det.abs() > EPSILON {
        Some(m.inverse())
    } else {
        None
    }
}

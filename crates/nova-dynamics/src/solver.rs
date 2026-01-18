//! Contact constraint solver using sequential impulses

use nova_collision::narrow_phase::{ContactManifold, ContactPoint};
use nova_core::RigidBodyHandle;
use nova_math::{orthonormal_basis, Vec3, EPSILON};
use smallvec::SmallVec;

use crate::body::RigidBody;

/// Solver configuration
#[derive(Debug, Clone)]
pub struct SolverConfig {
    /// Number of velocity solver iterations
    pub velocity_iterations: u32,
    /// Number of position correction iterations
    pub position_iterations: u32,
    /// Baumgarte stabilization factor (0-1)
    pub baumgarte: f32,
    /// Allowed penetration before position correction kicks in
    pub slop: f32,
    /// Maximum position correction per step
    pub max_correction: f32,
    /// Warm starting factor (0-1)
    pub warm_start_factor: f32,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            velocity_iterations: 20,
            position_iterations: 10,
            baumgarte: 0.4,
            slop: 0.001,
            max_correction: 0.3,
            warm_start_factor: 0.9,
        }
    }
}

/// Preprocessed contact constraint for solving
#[derive(Debug, Clone)]
pub struct ContactConstraint {
    pub body_a: RigidBodyHandle,
    pub body_b: RigidBodyHandle,
    pub normal: Vec3,
    pub tangent1: Vec3,
    pub tangent2: Vec3,
    pub friction: f32,
    pub restitution: f32,
    pub points: SmallVec<[ContactConstraintPoint; 4]>,
}

#[derive(Debug, Clone, Copy)]
pub struct ContactConstraintPoint {
    pub local_a: Vec3,
    pub local_b: Vec3,
    pub r_a: Vec3, // Vector from body A center to contact point
    pub r_b: Vec3, // Vector from body B center to contact point
    pub depth: f32,
    pub normal_mass: f32,
    pub tangent_mass_1: f32,
    pub tangent_mass_2: f32,
    pub normal_impulse: f32,
    pub tangent_impulse_1: f32,
    pub tangent_impulse_2: f32,
    pub velocity_bias: f32,
}

/// The contact solver
#[derive(Debug)]
pub struct ContactSolver {
    config: SolverConfig,
    constraints: Vec<ContactConstraint>,
}

impl Default for ContactSolver {
    fn default() -> Self {
        Self::new(SolverConfig::default())
    }
}

impl ContactSolver {
    pub fn new(config: SolverConfig) -> Self {
        Self {
            config,
            constraints: Vec::new(),
        }
    }

    /// Get solver configuration
    pub fn config(&self) -> &SolverConfig {
        &self.config
    }

    /// Set solver configuration
    pub fn set_config(&mut self, config: SolverConfig) {
        self.config = config;
    }

    /// Clear all constraints
    pub fn clear(&mut self) {
        self.constraints.clear();
    }

    /// Prepare constraints from contact manifolds
    pub fn prepare_constraints<F>(
        &mut self,
        manifolds: impl Iterator<Item = ContactManifold>,
        get_body: F,
        dt: f32,
    ) where
        F: Fn(RigidBodyHandle) -> Option<RigidBody>,
    {
        self.constraints.clear();

        for manifold in manifolds {
            let Some(body_a) = get_body(manifold.body_a) else {
                continue;
            };
            let Some(body_b) = get_body(manifold.body_b) else {
                continue;
            };

            let normal = manifold.normal;
            let (tangent1, tangent2) = orthonormal_basis(normal);

            let mut constraint = ContactConstraint {
                body_a: manifold.body_a,
                body_b: manifold.body_b,
                normal,
                tangent1,
                tangent2,
                friction: manifold.friction,
                restitution: manifold.restitution,
                points: SmallVec::new(),
            };

            for contact in &manifold.contacts {
                let point = prepare_contact_point(
                    contact,
                    &body_a,
                    &body_b,
                    &normal,
                    &tangent1,
                    &tangent2,
                    manifold.restitution,
                    dt,
                    &self.config,
                );
                constraint.points.push(point);
            }

            self.constraints.push(constraint);
        }
    }

    /// Apply warm starting impulses
    pub fn warm_start<F>(&self, mut apply_impulse: F)
    where
        F: FnMut(RigidBodyHandle, Vec3, Vec3), // body, impulse, point relative to center
    {
        let factor = self.config.warm_start_factor;

        for constraint in &self.constraints {
            for point in &constraint.points {
                // Normal impulse
                // Normal points from A to B, so A gets pushed opposite to normal, B along normal
                let normal_impulse = constraint.normal * point.normal_impulse * factor;
                apply_impulse(constraint.body_a, -normal_impulse, point.r_a);
                apply_impulse(constraint.body_b, normal_impulse, point.r_b);

                // Tangent impulses
                let tangent_impulse = constraint.tangent1 * point.tangent_impulse_1 * factor
                    + constraint.tangent2 * point.tangent_impulse_2 * factor;
                apply_impulse(constraint.body_a, -tangent_impulse, point.r_a);
                apply_impulse(constraint.body_b, tangent_impulse, point.r_b);
            }
        }
    }

    /// Solve velocity constraints with direct body access
    pub fn solve_velocities_direct(
        &mut self,
        bodies: &mut slotmap::SlotMap<RigidBodyHandle, RigidBody>,
    ) {
        for _ in 0..self.config.velocity_iterations {
            for constraint in &mut self.constraints {
                let Some(body_a) = bodies.get(constraint.body_a) else { continue; };
                let Some(body_b) = bodies.get(constraint.body_b) else { continue; };

                let vel_a_lin = body_a.linear_velocity;
                let vel_a_ang = body_a.angular_velocity;
                let vel_b_lin = body_b.linear_velocity;
                let vel_b_ang = body_b.angular_velocity;
                let inv_mass_a = body_a.inv_mass;
                let inv_mass_b = body_b.inv_mass;
                let inv_inertia_a = body_a.inv_inertia_world;
                let inv_inertia_b = body_b.inv_inertia_world;
                let body_a_dynamic = body_a.body_type.is_dynamic();
                let body_b_dynamic = body_b.body_type.is_dynamic();

                for point in &mut constraint.points {
                    // Compute relative velocity at contact point
                    let vel_a = vel_a_lin + vel_a_ang.cross(point.r_a);
                    let vel_b = vel_b_lin + vel_b_ang.cross(point.r_b);
                    let rel_vel = vel_b - vel_a;

                    // Normal impulse
                    let vn = rel_vel.dot(constraint.normal);
                    let mut lambda_n = point.normal_mass * (-vn + point.velocity_bias);
                    let old_impulse = point.normal_impulse;
                    point.normal_impulse = (old_impulse + lambda_n).max(0.0);
                    lambda_n = point.normal_impulse - old_impulse;

                    let impulse_n = constraint.normal * lambda_n;

                    // Apply normal impulse
                    // Normal points from A to B, so A gets pushed opposite to normal, B along normal
                    if body_a_dynamic {
                        if let Some(body) = bodies.get_mut(constraint.body_a) {
                            body.linear_velocity -= impulse_n * inv_mass_a;
                            body.angular_velocity -= inv_inertia_a * point.r_a.cross(impulse_n);
                        }
                    }
                    if body_b_dynamic {
                        if let Some(body) = bodies.get_mut(constraint.body_b) {
                            body.linear_velocity += impulse_n * inv_mass_b;
                            body.angular_velocity += inv_inertia_b * point.r_b.cross(impulse_n);
                        }
                    }

                    // Refresh relative velocity
                    let vel_a_lin = bodies.get(constraint.body_a).map_or(Vec3::ZERO, |b| b.linear_velocity);
                    let vel_a_ang = bodies.get(constraint.body_a).map_or(Vec3::ZERO, |b| b.angular_velocity);
                    let vel_b_lin = bodies.get(constraint.body_b).map_or(Vec3::ZERO, |b| b.linear_velocity);
                    let vel_b_ang = bodies.get(constraint.body_b).map_or(Vec3::ZERO, |b| b.angular_velocity);
                    let vel_a = vel_a_lin + vel_a_ang.cross(point.r_a);
                    let vel_b = vel_b_lin + vel_b_ang.cross(point.r_b);
                    let rel_vel = vel_b - vel_a;

                    // Friction impulse (tangent 1)
                    let vt1 = rel_vel.dot(constraint.tangent1);
                    let mut lambda_t1 = point.tangent_mass_1 * (-vt1);

                    let max_friction = constraint.friction * point.normal_impulse;
                    let old_tangent_1 = point.tangent_impulse_1;
                    point.tangent_impulse_1 =
                        (old_tangent_1 + lambda_t1).clamp(-max_friction, max_friction);
                    lambda_t1 = point.tangent_impulse_1 - old_tangent_1;

                    // Friction impulse (tangent 2)
                    let vt2 = rel_vel.dot(constraint.tangent2);
                    let mut lambda_t2 = point.tangent_mass_2 * (-vt2);

                    let old_tangent_2 = point.tangent_impulse_2;
                    point.tangent_impulse_2 =
                        (old_tangent_2 + lambda_t2).clamp(-max_friction, max_friction);
                    lambda_t2 = point.tangent_impulse_2 - old_tangent_2;

                    let impulse_t =
                        constraint.tangent1 * lambda_t1 + constraint.tangent2 * lambda_t2;

                    // Apply tangent impulse (friction)
                    // Same convention as normal impulse
                    if body_a_dynamic {
                        if let Some(body) = bodies.get_mut(constraint.body_a) {
                            body.linear_velocity -= impulse_t * inv_mass_a;
                            body.angular_velocity -= inv_inertia_a * point.r_a.cross(impulse_t);
                        }
                    }
                    if body_b_dynamic {
                        if let Some(body) = bodies.get_mut(constraint.body_b) {
                            body.linear_velocity += impulse_t * inv_mass_b;
                            body.angular_velocity += inv_inertia_b * point.r_b.cross(impulse_t);
                        }
                    }
                }
            }
        }
    }

    /// Solve position constraints (pseudo-velocity)
    pub fn solve_positions<F, G>(
        &self,
        mut get_position: F,
        mut apply_correction: G,
    ) -> bool
    where
        F: FnMut(RigidBodyHandle) -> (Vec3, nova_math::Quat), // Returns (position, rotation)
        G: FnMut(RigidBodyHandle, Vec3, Vec3),                // body, correction, r
    {
        let mut solved = true;

        for _ in 0..self.config.position_iterations {
            for constraint in &self.constraints {
                let (pos_a, rot_a) = get_position(constraint.body_a);
                let (pos_b, rot_b) = get_position(constraint.body_b);

                for point in &constraint.points {
                    // Recompute world positions
                    let world_a = pos_a + rot_a * point.local_a;
                    let world_b = pos_b + rot_b * point.local_b;

                    // Compute separation
                    let separation = (world_b - world_a).dot(constraint.normal) - 0.0;

                    if separation < -self.config.slop {
                        solved = false;

                        // Compute correction
                        let correction = ((-separation - self.config.slop) * self.config.baumgarte)
                            .min(self.config.max_correction);

                        let r_a = world_a - pos_a;
                        let r_b = world_b - pos_b;

                        let impulse = constraint.normal * correction * point.normal_mass;
                        // Normal points from A to B, so A is pushed opposite to normal, B along normal
                        apply_correction(constraint.body_a, -impulse, r_a);
                        apply_correction(constraint.body_b, impulse, r_b);
                    }
                }
            }
        }

        solved
    }

    /// Solve position constraints with direct body access
    pub fn solve_positions_direct(
        &self,
        bodies: &mut slotmap::SlotMap<RigidBodyHandle, RigidBody>,
    ) -> bool {
        let mut solved = true;

        for _ in 0..self.config.position_iterations {
            for constraint in &self.constraints {
                let Some(body_a) = bodies.get(constraint.body_a) else { continue; };
                let Some(body_b) = bodies.get(constraint.body_b) else { continue; };

                let pos_a = body_a.position;
                let rot_a = body_a.rotation;
                let pos_b = body_b.position;
                let rot_b = body_b.rotation;
                let inv_mass_a = body_a.inv_mass;
                let inv_mass_b = body_b.inv_mass;
                let inv_inertia_a = body_a.inv_inertia_world;
                let inv_inertia_b = body_b.inv_inertia_world;
                let body_a_dynamic = body_a.body_type.is_dynamic();
                let body_b_dynamic = body_b.body_type.is_dynamic();

                for point in &constraint.points {
                    // Recompute world positions
                    let world_a = pos_a + rot_a * point.local_a;
                    let world_b = pos_b + rot_b * point.local_b;

                    // Compute separation
                    let separation = (world_b - world_a).dot(constraint.normal);

                    if separation < -self.config.slop {
                        solved = false;

                        // Compute correction
                        let correction = ((-separation - self.config.slop) * self.config.baumgarte)
                            .min(self.config.max_correction);

                        let r_a = world_a - pos_a;
                        let r_b = world_b - pos_b;

                        let impulse = constraint.normal * correction * point.normal_mass;

                        // Apply position correction
                        // Normal points from A to B, so A is pushed opposite to normal, B along normal
                        if body_a_dynamic {
                            if let Some(body) = bodies.get_mut(constraint.body_a) {
                                body.position -= impulse * inv_mass_a;
                                let ang = inv_inertia_a * r_a.cross(-impulse);
                                let angle = ang.length();
                                if angle > EPSILON {
                                    let axis = ang / angle;
                                    body.rotation = (nova_math::Quat::from_axis_angle(axis, angle * 0.2)
                                        * body.rotation)
                                        .normalize();
                                }
                            }
                        }
                        if body_b_dynamic {
                            if let Some(body) = bodies.get_mut(constraint.body_b) {
                                body.position += impulse * inv_mass_b;
                                let ang = inv_inertia_b * r_b.cross(impulse);
                                let angle = ang.length();
                                if angle > EPSILON {
                                    let axis = ang / angle;
                                    body.rotation = (nova_math::Quat::from_axis_angle(axis, angle * 0.2)
                                        * body.rotation)
                                        .normalize();
                                }
                            }
                        }
                    }
                }
            }
        }

        solved
    }

    /// Get cached impulses for warm starting
    pub fn get_cached_impulses(&self) -> Vec<(RigidBodyHandle, RigidBodyHandle, Vec<(f32, f32, f32)>)> {
        self.constraints
            .iter()
            .map(|c| {
                let impulses: Vec<_> = c
                    .points
                    .iter()
                    .map(|p| (p.normal_impulse, p.tangent_impulse_1, p.tangent_impulse_2))
                    .collect();
                (c.body_a, c.body_b, impulses)
            })
            .collect()
    }
}

fn prepare_contact_point(
    contact: &ContactPoint,
    body_a: &RigidBody,
    body_b: &RigidBody,
    normal: &Vec3,
    tangent1: &Vec3,
    tangent2: &Vec3,
    restitution: f32,
    dt: f32,
    config: &SolverConfig,
) -> ContactConstraintPoint {
    let r_a = contact.world_a - body_a.position;
    let r_b = contact.world_b - body_b.position;

    // Compute effective mass for normal
    let rn_a = r_a.cross(*normal);
    let rn_b = r_b.cross(*normal);

    let k_normal = body_a.inv_mass
        + body_b.inv_mass
        + rn_a.dot(body_a.inv_inertia_world * rn_a)
        + rn_b.dot(body_b.inv_inertia_world * rn_b);

    let normal_mass = if k_normal > EPSILON {
        1.0 / k_normal
    } else {
        0.0
    };

    // Compute effective mass for tangent 1
    let rt1_a = r_a.cross(*tangent1);
    let rt1_b = r_b.cross(*tangent1);

    let k_tangent1 = body_a.inv_mass
        + body_b.inv_mass
        + rt1_a.dot(body_a.inv_inertia_world * rt1_a)
        + rt1_b.dot(body_b.inv_inertia_world * rt1_b);

    let tangent_mass_1 = if k_tangent1 > EPSILON {
        1.0 / k_tangent1
    } else {
        0.0
    };

    // Compute effective mass for tangent 2
    let rt2_a = r_a.cross(*tangent2);
    let rt2_b = r_b.cross(*tangent2);

    let k_tangent2 = body_a.inv_mass
        + body_b.inv_mass
        + rt2_a.dot(body_a.inv_inertia_world * rt2_a)
        + rt2_b.dot(body_b.inv_inertia_world * rt2_b);

    let tangent_mass_2 = if k_tangent2 > EPSILON {
        1.0 / k_tangent2
    } else {
        0.0
    };

    // Compute velocity bias for restitution
    let vel_a = body_a.linear_velocity + body_a.angular_velocity.cross(r_a);
    let vel_b = body_b.linear_velocity + body_b.angular_velocity.cross(r_b);
    let rel_vel = vel_b - vel_a;
    let vn = rel_vel.dot(*normal);

    let velocity_bias = if vn < -1.0 {
        -restitution * vn
    } else {
        0.0
    };

    ContactConstraintPoint {
        local_a: contact.local_a,
        local_b: contact.local_b,
        r_a,
        r_b,
        depth: contact.depth,
        normal_mass,
        tangent_mass_1,
        tangent_mass_2,
        normal_impulse: contact.normal_impulse,
        tangent_impulse_1: contact.tangent_impulse[0],
        tangent_impulse_2: contact.tangent_impulse[1],
        velocity_bias,
    }
}

//! Physics pipeline orchestration

use nova_collision::broad_phase::BroadPhase;
use nova_collision::narrow_phase::NarrowPhase;
use nova_collision::Collider;
use nova_constraints::{Joint, JointSolver};
use nova_core::{ColliderHandle, JointHandle, PhysicsEvents, RigidBodyHandle};
use nova_dynamics::body::RigidBody;
use nova_dynamics::integrator::{apply_impulse_at_point, Integrator, IntegratorType};
use nova_dynamics::island::IslandManager;
use nova_dynamics::sleeping::{SleepingConfig, SleepingManager};
use nova_dynamics::solver::{ContactSolver, SolverConfig};
use nova_math::Vec3;
use slotmap::SlotMap;

use super::SimulationStep;

/// Pipeline configuration
#[derive(Debug, Clone)]
pub struct PipelineConfig {
    /// Gravity vector
    pub gravity: Vec3,
    /// Number of substeps per step
    pub substeps: u32,
    /// Contact solver configuration
    pub solver_config: SolverConfig,
    /// Sleeping configuration
    pub sleeping_config: SleepingConfig,
    /// Integrator type
    pub integrator_type: IntegratorType,
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.81, 0.0),
            substeps: 2,
            solver_config: SolverConfig::default(),
            sleeping_config: SleepingConfig::default(),
            integrator_type: IntegratorType::SemiImplicitEuler,
        }
    }
}

/// The main physics simulation pipeline
pub struct PhysicsPipeline {
    config: PipelineConfig,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    contact_solver: ContactSolver,
    joint_solver: JointSolver,
    island_manager: IslandManager,
    sleeping_manager: SleepingManager,
    integrator: Integrator,
    step_number: u64,
    time: f64,
}

impl Default for PhysicsPipeline {
    fn default() -> Self {
        Self::new(PipelineConfig::default())
    }
}

impl PhysicsPipeline {
    pub fn new(config: PipelineConfig) -> Self {
        Self {
            contact_solver: ContactSolver::new(config.solver_config.clone()),
            joint_solver: JointSolver::default(),
            sleeping_manager: SleepingManager::new(config.sleeping_config.clone()),
            integrator: Integrator::new(config.integrator_type),
            config,
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            island_manager: IslandManager::new(),
            step_number: 0,
            time: 0.0,
        }
    }

    /// Get the pipeline configuration
    pub fn config(&self) -> &PipelineConfig {
        &self.config
    }

    /// Set gravity
    pub fn set_gravity(&mut self, gravity: Vec3) {
        self.config.gravity = gravity;
    }

    /// Get the broad phase
    pub fn broad_phase(&self) -> &BroadPhase {
        &self.broad_phase
    }

    /// Get the narrow phase
    pub fn narrow_phase(&self) -> &NarrowPhase {
        &self.narrow_phase
    }

    /// Get the island manager
    pub fn island_manager(&self) -> &IslandManager {
        &self.island_manager
    }

    /// Run one simulation step
    pub fn step(
        &mut self,
        dt: f32,
        bodies: &mut SlotMap<RigidBodyHandle, RigidBody>,
        colliders: &mut SlotMap<ColliderHandle, Collider>,
        joints: &mut SlotMap<JointHandle, Joint>,
        events: &PhysicsEvents,
    ) -> SimulationStep {
        let substep_dt = dt / self.config.substeps as f32;

        for _ in 0..self.config.substeps {
            self.substep(substep_dt, bodies, colliders, joints, events);
        }

        self.step_number += 1;
        self.time += dt as f64;

        SimulationStep::new(self.step_number, dt, self.time, self.config.substeps)
    }

    fn substep(
        &mut self,
        dt: f32,
        bodies: &mut SlotMap<RigidBodyHandle, RigidBody>,
        colliders: &mut SlotMap<ColliderHandle, Collider>,
        joints: &mut SlotMap<JointHandle, Joint>,
        events: &PhysicsEvents,
    ) {
        // 1. Integrate velocities (apply forces, gravity)
        for (_, body) in bodies.iter_mut() {
            self.integrator
                .integrate_velocities(body, self.config.gravity, dt);
        }

        // 2. Update broad phase
        for (handle, collider) in colliders.iter_mut() {
            if let Some(body) = bodies.get(collider.parent) {
                let aabb = collider.compute_world_aabb(&body.isometry());
                collider.set_cached_aabb(aabb);
                self.broad_phase.update(handle, aabb);
            }
        }

        // 3. Find potential collision pairs
        let pairs = self.broad_phase.compute_pairs();

        // 4. Narrow phase collision detection
        self.island_manager.clear();
        self.narrow_phase.begin_frame();

        for pair in &pairs {
            let Some(collider_a) = colliders.get(pair.first) else {
                continue;
            };
            let Some(collider_b) = colliders.get(pair.second) else {
                continue;
            };

            if !collider_a.can_collide_with(collider_b) {
                continue;
            }

            let Some(body_a) = bodies.get(collider_a.parent) else {
                continue;
            };
            let Some(body_b) = bodies.get(collider_b.parent) else {
                continue;
            };

            self.narrow_phase.update_pair(
                pair.first,
                pair.second,
                collider_a,
                collider_b,
                &body_a.isometry(),
                &body_b.isometry(),
            );

            // Update island graph
            if !body_a.body_type.is_static() && !body_b.body_type.is_static() {
                self.island_manager
                    .add_contact(collider_a.parent, collider_b.parent);
            }
        }

        // Remove stale manifolds for pairs no longer in broad phase
        self.narrow_phase.end_frame();

        // 5. Build islands
        self.island_manager
            .rebuild_islands(bodies.keys(), |h| {
                bodies.get(h).map_or(true, |b| b.body_type.is_static())
            });

        // 6. Prepare contact constraints
        let manifolds = self.narrow_phase.manifolds().map(|(_, m)| {
            let mut manifold = m.clone();
            // Get material properties
            if let (Some(ca), Some(cb)) = (
                colliders.get(m.collider_a),
                colliders.get(m.collider_b),
            ) {
                let (friction, restitution) = ca.material.combine(&cb.material);
                manifold.friction = friction;
                manifold.restitution = restitution;
            }
            manifold
        });

        self.contact_solver.prepare_constraints(
            manifolds,
            |h| bodies.get(h).cloned(),
            dt,
        );

        // 7. Warm start
        self.contact_solver.warm_start(|body_handle, impulse, r| {
            if let Some(body) = bodies.get_mut(body_handle) {
                apply_impulse_at_point(body, impulse, r);
            }
        });

        // 8. Solve velocities
        self.contact_solver.solve_velocities_direct(bodies);

        // 9. Solve joint constraints
        self.joint_solver.solve_direct(joints, bodies, dt);

        // 10. Integrate positions
        for (_, body) in bodies.iter_mut() {
            self.integrator.integrate_positions(body, dt);
        }

        // 11. Solve positions (pseudo-velocity correction)
        self.contact_solver.solve_positions_direct(bodies);

        // 12. Update sleeping
        for (handle, body) in bodies.iter_mut() {
            self.sleeping_manager.update_body(handle, body, dt);
        }
    }

    /// Add a collider to the broad phase
    pub fn add_collider(
        &mut self,
        handle: ColliderHandle,
        collider: &Collider,
        body: &RigidBody,
    ) {
        let aabb = collider.compute_world_aabb(&body.isometry());
        self.broad_phase.insert(handle, aabb);
    }

    /// Remove a collider from the broad phase
    pub fn remove_collider(&mut self, handle: ColliderHandle) {
        self.broad_phase.remove(handle);
    }

    /// Remove a body from tracking
    pub fn remove_body(&mut self, handle: RigidBodyHandle) {
        self.island_manager.remove_body(handle);
        self.sleeping_manager.remove_body(handle);
    }

    /// Get simulation time
    pub fn time(&self) -> f64 {
        self.time
    }

    /// Get step number
    pub fn step_number(&self) -> u64 {
        self.step_number
    }
}

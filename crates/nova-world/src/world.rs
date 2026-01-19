//! The main PhysicsWorld API

use nova_collision::query::{self, QueryFilter, RayHit};
use nova_collision::{Collider, CollisionShape};
use nova_constraints::{Joint, JointData};
use nova_core::{ColliderHandle, JointHandle, PhysicsEvents, RigidBodyHandle};
use nova_dynamics::body::RigidBody;
use nova_math::Vec3;
use nova_pipeline::{PhysicsPipeline, PipelineConfig, SimulationStep};
use slotmap::SlotMap;

use super::builders::{ColliderBuilder, RigidBodyBuilder};

/// The main physics world containing all simulation state
pub struct PhysicsWorld {
    bodies: SlotMap<RigidBodyHandle, RigidBody>,
    colliders: SlotMap<ColliderHandle, Collider>,
    joints: SlotMap<JointHandle, Joint>,
    pipeline: PhysicsPipeline,
    events: PhysicsEvents,
}

impl Default for PhysicsWorld {
    fn default() -> Self {
        Self::new()
    }
}

impl PhysicsWorld {
    /// Create a new physics world with default settings
    pub fn new() -> Self {
        Self::with_config(PipelineConfig::default())
    }

    /// Create a new physics world with custom configuration
    pub fn with_config(config: PipelineConfig) -> Self {
        Self {
            bodies: SlotMap::with_key(),
            colliders: SlotMap::with_key(),
            joints: SlotMap::with_key(),
            pipeline: PhysicsPipeline::new(config),
            events: PhysicsEvents::new(),
        }
    }

    // ==================== Configuration ====================

    /// Set gravity
    pub fn set_gravity(&mut self, gravity: Vec3) {
        self.pipeline.set_gravity(gravity);
    }

    /// Get gravity
    pub fn gravity(&self) -> Vec3 {
        self.pipeline.config().gravity
    }

    // ==================== Rigid Bodies ====================

    /// Create a body builder
    pub fn create_body(&self) -> RigidBodyBuilder {
        RigidBodyBuilder::new()
    }

    /// Insert a rigid body and return its handle
    pub fn insert_body(&mut self, body: RigidBody) -> RigidBodyHandle {
        self.bodies.insert(body)
    }

    /// Get a rigid body reference
    pub fn get_body(&self, handle: RigidBodyHandle) -> Option<&RigidBody> {
        self.bodies.get(handle)
    }

    /// Get a mutable rigid body reference
    pub fn get_body_mut(&mut self, handle: RigidBodyHandle) -> Option<&mut RigidBody> {
        self.bodies.get_mut(handle)
    }

    /// Remove a rigid body and all attached colliders
    pub fn remove_body(&mut self, handle: RigidBodyHandle) -> Option<RigidBody> {
        // Remove attached colliders
        let colliders_to_remove: Vec<_> = self
            .colliders
            .iter()
            .filter(|(_, c)| c.parent == handle)
            .map(|(h, _)| h)
            .collect();

        for collider_handle in colliders_to_remove {
            self.remove_collider(collider_handle);
        }

        // Remove attached joints
        let joints_to_remove: Vec<_> = self
            .joints
            .iter()
            .filter(|(_, j)| j.body_a == handle || j.body_b == handle)
            .map(|(h, _)| h)
            .collect();

        for joint_handle in joints_to_remove {
            self.remove_joint(joint_handle);
        }

        self.pipeline.remove_body(handle);
        self.bodies.remove(handle)
    }

    /// Get the number of bodies
    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    /// Iterate over all bodies
    pub fn bodies(&self) -> impl Iterator<Item = (RigidBodyHandle, &RigidBody)> {
        self.bodies.iter()
    }

    /// Iterate mutably over all bodies
    pub fn bodies_mut(&mut self) -> impl Iterator<Item = (RigidBodyHandle, &mut RigidBody)> {
        self.bodies.iter_mut()
    }

    // ==================== Colliders ====================

    /// Create a collider builder for a body
    pub fn create_collider(&self, body: RigidBodyHandle, shape: CollisionShape) -> ColliderBuilder {
        ColliderBuilder::new(body, shape)
    }

    /// Insert a collider and return its handle
    pub fn insert_collider(&mut self, collider: Collider) -> ColliderHandle {
        let handle = self.colliders.insert(collider);

        // Add to broad phase
        if let Some(body) = self.bodies.get(self.colliders[handle].parent) {
            self.pipeline.add_collider(handle, &self.colliders[handle], body);
        }

        // Update body inertia
        self.update_body_mass_properties(self.colliders[handle].parent);

        handle
    }

    /// Get a collider reference
    pub fn get_collider(&self, handle: ColliderHandle) -> Option<&Collider> {
        self.colliders.get(handle)
    }

    /// Get a mutable collider reference
    pub fn get_collider_mut(&mut self, handle: ColliderHandle) -> Option<&mut Collider> {
        self.colliders.get_mut(handle)
    }

    /// Remove a collider
    pub fn remove_collider(&mut self, handle: ColliderHandle) -> Option<Collider> {
        if let Some(collider) = self.colliders.remove(handle) {
            self.pipeline.remove_collider(handle);
            self.update_body_mass_properties(collider.parent);
            Some(collider)
        } else {
            None
        }
    }

    /// Get the number of colliders
    pub fn collider_count(&self) -> usize {
        self.colliders.len()
    }

    fn update_body_mass_properties(&mut self, body_handle: RigidBodyHandle) {
        let Some(body) = self.bodies.get_mut(body_handle) else {
            return;
        };

        if !body.body_type.is_dynamic() {
            return;
        }

        // Compute combined inertia from all colliders
        let mut total_inertia = nova_math::Mat3::ZERO;

        for (_, collider) in self.colliders.iter() {
            if collider.parent == body_handle {
                let shape_inertia = collider.shape.compute_inertia(body.mass);
                total_inertia = total_inertia + shape_inertia;
            }
        }

        body.set_inertia(total_inertia);
    }

    // ==================== Joints ====================

    /// Insert a joint and return its handle
    pub fn insert_joint(&mut self, joint: Joint) -> JointHandle {
        self.joints.insert(joint)
    }

    /// Create a ball joint between two bodies
    pub fn create_ball_joint(
        &mut self,
        body_a: RigidBodyHandle,
        body_b: RigidBodyHandle,
        anchor_a: Vec3,
        anchor_b: Vec3,
    ) -> JointHandle {
        let joint = Joint::new(
            body_a,
            body_b,
            JointData::Ball(nova_constraints::BallJoint::new()),
        )
        .with_anchors(anchor_a, anchor_b);
        self.insert_joint(joint)
    }

    /// Create a hinge joint
    pub fn create_hinge_joint(
        &mut self,
        body_a: RigidBodyHandle,
        body_b: RigidBodyHandle,
        anchor_a: Vec3,
        anchor_b: Vec3,
        axis: Vec3,
    ) -> JointHandle {
        let joint = Joint::new(
            body_a,
            body_b,
            JointData::Hinge(nova_constraints::HingeJoint::new(axis)),
        )
        .with_anchors(anchor_a, anchor_b);
        self.insert_joint(joint)
    }

    /// Create a fixed joint
    pub fn create_fixed_joint(
        &mut self,
        body_a: RigidBodyHandle,
        body_b: RigidBodyHandle,
        anchor_a: Vec3,
        anchor_b: Vec3,
    ) -> JointHandle {
        let joint = Joint::new(
            body_a,
            body_b,
            JointData::Fixed(nova_constraints::FixedJoint::new()),
        )
        .with_anchors(anchor_a, anchor_b);
        self.insert_joint(joint)
    }

    /// Create a distance joint
    pub fn create_distance_joint(
        &mut self,
        body_a: RigidBodyHandle,
        body_b: RigidBodyHandle,
        anchor_a: Vec3,
        anchor_b: Vec3,
        rest_length: f32,
    ) -> JointHandle {
        let joint = Joint::new(
            body_a,
            body_b,
            JointData::Distance(nova_constraints::DistanceJoint::new(rest_length)),
        )
        .with_anchors(anchor_a, anchor_b);
        self.insert_joint(joint)
    }

    /// Get a joint reference
    pub fn get_joint(&self, handle: JointHandle) -> Option<&Joint> {
        self.joints.get(handle)
    }

    /// Get a mutable joint reference
    pub fn get_joint_mut(&mut self, handle: JointHandle) -> Option<&mut Joint> {
        self.joints.get_mut(handle)
    }

    /// Remove a joint
    pub fn remove_joint(&mut self, handle: JointHandle) -> Option<Joint> {
        self.joints.remove(handle)
    }

    /// Get the number of joints
    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    /// Iterate over all joints
    pub fn joints(&self) -> impl Iterator<Item = (JointHandle, &Joint)> {
        self.joints.iter()
    }

    /// Iterate over all colliders
    pub fn colliders(&self) -> impl Iterator<Item = (ColliderHandle, &Collider)> {
        self.colliders.iter()
    }

    // ==================== Simulation ====================

    /// Step the simulation forward by dt seconds
    pub fn step(&mut self, dt: f32) -> SimulationStep {
        self.pipeline.step(
            dt,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.joints,
            &self.events,
        )
    }

    /// Get the simulation time
    pub fn time(&self) -> f64 {
        self.pipeline.time()
    }

    /// Get the step number
    pub fn step_number(&self) -> u64 {
        self.pipeline.step_number()
    }

    // ==================== Queries ====================

    /// Perform a raycast and return the first hit
    pub fn raycast(
        &self,
        origin: Vec3,
        direction: Vec3,
        max_distance: f32,
        filter: QueryFilter,
    ) -> Option<RayHit> {
        let direction = direction.normalize();

        // Query broad phase first
        let candidates = self.pipeline.broad_phase().query_ray(origin, direction, max_distance);

        let mut best_hit: Option<RayHit> = None;

        for collider_handle in candidates {
            let Some(collider) = self.colliders.get(collider_handle) else {
                continue;
            };

            if !filter.test(collider_handle, collider) {
                continue;
            }

            let Some(body) = self.bodies.get(collider.parent) else {
                continue;
            };

            if let Some((t, point, normal)) =
                query::raycast_collider(origin, direction, max_distance, collider, &body.isometry())
            {
                if best_hit.as_ref().map_or(true, |h| t < h.distance) {
                    best_hit = Some(RayHit {
                        collider: collider_handle,
                        distance: t,
                        point,
                        normal,
                    });
                }
            }
        }

        best_hit
    }

    /// Perform a raycast and return all hits
    pub fn raycast_all(
        &self,
        origin: Vec3,
        direction: Vec3,
        max_distance: f32,
        filter: QueryFilter,
    ) -> Vec<RayHit> {
        let direction = direction.normalize();
        let candidates = self.pipeline.broad_phase().query_ray(origin, direction, max_distance);

        let mut hits = Vec::new();

        for collider_handle in candidates {
            let Some(collider) = self.colliders.get(collider_handle) else {
                continue;
            };

            if !filter.test(collider_handle, collider) {
                continue;
            }

            let Some(body) = self.bodies.get(collider.parent) else {
                continue;
            };

            if let Some((t, point, normal)) =
                query::raycast_collider(origin, direction, max_distance, collider, &body.isometry())
            {
                hits.push(RayHit {
                    collider: collider_handle,
                    distance: t,
                    point,
                    normal,
                });
            }
        }

        hits.sort_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap());
        hits
    }

    /// Check if a point is inside any collider
    pub fn point_inside(&self, point: Vec3, filter: QueryFilter) -> Option<ColliderHandle> {
        let aabb = nova_math::Aabb::from_point(point);
        let candidates = self.pipeline.broad_phase().query_aabb(&aabb);

        for collider_handle in candidates {
            let Some(collider) = self.colliders.get(collider_handle) else {
                continue;
            };

            if !filter.test(collider_handle, collider) {
                continue;
            }

            let Some(body) = self.bodies.get(collider.parent) else {
                continue;
            };

            if query::point_in_collider(point, collider, &body.isometry()) {
                return Some(collider_handle);
            }
        }

        None
    }

    /// Query colliders that overlap with an AABB
    pub fn query_aabb(&self, aabb: &nova_math::Aabb, filter: QueryFilter) -> Vec<ColliderHandle> {
        let candidates = self.pipeline.broad_phase().query_aabb(aabb);

        candidates
            .into_iter()
            .filter(|&handle| {
                self.colliders
                    .get(handle)
                    .map_or(false, |c| filter.test(handle, c))
            })
            .collect()
    }

    // ==================== Events ====================

    /// Get the physics events
    pub fn events(&self) -> &PhysicsEvents {
        &self.events
    }

    /// Drain collision events
    pub fn drain_collision_events(&self) -> Vec<nova_core::CollisionEvent> {
        self.events.collisions.drain()
    }

    // ==================== Debug ====================

    /// Get debug information about the world
    pub fn debug_info(&self) -> WorldDebugInfo {
        WorldDebugInfo {
            body_count: self.bodies.len(),
            collider_count: self.colliders.len(),
            joint_count: self.joints.len(),
            island_count: self.pipeline.island_manager().island_count(),
            contact_count: self.pipeline.narrow_phase().manifolds().count(),
            time: self.pipeline.time(),
            step_number: self.pipeline.step_number(),
        }
    }
}

/// Debug information about the physics world
#[derive(Debug, Clone)]
pub struct WorldDebugInfo {
    pub body_count: usize,
    pub collider_count: usize,
    pub joint_count: usize,
    pub island_count: usize,
    pub contact_count: usize,
    pub time: f64,
    pub step_number: u64,
}

#[cfg(test)]
mod tests {
    use super::*;
    use nova_collision::{BoxShape, SphereShape};
    use nova_dynamics::RigidBodyType;

    #[test]
    fn test_basic_simulation() {
        let mut world = PhysicsWorld::new();
        world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

        // Create a dynamic body
        let body = world.create_body()
            .body_type(RigidBodyType::Dynamic)
            .position(Vec3::new(0.0, 10.0, 0.0))
            .mass(1.0)
            .build();
        let body_handle = world.insert_body(body);

        // Add a sphere collider
        let collider = world.create_collider(
            body_handle,
            CollisionShape::Sphere(SphereShape::new(0.5)),
        ).build();
        world.insert_collider(collider);

        // Step simulation
        for _ in 0..60 {
            world.step(1.0 / 60.0);
        }

        // Body should have fallen
        let body = world.get_body(body_handle).unwrap();
        assert!(body.position.y < 10.0);
    }

    #[test]
    fn test_raycast() {
        let mut world = PhysicsWorld::new();

        // Create a static body
        let body = world.create_body()
            .body_type(RigidBodyType::Static)
            .position(Vec3::ZERO)
            .build();
        let body_handle = world.insert_body(body);

        // Add a box collider
        let collider = world.create_collider(
            body_handle,
            CollisionShape::Box(BoxShape::new(Vec3::ONE)),
        ).build();
        world.insert_collider(collider);

        // Raycast towards the box
        let hit = world.raycast(
            Vec3::new(-5.0, 0.0, 0.0),
            Vec3::X,
            100.0,
            QueryFilter::default(),
        );

        assert!(hit.is_some());
        let hit = hit.unwrap();
        assert!((hit.distance - 4.0).abs() < 0.1);
    }
}

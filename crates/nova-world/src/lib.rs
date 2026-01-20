//! Nova World - High-level World API for the Nova Physics Engine
//!
//! Provides the main PhysicsWorld API for easy physics simulation.

mod builders;
mod world;

pub use builders::{ColliderBuilder, RigidBodyBuilder};
pub use world::PhysicsWorld;

// Re-export core types
pub use nova_collision::{
    BoxShape, CapsuleShape, CollisionGroups, CollisionShape, ConvexHull, QueryFilter, RayHit,
    SphereShape, TriMesh,
};
pub use nova_constraints::{
    BallJoint, DistanceJoint, FixedJoint, HingeJoint, Joint, JointData, JointLimits, JointMotor,
    PrismaticJoint,
};
pub use nova_core::{ColliderHandle, JointHandle, PhysicsEvents, RigidBodyHandle};
pub use nova_dynamics::{RigidBody, RigidBodyType, SolverConfig};
pub use nova_math::{Aabb, Isometry, Mat3, Quat, Vec3};
pub use nova_pipeline::{PipelineConfig, SimulationStep};

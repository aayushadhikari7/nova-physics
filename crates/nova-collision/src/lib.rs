//! Nova Collision - Collision detection for the Nova Physics Engine
//!
//! Provides shapes, broad phase, narrow phase, and spatial queries.

pub mod broad_phase;
pub mod collider;
pub mod narrow_phase;
pub mod query;
pub mod shapes;

pub use broad_phase::BroadPhase;
pub use collider::{Collider, ColliderBuilder, CollisionGroups, PhysicsMaterial, CombineRule};
pub use narrow_phase::{ContactManifold, NarrowPhase};
pub use query::{QueryFilter, RayHit, ShapeHit};
pub use shapes::{BoxShape, CapsuleShape, CollisionShape, ConvexHull, SphereShape, TriMesh};

// Re-export core types
pub use nova_core::{ColliderHandle, ColliderPair, RigidBodyHandle};
pub use nova_math::{Aabb, Isometry, Vec3};

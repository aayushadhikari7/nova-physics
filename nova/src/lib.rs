//! # Nova Physics Engine
//!
//! A production-grade 3D physics engine written in Rust.
//!
//! ## Quick Start
//!
//! ```rust
//! use nova::prelude::*;
//!
//! // Create a physics world
//! let mut world = PhysicsWorld::new();
//! world.set_gravity(Vec3::new(0.0, -9.81, 0.0));
//!
//! // Create a ground plane (static body)
//! let ground = world.create_body()
//!     .body_type(RigidBodyType::Static)
//!     .position(Vec3::ZERO)
//!     .build();
//! let ground_handle = world.insert_body(ground);
//!
//! world.insert_collider(
//!     world.create_collider(
//!         ground_handle,
//!         CollisionShape::Box(BoxShape::new(Vec3::new(50.0, 0.5, 50.0))),
//!     )
//!     .friction(0.5)
//!     .build()
//! );
//!
//! // Create a dynamic ball
//! let ball = world.create_body()
//!     .body_type(RigidBodyType::Dynamic)
//!     .position(Vec3::new(0.0, 10.0, 0.0))
//!     .mass(1.0)
//!     .build();
//! let ball_handle = world.insert_body(ball);
//!
//! world.insert_collider(
//!     world.create_collider(
//!         ball_handle,
//!         CollisionShape::Sphere(SphereShape::new(0.5)),
//!     )
//!     .restitution(0.5)
//!     .build()
//! );
//!
//! // Step the simulation
//! for _ in 0..600 {
//!     world.step(1.0 / 60.0);
//! }
//! ```
//!
//! ## Modules
//!
//! - [`math`] - Math primitives (Vec3, Mat3, Quat, AABB, Isometry)
//! - [`core`] - Handles, arenas, and events
//! - [`collision`] - Shapes, broad phase, narrow phase, queries
//! - [`dynamics`] - Rigid bodies, solvers, integrators
//! - [`constraints`] - Joints and constraints
//! - [`pipeline`] - Simulation orchestration
//! - [`world`] - High-level PhysicsWorld API

pub mod collision {
    //! Collision detection module
    pub use nova_collision::*;
}

pub mod constraints {
    //! Joints and constraints module
    pub use nova_constraints::*;
}

pub mod core {
    //! Core types module
    pub use nova_core::*;
}

pub mod dynamics {
    //! Rigid body dynamics module
    pub use nova_dynamics::*;
}

pub mod math {
    //! Math primitives module
    pub use nova_math::*;
}

pub mod pipeline {
    //! Pipeline orchestration module
    pub use nova_pipeline::*;
}

pub mod world {
    //! High-level world API module
    pub use nova_world::*;
}

/// Prelude module - import this for common types
pub mod prelude {
    // Math types
    pub use nova_math::{Aabb, Isometry, Mat3, Mat4, Quat, Vec3, Vec3A};

    // Handles
    pub use nova_core::{ColliderHandle, JointHandle, RigidBodyHandle};

    // Collision shapes
    pub use nova_collision::{
        BoxShape, CapsuleShape, CollisionShape, ConvexHull, SphereShape, TriMesh,
    };

    // Collision queries
    pub use nova_collision::{QueryFilter, RayHit};

    // Collision groups
    pub use nova_collision::CollisionGroups;

    // Rigid body types
    pub use nova_dynamics::{RigidBody, RigidBodyType};

    // Joints
    pub use nova_constraints::{
        BallJoint, DistanceJoint, FixedJoint, HingeJoint, Joint, JointData, PrismaticJoint,
    };

    // World API
    pub use nova_world::{PhysicsWorld, ColliderBuilder, RigidBodyBuilder};

    // Pipeline
    pub use nova_pipeline::{PipelineConfig, SimulationStep};

    // Events
    pub use nova_core::{CollisionEvent, CollisionEventType, PhysicsEvents};
}

//! Nova Dynamics - Rigid body dynamics for the Nova Physics Engine
//!
//! Provides rigid body simulation, solvers, and integrators.

pub mod body;
pub mod integrator;
pub mod island;
pub mod sleeping;
pub mod solver;

pub use body::{RigidBody, RigidBodyBuilder, RigidBodyType};
pub use integrator::{Integrator, IntegratorType};
pub use island::IslandManager;
pub use sleeping::SleepingManager;
pub use solver::{ContactSolver, SolverConfig};

// Re-export core types
pub use nova_core::{RigidBodyHandle, ColliderHandle};
pub use nova_math::{Isometry, Mat3, Quat, Vec3};

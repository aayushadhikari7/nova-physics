//! Nova Core - Core utilities for the Nova Physics Engine
//!
//! Provides handles, arenas, pools, and event systems.

mod arena;
mod events;
mod handles;
mod pool;

pub use arena::*;
pub use events::*;
pub use handles::*;
pub use pool::*;

// Re-export commonly used types
pub use nova_math::{Aabb, Isometry, Mat3, Mat4, Quat, Vec3, Vec3A};

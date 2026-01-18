//! Nova Math - Math primitives for the Nova Physics Engine
//!
//! This crate provides math types wrapping `glam` for physics simulations.

mod aabb;
mod isometry;
mod types;

pub use aabb::Aabb;
pub use isometry::Isometry;
pub use types::*;

// Re-export glam types we use directly
pub use glam::{Mat3, Mat4, Quat, Vec3, Vec3A};

/// Small epsilon for floating point comparisons
pub const EPSILON: f32 = 1e-6;

/// Larger epsilon for physics stability
pub const PHYSICS_EPSILON: f32 = 1e-4;

/// Clamp a value between min and max
#[inline]
pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    value.max(min).min(max)
}

/// Check if two floats are approximately equal
#[inline]
pub fn approx_eq(a: f32, b: f32, epsilon: f32) -> bool {
    (a - b).abs() < epsilon
}

/// Safe normalization that returns a default if the vector is too small
#[inline]
pub fn safe_normalize(v: Vec3, default: Vec3) -> Vec3 {
    let len_sq = v.length_squared();
    if len_sq > EPSILON * EPSILON {
        v / len_sq.sqrt()
    } else {
        default
    }
}

/// Compute an orthonormal basis from a single vector
#[inline]
pub fn orthonormal_basis(n: Vec3) -> (Vec3, Vec3) {
    let sign = if n.z >= 0.0 { 1.0 } else { -1.0 };
    let a = -1.0 / (sign + n.z);
    let b = n.x * n.y * a;

    let t1 = Vec3::new(1.0 + sign * n.x * n.x * a, sign * b, -sign * n.x);
    let t2 = Vec3::new(b, sign + n.y * n.y * a, -n.y);

    (t1, t2)
}

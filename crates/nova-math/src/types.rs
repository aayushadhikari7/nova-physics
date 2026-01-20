//! Type aliases and common constants

use glam::{Mat3, Quat, Vec3};

/// Standard gravity constant (m/s²)
pub const GRAVITY: f32 = 9.81;

/// Default linear damping factor
pub const DEFAULT_LINEAR_DAMPING: f32 = 0.01;

/// Default angular damping factor
pub const DEFAULT_ANGULAR_DAMPING: f32 = 0.01;

/// Maximum linear velocity (for stability)
/// Capped to prevent tunneling - at 50 m/s with 1/960s substeps,
/// objects move max 5.2cm per substep which is safe for most colliders
pub const MAX_LINEAR_VELOCITY: f32 = 50.0;

/// Maximum angular velocity (for stability)
pub const MAX_ANGULAR_VELOCITY: f32 = 100.0;

/// Compute the inertia tensor for a box with given half-extents and mass
#[inline]
pub fn box_inertia(half_extents: Vec3, mass: f32) -> Mat3 {
    let x2 = half_extents.x * half_extents.x;
    let y2 = half_extents.y * half_extents.y;
    let z2 = half_extents.z * half_extents.z;
    let factor = mass / 3.0;

    Mat3::from_diagonal(Vec3::new(
        factor * (y2 + z2),
        factor * (x2 + z2),
        factor * (x2 + y2),
    ))
}

/// Compute the inertia tensor for a sphere with given radius and mass
#[inline]
pub fn sphere_inertia(radius: f32, mass: f32) -> Mat3 {
    let i = 0.4 * mass * radius * radius;
    Mat3::from_diagonal(Vec3::splat(i))
}

/// Compute the inertia tensor for a capsule (approximation)
#[inline]
pub fn capsule_inertia(radius: f32, half_height: f32, mass: f32) -> Mat3 {
    let cylinder_mass = mass * half_height / (half_height + radius * 2.0 / 3.0);
    let sphere_mass = mass - cylinder_mass;

    let r2 = radius * radius;
    let h2 = half_height * half_height;

    // Cylinder contribution
    let cyl_i_axial = 0.5 * cylinder_mass * r2;
    let cyl_i_radial = cylinder_mass * (r2 / 4.0 + h2 / 3.0);

    // Hemisphere contributions (parallel axis theorem)
    let sphere_i = 0.4 * sphere_mass * r2;
    let sphere_offset = half_height + 3.0 * radius / 8.0;
    let sphere_i_offset = sphere_i + sphere_mass * sphere_offset * sphere_offset;

    Mat3::from_diagonal(Vec3::new(
        cyl_i_radial + sphere_i_offset,
        cyl_i_axial + sphere_i,
        cyl_i_radial + sphere_i_offset,
    ))
}

/// Rotate an inertia tensor by a quaternion
#[inline]
pub fn rotate_inertia(inertia: Mat3, rotation: Quat) -> Mat3 {
    let rot_mat = Mat3::from_quat(rotation);
    rot_mat * inertia * rot_mat.transpose()
}

/// Compute the inverse inertia tensor, returning zero matrix for infinite inertia
#[inline]
pub fn inverse_inertia(inertia: Mat3) -> Mat3 {
    let det = inertia.determinant();
    if det.abs() > crate::EPSILON {
        inertia.inverse()
    } else {
        Mat3::ZERO
    }
}

//! Collision shapes

mod box_shape;
mod capsule;
mod compound;
mod convex_hull;
mod sphere;
mod trimesh;

pub use box_shape::BoxShape;
pub use capsule::CapsuleShape;
pub use compound::CompoundShape;
pub use convex_hull::ConvexHull;
pub use sphere::SphereShape;
pub use trimesh::TriMesh;

use nova_math::{Aabb, Isometry, Mat3, Vec3};

/// Enumeration of all supported collision shapes
#[derive(Debug, Clone)]
pub enum CollisionShape {
    Sphere(SphereShape),
    Capsule(CapsuleShape),
    Box(BoxShape),
    ConvexHull(ConvexHull),
    TriMesh(TriMesh),
    Compound(CompoundShape),
}

impl CollisionShape {
    /// Compute the AABB of this shape at the given transform
    pub fn compute_aabb(&self, transform: &Isometry) -> Aabb {
        match self {
            CollisionShape::Sphere(s) => s.compute_aabb(transform),
            CollisionShape::Capsule(c) => c.compute_aabb(transform),
            CollisionShape::Box(b) => b.compute_aabb(transform),
            CollisionShape::ConvexHull(h) => h.compute_aabb(transform),
            CollisionShape::TriMesh(t) => t.compute_aabb(transform),
            CollisionShape::Compound(c) => c.compute_aabb(transform),
        }
    }

    /// Get the support point in a given direction (for GJK)
    pub fn support_point(&self, transform: &Isometry, direction: Vec3) -> Vec3 {
        match self {
            CollisionShape::Sphere(s) => s.support_point(transform, direction),
            CollisionShape::Capsule(c) => c.support_point(transform, direction),
            CollisionShape::Box(b) => b.support_point(transform, direction),
            CollisionShape::ConvexHull(h) => h.support_point(transform, direction),
            CollisionShape::TriMesh(_) => {
                // TriMesh doesn't support GJK directly
                transform.translation
            }
            CollisionShape::Compound(c) => c.support_point(transform, direction),
        }
    }

    /// Compute the inertia tensor for this shape with the given mass
    pub fn compute_inertia(&self, mass: f32) -> Mat3 {
        match self {
            CollisionShape::Sphere(s) => s.compute_inertia(mass),
            CollisionShape::Capsule(c) => c.compute_inertia(mass),
            CollisionShape::Box(b) => b.compute_inertia(mass),
            CollisionShape::ConvexHull(h) => h.compute_inertia(mass),
            CollisionShape::TriMesh(t) => t.compute_inertia(mass),
            CollisionShape::Compound(c) => c.compute_inertia(mass),
        }
    }

    /// Check if this is a convex shape (supports GJK)
    pub fn is_convex(&self) -> bool {
        !matches!(self, CollisionShape::TriMesh(_) | CollisionShape::Compound(_))
    }

    /// Get the center of mass in local space
    pub fn local_center_of_mass(&self) -> Vec3 {
        match self {
            CollisionShape::Sphere(_) => Vec3::ZERO,
            CollisionShape::Capsule(_) => Vec3::ZERO,
            CollisionShape::Box(_) => Vec3::ZERO,
            CollisionShape::ConvexHull(h) => h.center_of_mass(),
            CollisionShape::TriMesh(t) => t.center_of_mass(),
            CollisionShape::Compound(c) => c.center_of_mass(),
        }
    }
}

/// Trait for shapes that support the support point operation
pub trait SupportMap {
    /// Get the point on the shape furthest in the given direction
    fn support_point(&self, transform: &Isometry, direction: Vec3) -> Vec3;

    /// Get the support point in local space
    fn local_support_point(&self, direction: Vec3) -> Vec3;
}

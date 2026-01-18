//! Narrow phase collision detection

mod contact;
mod epa;
mod gjk;
mod sat;

pub use contact::{ContactManifold, ContactPoint};
pub use epa::epa;
pub use gjk::{gjk_intersection, GjkResult};
pub use sat::sat_box_box;

use nova_core::{ColliderHandle, ColliderPair};
use nova_math::{Isometry, Vec3};
use smallvec::SmallVec;

use crate::collider::Collider;
use crate::shapes::{BoxShape, CapsuleShape, CollisionShape, SphereShape};

/// The narrow phase collision detection system
#[derive(Debug, Default)]
pub struct NarrowPhase {
    /// Active contact manifolds
    manifolds: hashbrown::HashMap<ColliderPair, ContactManifold>,
    /// Pairs that were updated this frame (for stale manifold cleanup)
    updated_pairs: hashbrown::HashSet<ColliderPair>,
}

impl NarrowPhase {
    pub fn new() -> Self {
        Self::default()
    }

    /// Begin a new frame - clears the set of updated pairs
    pub fn begin_frame(&mut self) {
        self.updated_pairs.clear();
    }

    /// End a frame - removes manifolds for pairs that weren't updated
    pub fn end_frame(&mut self) {
        self.manifolds
            .retain(|pair, _| self.updated_pairs.contains(pair));
    }

    /// Get all contact manifolds
    pub fn manifolds(&self) -> impl Iterator<Item = (&ColliderPair, &ContactManifold)> {
        self.manifolds.iter()
    }

    /// Get a specific manifold
    pub fn get_manifold(&self, pair: &ColliderPair) -> Option<&ContactManifold> {
        self.manifolds.get(pair)
    }

    /// Clear all manifolds
    pub fn clear(&mut self) {
        self.manifolds.clear();
        self.updated_pairs.clear();
    }

    /// Update collision detection for a pair of colliders
    pub fn update_pair(
        &mut self,
        handle_a: ColliderHandle,
        handle_b: ColliderHandle,
        collider_a: &Collider,
        collider_b: &Collider,
        transform_a: &Isometry,
        transform_b: &Isometry,
    ) {
        let pair = ColliderPair::new(handle_a, handle_b);

        // Mark this pair as processed this frame
        self.updated_pairs.insert(pair);

        let world_transform_a = transform_a.mul(&collider_a.local_transform);
        let world_transform_b = transform_b.mul(&collider_b.local_transform);

        if let Some(manifold) = compute_contacts(
            &collider_a.shape,
            &world_transform_a,
            &collider_b.shape,
            &world_transform_b,
        ) {
            let mut full_manifold = manifold;
            full_manifold.collider_a = handle_a;
            full_manifold.collider_b = handle_b;
            full_manifold.body_a = collider_a.parent;
            full_manifold.body_b = collider_b.parent;

            // Warm starting: preserve cached impulses from previous frame
            if let Some(old_manifold) = self.manifolds.get(&pair) {
                full_manifold.warm_start(old_manifold);
            }

            self.manifolds.insert(pair, full_manifold);
        } else {
            self.manifolds.remove(&pair);
        }
    }

    /// Remove a pair from tracking
    pub fn remove_pair(&mut self, pair: &ColliderPair) {
        self.manifolds.remove(pair);
    }
}

/// Compute contacts between two shapes
pub fn compute_contacts(
    shape_a: &CollisionShape,
    transform_a: &Isometry,
    shape_b: &CollisionShape,
    transform_b: &Isometry,
) -> Option<ContactManifold> {
    use CollisionShape::*;

    match (shape_a, shape_b) {
        // Sphere-Sphere
        (Sphere(a), Sphere(b)) => sphere_sphere(a, transform_a, b, transform_b),

        // Sphere-Capsule
        (Sphere(a), Capsule(b)) => sphere_capsule(a, transform_a, b, transform_b),
        (Capsule(a), Sphere(b)) => sphere_capsule(b, transform_b, a, transform_a).map(|m| m.flip()),

        // Sphere-Box
        (Sphere(a), Box(b)) => sphere_box(a, transform_a, b, transform_b),
        (Box(a), Sphere(b)) => sphere_box(b, transform_b, a, transform_a).map(|m| m.flip()),

        // Capsule-Capsule
        (Capsule(a), Capsule(b)) => capsule_capsule(a, transform_a, b, transform_b),

        // Box-Box (SAT fast path)
        (Box(a), Box(b)) => sat_box_box(a, transform_a, b, transform_b),

        // Convex-Convex (GJK + EPA)
        _ if shape_a.is_convex() && shape_b.is_convex() => {
            convex_convex(shape_a, transform_a, shape_b, transform_b)
        }

        // Unsupported pairs
        _ => None,
    }
}

fn sphere_sphere(
    a: &SphereShape,
    transform_a: &Isometry,
    b: &SphereShape,
    transform_b: &Isometry,
) -> Option<ContactManifold> {
    let center_a = transform_a.translation;
    let center_b = transform_b.translation;

    let diff = center_b - center_a;
    let dist_sq = diff.length_squared();
    let sum_radii = a.radius + b.radius;

    if dist_sq >= sum_radii * sum_radii {
        return None;
    }

    let dist = dist_sq.sqrt();
    let normal = if dist > nova_math::EPSILON {
        diff / dist
    } else {
        Vec3::Y
    };

    let depth = sum_radii - dist;
    let point_a = center_a + normal * a.radius;
    let point_b = center_b - normal * b.radius;

    let mut manifold = ContactManifold::new();
    manifold.normal = normal;
    manifold.add_contact(ContactPoint {
        local_a: transform_a.inverse_transform_point(point_a),
        local_b: transform_b.inverse_transform_point(point_b),
        world_a: point_a,
        world_b: point_b,
        depth,
        normal_impulse: 0.0,
        tangent_impulse: [0.0, 0.0],
    });

    Some(manifold)
}

fn sphere_capsule(
    sphere: &SphereShape,
    sphere_transform: &Isometry,
    capsule: &CapsuleShape,
    capsule_transform: &Isometry,
) -> Option<ContactManifold> {
    let sphere_center = sphere_transform.translation;

    // Get capsule segment in world space
    let (a, b) = capsule.segment();
    let world_a = capsule_transform.transform_point(a);
    let world_b = capsule_transform.transform_point(b);

    // Find closest point on segment to sphere center
    let ab = world_b - world_a;
    let t = ((sphere_center - world_a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0);
    let closest_on_segment = world_a + ab * t;

    let diff = closest_on_segment - sphere_center; // From sphere (A) to capsule (B)
    let dist_sq = diff.length_squared();
    let sum_radii = sphere.radius + capsule.radius;

    if dist_sq >= sum_radii * sum_radii {
        return None;
    }

    let dist = dist_sq.sqrt();
    // Normal points from A (sphere) to B (capsule)
    let normal = if dist > nova_math::EPSILON {
        diff / dist
    } else {
        Vec3::Y
    };

    let depth = sum_radii - dist;
    // point_a is on sphere surface in direction of normal (toward capsule)
    let point_a = sphere_center + normal * sphere.radius;
    // point_b is on capsule surface in opposite direction of normal (toward sphere)
    let point_b = closest_on_segment - normal * capsule.radius;

    let mut manifold = ContactManifold::new();
    manifold.normal = normal;
    manifold.add_contact(ContactPoint {
        local_a: sphere_transform.inverse_transform_point(point_a),
        local_b: capsule_transform.inverse_transform_point(point_b),
        world_a: point_a,
        world_b: point_b,
        depth,
        normal_impulse: 0.0,
        tangent_impulse: [0.0, 0.0],
    });

    Some(manifold)
}

fn sphere_box(
    sphere: &SphereShape,
    sphere_transform: &Isometry,
    box_shape: &BoxShape,
    box_transform: &Isometry,
) -> Option<ContactManifold> {
    let sphere_center = sphere_transform.translation;

    // Transform sphere center to box local space
    let local_center = box_transform.inverse_transform_point(sphere_center);

    // Find closest point on box
    let closest_local = box_shape.closest_local_point(local_center);
    let closest_world = box_transform.transform_point(closest_local);

    let diff = closest_world - sphere_center; // From sphere (A) to box (B)
    let dist_sq = diff.length_squared();

    if dist_sq >= sphere.radius * sphere.radius {
        return None;
    }

    let dist = dist_sq.sqrt();

    // Handle case when sphere center is inside box
    // Normal should point from A (sphere) to B (box)
    let (normal, depth) = if dist < nova_math::EPSILON {
        // Find the closest face - normal points toward the face (outward from sphere's perspective)
        let distances = [
            box_shape.half_extents.x - local_center.x.abs(),
            box_shape.half_extents.y - local_center.y.abs(),
            box_shape.half_extents.z - local_center.z.abs(),
        ];

        let (min_axis, min_dist) = distances
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap();

        let mut normal_local = Vec3::ZERO;
        // Normal points toward the box face the sphere is closest to (from sphere toward box surface)
        normal_local[min_axis] = if local_center[min_axis] >= 0.0 {
            1.0
        } else {
            -1.0
        };

        let normal_world = box_transform.transform_vector(normal_local);
        (normal_world, sphere.radius + min_dist)
    } else {
        (diff / dist, sphere.radius - dist) // Normal points from sphere to box
    };

    // point_a is on sphere surface in direction of normal (toward box)
    let point_a = sphere_center + normal * sphere.radius;

    let mut manifold = ContactManifold::new();
    manifold.normal = normal;
    manifold.add_contact(ContactPoint {
        local_a: sphere_transform.inverse_transform_point(point_a),
        local_b: closest_local,
        world_a: point_a,
        world_b: closest_world,
        depth,
        normal_impulse: 0.0,
        tangent_impulse: [0.0, 0.0],
    });

    Some(manifold)
}

fn capsule_capsule(
    a: &CapsuleShape,
    transform_a: &Isometry,
    b: &CapsuleShape,
    transform_b: &Isometry,
) -> Option<ContactManifold> {
    // Get capsule segments in world space
    let (a1, a2) = a.segment();
    let world_a1 = transform_a.transform_point(a1);
    let world_a2 = transform_a.transform_point(a2);

    let (b1, b2) = b.segment();
    let world_b1 = transform_b.transform_point(b1);
    let world_b2 = transform_b.transform_point(b2);

    // Find closest points between segments
    let (closest_a, closest_b) =
        closest_points_segment_segment(world_a1, world_a2, world_b1, world_b2);

    let diff = closest_b - closest_a;
    let dist_sq = diff.length_squared();
    let sum_radii = a.radius + b.radius;

    if dist_sq >= sum_radii * sum_radii {
        return None;
    }

    let dist = dist_sq.sqrt();
    let normal = if dist > nova_math::EPSILON {
        diff / dist
    } else {
        Vec3::Y
    };

    let depth = sum_radii - dist;
    let point_a = closest_a + normal * a.radius;
    let point_b = closest_b - normal * b.radius;

    let mut manifold = ContactManifold::new();
    manifold.normal = normal;
    manifold.add_contact(ContactPoint {
        local_a: transform_a.inverse_transform_point(point_a),
        local_b: transform_b.inverse_transform_point(point_b),
        world_a: point_a,
        world_b: point_b,
        depth,
        normal_impulse: 0.0,
        tangent_impulse: [0.0, 0.0],
    });

    Some(manifold)
}

fn convex_convex(
    shape_a: &CollisionShape,
    transform_a: &Isometry,
    shape_b: &CollisionShape,
    transform_b: &Isometry,
) -> Option<ContactManifold> {
    // Run GJK to check for intersection
    let gjk_result = gjk_intersection(shape_a, transform_a, shape_b, transform_b);

    match gjk_result {
        GjkResult::NoIntersection => None,
        GjkResult::Intersection(simplex) => {
            // Run EPA to get penetration info
            if let Some((normal, depth, point_a, point_b)) =
                epa(shape_a, transform_a, shape_b, transform_b, simplex)
            {
                let mut manifold = ContactManifold::new();
                manifold.normal = normal;
                manifold.add_contact(ContactPoint {
                    local_a: transform_a.inverse_transform_point(point_a),
                    local_b: transform_b.inverse_transform_point(point_b),
                    world_a: point_a,
                    world_b: point_b,
                    depth,
                    normal_impulse: 0.0,
                    tangent_impulse: [0.0, 0.0],
                });
                Some(manifold)
            } else {
                None
            }
        }
    }
}

/// Find closest points between two line segments
fn closest_points_segment_segment(
    a1: Vec3,
    a2: Vec3,
    b1: Vec3,
    b2: Vec3,
) -> (Vec3, Vec3) {
    let d1 = a2 - a1;
    let d2 = b2 - b1;
    let r = a1 - b1;

    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);

    let epsilon = 1e-10;

    let (s, t) = if a <= epsilon && e <= epsilon {
        (0.0, 0.0)
    } else if a <= epsilon {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(r);
        if e <= epsilon {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;

            let s = if denom.abs() > epsilon {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };

            let t_num = b * s + f;
            let t = if t_num < 0.0 {
                let s = (-c / a).clamp(0.0, 1.0);
                (s, 0.0)
            } else if t_num > e {
                let s = ((b - c) / a).clamp(0.0, 1.0);
                (s, 1.0)
            } else {
                (s, t_num / e)
            };
            t
        }
    };

    (a1 + d1 * s, b1 + d2 * t)
}

//! Spatial queries (raycasting, shapecasting, overlap tests)

use nova_core::ColliderHandle;
use nova_math::{Aabb, Isometry, Vec3};

use crate::collider::{Collider, CollisionGroups};
use crate::shapes::{CollisionShape, SphereShape, BoxShape, CapsuleShape, TriMesh};

/// Filter for spatial queries
#[derive(Debug, Clone)]
pub struct QueryFilter {
    /// Groups the query belongs to
    pub membership: CollisionGroups,
    /// Groups the query can interact with
    pub filter: CollisionGroups,
    /// Specific colliders to exclude
    pub exclude: Vec<ColliderHandle>,
    /// Whether to include sensors
    pub include_sensors: bool,
}

impl Default for QueryFilter {
    fn default() -> Self {
        Self {
            membership: CollisionGroups::ALL,
            filter: CollisionGroups::ALL,
            exclude: Vec::new(),
            include_sensors: false,
        }
    }
}

impl QueryFilter {
    pub fn new() -> Self {
        Self::default()
    }

    /// Set group membership
    pub fn with_membership(mut self, groups: CollisionGroups) -> Self {
        self.membership = groups;
        self
    }

    /// Set filter
    pub fn with_filter(mut self, groups: CollisionGroups) -> Self {
        self.filter = groups;
        self
    }

    /// Exclude specific colliders
    pub fn exclude(mut self, handles: impl IntoIterator<Item = ColliderHandle>) -> Self {
        self.exclude.extend(handles);
        self
    }

    /// Include sensors in query results
    pub fn include_sensors(mut self, include: bool) -> Self {
        self.include_sensors = include;
        self
    }

    /// Check if a collider passes the filter
    pub fn test(&self, handle: ColliderHandle, collider: &Collider) -> bool {
        if self.exclude.contains(&handle) {
            return false;
        }

        if !self.include_sensors && collider.is_sensor {
            return false;
        }

        self.membership.intersects(collider.filter) && collider.membership.intersects(self.filter)
    }
}

/// Result of a raycast
#[derive(Debug, Clone, Copy)]
pub struct RayHit {
    /// The collider that was hit
    pub collider: ColliderHandle,
    /// Distance along the ray
    pub distance: f32,
    /// World-space hit point
    pub point: Vec3,
    /// World-space surface normal at hit point
    pub normal: Vec3,
}

/// Result of a shapecast
#[derive(Debug, Clone, Copy)]
pub struct ShapeHit {
    /// The collider that was hit
    pub collider: ColliderHandle,
    /// Distance along the cast direction
    pub distance: f32,
    /// World-space hit point
    pub point: Vec3,
    /// World-space surface normal at hit point
    pub normal: Vec3,
}

/// Raycast against a single collider
pub fn raycast_collider(
    origin: Vec3,
    direction: Vec3,
    max_distance: f32,
    collider: &Collider,
    body_transform: &Isometry,
) -> Option<(f32, Vec3, Vec3)> {
    let world_transform = body_transform.mul(&collider.local_transform);

    match &collider.shape {
        CollisionShape::Sphere(sphere) => {
            raycast_sphere(origin, direction, max_distance, sphere, &world_transform)
        }
        CollisionShape::Capsule(capsule) => {
            raycast_capsule(origin, direction, max_distance, capsule, &world_transform)
        }
        CollisionShape::Box(box_shape) => {
            raycast_box(origin, direction, max_distance, box_shape, &world_transform)
        }
        CollisionShape::TriMesh(mesh) => {
            raycast_trimesh(origin, direction, max_distance, mesh, &world_transform)
        }
        CollisionShape::ConvexHull(hull) => {
            // Use AABB approximation for convex hull
            let aabb = hull.compute_aabb(&world_transform);
            let inv_dir = Vec3::new(
                if direction.x.abs() > 1e-10 { 1.0 / direction.x } else { f32::MAX },
                if direction.y.abs() > 1e-10 { 1.0 / direction.y } else { f32::MAX },
                if direction.z.abs() > 1e-10 { 1.0 / direction.z } else { f32::MAX },
            );
            aabb.ray_intersection(origin, inv_dir).and_then(|(t, _)| {
                if t <= max_distance {
                    let point = origin + direction * t;
                    // Approximate normal from AABB
                    let diff = point - aabb.center();
                    let half = aabb.half_extents();
                    let normal = Vec3::new(
                        if (diff.x.abs() - half.x).abs() < 0.01 { diff.x.signum() } else { 0.0 },
                        if (diff.y.abs() - half.y).abs() < 0.01 { diff.y.signum() } else { 0.0 },
                        if (diff.z.abs() - half.z).abs() < 0.01 { diff.z.signum() } else { 0.0 },
                    ).normalize_or(Vec3::Y);
                    Some((t, point, normal))
                } else {
                    None
                }
            })
        }
        CollisionShape::Compound(compound) => {
            let mut best: Option<(f32, Vec3, Vec3)> = None;
            for child in compound.children() {
                let child_transform = world_transform.mul(&child.local_transform);
                // Recursively raycast child - simplified for compounds
                let child_collider = Collider::new(
                    nova_core::RigidBodyHandle::default(),
                    child.shape.clone(),
                );
                if let Some((t, p, n)) = raycast_collider(
                    origin, direction, max_distance,
                    &child_collider, &child_transform,
                ) {
                    if best.map_or(true, |(best_t, _, _)| t < best_t) {
                        best = Some((t, p, n));
                    }
                }
            }
            best
        }
    }
}

fn raycast_sphere(
    origin: Vec3,
    direction: Vec3,
    max_distance: f32,
    sphere: &SphereShape,
    transform: &Isometry,
) -> Option<(f32, Vec3, Vec3)> {
    let center = transform.translation;
    let oc = origin - center;

    let a = direction.length_squared();
    let b = 2.0 * oc.dot(direction);
    let c = oc.length_squared() - sphere.radius * sphere.radius;

    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return None;
    }

    let sqrt_disc = discriminant.sqrt();
    let t1 = (-b - sqrt_disc) / (2.0 * a);
    let t2 = (-b + sqrt_disc) / (2.0 * a);

    let t = if t1 >= 0.0 && t1 <= max_distance {
        t1
    } else if t2 >= 0.0 && t2 <= max_distance {
        t2
    } else {
        return None;
    };

    let point = origin + direction * t;
    let normal = (point - center).normalize();

    Some((t, point, normal))
}

fn raycast_capsule(
    origin: Vec3,
    direction: Vec3,
    max_distance: f32,
    capsule: &CapsuleShape,
    transform: &Isometry,
) -> Option<(f32, Vec3, Vec3)> {
    // Transform ray to capsule local space
    let local_origin = transform.inverse_transform_point(origin);
    let local_dir = transform.inverse_transform_vector(direction);

    let (seg_a, seg_b) = capsule.segment();

    // Ray-cylinder test (infinite cylinder along Y)
    let dir_xz = Vec3::new(local_dir.x, 0.0, local_dir.z);
    let origin_xz = Vec3::new(local_origin.x, 0.0, local_origin.z);

    let a = dir_xz.length_squared();
    let b = 2.0 * origin_xz.dot(dir_xz);
    let c = origin_xz.length_squared() - capsule.radius * capsule.radius;

    let mut best: Option<(f32, Vec3, Vec3)> = None;

    // Check cylinder body
    if a > 1e-10 {
        let discriminant = b * b - 4.0 * a * c;
        if discriminant >= 0.0 {
            let sqrt_disc = discriminant.sqrt();
            for sign in [-1.0, 1.0] {
                let t = (-b + sign * sqrt_disc) / (2.0 * a);
                if t >= 0.0 && t <= max_distance {
                    let local_point = local_origin + local_dir * t;
                    if local_point.y >= -capsule.half_height && local_point.y <= capsule.half_height {
                        let local_normal = Vec3::new(local_point.x, 0.0, local_point.z).normalize();
                        let world_point = transform.transform_point(local_point);
                        let world_normal = transform.transform_vector(local_normal);

                        if best.map_or(true, |(best_t, _, _)| t < best_t) {
                            best = Some((t, world_point, world_normal));
                        }
                    }
                }
            }
        }
    }

    // Check hemisphere caps
    for cap_center in [seg_a, seg_b] {
        let oc = local_origin - cap_center;
        let a = local_dir.length_squared();
        let b = 2.0 * oc.dot(local_dir);
        let c = oc.length_squared() - capsule.radius * capsule.radius;

        let discriminant = b * b - 4.0 * a * c;
        if discriminant >= 0.0 {
            let sqrt_disc = discriminant.sqrt();
            let t = (-b - sqrt_disc) / (2.0 * a);

            if t >= 0.0 && t <= max_distance {
                let local_point = local_origin + local_dir * t;
                // Only count if outside cylinder portion
                let in_cylinder = local_point.y >= -capsule.half_height
                    && local_point.y <= capsule.half_height
                    && Vec3::new(local_point.x, 0.0, local_point.z).length_squared()
                        <= capsule.radius * capsule.radius;

                if !in_cylinder || best.is_none() {
                    let local_normal = (local_point - cap_center).normalize();
                    let world_point = transform.transform_point(local_point);
                    let world_normal = transform.transform_vector(local_normal);

                    if best.map_or(true, |(best_t, _, _)| t < best_t) {
                        best = Some((t, world_point, world_normal));
                    }
                }
            }
        }
    }

    best
}

fn raycast_box(
    origin: Vec3,
    direction: Vec3,
    max_distance: f32,
    box_shape: &BoxShape,
    transform: &Isometry,
) -> Option<(f32, Vec3, Vec3)> {
    // Transform ray to box local space
    let local_origin = transform.inverse_transform_point(origin);
    let local_dir = transform.inverse_transform_vector(direction);

    let inv_dir = Vec3::new(
        if local_dir.x.abs() > 1e-10 { 1.0 / local_dir.x } else { f32::MAX * local_dir.x.signum() },
        if local_dir.y.abs() > 1e-10 { 1.0 / local_dir.y } else { f32::MAX * local_dir.y.signum() },
        if local_dir.z.abs() > 1e-10 { 1.0 / local_dir.z } else { f32::MAX * local_dir.z.signum() },
    );

    let t1 = (-box_shape.half_extents - local_origin) * inv_dir;
    let t2 = (box_shape.half_extents - local_origin) * inv_dir;

    let t_min = t1.min(t2);
    let t_max = t1.max(t2);

    let t_enter = t_min.x.max(t_min.y).max(t_min.z);
    let t_exit = t_max.x.min(t_max.y).min(t_max.z);

    if t_enter > t_exit || t_exit < 0.0 || t_enter > max_distance {
        return None;
    }

    let t = if t_enter >= 0.0 { t_enter } else { t_exit };
    if t > max_distance {
        return None;
    }

    let local_point = local_origin + local_dir * t;

    // Determine hit face for normal
    let abs_point = local_point.abs();
    let h = box_shape.half_extents;
    let local_normal = if (abs_point.x - h.x).abs() < 0.001 {
        Vec3::new(local_point.x.signum(), 0.0, 0.0)
    } else if (abs_point.y - h.y).abs() < 0.001 {
        Vec3::new(0.0, local_point.y.signum(), 0.0)
    } else {
        Vec3::new(0.0, 0.0, local_point.z.signum())
    };

    let world_point = transform.transform_point(local_point);
    let world_normal = transform.transform_vector(local_normal);

    Some((t, world_point, world_normal))
}

fn raycast_trimesh(
    origin: Vec3,
    direction: Vec3,
    max_distance: f32,
    mesh: &TriMesh,
    transform: &Isometry,
) -> Option<(f32, Vec3, Vec3)> {
    let local_origin = transform.inverse_transform_point(origin);
    let local_dir = transform.inverse_transform_vector(direction);

    mesh.raycast(local_origin, local_dir, max_distance)
        .map(|(_, t, local_normal)| {
            let local_point = local_origin + local_dir * t;
            let world_point = transform.transform_point(local_point);
            let world_normal = transform.transform_vector(local_normal);
            (t, world_point, world_normal)
        })
}

/// Check if a point is inside a collider
pub fn point_in_collider(
    point: Vec3,
    collider: &Collider,
    body_transform: &Isometry,
) -> bool {
    let world_transform = body_transform.mul(&collider.local_transform);
    let local_point = world_transform.inverse_transform_point(point);

    match &collider.shape {
        CollisionShape::Sphere(sphere) => {
            local_point.length_squared() <= sphere.radius * sphere.radius
        }
        CollisionShape::Box(box_shape) => {
            box_shape.contains_local_point(local_point)
        }
        CollisionShape::Capsule(capsule) => {
            let closest_on_axis = capsule.closest_point_on_axis(local_point);
            (local_point - closest_on_axis).length_squared() <= capsule.radius * capsule.radius
        }
        _ => {
            // Use AABB as approximation for complex shapes
            let aabb = collider.shape.compute_aabb(&Isometry::IDENTITY);
            aabb.contains_point(local_point)
        }
    }
}

/// Find the closest point on a collider to a query point
pub fn closest_point_on_collider(
    point: Vec3,
    collider: &Collider,
    body_transform: &Isometry,
) -> Vec3 {
    let world_transform = body_transform.mul(&collider.local_transform);

    match &collider.shape {
        CollisionShape::Sphere(sphere) => {
            sphere.closest_point(&world_transform, point)
        }
        CollisionShape::Box(box_shape) => {
            let local_point = world_transform.inverse_transform_point(point);
            let local_closest = box_shape.closest_local_point(local_point);
            world_transform.transform_point(local_closest)
        }
        _ => {
            // Approximation: project to AABB surface
            let aabb = collider.shape.compute_aabb(&world_transform);
            aabb.closest_point(point)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shapes::SphereShape;

    #[test]
    fn test_raycast_sphere() {
        let collider = Collider::new(
            nova_core::RigidBodyHandle::default(),
            CollisionShape::Sphere(SphereShape::new(1.0)),
        );
        let transform = Isometry::IDENTITY;

        let origin = Vec3::new(-5.0, 0.0, 0.0);
        let direction = Vec3::X;

        let hit = raycast_collider(origin, direction, 100.0, &collider, &transform);
        assert!(hit.is_some());

        let (t, point, normal) = hit.unwrap();
        assert!((t - 4.0).abs() < 0.01);
        assert!((point - Vec3::new(-1.0, 0.0, 0.0)).length() < 0.01);
        assert!((normal - Vec3::NEG_X).length() < 0.01);
    }

    #[test]
    fn test_point_in_sphere() {
        let collider = Collider::new(
            nova_core::RigidBodyHandle::default(),
            CollisionShape::Sphere(SphereShape::new(1.0)),
        );
        let transform = Isometry::IDENTITY;

        assert!(point_in_collider(Vec3::ZERO, &collider, &transform));
        assert!(point_in_collider(Vec3::new(0.5, 0.0, 0.0), &collider, &transform));
        assert!(!point_in_collider(Vec3::new(1.5, 0.0, 0.0), &collider, &transform));
    }
}

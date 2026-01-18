//! Convex hull collision shape

use nova_math::{Aabb, Isometry, Mat3, Vec3};
use smallvec::SmallVec;

use super::SupportMap;

/// A convex hull defined by a set of vertices
#[derive(Debug, Clone)]
pub struct ConvexHull {
    /// Vertices of the hull
    vertices: Vec<Vec3>,
    /// Center of mass
    center: Vec3,
    /// Local AABB for quick culling
    local_aabb: Aabb,
}

impl ConvexHull {
    /// Create a convex hull from a set of points
    /// Note: This does not compute the convex hull, it assumes the points already form one
    pub fn new(vertices: Vec<Vec3>) -> Self {
        assert!(
            vertices.len() >= 4,
            "Convex hull must have at least 4 vertices"
        );

        let center = vertices.iter().fold(Vec3::ZERO, |acc, &v| acc + v) / vertices.len() as f32;

        let local_aabb = Aabb::from_points(&vertices);

        Self {
            vertices,
            center,
            local_aabb,
        }
    }

    /// Create a convex hull from a set of points, computing the actual hull
    pub fn from_points(points: &[Vec3]) -> Self {
        // Simple quickhull-like implementation
        // For a full implementation, use a proper convex hull algorithm
        let hull_points = compute_convex_hull(points);
        Self::new(hull_points)
    }

    /// Get the vertices
    pub fn vertices(&self) -> &[Vec3] {
        &self.vertices
    }

    /// Get the center of mass
    pub fn center_of_mass(&self) -> Vec3 {
        self.center
    }

    /// Compute the AABB
    pub fn compute_aabb(&self, transform: &Isometry) -> Aabb {
        let mut aabb = Aabb::EMPTY;
        for &vertex in &self.vertices {
            let world_vertex = transform.transform_point(vertex);
            aabb = aabb.expand_to_include_point(world_vertex);
        }
        aabb
    }

    /// Compute approximate inertia tensor (using bounding box approximation)
    pub fn compute_inertia(&self, mass: f32) -> Mat3 {
        let half_extents = self.local_aabb.half_extents();
        nova_math::box_inertia(half_extents, mass)
    }

    /// Get the number of vertices
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }
}

impl SupportMap for ConvexHull {
    fn support_point(&self, transform: &Isometry, direction: Vec3) -> Vec3 {
        let local_dir = transform.inverse_transform_vector(direction);
        let local_support = self.local_support_point(local_dir);
        transform.transform_point(local_support)
    }

    fn local_support_point(&self, direction: Vec3) -> Vec3 {
        let mut best_point = self.vertices[0];
        let mut best_dot = direction.dot(best_point);

        for &vertex in &self.vertices[1..] {
            let dot = direction.dot(vertex);
            if dot > best_dot {
                best_dot = dot;
                best_point = vertex;
            }
        }

        best_point
    }
}

impl PartialEq for ConvexHull {
    fn eq(&self, other: &Self) -> bool {
        self.vertices == other.vertices
    }
}

/// Simple convex hull computation (gift wrapping / Jarvis march in 3D simplified)
fn compute_convex_hull(points: &[Vec3]) -> Vec<Vec3> {
    if points.len() <= 4 {
        return points.to_vec();
    }

    // Find extreme points
    let mut min_x = points[0];
    let mut max_x = points[0];
    let mut min_y = points[0];
    let mut max_y = points[0];
    let mut min_z = points[0];
    let mut max_z = points[0];

    for &p in points {
        if p.x < min_x.x {
            min_x = p;
        }
        if p.x > max_x.x {
            max_x = p;
        }
        if p.y < min_y.y {
            min_y = p;
        }
        if p.y > max_y.y {
            max_y = p;
        }
        if p.z < min_z.z {
            min_z = p;
        }
        if p.z > max_z.z {
            max_z = p;
        }
    }

    // Collect unique extreme points
    let mut hull: SmallVec<[Vec3; 8]> = SmallVec::new();
    for p in [min_x, max_x, min_y, max_y, min_z, max_z] {
        if !hull.iter().any(|&h| (h - p).length_squared() < 1e-10) {
            hull.push(p);
        }
    }

    // Add points that are significantly outside the current hull
    for &p in points {
        let center =
            hull.iter().fold(Vec3::ZERO, |acc, &v| acc + v) / hull.len().max(1) as f32;
        let dir = p - center;
        if dir.length_squared() > 1e-10 {
            let mut is_extreme = false;
            for &h in &hull {
                let to_h = h - center;
                if dir.dot(to_h) > to_h.length_squared() * 0.99 {
                    // Point is more extreme in some direction
                    is_extreme = true;
                    break;
                }
            }
            if is_extreme && !hull.iter().any(|&h| (h - p).length_squared() < 1e-10) {
                hull.push(p);
            }
        }
    }

    hull.to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_convex_hull_support() {
        let vertices = vec![
            Vec3::new(-1.0, -1.0, -1.0),
            Vec3::new(1.0, -1.0, -1.0),
            Vec3::new(1.0, 1.0, -1.0),
            Vec3::new(-1.0, 1.0, -1.0),
            Vec3::new(-1.0, -1.0, 1.0),
            Vec3::new(1.0, -1.0, 1.0),
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(-1.0, 1.0, 1.0),
        ];

        let hull = ConvexHull::new(vertices);
        let transform = Isometry::IDENTITY;

        let support = hull.support_point(&transform, Vec3::new(1.0, 1.0, 1.0).normalize());
        assert_eq!(support, Vec3::new(1.0, 1.0, 1.0));
    }
}

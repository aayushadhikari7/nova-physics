//! Triangle mesh collision shape

use nova_math::{Aabb, Isometry, Mat3, Vec3};

/// A triangle in the mesh
#[derive(Debug, Clone, Copy)]
pub struct Triangle {
    pub vertices: [Vec3; 3],
}

impl Triangle {
    pub fn new(a: Vec3, b: Vec3, c: Vec3) -> Self {
        Self { vertices: [a, b, c] }
    }

    pub fn normal(&self) -> Vec3 {
        let edge1 = self.vertices[1] - self.vertices[0];
        let edge2 = self.vertices[2] - self.vertices[0];
        edge1.cross(edge2).normalize()
    }

    pub fn center(&self) -> Vec3 {
        (self.vertices[0] + self.vertices[1] + self.vertices[2]) / 3.0
    }

    pub fn aabb(&self) -> Aabb {
        Aabb::from_points(&self.vertices)
    }
}

/// BVH node for triangle mesh acceleration
#[derive(Debug, Clone)]
struct BvhNode {
    aabb: Aabb,
    /// If leaf: indices into triangles array. If internal: child node indices
    data: BvhNodeData,
}

#[derive(Debug, Clone)]
enum BvhNodeData {
    Leaf { triangle_indices: Vec<usize> },
    Internal { left: usize, right: usize },
}

/// A triangle mesh collision shape with BVH acceleration
#[derive(Debug, Clone)]
pub struct TriMesh {
    vertices: Vec<Vec3>,
    indices: Vec<[u32; 3]>,
    triangles: Vec<Triangle>,
    bvh_nodes: Vec<BvhNode>,
    root_aabb: Aabb,
    center: Vec3,
}

impl TriMesh {
    /// Create a new triangle mesh from vertices and indices
    pub fn new(vertices: Vec<Vec3>, indices: Vec<[u32; 3]>) -> Self {
        let triangles: Vec<Triangle> = indices
            .iter()
            .map(|idx| {
                Triangle::new(
                    vertices[idx[0] as usize],
                    vertices[idx[1] as usize],
                    vertices[idx[2] as usize],
                )
            })
            .collect();

        let root_aabb = Aabb::from_points(&vertices);
        let center =
            triangles.iter().map(|t| t.center()).fold(Vec3::ZERO, |a, b| a + b) / triangles.len().max(1) as f32;

        let mut mesh = Self {
            vertices,
            indices,
            triangles,
            bvh_nodes: Vec::new(),
            root_aabb,
            center,
        };

        mesh.build_bvh();
        mesh
    }

    /// Build the BVH for collision queries
    fn build_bvh(&mut self) {
        if self.triangles.is_empty() {
            return;
        }

        let indices: Vec<usize> = (0..self.triangles.len()).collect();
        self.bvh_nodes.clear();
        self.build_bvh_node(&indices);
    }

    fn build_bvh_node(&mut self, triangle_indices: &[usize]) -> usize {
        let node_index = self.bvh_nodes.len();

        // Compute AABB for this set of triangles
        let mut aabb = Aabb::EMPTY;
        for &idx in triangle_indices {
            aabb = aabb.merge(&self.triangles[idx].aabb());
        }

        // Leaf node if few triangles
        if triangle_indices.len() <= 4 {
            self.bvh_nodes.push(BvhNode {
                aabb,
                data: BvhNodeData::Leaf {
                    triangle_indices: triangle_indices.to_vec(),
                },
            });
            return node_index;
        }

        // Split along longest axis
        let axis = aabb.longest_axis();
        let mut sorted_indices = triangle_indices.to_vec();
        sorted_indices.sort_by(|&a, &b| {
            let ca = self.triangles[a].center();
            let cb = self.triangles[b].center();
            let va = match axis {
                0 => ca.x,
                1 => ca.y,
                _ => ca.z,
            };
            let vb = match axis {
                0 => cb.x,
                1 => cb.y,
                _ => cb.z,
            };
            va.partial_cmp(&vb).unwrap_or(std::cmp::Ordering::Equal)
        });

        let mid = sorted_indices.len() / 2;
        let (left_indices, right_indices) = sorted_indices.split_at(mid);

        // Reserve space for this node
        self.bvh_nodes.push(BvhNode {
            aabb,
            data: BvhNodeData::Leaf {
                triangle_indices: Vec::new(),
            },
        });

        let left = self.build_bvh_node(left_indices);
        let right = self.build_bvh_node(right_indices);

        self.bvh_nodes[node_index].data = BvhNodeData::Internal { left, right };

        node_index
    }

    /// Get vertices
    pub fn vertices(&self) -> &[Vec3] {
        &self.vertices
    }

    /// Get indices
    pub fn indices(&self) -> &[[u32; 3]] {
        &self.indices
    }

    /// Get triangles
    pub fn triangles(&self) -> &[Triangle] {
        &self.triangles
    }

    /// Get center of mass
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

    /// Compute approximate inertia (using AABB approximation)
    pub fn compute_inertia(&self, mass: f32) -> Mat3 {
        let half_extents = self.root_aabb.half_extents();
        nova_math::box_inertia(half_extents, mass)
    }

    /// Query triangles that potentially intersect an AABB
    pub fn query_aabb(&self, query_aabb: &Aabb) -> Vec<usize> {
        let mut result = Vec::new();
        if !self.bvh_nodes.is_empty() {
            self.query_aabb_recursive(0, query_aabb, &mut result);
        }
        result
    }

    fn query_aabb_recursive(&self, node_idx: usize, query_aabb: &Aabb, result: &mut Vec<usize>) {
        let node = &self.bvh_nodes[node_idx];

        if !node.aabb.intersects(query_aabb) {
            return;
        }

        match &node.data {
            BvhNodeData::Leaf { triangle_indices } => {
                for &idx in triangle_indices {
                    if self.triangles[idx].aabb().intersects(query_aabb) {
                        result.push(idx);
                    }
                }
            }
            BvhNodeData::Internal { left, right } => {
                self.query_aabb_recursive(*left, query_aabb, result);
                self.query_aabb_recursive(*right, query_aabb, result);
            }
        }
    }

    /// Raycast against the mesh
    pub fn raycast(&self, origin: Vec3, direction: Vec3, max_dist: f32) -> Option<(usize, f32, Vec3)> {
        let inv_dir = Vec3::new(
            if direction.x.abs() > 1e-10 {
                1.0 / direction.x
            } else {
                f32::MAX
            },
            if direction.y.abs() > 1e-10 {
                1.0 / direction.y
            } else {
                f32::MAX
            },
            if direction.z.abs() > 1e-10 {
                1.0 / direction.z
            } else {
                f32::MAX
            },
        );

        let mut best_hit: Option<(usize, f32, Vec3)> = None;

        if !self.bvh_nodes.is_empty() {
            self.raycast_recursive(0, origin, direction, inv_dir, max_dist, &mut best_hit);
        }

        best_hit
    }

    fn raycast_recursive(
        &self,
        node_idx: usize,
        origin: Vec3,
        direction: Vec3,
        inv_dir: Vec3,
        max_dist: f32,
        best_hit: &mut Option<(usize, f32, Vec3)>,
    ) {
        let node = &self.bvh_nodes[node_idx];

        let current_max = best_hit.map_or(max_dist, |(_, t, _)| t);
        if node.aabb.ray_intersection(origin, inv_dir).is_none() {
            return;
        }

        match &node.data {
            BvhNodeData::Leaf { triangle_indices } => {
                for &idx in triangle_indices {
                    if let Some((t, normal)) = ray_triangle_intersection(
                        origin,
                        direction,
                        &self.triangles[idx],
                        current_max,
                    ) {
                        if best_hit.map_or(true, |(_, best_t, _)| t < best_t) {
                            *best_hit = Some((idx, t, normal));
                        }
                    }
                }
            }
            BvhNodeData::Internal { left, right } => {
                self.raycast_recursive(*left, origin, direction, inv_dir, max_dist, best_hit);
                self.raycast_recursive(*right, origin, direction, inv_dir, max_dist, best_hit);
            }
        }
    }
}

impl PartialEq for TriMesh {
    fn eq(&self, other: &Self) -> bool {
        self.vertices == other.vertices && self.indices == other.indices
    }
}

/// Möller–Trumbore ray-triangle intersection
fn ray_triangle_intersection(
    origin: Vec3,
    direction: Vec3,
    triangle: &Triangle,
    max_dist: f32,
) -> Option<(f32, Vec3)> {
    let edge1 = triangle.vertices[1] - triangle.vertices[0];
    let edge2 = triangle.vertices[2] - triangle.vertices[0];
    let h = direction.cross(edge2);
    let a = edge1.dot(h);

    if a.abs() < 1e-10 {
        return None; // Ray parallel to triangle
    }

    let f = 1.0 / a;
    let s = origin - triangle.vertices[0];
    let u = f * s.dot(h);

    if !(0.0..=1.0).contains(&u) {
        return None;
    }

    let q = s.cross(edge1);
    let v = f * direction.dot(q);

    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = f * edge2.dot(q);

    if t > 1e-10 && t <= max_dist {
        let normal = edge1.cross(edge2).normalize();
        Some((t, normal))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trimesh_raycast() {
        let vertices = vec![
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.5, 1.0, 0.0),
        ];
        let indices = vec![[0, 1, 2]];

        let mesh = TriMesh::new(vertices, indices);

        // Ray hitting the triangle
        let hit = mesh.raycast(Vec3::new(0.5, 0.5, -1.0), Vec3::Z, 10.0);
        assert!(hit.is_some());
        let (idx, t, _) = hit.unwrap();
        assert_eq!(idx, 0);
        assert!((t - 1.0).abs() < 1e-5);

        // Ray missing the triangle
        let miss = mesh.raycast(Vec3::new(5.0, 5.0, -1.0), Vec3::Z, 10.0);
        assert!(miss.is_none());
    }
}

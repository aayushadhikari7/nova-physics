//! EPA (Expanding Polytope Algorithm) for penetration depth calculation

use nova_math::{Isometry, Vec3, EPSILON};
use smallvec::SmallVec;

use super::gjk::Simplex;
use crate::shapes::CollisionShape;

const MAX_EPA_ITERATIONS: u32 = 64;
const EPA_TOLERANCE: f32 = 0.0001;

/// A face in the EPA polytope
#[derive(Debug, Clone, Copy)]
struct EpaFace {
    indices: [usize; 3],
    normal: Vec3,
    distance: f32,
}

/// Run EPA algorithm to find penetration depth and contact points
pub fn epa(
    shape_a: &CollisionShape,
    transform_a: &Isometry,
    shape_b: &CollisionShape,
    transform_b: &Isometry,
    simplex: Simplex,
) -> Option<(Vec3, f32, Vec3, Vec3)> {
    if simplex.size < 4 {
        return None;
    }

    // Initialize polytope from simplex
    let mut vertices: SmallVec<[Vec3; 64]> = SmallVec::new();
    let mut support_a: SmallVec<[Vec3; 64]> = SmallVec::new();
    let mut support_b: SmallVec<[Vec3; 64]> = SmallVec::new();

    for i in 0..4 {
        vertices.push(simplex.points[i]);
        support_a.push(simplex.support_a[i]);
        support_b.push(simplex.support_b[i]);
    }

    // Initial faces (tetrahedron)
    let mut faces: SmallVec<[EpaFace; 64]> = SmallVec::new();

    // Create faces with consistent winding
    let face_indices = [[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]];

    for indices in &face_indices {
        if let Some(face) = create_face(&vertices, *indices) {
            faces.push(face);
        }
    }

    if faces.is_empty() {
        return None;
    }

    for _ in 0..MAX_EPA_ITERATIONS {
        // Find face closest to origin
        let (closest_idx, closest_face) = faces
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.distance.partial_cmp(&b.distance).unwrap())?;

        let closest_face = *closest_face;

        // Get support point in face normal direction
        let support_point_a = shape_a.support_point(transform_a, closest_face.normal);
        let support_point_b = shape_b.support_point(transform_b, -closest_face.normal);
        let new_point = support_point_a - support_point_b;

        let distance_to_face = new_point.dot(closest_face.normal);

        // Check for convergence
        if distance_to_face - closest_face.distance < EPA_TOLERANCE {
            // Found the closest face, compute contact points
            let (point_a, point_b) = compute_contact_points(
                &closest_face,
                &vertices,
                &support_a,
                &support_b,
            );

            return Some((closest_face.normal, closest_face.distance, point_a, point_b));
        }

        // Add new point and expand polytope
        let new_idx = vertices.len();
        vertices.push(new_point);
        support_a.push(support_point_a);
        support_b.push(support_point_b);

        // Find and remove faces visible from new point
        let mut edges: SmallVec<[(usize, usize); 32]> = SmallVec::new();

        faces.retain(|face| {
            let to_point = new_point - vertices[face.indices[0]];
            if face.normal.dot(to_point) > 0.0 {
                // Face is visible, save its edges
                add_edge(&mut edges, face.indices[0], face.indices[1]);
                add_edge(&mut edges, face.indices[1], face.indices[2]);
                add_edge(&mut edges, face.indices[2], face.indices[0]);
                false
            } else {
                true
            }
        });

        // Create new faces from horizon edges
        for (i, j) in edges {
            if let Some(face) = create_face(&vertices, [i, j, new_idx]) {
                faces.push(face);
            }
        }

        if faces.is_empty() {
            return None;
        }
    }

    // Max iterations reached, return best approximation
    let closest_face = faces
        .iter()
        .min_by(|a, b| a.distance.partial_cmp(&b.distance).unwrap())?;

    let (point_a, point_b) =
        compute_contact_points(closest_face, &vertices, &support_a, &support_b);

    Some((closest_face.normal, closest_face.distance, point_a, point_b))
}

fn create_face(vertices: &[Vec3], indices: [usize; 3]) -> Option<EpaFace> {
    let a = vertices[indices[0]];
    let b = vertices[indices[1]];
    let c = vertices[indices[2]];

    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(ac);

    let len = normal.length();
    if len < EPSILON {
        return None;
    }

    let normal = normal / len;

    // Ensure normal points away from origin
    let distance = normal.dot(a);
    let (normal, distance) = if distance < 0.0 {
        (-normal, -distance)
    } else {
        (normal, distance)
    };

    Some(EpaFace {
        indices,
        normal,
        distance,
    })
}

fn add_edge(edges: &mut SmallVec<[(usize, usize); 32]>, a: usize, b: usize) {
    // Check if reverse edge exists
    if let Some(pos) = edges.iter().position(|&(i, j)| i == b && j == a) {
        // Remove the reverse edge (it's shared between two visible faces)
        edges.swap_remove(pos);
    } else {
        edges.push((a, b));
    }
}

fn compute_contact_points(
    face: &EpaFace,
    vertices: &[Vec3],
    support_a: &[Vec3],
    support_b: &[Vec3],
) -> (Vec3, Vec3) {
    let a = vertices[face.indices[0]];
    let b = vertices[face.indices[1]];
    let c = vertices[face.indices[2]];

    // Project origin onto face and compute barycentric coordinates
    let origin_on_face = face.normal * face.distance;

    let v0 = b - a;
    let v1 = c - a;
    let v2 = origin_on_face - a;

    let d00 = v0.dot(v0);
    let d01 = v0.dot(v1);
    let d11 = v1.dot(v1);
    let d20 = v2.dot(v0);
    let d21 = v2.dot(v1);

    let denom = d00 * d11 - d01 * d01;
    if denom.abs() < EPSILON {
        // Degenerate face, return centroid
        let point_a = (support_a[face.indices[0]] + support_a[face.indices[1]] + support_a[face.indices[2]]) / 3.0;
        let point_b = (support_b[face.indices[0]] + support_b[face.indices[1]] + support_b[face.indices[2]]) / 3.0;
        return (point_a, point_b);
    }

    let v = (d11 * d20 - d01 * d21) / denom;
    let w = (d00 * d21 - d01 * d20) / denom;
    let u = 1.0 - v - w;

    // Clamp to valid barycentric range
    let u = u.clamp(0.0, 1.0);
    let v = v.clamp(0.0, 1.0);
    let w = w.clamp(0.0, 1.0);
    let sum = u + v + w;
    let (u, v, w) = if sum > EPSILON {
        (u / sum, v / sum, w / sum)
    } else {
        (1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0)
    };

    let point_a = support_a[face.indices[0]] * u
        + support_a[face.indices[1]] * v
        + support_a[face.indices[2]] * w;
    let point_b = support_b[face.indices[0]] * u
        + support_b[face.indices[1]] * v
        + support_b[face.indices[2]] * w;

    (point_a, point_b)
}

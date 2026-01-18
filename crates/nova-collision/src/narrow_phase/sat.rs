//! SAT (Separating Axis Theorem) for box-box collision

use nova_math::{Isometry, Vec3, EPSILON};

use super::contact::{ContactManifold, ContactPoint};
use crate::shapes::BoxShape;

/// SAT-based box-box collision detection
pub fn sat_box_box(
    box_a: &BoxShape,
    transform_a: &Isometry,
    box_b: &BoxShape,
    transform_b: &Isometry,
) -> Option<ContactManifold> {
    // Get axes for both boxes
    let axes_a = [
        transform_a.rotation * Vec3::X,
        transform_a.rotation * Vec3::Y,
        transform_a.rotation * Vec3::Z,
    ];
    let axes_b = [
        transform_b.rotation * Vec3::X,
        transform_b.rotation * Vec3::Y,
        transform_b.rotation * Vec3::Z,
    ];

    let center_a = transform_a.translation;
    let center_b = transform_b.translation;
    let d = center_b - center_a;

    let ha = box_a.half_extents;
    let hb = box_b.half_extents;

    // Precompute rotation matrix from A to B
    let mut r = [[0.0f32; 3]; 3];
    let mut abs_r = [[0.0f32; 3]; 3];

    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = axes_a[i].dot(axes_b[j]);
            abs_r[i][j] = r[i][j].abs() + EPSILON;
        }
    }

    let mut min_depth = f32::MAX;
    let mut min_axis = Vec3::ZERO;
    let mut min_axis_type = 0u8; // 0-2: face A, 3-5: face B, 6+: edge-edge

    // Test axes of A
    for i in 0..3 {
        let ra = ha[i];
        let rb = hb[0] * abs_r[i][0] + hb[1] * abs_r[i][1] + hb[2] * abs_r[i][2];
        let dist = d.dot(axes_a[i]).abs();
        let depth = ra + rb - dist;

        if depth < 0.0 {
            return None;
        }

        if depth < min_depth {
            min_depth = depth;
            min_axis = if d.dot(axes_a[i]) < 0.0 {
                -axes_a[i]
            } else {
                axes_a[i]
            };
            min_axis_type = i as u8;
        }
    }

    // Test axes of B
    for i in 0..3 {
        let ra = ha[0] * abs_r[0][i] + ha[1] * abs_r[1][i] + ha[2] * abs_r[2][i];
        let rb = hb[i];
        let dist = d.dot(axes_b[i]).abs();
        let depth = ra + rb - dist;

        if depth < 0.0 {
            return None;
        }

        if depth < min_depth {
            min_depth = depth;
            min_axis = if d.dot(axes_b[i]) < 0.0 {
                -axes_b[i]
            } else {
                axes_b[i]
            };
            min_axis_type = (i + 3) as u8;
        }
    }

    // Test cross product axes (edge-edge)
    for i in 0..3 {
        for j in 0..3 {
            let axis = axes_a[i].cross(axes_b[j]);
            let len = axis.length();
            if len < EPSILON {
                continue;
            }
            let axis = axis / len;

            let ra = ha[(i + 1) % 3] * abs_r[(i + 2) % 3][j]
                + ha[(i + 2) % 3] * abs_r[(i + 1) % 3][j];
            let rb = hb[(j + 1) % 3] * abs_r[i][(j + 2) % 3]
                + hb[(j + 2) % 3] * abs_r[i][(j + 1) % 3];
            let dist = d.dot(axis).abs();
            let depth = ra + rb - dist;

            if depth < 0.0 {
                return None;
            }

            // Prefer face contacts over edge contacts
            let depth_with_preference = depth * 0.99;
            if depth_with_preference < min_depth {
                min_depth = depth;
                min_axis = if d.dot(axis) < 0.0 { -axis } else { axis };
                min_axis_type = (6 + i * 3 + j) as u8;
            }
        }
    }

    // Generate contact manifold
    let normal = min_axis;

    if min_axis_type < 3 {
        // Face of A
        generate_face_contacts(box_a, transform_a, box_b, transform_b, normal, min_depth)
    } else if min_axis_type < 6 {
        // Face of B
        generate_face_contacts(box_b, transform_b, box_a, transform_a, -normal, min_depth)
            .map(|m| m.flip())
    } else {
        // Edge-edge
        generate_edge_contact(box_a, transform_a, box_b, transform_b, normal, min_depth)
    }
}

fn generate_face_contacts(
    reference_box: &BoxShape,
    reference_transform: &Isometry,
    incident_box: &BoxShape,
    incident_transform: &Isometry,
    normal: Vec3,
    depth: f32,
) -> Option<ContactManifold> {
    // Find incident face (face of incident box most anti-parallel to normal)
    let local_normal_incident = incident_transform.inverse_transform_vector(normal);

    let mut min_dot = f32::MAX;
    let mut incident_axis = 0;
    let mut incident_sign = 1.0f32;

    for i in 0..3 {
        let axis = Vec3::ZERO.with(i, 1.0);
        let dot = local_normal_incident.dot(axis);
        if dot < min_dot {
            min_dot = dot;
            incident_axis = i;
            incident_sign = 1.0;
        }
        if -dot < min_dot {
            min_dot = -dot;
            incident_axis = i;
            incident_sign = -1.0;
        }
    }

    // Get incident face vertices
    let h = incident_box.half_extents;
    let mut face_center = Vec3::ZERO;
    face_center[incident_axis] = h[incident_axis] * incident_sign;

    let axis1 = (incident_axis + 1) % 3;
    let axis2 = (incident_axis + 2) % 3;

    let offsets = [
        Vec3::ZERO.with(axis1, h[axis1]).with(axis2, h[axis2]),
        Vec3::ZERO.with(axis1, -h[axis1]).with(axis2, h[axis2]),
        Vec3::ZERO.with(axis1, -h[axis1]).with(axis2, -h[axis2]),
        Vec3::ZERO.with(axis1, h[axis1]).with(axis2, -h[axis2]),
    ];

    let mut incident_verts: [Vec3; 4] = [Vec3::ZERO; 4];
    for (i, offset) in offsets.iter().enumerate() {
        incident_verts[i] = incident_transform.transform_point(face_center + *offset);
    }

    // Clip incident face against reference face side planes
    let ref_h = reference_box.half_extents;
    let ref_axes = [
        reference_transform.rotation * Vec3::X,
        reference_transform.rotation * Vec3::Y,
        reference_transform.rotation * Vec3::Z,
    ];

    // Find reference face axis
    let local_normal_ref = reference_transform.inverse_transform_vector(normal);
    let mut ref_axis = 0;
    let mut ref_sign = 1.0f32;
    let mut max_dot = 0.0f32;
    for i in 0..3 {
        let dot = local_normal_ref[i].abs();
        if dot > max_dot {
            max_dot = dot;
            ref_axis = i;
            ref_sign = local_normal_ref[i].signum();
        }
    }

    let clip_axes = [(ref_axis + 1) % 3, (ref_axis + 2) % 3];

    let mut clipped_verts = incident_verts.to_vec();

    for &clip_axis in &clip_axes {
        let axis = ref_axes[clip_axis];
        let extent = ref_h[clip_axis];
        let center_proj = reference_transform.translation.dot(axis);

        // Clip against positive side
        clipped_verts = clip_polygon_against_plane(&clipped_verts, axis, center_proj + extent);
        if clipped_verts.is_empty() {
            return None;
        }

        // Clip against negative side
        clipped_verts = clip_polygon_against_plane(&clipped_verts, -axis, -center_proj + extent);
        if clipped_verts.is_empty() {
            return None;
        }
    }

    // Keep only points below the reference face
    let ref_face_normal = ref_axes[ref_axis] * ref_sign;
    let ref_face_offset = reference_transform.translation.dot(ref_face_normal) + ref_h[ref_axis];

    let mut manifold = ContactManifold::new();
    manifold.normal = normal;

    for vert in &clipped_verts {
        let dist = vert.dot(ref_face_normal) - ref_face_offset;
        if dist < EPSILON {
            let world_a = *vert - ref_face_normal * dist;
            let world_b = *vert;

            manifold.add_contact(ContactPoint {
                local_a: reference_transform.inverse_transform_point(world_a),
                local_b: incident_transform.inverse_transform_point(world_b),
                world_a,
                world_b,
                depth: -dist,
                normal_impulse: 0.0,
                tangent_impulse: [0.0, 0.0],
            });
        }
    }

    if manifold.is_empty() {
        None
    } else {
        Some(manifold)
    }
}

fn clip_polygon_against_plane(verts: &[Vec3], plane_normal: Vec3, plane_offset: f32) -> Vec<Vec3> {
    if verts.is_empty() {
        return Vec::new();
    }

    let mut result = Vec::with_capacity(verts.len() + 1);

    for i in 0..verts.len() {
        let v1 = verts[i];
        let v2 = verts[(i + 1) % verts.len()];

        let d1 = v1.dot(plane_normal) - plane_offset;
        let d2 = v2.dot(plane_normal) - plane_offset;

        if d1 <= 0.0 {
            result.push(v1);
        }

        if (d1 > 0.0) != (d2 > 0.0) {
            let t = d1 / (d1 - d2);
            result.push(v1 + (v2 - v1) * t);
        }
    }

    result
}

fn generate_edge_contact(
    box_a: &BoxShape,
    transform_a: &Isometry,
    box_b: &BoxShape,
    transform_b: &Isometry,
    normal: Vec3,
    depth: f32,
) -> Option<ContactManifold> {
    // Find the edges that are closest
    let center_a = transform_a.translation;
    let center_b = transform_b.translation;

    // Simple approximation: use centers offset by normal
    let point_a = center_a + normal * depth * 0.5;
    let point_b = center_b - normal * depth * 0.5;

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

/// Helper trait to create a Vec3 with one component set
trait Vec3Ext {
    fn with(self, index: usize, value: f32) -> Self;
}

impl Vec3Ext for Vec3 {
    fn with(mut self, index: usize, value: f32) -> Self {
        self[index] = value;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sat_boxes_intersecting() {
        let box_a = BoxShape::new(Vec3::ONE);
        let box_b = BoxShape::new(Vec3::ONE);

        let transform_a = Isometry::IDENTITY;
        let transform_b = Isometry::from_translation(Vec3::new(1.5, 0.0, 0.0));

        let result = sat_box_box(&box_a, &transform_a, &box_b, &transform_b);
        assert!(result.is_some());

        let manifold = result.unwrap();
        assert!(!manifold.is_empty());
        assert!((manifold.normal.x.abs() - 1.0).abs() < 0.1);
    }

    #[test]
    fn test_sat_boxes_separated() {
        let box_a = BoxShape::new(Vec3::ONE);
        let box_b = BoxShape::new(Vec3::ONE);

        let transform_a = Isometry::IDENTITY;
        let transform_b = Isometry::from_translation(Vec3::new(3.0, 0.0, 0.0));

        let result = sat_box_box(&box_a, &transform_a, &box_b, &transform_b);
        assert!(result.is_none());
    }
}

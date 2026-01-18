//! GJK (Gilbert-Johnson-Keerthi) algorithm for intersection testing

use nova_math::{Isometry, Vec3, EPSILON};

use crate::shapes::CollisionShape;

const MAX_GJK_ITERATIONS: u32 = 32;

/// Result of GJK intersection test
#[derive(Debug, Clone)]
pub enum GjkResult {
    NoIntersection,
    Intersection(Simplex),
}

/// A simplex used in GJK (1-4 points)
#[derive(Debug, Clone)]
pub struct Simplex {
    pub points: [Vec3; 4],
    pub support_a: [Vec3; 4],
    pub support_b: [Vec3; 4],
    pub size: usize,
}

impl Default for Simplex {
    fn default() -> Self {
        Self::new()
    }
}

impl Simplex {
    pub fn new() -> Self {
        Self {
            points: [Vec3::ZERO; 4],
            support_a: [Vec3::ZERO; 4],
            support_b: [Vec3::ZERO; 4],
            size: 0,
        }
    }

    pub fn push(&mut self, point: Vec3, support_a: Vec3, support_b: Vec3) {
        if self.size < 4 {
            self.points[self.size] = point;
            self.support_a[self.size] = support_a;
            self.support_b[self.size] = support_b;
            self.size += 1;
        }
    }

    pub fn set(&mut self, index: usize, point: Vec3, support_a: Vec3, support_b: Vec3) {
        self.points[index] = point;
        self.support_a[index] = support_a;
        self.support_b[index] = support_b;
    }
}

/// Compute the Minkowski difference support point
fn support(
    shape_a: &CollisionShape,
    transform_a: &Isometry,
    shape_b: &CollisionShape,
    transform_b: &Isometry,
    direction: Vec3,
) -> (Vec3, Vec3, Vec3) {
    let support_a = shape_a.support_point(transform_a, direction);
    let support_b = shape_b.support_point(transform_b, -direction);
    (support_a - support_b, support_a, support_b)
}

/// Run GJK algorithm to test for intersection
pub fn gjk_intersection(
    shape_a: &CollisionShape,
    transform_a: &Isometry,
    shape_b: &CollisionShape,
    transform_b: &Isometry,
) -> GjkResult {
    let mut simplex = Simplex::new();

    // Initial direction: from A's center to B's center
    let mut direction = transform_b.translation - transform_a.translation;
    if direction.length_squared() < EPSILON {
        direction = Vec3::X;
    }

    // Get first support point
    let (point, sa, sb) = support(shape_a, transform_a, shape_b, transform_b, direction);
    simplex.push(point, sa, sb);

    // New search direction towards origin
    direction = -point;

    for _ in 0..MAX_GJK_ITERATIONS {
        // Get new support point
        let (new_point, sa, sb) = support(shape_a, transform_a, shape_b, transform_b, direction);

        // If the new point didn't pass the origin, no intersection
        if new_point.dot(direction) < 0.0 {
            return GjkResult::NoIntersection;
        }

        simplex.push(new_point, sa, sb);

        // Check if simplex contains origin and update simplex/direction
        if do_simplex(&mut simplex, &mut direction) {
            return GjkResult::Intersection(simplex);
        }
    }

    GjkResult::NoIntersection
}

/// Process the simplex and update the search direction
/// Returns true if the simplex contains the origin
fn do_simplex(simplex: &mut Simplex, direction: &mut Vec3) -> bool {
    match simplex.size {
        2 => do_simplex_line(simplex, direction),
        3 => do_simplex_triangle(simplex, direction),
        4 => do_simplex_tetrahedron(simplex, direction),
        _ => false,
    }
}

fn do_simplex_line(simplex: &mut Simplex, direction: &mut Vec3) -> bool {
    let a = simplex.points[1];
    let b = simplex.points[0];

    let ab = b - a;
    let ao = -a;

    if ab.dot(ao) > 0.0 {
        *direction = ab.cross(ao).cross(ab);
    } else {
        simplex.size = 1;
        simplex.points[0] = a;
        simplex.support_a[0] = simplex.support_a[1];
        simplex.support_b[0] = simplex.support_b[1];
        *direction = ao;
    }

    false
}

fn do_simplex_triangle(simplex: &mut Simplex, direction: &mut Vec3) -> bool {
    let a = simplex.points[2];
    let b = simplex.points[1];
    let c = simplex.points[0];

    let ab = b - a;
    let ac = c - a;
    let ao = -a;

    let abc = ab.cross(ac);

    if abc.cross(ac).dot(ao) > 0.0 {
        if ac.dot(ao) > 0.0 {
            // Edge AC
            simplex.points[1] = a;
            simplex.support_a[1] = simplex.support_a[2];
            simplex.support_b[1] = simplex.support_b[2];
            simplex.size = 2;
            *direction = ac.cross(ao).cross(ac);
        } else {
            return handle_line_case(simplex, direction, a, b, ab, ao);
        }
    } else if ab.cross(abc).dot(ao) > 0.0 {
        return handle_line_case(simplex, direction, a, b, ab, ao);
    } else if abc.dot(ao) > 0.0 {
        *direction = abc;
    } else {
        // Swap B and C
        simplex.points[0] = b;
        simplex.points[1] = c;
        let temp_a = simplex.support_a[0];
        let temp_b = simplex.support_b[0];
        simplex.support_a[0] = simplex.support_a[1];
        simplex.support_b[0] = simplex.support_b[1];
        simplex.support_a[1] = temp_a;
        simplex.support_b[1] = temp_b;
        *direction = -abc;
    }

    false
}

fn handle_line_case(
    simplex: &mut Simplex,
    direction: &mut Vec3,
    a: Vec3,
    b: Vec3,
    ab: Vec3,
    ao: Vec3,
) -> bool {
    if ab.dot(ao) > 0.0 {
        simplex.points[0] = b;
        simplex.points[1] = a;
        simplex.support_a[0] = simplex.support_a[1];
        simplex.support_b[0] = simplex.support_b[1];
        simplex.support_a[1] = simplex.support_a[2];
        simplex.support_b[1] = simplex.support_b[2];
        simplex.size = 2;
        *direction = ab.cross(ao).cross(ab);
    } else {
        simplex.points[0] = a;
        simplex.support_a[0] = simplex.support_a[2];
        simplex.support_b[0] = simplex.support_b[2];
        simplex.size = 1;
        *direction = ao;
    }
    false
}

fn do_simplex_tetrahedron(simplex: &mut Simplex, direction: &mut Vec3) -> bool {
    let a = simplex.points[3];
    let b = simplex.points[2];
    let c = simplex.points[1];
    let d = simplex.points[0];

    let ab = b - a;
    let ac = c - a;
    let ad = d - a;
    let ao = -a;

    let abc = ab.cross(ac);
    let acd = ac.cross(ad);
    let adb = ad.cross(ab);

    // Check each face
    if abc.dot(ao) > 0.0 {
        // Origin is on ABC side
        simplex.points[0] = c;
        simplex.points[1] = b;
        simplex.points[2] = a;
        simplex.support_a[0] = simplex.support_a[1];
        simplex.support_b[0] = simplex.support_b[1];
        simplex.support_a[1] = simplex.support_a[2];
        simplex.support_b[1] = simplex.support_b[2];
        simplex.support_a[2] = simplex.support_a[3];
        simplex.support_b[2] = simplex.support_b[3];
        simplex.size = 3;
        return do_simplex_triangle(simplex, direction);
    }

    if acd.dot(ao) > 0.0 {
        // Origin is on ACD side
        simplex.points[0] = d;
        simplex.points[1] = c;
        simplex.points[2] = a;
        simplex.support_a[0] = simplex.support_a[0];
        simplex.support_b[0] = simplex.support_b[0];
        simplex.support_a[1] = simplex.support_a[1];
        simplex.support_b[1] = simplex.support_b[1];
        simplex.support_a[2] = simplex.support_a[3];
        simplex.support_b[2] = simplex.support_b[3];
        simplex.size = 3;
        return do_simplex_triangle(simplex, direction);
    }

    if adb.dot(ao) > 0.0 {
        // Origin is on ADB side
        simplex.points[0] = b;
        simplex.points[1] = d;
        simplex.points[2] = a;
        simplex.support_a[0] = simplex.support_a[2];
        simplex.support_b[0] = simplex.support_b[2];
        simplex.support_a[1] = simplex.support_a[0];
        simplex.support_b[1] = simplex.support_b[0];
        simplex.support_a[2] = simplex.support_a[3];
        simplex.support_b[2] = simplex.support_b[3];
        simplex.size = 3;
        return do_simplex_triangle(simplex, direction);
    }

    // Origin is inside tetrahedron
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shapes::SphereShape;

    #[test]
    fn test_gjk_spheres_intersecting() {
        let sphere_a = CollisionShape::Sphere(SphereShape::new(1.0));
        let sphere_b = CollisionShape::Sphere(SphereShape::new(1.0));

        let transform_a = Isometry::IDENTITY;
        let transform_b = Isometry::from_translation(Vec3::new(1.5, 0.0, 0.0));

        let result = gjk_intersection(&sphere_a, &transform_a, &sphere_b, &transform_b);
        assert!(matches!(result, GjkResult::Intersection(_)));
    }

    #[test]
    fn test_gjk_spheres_not_intersecting() {
        let sphere_a = CollisionShape::Sphere(SphereShape::new(1.0));
        let sphere_b = CollisionShape::Sphere(SphereShape::new(1.0));

        let transform_a = Isometry::IDENTITY;
        let transform_b = Isometry::from_translation(Vec3::new(3.0, 0.0, 0.0));

        let result = gjk_intersection(&sphere_a, &transform_a, &sphere_b, &transform_b);
        assert!(matches!(result, GjkResult::NoIntersection));
    }
}

//! AABB - Axis-Aligned Bounding Box

use glam::Vec3;

/// An axis-aligned bounding box defined by min and max corners.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    /// Minimum corner
    pub min: Vec3,
    /// Maximum corner
    pub max: Vec3,
}

impl Default for Aabb {
    fn default() -> Self {
        Self::EMPTY
    }
}

impl Aabb {
    /// An invalid/empty AABB
    pub const EMPTY: Self = Self {
        min: Vec3::splat(f32::INFINITY),
        max: Vec3::splat(f32::NEG_INFINITY),
    };

    /// Create a new AABB from min and max corners
    #[inline]
    pub fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    /// Create an AABB from center and half-extents
    #[inline]
    pub fn from_center_half_extents(center: Vec3, half_extents: Vec3) -> Self {
        Self {
            min: center - half_extents,
            max: center + half_extents,
        }
    }

    /// Create an AABB containing a single point
    #[inline]
    pub fn from_point(point: Vec3) -> Self {
        Self {
            min: point,
            max: point,
        }
    }

    /// Create an AABB containing a set of points
    pub fn from_points(points: &[Vec3]) -> Self {
        let mut aabb = Self::EMPTY;
        for &point in points {
            aabb = aabb.expand_to_include_point(point);
        }
        aabb
    }

    /// Check if this AABB is valid (min <= max for all axes)
    #[inline]
    pub fn is_valid(&self) -> bool {
        self.min.x <= self.max.x && self.min.y <= self.max.y && self.min.z <= self.max.z
    }

    /// Get the center of the AABB
    #[inline]
    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    /// Get the half-extents (half the size on each axis)
    #[inline]
    pub fn half_extents(&self) -> Vec3 {
        (self.max - self.min) * 0.5
    }

    /// Get the full size of the AABB
    #[inline]
    pub fn size(&self) -> Vec3 {
        self.max - self.min
    }

    /// Get the volume of the AABB
    #[inline]
    pub fn volume(&self) -> f32 {
        let size = self.size();
        size.x * size.y * size.z
    }

    /// Get the surface area of the AABB (useful for BVH heuristics)
    #[inline]
    pub fn surface_area(&self) -> f32 {
        let size = self.size();
        2.0 * (size.x * size.y + size.y * size.z + size.z * size.x)
    }

    /// Check if this AABB contains a point
    #[inline]
    pub fn contains_point(&self, point: Vec3) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }

    /// Check if this AABB fully contains another AABB
    #[inline]
    pub fn contains_aabb(&self, other: &Aabb) -> bool {
        self.min.x <= other.min.x
            && self.min.y <= other.min.y
            && self.min.z <= other.min.z
            && self.max.x >= other.max.x
            && self.max.y >= other.max.y
            && self.max.z >= other.max.z
    }

    /// Check if this AABB intersects another AABB
    #[inline]
    pub fn intersects(&self, other: &Aabb) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    /// Compute the intersection of two AABBs
    #[inline]
    pub fn intersection(&self, other: &Aabb) -> Self {
        Self {
            min: self.min.max(other.min),
            max: self.max.min(other.max),
        }
    }

    /// Merge two AABBs into one that contains both
    #[inline]
    pub fn merge(&self, other: &Aabb) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    /// Expand the AABB to include a point
    #[inline]
    pub fn expand_to_include_point(&self, point: Vec3) -> Self {
        Self {
            min: self.min.min(point),
            max: self.max.max(point),
        }
    }

    /// Expand the AABB by a margin on all sides
    #[inline]
    pub fn expand_by(&self, margin: f32) -> Self {
        Self {
            min: self.min - Vec3::splat(margin),
            max: self.max + Vec3::splat(margin),
        }
    }

    /// Translate the AABB
    #[inline]
    pub fn translate(&self, offset: Vec3) -> Self {
        Self {
            min: self.min + offset,
            max: self.max + offset,
        }
    }

    /// Ray intersection test
    /// Returns Some((t_min, t_max)) if ray intersects, None otherwise
    /// Ray is defined as: origin + direction * t
    #[inline]
    pub fn ray_intersection(&self, origin: Vec3, inv_direction: Vec3) -> Option<(f32, f32)> {
        let t1 = (self.min - origin) * inv_direction;
        let t2 = (self.max - origin) * inv_direction;

        let t_min = t1.min(t2);
        let t_max = t1.max(t2);

        let t_enter = t_min.x.max(t_min.y).max(t_min.z);
        let t_exit = t_max.x.min(t_max.y).min(t_max.z);

        if t_enter <= t_exit && t_exit >= 0.0 {
            Some((t_enter.max(0.0), t_exit))
        } else {
            None
        }
    }

    /// Get the closest point on the AABB to a given point
    #[inline]
    pub fn closest_point(&self, point: Vec3) -> Vec3 {
        point.clamp(self.min, self.max)
    }

    /// Get the squared distance from a point to the AABB
    #[inline]
    pub fn distance_squared_to_point(&self, point: Vec3) -> f32 {
        let closest = self.closest_point(point);
        (point - closest).length_squared()
    }

    /// Get the longest axis (0=x, 1=y, 2=z)
    #[inline]
    pub fn longest_axis(&self) -> usize {
        let size = self.size();
        if size.x >= size.y && size.x >= size.z {
            0
        } else if size.y >= size.z {
            1
        } else {
            2
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_intersection() {
        let a = Aabb::new(Vec3::ZERO, Vec3::ONE);
        let b = Aabb::new(Vec3::splat(0.5), Vec3::splat(1.5));
        assert!(a.intersects(&b));
        assert!(b.intersects(&a));

        let c = Aabb::new(Vec3::splat(2.0), Vec3::splat(3.0));
        assert!(!a.intersects(&c));
    }

    #[test]
    fn test_contains() {
        let outer = Aabb::new(Vec3::ZERO, Vec3::splat(10.0));
        let inner = Aabb::new(Vec3::ONE, Vec3::splat(5.0));
        assert!(outer.contains_aabb(&inner));
        assert!(!inner.contains_aabb(&outer));
    }

    #[test]
    fn test_ray() {
        let aabb = Aabb::new(Vec3::ZERO, Vec3::ONE);
        let origin = Vec3::new(-1.0, 0.5, 0.5);
        let direction = Vec3::X;
        let inv_dir = Vec3::ONE / direction;

        let hit = aabb.ray_intersection(origin, inv_dir);
        assert!(hit.is_some());
        let (t_min, t_max) = hit.unwrap();
        assert!((t_min - 1.0).abs() < 1e-5);
        assert!((t_max - 2.0).abs() < 1e-5);
    }
}

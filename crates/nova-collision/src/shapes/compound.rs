//! Compound collision shape (multiple shapes combined)

use nova_math::{Aabb, Isometry, Mat3, Vec3};

use super::{CollisionShape, SupportMap};

/// A child shape within a compound
#[derive(Debug, Clone)]
pub struct CompoundChild {
    /// Local transform relative to the compound's origin
    pub local_transform: Isometry,
    /// The shape
    pub shape: CollisionShape,
}

/// A compound collision shape made of multiple child shapes
#[derive(Debug, Clone)]
pub struct CompoundShape {
    children: Vec<CompoundChild>,
    local_aabb: Aabb,
    center: Vec3,
}

impl CompoundShape {
    /// Create a new empty compound shape
    pub fn new() -> Self {
        Self {
            children: Vec::new(),
            local_aabb: Aabb::EMPTY,
            center: Vec3::ZERO,
        }
    }

    /// Create a compound shape from children
    pub fn from_children(children: Vec<CompoundChild>) -> Self {
        let mut compound = Self::new();
        for child in children {
            compound.add_child(child.local_transform, child.shape);
        }
        compound
    }

    /// Add a child shape
    pub fn add_child(&mut self, local_transform: Isometry, shape: CollisionShape) {
        let child_aabb = shape.compute_aabb(&local_transform);
        self.local_aabb = if self.children.is_empty() {
            child_aabb
        } else {
            self.local_aabb.merge(&child_aabb)
        };

        self.children.push(CompoundChild {
            local_transform,
            shape,
        });

        // Recompute center
        self.center = self
            .children
            .iter()
            .map(|c| c.local_transform.translation)
            .fold(Vec3::ZERO, |a, b| a + b)
            / self.children.len() as f32;
    }

    /// Get the children
    pub fn children(&self) -> &[CompoundChild] {
        &self.children
    }

    /// Get the number of children
    pub fn len(&self) -> usize {
        self.children.len()
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.children.is_empty()
    }

    /// Get center of mass
    pub fn center_of_mass(&self) -> Vec3 {
        self.center
    }

    /// Compute the AABB
    pub fn compute_aabb(&self, transform: &Isometry) -> Aabb {
        let mut aabb = Aabb::EMPTY;
        for child in &self.children {
            let child_world_transform = transform.mul(&child.local_transform);
            let child_aabb = child.shape.compute_aabb(&child_world_transform);
            aabb = aabb.merge(&child_aabb);
        }
        aabb
    }

    /// Compute approximate inertia (sum of child inertias)
    pub fn compute_inertia(&self, mass: f32) -> Mat3 {
        if self.children.is_empty() {
            return Mat3::ZERO;
        }

        let mass_per_child = mass / self.children.len() as f32;
        let mut total_inertia = Mat3::ZERO;

        for child in &self.children {
            let child_inertia = child.shape.compute_inertia(mass_per_child);
            // Apply parallel axis theorem for offset
            let offset = child.local_transform.translation;
            let offset_sq = offset.length_squared();
            let parallel_axis = Mat3::from_diagonal(Vec3::splat(offset_sq))
                - Mat3::from_cols(
                    offset * offset.x,
                    offset * offset.y,
                    offset * offset.z,
                );
            total_inertia =
                total_inertia + child_inertia + parallel_axis * mass_per_child;
        }

        total_inertia
    }

    /// Get support point by finding the best among all children
    pub fn support_point(&self, transform: &Isometry, direction: Vec3) -> Vec3 {
        let mut best_point = Vec3::ZERO;
        let mut best_dot = f32::NEG_INFINITY;

        for child in &self.children {
            let child_world_transform = transform.mul(&child.local_transform);
            let support = child.shape.support_point(&child_world_transform, direction);
            let dot = direction.dot(support);
            if dot > best_dot {
                best_dot = dot;
                best_point = support;
            }
        }

        best_point
    }
}

impl Default for CompoundShape {
    fn default() -> Self {
        Self::new()
    }
}

impl PartialEq for CompoundShape {
    fn eq(&self, other: &Self) -> bool {
        if self.children.len() != other.children.len() {
            return false;
        }
        // Simplified equality check
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shapes::SphereShape;

    #[test]
    fn test_compound_aabb() {
        let mut compound = CompoundShape::new();

        compound.add_child(
            Isometry::from_translation(Vec3::new(-1.0, 0.0, 0.0)),
            CollisionShape::Sphere(SphereShape::new(0.5)),
        );
        compound.add_child(
            Isometry::from_translation(Vec3::new(1.0, 0.0, 0.0)),
            CollisionShape::Sphere(SphereShape::new(0.5)),
        );

        let aabb = compound.compute_aabb(&Isometry::IDENTITY);

        assert!((aabb.min.x - (-1.5)).abs() < 1e-5);
        assert!((aabb.max.x - 1.5).abs() < 1e-5);
    }
}

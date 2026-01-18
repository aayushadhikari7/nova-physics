//! Contact manifold and contact points

use nova_core::{ColliderHandle, RigidBodyHandle};
use nova_math::Vec3;
use smallvec::SmallVec;

/// Maximum number of contact points in a manifold
pub const MAX_CONTACTS: usize = 4;

/// A single contact point
#[derive(Debug, Clone, Copy)]
pub struct ContactPoint {
    /// Contact point in local space of body A
    pub local_a: Vec3,
    /// Contact point in local space of body B
    pub local_b: Vec3,
    /// Contact point in world space on body A
    pub world_a: Vec3,
    /// Contact point in world space on body B
    pub world_b: Vec3,
    /// Penetration depth (positive = penetrating)
    pub depth: f32,
    /// Cached normal impulse for warm starting
    pub normal_impulse: f32,
    /// Cached tangent impulses for warm starting
    pub tangent_impulse: [f32; 2],
}

impl Default for ContactPoint {
    fn default() -> Self {
        Self {
            local_a: Vec3::ZERO,
            local_b: Vec3::ZERO,
            world_a: Vec3::ZERO,
            world_b: Vec3::ZERO,
            depth: 0.0,
            normal_impulse: 0.0,
            tangent_impulse: [0.0, 0.0],
        }
    }
}

/// A contact manifold containing up to MAX_CONTACTS contact points
#[derive(Debug, Clone)]
pub struct ContactManifold {
    /// Collider A handle
    pub collider_a: ColliderHandle,
    /// Collider B handle
    pub collider_b: ColliderHandle,
    /// Body A handle
    pub body_a: RigidBodyHandle,
    /// Body B handle
    pub body_b: RigidBodyHandle,
    /// Contact normal (from A to B)
    pub normal: Vec3,
    /// Contact points
    pub contacts: SmallVec<[ContactPoint; MAX_CONTACTS]>,
    /// Combined friction
    pub friction: f32,
    /// Combined restitution
    pub restitution: f32,
}

impl Default for ContactManifold {
    fn default() -> Self {
        Self::new()
    }
}

impl ContactManifold {
    pub fn new() -> Self {
        Self {
            collider_a: ColliderHandle::default(),
            collider_b: ColliderHandle::default(),
            body_a: RigidBodyHandle::default(),
            body_b: RigidBodyHandle::default(),
            normal: Vec3::Y,
            contacts: SmallVec::new(),
            friction: 0.5,
            restitution: 0.0,
        }
    }

    /// Add a contact point, potentially replacing the worst one if full
    pub fn add_contact(&mut self, contact: ContactPoint) {
        if self.contacts.len() < MAX_CONTACTS {
            self.contacts.push(contact);
        } else {
            // Find the contact that results in the largest area when removed
            // and replace it if the new contact would give a better manifold
            let mut worst_idx = 0;
            let mut worst_area = f32::MAX;

            for i in 0..self.contacts.len() {
                let area = self.manifold_area_without(i, &contact);
                if area < worst_area {
                    worst_area = area;
                    worst_idx = i;
                }
            }

            let new_area = self.manifold_area_without(worst_idx, &contact);
            if new_area > worst_area {
                self.contacts[worst_idx] = contact;
            }
        }
    }

    /// Calculate the area of the manifold if we removed contact at index and added new_contact
    fn manifold_area_without(&self, exclude_idx: usize, new_contact: &ContactPoint) -> f32 {
        let mut points: SmallVec<[Vec3; MAX_CONTACTS]> = SmallVec::new();

        for (i, c) in self.contacts.iter().enumerate() {
            if i != exclude_idx {
                points.push(c.world_a);
            }
        }
        points.push(new_contact.world_a);

        // Compute approximate area (sum of cross products)
        if points.len() < 3 {
            return 0.0;
        }

        let center = points.iter().fold(Vec3::ZERO, |a, &b| a + b) / points.len() as f32;

        points
            .windows(2)
            .map(|w| {
                let v1 = w[0] - center;
                let v2 = w[1] - center;
                v1.cross(v2).length() * 0.5
            })
            .sum()
    }

    /// Flip the manifold (swap A and B)
    pub fn flip(mut self) -> Self {
        std::mem::swap(&mut self.collider_a, &mut self.collider_b);
        std::mem::swap(&mut self.body_a, &mut self.body_b);
        self.normal = -self.normal;

        for contact in &mut self.contacts {
            std::mem::swap(&mut contact.local_a, &mut contact.local_b);
            std::mem::swap(&mut contact.world_a, &mut contact.world_b);
        }

        self
    }

    /// Copy cached impulses from a previous manifold for warm starting
    pub fn warm_start(&mut self, old: &ContactManifold) {
        const MATCH_THRESHOLD: f32 = 0.01; // 1cm threshold

        for contact in &mut self.contacts {
            // Find matching contact in old manifold
            for old_contact in &old.contacts {
                let dist_a = (contact.local_a - old_contact.local_a).length_squared();
                let dist_b = (contact.local_b - old_contact.local_b).length_squared();

                if dist_a < MATCH_THRESHOLD && dist_b < MATCH_THRESHOLD {
                    contact.normal_impulse = old_contact.normal_impulse;
                    contact.tangent_impulse = old_contact.tangent_impulse;
                    break;
                }
            }
        }
    }

    /// Clear all contacts
    pub fn clear(&mut self) {
        self.contacts.clear();
    }

    /// Get the deepest penetration depth
    pub fn max_depth(&self) -> f32 {
        self.contacts
            .iter()
            .map(|c| c.depth)
            .fold(0.0, f32::max)
    }

    /// Get number of contacts
    pub fn len(&self) -> usize {
        self.contacts.len()
    }

    /// Check if manifold is empty
    pub fn is_empty(&self) -> bool {
        self.contacts.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_contact_manifold() {
        let mut manifold = ContactManifold::new();

        for i in 0..5 {
            manifold.add_contact(ContactPoint {
                world_a: Vec3::new(i as f32, 0.0, 0.0),
                depth: 0.1,
                ..Default::default()
            });
        }

        // Should have at most MAX_CONTACTS
        assert!(manifold.len() <= MAX_CONTACTS);
    }
}

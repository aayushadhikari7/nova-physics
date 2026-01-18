//! Dynamic Bounding Volume Hierarchy for broad phase

use hashbrown::HashMap;
use nova_core::ColliderHandle;
use nova_math::Aabb;
use slotmap::Key;
use smallvec::SmallVec;

use super::BroadPhasePair;

/// Fat AABB margin for reducing updates
const FAT_AABB_MARGIN: f32 = 0.1;

/// A node in the BVH tree
#[derive(Debug, Clone)]
pub struct BvhNode {
    /// The AABB for this node (fat AABB for leaves)
    pub aabb: Aabb,
    /// Parent node index (None for root)
    pub parent: Option<usize>,
    /// Node data
    pub data: BvhNodeData,
    /// Height of the subtree
    pub height: i32,
}

#[derive(Debug, Clone)]
pub enum BvhNodeData {
    Internal {
        left: usize,
        right: usize,
    },
    Leaf {
        collider: ColliderHandle,
        tight_aabb: Aabb,
    },
}

impl BvhNode {
    fn is_leaf(&self) -> bool {
        matches!(self.data, BvhNodeData::Leaf { .. })
    }
}

/// The broad phase collision detection system using a dynamic BVH
#[derive(Debug)]
pub struct BroadPhase {
    nodes: Vec<Option<BvhNode>>,
    root: Option<usize>,
    free_list: Vec<usize>,
    collider_to_node: HashMap<ColliderHandle, usize>,
}

impl Default for BroadPhase {
    fn default() -> Self {
        Self::new()
    }
}

impl BroadPhase {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            root: None,
            free_list: Vec::new(),
            collider_to_node: HashMap::new(),
        }
    }

    /// Insert a collider into the broad phase
    pub fn insert(&mut self, handle: ColliderHandle, aabb: Aabb) {
        let fat_aabb = aabb.expand_by(FAT_AABB_MARGIN);

        let node_idx = self.allocate_node();
        self.nodes[node_idx] = Some(BvhNode {
            aabb: fat_aabb,
            parent: None,
            data: BvhNodeData::Leaf {
                collider: handle,
                tight_aabb: aabb,
            },
            height: 0,
        });

        self.collider_to_node.insert(handle, node_idx);
        self.insert_leaf(node_idx);
    }

    /// Remove a collider from the broad phase
    pub fn remove(&mut self, handle: ColliderHandle) {
        if let Some(node_idx) = self.collider_to_node.remove(&handle) {
            self.remove_leaf(node_idx);
            self.free_node(node_idx);
        }
    }

    /// Update a collider's AABB
    pub fn update(&mut self, handle: ColliderHandle, aabb: Aabb) -> bool {
        let Some(&node_idx) = self.collider_to_node.get(&handle) else {
            return false;
        };

        let node = self.nodes[node_idx].as_ref().unwrap();

        // Check if the new tight AABB fits within the fat AABB
        if node.aabb.contains_aabb(&aabb) {
            // Still fits within fat AABB, just update tight AABB
            if let Some(node) = &mut self.nodes[node_idx] {
                if let BvhNodeData::Leaf { tight_aabb, .. } = &mut node.data {
                    *tight_aabb = aabb;
                }
            }
            return false;
        }

        // Need to reinsert - tight AABB no longer fits within fat AABB
        self.remove_leaf(node_idx);

        let fat_aabb = aabb.expand_by(FAT_AABB_MARGIN);
        if let Some(node) = &mut self.nodes[node_idx] {
            node.aabb = fat_aabb;
            if let BvhNodeData::Leaf { tight_aabb, .. } = &mut node.data {
                *tight_aabb = aabb;
            }
        }

        self.insert_leaf(node_idx);
        true
    }

    /// Query all potential collision pairs
    pub fn compute_pairs(&self) -> Vec<BroadPhasePair> {
        let mut pairs = Vec::new();

        if self.root.is_none() {
            return pairs;
        }

        // For each leaf, query the tree for overlapping leaves
        for (_, &node_idx) in &self.collider_to_node {
            let node = self.nodes[node_idx].as_ref().unwrap();
            let aabb = &node.aabb;

            if let BvhNodeData::Leaf { collider: handle_a, .. } = &node.data {
                let mut stack: SmallVec<[usize; 64]> = SmallVec::new();
                if let Some(root) = self.root {
                    stack.push(root);
                }

                while let Some(current) = stack.pop() {
                    if current == node_idx {
                        continue;
                    }

                    let current_node = self.nodes[current].as_ref().unwrap();

                    if !aabb.intersects(&current_node.aabb) {
                        continue;
                    }

                    match &current_node.data {
                        BvhNodeData::Leaf { collider: handle_b, .. } => {
                            // Only add pair once (when a < b)
                            let a_data = Key::data(handle_a);
                            let b_data = Key::data(handle_b);

                            if a_data.as_ffi() < b_data.as_ffi() {
                                pairs.push(BroadPhasePair::new(*handle_a, *handle_b));
                            }
                        }
                        BvhNodeData::Internal { left, right } => {
                            stack.push(*left);
                            stack.push(*right);
                        }
                    }
                }
            }
        }

        pairs
    }

    /// Query colliders that overlap with an AABB
    pub fn query_aabb(&self, aabb: &Aabb) -> Vec<ColliderHandle> {
        let mut result = Vec::new();

        let mut stack: SmallVec<[usize; 64]> = SmallVec::new();
        if let Some(root) = self.root {
            stack.push(root);
        }

        while let Some(current) = stack.pop() {
            let node = self.nodes[current].as_ref().unwrap();

            if !aabb.intersects(&node.aabb) {
                continue;
            }

            match &node.data {
                BvhNodeData::Leaf { collider, tight_aabb } => {
                    if aabb.intersects(tight_aabb) {
                        result.push(*collider);
                    }
                }
                BvhNodeData::Internal { left, right } => {
                    stack.push(*left);
                    stack.push(*right);
                }
            }
        }

        result
    }

    /// Query colliders along a ray
    pub fn query_ray(
        &self,
        origin: nova_math::Vec3,
        direction: nova_math::Vec3,
        max_dist: f32,
    ) -> Vec<ColliderHandle> {
        let mut result = Vec::new();

        let inv_dir = nova_math::Vec3::new(
            if direction.x.abs() > 1e-10 { 1.0 / direction.x } else { f32::MAX },
            if direction.y.abs() > 1e-10 { 1.0 / direction.y } else { f32::MAX },
            if direction.z.abs() > 1e-10 { 1.0 / direction.z } else { f32::MAX },
        );

        let mut stack: SmallVec<[usize; 64]> = SmallVec::new();
        if let Some(root) = self.root {
            stack.push(root);
        }

        while let Some(current) = stack.pop() {
            let node = self.nodes[current].as_ref().unwrap();

            if node.aabb.ray_intersection(origin, inv_dir).is_none() {
                continue;
            }

            match &node.data {
                BvhNodeData::Leaf { collider, tight_aabb } => {
                    if tight_aabb.ray_intersection(origin, inv_dir).is_some() {
                        result.push(*collider);
                    }
                }
                BvhNodeData::Internal { left, right } => {
                    stack.push(*left);
                    stack.push(*right);
                }
            }
        }

        result
    }

    /// Get the number of colliders in the broad phase
    pub fn len(&self) -> usize {
        self.collider_to_node.len()
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.collider_to_node.is_empty()
    }

    /// Clear all colliders
    pub fn clear(&mut self) {
        self.nodes.clear();
        self.root = None;
        self.free_list.clear();
        self.collider_to_node.clear();
    }

    // Internal methods

    fn allocate_node(&mut self) -> usize {
        if let Some(idx) = self.free_list.pop() {
            idx
        } else {
            let idx = self.nodes.len();
            self.nodes.push(None);
            idx
        }
    }

    fn free_node(&mut self, idx: usize) {
        self.nodes[idx] = None;
        self.free_list.push(idx);
    }

    fn insert_leaf(&mut self, leaf_idx: usize) {
        if self.root.is_none() {
            self.root = Some(leaf_idx);
            return;
        }

        let leaf_aabb = self.nodes[leaf_idx].as_ref().unwrap().aabb;

        // Find the best sibling
        let mut best_sibling = self.root.unwrap();
        let mut best_cost = self.nodes[best_sibling]
            .as_ref()
            .unwrap()
            .aabb
            .merge(&leaf_aabb)
            .surface_area();

        let mut stack: SmallVec<[(usize, f32); 64]> = SmallVec::new();
        stack.push((self.root.unwrap(), 0.0));

        while let Some((current, inherited_cost)) = stack.pop() {
            let node = self.nodes[current].as_ref().unwrap();
            let combined_aabb = node.aabb.merge(&leaf_aabb);
            let direct_cost = combined_aabb.surface_area();
            let cost = direct_cost + inherited_cost;

            if cost < best_cost {
                best_cost = cost;
                best_sibling = current;
            }

            // Lower bound for children
            let lower_bound = leaf_aabb.surface_area() + inherited_cost + direct_cost - node.aabb.surface_area();

            if lower_bound < best_cost {
                if let BvhNodeData::Internal { left, right } = &node.data {
                    let new_inherited = inherited_cost + direct_cost - node.aabb.surface_area();
                    stack.push((*left, new_inherited));
                    stack.push((*right, new_inherited));
                }
            }
        }

        // Create new parent
        let old_parent = self.nodes[best_sibling].as_ref().unwrap().parent;
        let new_parent_idx = self.allocate_node();

        let combined_aabb = self.nodes[best_sibling]
            .as_ref()
            .unwrap()
            .aabb
            .merge(&leaf_aabb);

        self.nodes[new_parent_idx] = Some(BvhNode {
            aabb: combined_aabb,
            parent: old_parent,
            data: BvhNodeData::Internal {
                left: best_sibling,
                right: leaf_idx,
            },
            height: self.nodes[best_sibling].as_ref().unwrap().height + 1,
        });

        if let Some(node) = &mut self.nodes[best_sibling] {
            node.parent = Some(new_parent_idx);
        }
        if let Some(node) = &mut self.nodes[leaf_idx] {
            node.parent = Some(new_parent_idx);
        }

        if let Some(parent_idx) = old_parent {
            if let Some(parent) = &mut self.nodes[parent_idx] {
                if let BvhNodeData::Internal { left, right } = &mut parent.data {
                    if *left == best_sibling {
                        *left = new_parent_idx;
                    } else {
                        *right = new_parent_idx;
                    }
                }
            }
        } else {
            self.root = Some(new_parent_idx);
        }

        // Refit ancestors
        self.refit_ancestors(new_parent_idx);
    }

    fn remove_leaf(&mut self, leaf_idx: usize) {
        if self.root == Some(leaf_idx) {
            self.root = None;
            return;
        }

        let parent_idx = self.nodes[leaf_idx].as_ref().unwrap().parent.unwrap();
        let grandparent = self.nodes[parent_idx].as_ref().unwrap().parent;

        // Find sibling
        let sibling_idx = if let BvhNodeData::Internal { left, right } =
            &self.nodes[parent_idx].as_ref().unwrap().data
        {
            if *left == leaf_idx {
                *right
            } else {
                *left
            }
        } else {
            return;
        };

        if let Some(grandparent_idx) = grandparent {
            // Replace parent with sibling in grandparent
            if let Some(gp) = &mut self.nodes[grandparent_idx] {
                if let BvhNodeData::Internal { left, right } = &mut gp.data {
                    if *left == parent_idx {
                        *left = sibling_idx;
                    } else {
                        *right = sibling_idx;
                    }
                }
            }
            if let Some(sibling) = &mut self.nodes[sibling_idx] {
                sibling.parent = Some(grandparent_idx);
            }

            self.free_node(parent_idx);
            self.refit_ancestors(grandparent_idx);
        } else {
            // Parent was root
            self.root = Some(sibling_idx);
            if let Some(sibling) = &mut self.nodes[sibling_idx] {
                sibling.parent = None;
            }
            self.free_node(parent_idx);
        }
    }

    fn refit_ancestors(&mut self, mut idx: usize) {
        while let Some(node) = &self.nodes[idx] {
            let parent = node.parent;

            if let BvhNodeData::Internal { left, right } = &node.data {
                let left_node = self.nodes[*left].as_ref().unwrap();
                let right_node = self.nodes[*right].as_ref().unwrap();

                let combined = left_node.aabb.merge(&right_node.aabb);
                let height = left_node.height.max(right_node.height) + 1;

                if let Some(node) = &mut self.nodes[idx] {
                    node.aabb = combined;
                    node.height = height;
                }
            }

            if let Some(parent_idx) = parent {
                idx = parent_idx;
            } else {
                break;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nova_math::Vec3;
    use slotmap::SlotMap;

    #[test]
    fn test_broad_phase_pairs() {
        let mut handles: SlotMap<ColliderHandle, ()> = SlotMap::with_key();
        let h1 = handles.insert(());
        let h2 = handles.insert(());
        let h3 = handles.insert(());

        let mut bp = BroadPhase::new();

        // Two overlapping boxes
        bp.insert(h1, Aabb::new(Vec3::ZERO, Vec3::ONE));
        bp.insert(h2, Aabb::new(Vec3::splat(0.5), Vec3::splat(1.5)));

        // One separate box
        bp.insert(h3, Aabb::new(Vec3::splat(10.0), Vec3::splat(11.0)));

        let pairs = bp.compute_pairs();

        // Should have exactly one pair (h1, h2)
        assert_eq!(pairs.len(), 1);
    }

    #[test]
    fn test_broad_phase_update() {
        let mut handles: SlotMap<ColliderHandle, ()> = SlotMap::with_key();
        let h1 = handles.insert(());

        let mut bp = BroadPhase::new();
        bp.insert(h1, Aabb::new(Vec3::ZERO, Vec3::ONE));

        // Small movement within fat margin - should not reinsert
        // Fat margin is 0.1, so moving within that range keeps the tight AABB inside fat AABB
        let changed = bp.update(h1, Aabb::new(Vec3::splat(0.05), Vec3::splat(1.05)));
        assert!(!changed, "Small movement within fat margin should not trigger reinsert");

        // Large movement - should reinsert (tight AABB no longer fits in fat AABB)
        let changed = bp.update(h1, Aabb::new(Vec3::splat(5.0), Vec3::splat(6.0)));
        assert!(changed, "Large movement should trigger reinsert");
    }
}

//! Island detection for parallel solving and sleeping

use hashbrown::{HashMap, HashSet};
use nova_core::{IslandHandle, RigidBodyHandle};
use smallvec::SmallVec;
use slotmap::SlotMap;

/// An island of interconnected bodies
#[derive(Debug, Clone)]
pub struct Island {
    pub bodies: Vec<RigidBodyHandle>,
    pub is_sleeping: bool,
}

impl Island {
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            is_sleeping: false,
        }
    }
}

impl Default for Island {
    fn default() -> Self {
        Self::new()
    }
}

/// Manages island detection and updates
#[derive(Debug)]
pub struct IslandManager {
    /// Islands storage
    islands: SlotMap<IslandHandle, Island>,
    /// Body to island mapping
    body_to_island: HashMap<RigidBodyHandle, IslandHandle>,
    /// Contact graph edges (body pairs that are in contact)
    contact_graph: HashMap<RigidBodyHandle, HashSet<RigidBodyHandle>>,
}

impl Default for IslandManager {
    fn default() -> Self {
        Self::new()
    }
}

impl IslandManager {
    pub fn new() -> Self {
        Self {
            islands: SlotMap::with_key(),
            body_to_island: HashMap::new(),
            contact_graph: HashMap::new(),
        }
    }

    /// Clear all island data
    pub fn clear(&mut self) {
        self.islands.clear();
        self.body_to_island.clear();
        self.contact_graph.clear();
    }

    /// Add a contact edge between two bodies
    pub fn add_contact(&mut self, body_a: RigidBodyHandle, body_b: RigidBodyHandle) {
        self.contact_graph
            .entry(body_a)
            .or_default()
            .insert(body_b);
        self.contact_graph
            .entry(body_b)
            .or_default()
            .insert(body_a);
    }

    /// Remove a contact edge
    pub fn remove_contact(&mut self, body_a: RigidBodyHandle, body_b: RigidBodyHandle) {
        if let Some(neighbors) = self.contact_graph.get_mut(&body_a) {
            neighbors.remove(&body_b);
        }
        if let Some(neighbors) = self.contact_graph.get_mut(&body_b) {
            neighbors.remove(&body_a);
        }
    }

    /// Remove a body from the island system
    pub fn remove_body(&mut self, body: RigidBodyHandle) {
        self.contact_graph.remove(&body);
        for neighbors in self.contact_graph.values_mut() {
            neighbors.remove(&body);
        }
        self.body_to_island.remove(&body);
    }

    /// Rebuild islands from the contact graph
    pub fn rebuild_islands<F>(&mut self, bodies: impl Iterator<Item = RigidBodyHandle>, is_static: F)
    where
        F: Fn(RigidBodyHandle) -> bool,
    {
        self.islands.clear();
        self.body_to_island.clear();

        let all_bodies: Vec<_> = bodies.collect();
        let mut visited: HashSet<RigidBodyHandle> = HashSet::new();

        for &body in &all_bodies {
            if visited.contains(&body) || is_static(body) {
                continue;
            }

            // BFS to find connected component
            let mut island = Island::new();
            let mut queue: SmallVec<[RigidBodyHandle; 64]> = SmallVec::new();
            queue.push(body);
            visited.insert(body);

            while let Some(current) = queue.pop() {
                island.bodies.push(current);

                if let Some(neighbors) = self.contact_graph.get(&current) {
                    for &neighbor in neighbors {
                        if !visited.contains(&neighbor) && !is_static(neighbor) {
                            visited.insert(neighbor);
                            queue.push(neighbor);
                        }
                    }
                }
            }

            let island_handle = self.islands.insert(island);
            for &body in &self.islands[island_handle].bodies {
                self.body_to_island.insert(body, island_handle);
            }
        }
    }

    /// Get all islands
    pub fn islands(&self) -> impl Iterator<Item = (IslandHandle, &Island)> {
        self.islands.iter()
    }

    /// Get a specific island
    pub fn get_island(&self, handle: IslandHandle) -> Option<&Island> {
        self.islands.get(handle)
    }

    /// Get the island a body belongs to
    pub fn get_body_island(&self, body: RigidBodyHandle) -> Option<IslandHandle> {
        self.body_to_island.get(&body).copied()
    }

    /// Get the number of islands
    pub fn island_count(&self) -> usize {
        self.islands.len()
    }

    /// Mark an island as sleeping
    pub fn set_island_sleeping(&mut self, handle: IslandHandle, sleeping: bool) {
        if let Some(island) = self.islands.get_mut(handle) {
            island.is_sleeping = sleeping;
        }
    }

    /// Wake up all islands containing the given body
    pub fn wake_island_with_body(&mut self, body: RigidBodyHandle) {
        if let Some(&island_handle) = self.body_to_island.get(&body) {
            if let Some(island) = self.islands.get_mut(island_handle) {
                island.is_sleeping = false;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use slotmap::SlotMap;

    #[test]
    fn test_island_building() {
        let mut body_handles: SlotMap<RigidBodyHandle, ()> = SlotMap::with_key();
        let h1 = body_handles.insert(());
        let h2 = body_handles.insert(());
        let h3 = body_handles.insert(());

        let mut manager = IslandManager::new();

        // Create a contact between h1 and h2
        manager.add_contact(h1, h2);

        // h3 is isolated
        manager.rebuild_islands(body_handles.keys(), |_| false);

        // Should have 2 islands: one with h1+h2, one with h3
        assert_eq!(manager.island_count(), 2);

        // h1 and h2 should be in the same island
        assert_eq!(
            manager.get_body_island(h1),
            manager.get_body_island(h2)
        );

        // h3 should be in a different island
        assert_ne!(
            manager.get_body_island(h1),
            manager.get_body_island(h3)
        );
    }
}

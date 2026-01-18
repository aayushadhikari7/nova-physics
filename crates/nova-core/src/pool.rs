//! Object pools for reusing allocations

use std::mem::MaybeUninit;

/// A simple object pool that reuses allocations
#[derive(Debug)]
pub struct Pool<T> {
    items: Vec<T>,
    free_list: Vec<usize>,
}

impl<T> Default for Pool<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T> Pool<T> {
    /// Create a new empty pool
    pub fn new() -> Self {
        Self {
            items: Vec::new(),
            free_list: Vec::new(),
        }
    }

    /// Create a pool with pre-allocated capacity
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            items: Vec::with_capacity(capacity),
            free_list: Vec::new(),
        }
    }

    /// Get the number of active items
    pub fn len(&self) -> usize {
        self.items.len() - self.free_list.len()
    }

    /// Check if pool is empty
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Clear all items from the pool
    pub fn clear(&mut self) {
        self.items.clear();
        self.free_list.clear();
    }
}

impl<T: Default> Pool<T> {
    /// Allocate a new item or reuse from free list
    pub fn allocate(&mut self) -> (usize, &mut T) {
        if let Some(index) = self.free_list.pop() {
            self.items[index] = T::default();
            (index, &mut self.items[index])
        } else {
            let index = self.items.len();
            self.items.push(T::default());
            (index, &mut self.items[index])
        }
    }

    /// Free an item back to the pool
    pub fn free(&mut self, index: usize) {
        if index < self.items.len() {
            self.free_list.push(index);
        }
    }

    /// Get an item by index
    pub fn get(&self, index: usize) -> Option<&T> {
        if self.free_list.contains(&index) {
            None
        } else {
            self.items.get(index)
        }
    }

    /// Get a mutable item by index
    pub fn get_mut(&mut self, index: usize) -> Option<&mut T> {
        if self.free_list.contains(&index) {
            None
        } else {
            self.items.get_mut(index)
        }
    }
}

/// A typed pool for specific object types with reset capability
pub struct TypedPool<T, F: Fn() -> T> {
    items: Vec<T>,
    factory: F,
}

impl<T, F: Fn() -> T> TypedPool<T, F> {
    /// Create a new typed pool with a factory function
    pub fn new(factory: F) -> Self {
        Self {
            items: Vec::new(),
            factory,
        }
    }

    /// Get an item from the pool, creating a new one if empty
    pub fn get(&mut self) -> T {
        self.items.pop().unwrap_or_else(&self.factory)
    }

    /// Return an item to the pool
    pub fn put(&mut self, item: T) {
        self.items.push(item);
    }

    /// Pre-allocate items in the pool
    pub fn preallocate(&mut self, count: usize) {
        for _ in 0..count {
            let item = (self.factory)();
            self.items.push(item);
        }
    }

    /// Clear the pool
    pub fn clear(&mut self) {
        self.items.clear();
    }

    /// Get current pool size
    pub fn len(&self) -> usize {
        self.items.len()
    }

    /// Check if pool is empty
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
}

impl<T, F: Fn() -> T> std::fmt::Debug for TypedPool<T, F> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TypedPool")
            .field("items_count", &self.items.len())
            .finish()
    }
}

/// A fixed-size ring buffer for storing recent values
#[derive(Debug)]
pub struct RingBuffer<T, const N: usize> {
    data: [MaybeUninit<T>; N],
    head: usize,
    len: usize,
}

impl<T, const N: usize> Default for RingBuffer<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T, const N: usize> RingBuffer<T, N> {
    /// Create a new empty ring buffer
    pub fn new() -> Self {
        Self {
            // SAFETY: MaybeUninit doesn't require initialization
            data: unsafe { MaybeUninit::uninit().assume_init() },
            head: 0,
            len: 0,
        }
    }

    /// Push a value, overwriting the oldest if full
    pub fn push(&mut self, value: T) {
        let index = (self.head + self.len) % N;

        if self.len == N {
            // Overwriting oldest value, drop it first
            unsafe {
                self.data[self.head].assume_init_drop();
            }
            self.head = (self.head + 1) % N;
        } else {
            self.len += 1;
        }

        self.data[index].write(value);
    }

    /// Get the most recent value
    pub fn last(&self) -> Option<&T> {
        if self.len == 0 {
            None
        } else {
            let index = (self.head + self.len - 1) % N;
            Some(unsafe { self.data[index].assume_init_ref() })
        }
    }

    /// Get the number of items
    pub fn len(&self) -> usize {
        self.len
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Check if full
    pub fn is_full(&self) -> bool {
        self.len == N
    }

    /// Clear the buffer
    pub fn clear(&mut self) {
        for i in 0..self.len {
            let index = (self.head + i) % N;
            unsafe {
                self.data[index].assume_init_drop();
            }
        }
        self.head = 0;
        self.len = 0;
    }
}

impl<T: Clone, const N: usize> RingBuffer<T, N> {
    /// Iterate over all values from oldest to newest
    pub fn iter(&self) -> impl Iterator<Item = T> + '_ {
        (0..self.len).map(move |i| {
            let index = (self.head + i) % N;
            unsafe { self.data[index].assume_init_ref().clone() }
        })
    }
}

impl<T, const N: usize> Drop for RingBuffer<T, N> {
    fn drop(&mut self) {
        self.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pool() {
        let mut pool: Pool<i32> = Pool::new();

        let (idx1, val1) = pool.allocate();
        *val1 = 42;
        assert_eq!(pool.len(), 1);

        let (idx2, _) = pool.allocate();
        assert_eq!(pool.len(), 2);

        pool.free(idx1);
        assert_eq!(pool.len(), 1);

        // Reuses freed slot
        let (idx3, _) = pool.allocate();
        assert_eq!(idx3, idx1);
    }

    #[test]
    fn test_ring_buffer() {
        let mut buf: RingBuffer<i32, 3> = RingBuffer::new();

        buf.push(1);
        buf.push(2);
        buf.push(3);
        assert!(buf.is_full());
        assert_eq!(*buf.last().unwrap(), 3);

        buf.push(4); // Overwrites 1
        assert_eq!(buf.iter().collect::<Vec<_>>(), vec![2, 3, 4]);
    }
}

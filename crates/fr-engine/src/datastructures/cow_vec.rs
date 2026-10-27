//! fastroute: a vector whose clones share their elements until they are written.
//!
//! The parallel autorouting pass, the optimizer and multi-start route on clones of the board.
//! With plain vectors every clone copied the item slots and the search tree nodes (37 MB per
//! clone on an 874-part board, 36 clones in flight in a parallel pass). A `CowVec` stores its
//! elements in full chunks behind `Arc`s plus an owned tail: a clone copies the chunk pointers
//! and the tail, and writing an element copies only its chunk if that chunk is still shared.
//!
//! Reads are on hot paths (tree traversals): a full chunk is an `Arc<[T]>`, so an element is
//! two dependent loads away (the chunk pointer, then the element), as with `Arc<Vec<T>>` it
//! would be three.

use std::fmt;
use std::ops::{Index, IndexMut};
use std::sync::Arc;

const SHIFT: usize = 6;
/// Elements per chunk.
const CHUNK: usize = 1 << SHIFT;
const MASK: usize = CHUNK - 1;

#[derive(Clone)]
pub struct CowVec<T> {
    /// Full chunks of `CHUNK` elements.
    chunks: Vec<Arc<[T]>>,
    /// The last, partial chunk (fewer than `CHUNK` elements), not shared.
    tail: Vec<T>,
}

impl<T> Default for CowVec<T> {
    fn default() -> Self {
        CowVec { chunks: Vec::new(), tail: Vec::new() }
    }
}

/// The chunk as a mutable slice, copied first if it is shared.
#[inline]
fn chunk_mut<T: Clone>(chunk: &mut Arc<[T]>) -> &mut [T] {
    // one atomic compare-exchange for an unshared chunk (`Arc::get_mut` twice took two)
    Arc::make_mut(chunk)
}

impl<T: Clone> CowVec<T> {
    pub fn new() -> Self {
        Self::default()
    }

    #[inline]
    pub fn len(&self) -> usize {
        (self.chunks.len() << SHIFT) + self.tail.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.chunks.is_empty() && self.tail.is_empty()
    }

    #[inline]
    pub fn get(&self, i: usize) -> Option<&T> {
        match self.chunks.get(i >> SHIFT) {
            Some(c) => Some(&c[i & MASK]),
            None => self.tail.get(i - (self.chunks.len() << SHIFT)),
        }
    }

    /// Copies the element's chunk first if it is shared with a clone.
    #[inline]
    pub fn get_mut(&mut self, i: usize) -> Option<&mut T> {
        let full = self.chunks.len() << SHIFT;
        match self.chunks.get_mut(i >> SHIFT) {
            Some(c) => Some(&mut chunk_mut(c)[i & MASK]),
            None => self.tail.get_mut(i - full),
        }
    }

    pub fn push(&mut self, value: T) {
        if self.tail.capacity() == 0 {
            self.tail.reserve_exact(CHUNK);
        }
        self.tail.push(value);
        if self.tail.len() == CHUNK {
            let full = std::mem::replace(&mut self.tail, Vec::with_capacity(CHUNK));
            self.chunks.push(Arc::from(full));
        }
    }

    fn pop(&mut self) {
        if self.tail.is_empty() {
            let Some(last) = self.chunks.pop() else { return };
            self.tail = last.to_vec();
        }
        self.tail.pop();
    }

    /// `Vec::resize`.
    pub fn resize(&mut self, new_len: usize, value: T) {
        while self.len() > new_len {
            self.pop();
        }
        while self.len() < new_len {
            self.push(value.clone());
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = &T> + '_ {
        self.chunks.iter().flat_map(|c| c.iter()).chain(self.tail.iter())
    }

    /// Unshares every chunk.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut T> + '_ {
        self.chunks.iter_mut().flat_map(|c| chunk_mut(c).iter_mut()).chain(self.tail.iter_mut())
    }
}

impl<T> Index<usize> for CowVec<T> {
    type Output = T;
    #[inline]
    fn index(&self, i: usize) -> &T {
        match self.chunks.get(i >> SHIFT) {
            Some(c) => &c[i & MASK],
            None => &self.tail[i - (self.chunks.len() << SHIFT)],
        }
    }
}

impl<T: Clone> IndexMut<usize> for CowVec<T> {
    #[inline]
    fn index_mut(&mut self, i: usize) -> &mut T {
        let full = self.chunks.len() << SHIFT;
        match self.chunks.get_mut(i >> SHIFT) {
            Some(c) => &mut chunk_mut(c)[i & MASK],
            None => &mut self.tail[i - full],
        }
    }
}

impl<T: fmt::Debug> fmt::Debug for CowVec<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_list().entries(self.chunks.iter().flat_map(|c| c.iter()).chain(self.tail.iter())).finish()
    }
}

impl<T: Clone> FromIterator<T> for CowVec<T> {
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        let mut v = CowVec::new();
        for x in iter {
            v.push(x);
        }
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clones_share_until_written() {
        let mut a: CowVec<u32> = (0..200).collect();
        let b = a.clone();
        a[5] = 1000;
        a.push(7);
        assert_eq!(b[5], 5);
        assert_eq!(b.len(), 200);
        assert_eq!(a[5], 1000);
        assert_eq!(a[200], 7);
        // only the written chunk was copied
        assert!(Arc::ptr_eq(&a.chunks[1], &b.chunks[1]));
        assert!(!Arc::ptr_eq(&a.chunks[0], &b.chunks[0]));
    }

    #[test]
    fn resize_and_iterate() {
        let mut v: CowVec<i32> = CowVec::new();
        v.resize(130, 1);
        assert_eq!(v.len(), 130);
        assert_eq!(v.iter().sum::<i32>(), 130);
        v.resize(64, 0);
        assert_eq!(v.len(), 64);
        assert_eq!(v.chunks.len(), 1);
        assert!(v.tail.is_empty());
        v.resize(63, 0);
        assert_eq!((v.chunks.len(), v.tail.len()), (0, 63));
        for x in v.iter_mut() {
            *x = 2;
        }
        assert_eq!(v.iter().sum::<i32>(), 126);
        assert_eq!(v.get(63), None);
        assert_eq!(v.get_mut(62).copied(), Some(2));
    }

    #[test]
    #[should_panic]
    fn index_out_of_range_panics() {
        let v: CowVec<u8> = (0..70).collect();
        let _ = v[70];
    }
}

pub fn join_within_vec<I, E, F: Fn(&I, &I) -> Result<Option<I>, E>>(
    items: &mut Vec<I>,
    joiner: F,
) -> Result<(), E> {
    // do-until
    while {
        let mut could_join = false;

        let mut i = 0;

        while i + 1 < items.len() {
            let mut j = i + 1;

            while j < items.len() {
                if let Some(joined) =
                    joiner(&items[i], &items[j])?.or(joiner(&items[j], &items[i])?)
                {
                    items[i] = joined;
                    items.swap_remove(j);
                    could_join = true;
                } else {
                    j += 1;
                }
            }

            i += 1;
        }

        could_join
    } {}

    Ok(())
}

// smallset: a Rust crate for small unordered sets of elements, built on top of
// `smallvec`.
//
// Copyright (c) 2016 Chris Fallin <cfallin@c1f.net>. Released under the MIT license.
// Extendend by Anselm Eickhoff

use std::fmt;

use smallvec::{Array, SmallVec};

/// A `SmallSet` is an unordered set of elements. It is designed to work best
/// for very small sets (no more than ten or so elements). In order to support
/// small sets very efficiently, it stores elements in a simple unordered array.
/// When the set is smaller than the size of the array `A`, all elements are
/// stored inline, without heap allocation. This is accomplished by using a
/// `smallvec::SmallVec`.
///
/// The insert, remove, and query methods on `SmallSet` have `O(n)` time
/// complexity in the current set size: they perform a linear scan to determine
/// if the element in question is present. This is inefficient for large sets,
/// but fast and cache-friendly for small sets.
///
/// Example usage:
///
/// ```
/// use descartes::util::SmallSet;
///
/// // `s` and its elements will be completely stack-allocated in this example.
/// let mut s: SmallSet<[u32; 4]> = SmallSet::new();
/// s.insert(1);
/// s.insert(2);
/// s.insert(3);
/// assert!(s.len() == 3);
/// assert!(s.contains(&1));
/// ```
pub struct SmallSet<A: Array>
where
    A::Item: PartialEq + Eq,
{
    elements: SmallVec<A>,
}

impl<A: Array> SmallSet<A>
where
    A::Item: PartialEq + Eq,
{
    /// Creates a new, empty `SmallSet`.
    pub fn new() -> SmallSet<A> {
        SmallSet {
            elements: SmallVec::new(),
        }
    }

    /// Inserts `elem` into the set if not yet present. Returns `true` if the
    /// set did not have this element present, or `false` if it already had this
    /// element present.
    pub fn insert(&mut self, elem: A::Item) -> bool {
        if !self.contains(&elem) {
            self.elements.push(elem);
            true
        } else {
            false
        }
    }

    /// Removes `elem` from the set. Returns `true` if the element was removed,
    /// or `false` if it was not found.
    pub fn remove(&mut self, elem: &A::Item) -> bool {
        if let Some(pos) = self.elements.iter().position(|e| *e == *elem) {
            self.elements.swap_remove(pos);
            true
        } else {
            false
        }
    }

    /// Clears the set.
    pub fn clear(&mut self) {
        self.elements.clear();
    }

    pub fn retain<F: FnMut(&mut A::Item) -> bool>(&mut self, predicate: F) {
        self.elements.retain(predicate);
    }
}

impl<A: Array> ::std::ops::Deref for SmallSet<A>
where
    A::Item: Eq,
{
    type Target = [A::Item];

    fn deref(&self) -> &[A::Item] {
        &self.elements
    }
}

impl<A: Array> ::std::ops::DerefMut for SmallSet<A>
where
    A::Item: Eq,
{
    fn deref_mut(&mut self) -> &mut [A::Item] {
        &mut self.elements
    }
}

impl<A: Array> ::std::iter::IntoIterator for SmallSet<A>
where
    A::Item: Eq, {
        type IntoIter = ::smallvec::IntoIter<A>;
        type Item = A::Item;
        fn into_iter(self) -> Self::IntoIter {
            self.elements.into_iter()
        }
    }

impl<A: Array> Clone for SmallSet<A>
where
    A::Item: PartialEq + Eq + Clone,
{
    fn clone(&self) -> SmallSet<A> {
        SmallSet {
            elements: self.elements.clone(),
        }
    }
}

impl<A: Array> fmt::Debug for SmallSet<A>
where
    A::Item: PartialEq + Eq + fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        self.elements.fmt(f)
    }
}

pub struct SmallSortedSet<A: Array>
where
    A::Item: PartialEq + Eq,
{
    elements: SmallVec<A>,
}

impl<A: Array> SmallSortedSet<A>
where
    A::Item: PartialEq + Eq + Ord,
{
    /// Creates a new, empty `SmallSortedSet`.
    pub fn new() -> SmallSortedSet<A> {
        SmallSortedSet {
            elements: SmallVec::new(),
        }
    }

    /// Inserts `elem` into the set if not yet present. Returns `true` if the
    /// set did not have this element present, or `false` if it already had this
    /// element present.
    pub fn insert(&mut self, elem: A::Item) -> bool {
        match self.elements.binary_search(&elem) {
            Ok(_pos) => {false},
            Err(insert_pos) => {
                self.elements.insert(insert_pos, elem);
                true
            }
        }
    }

    /// Removes `elem` from the set. Returns `true` if the element was removed,
    /// or `false` if it was not found.
    pub fn remove(&mut self, elem: &A::Item) -> bool {
        match self.elements.binary_search(&elem) {
            Ok(pos) => {
                self.elements.remove(pos);
                true
            },
            Err(_pos) => {
                false
            }
        }
    }

    /// Override slice implementation, make use of binary search
    pub fn contains(&mut self, elem: &A::Item) -> bool {
        self.elements.binary_search(&elem).is_ok()
    }

    /// Clears the set.
    pub fn clear(&mut self) {
        self.elements.clear();
    }

    pub fn retain<F: FnMut(&mut A::Item) -> bool>(&mut self, predicate: F) {
        self.elements.retain(predicate);
    }
}

impl<A: Array> ::std::ops::Deref for SmallSortedSet<A>
where
    A::Item: Eq,
{
    type Target = [A::Item];

    fn deref(&self) -> &[A::Item] {
        &self.elements
    }
}

impl<A: Array> ::std::ops::DerefMut for SmallSortedSet<A>
where
    A::Item: Eq,
{
    fn deref_mut(&mut self) -> &mut [A::Item] {
        &mut self.elements
    }
}

impl<A: Array> ::std::iter::IntoIterator for SmallSortedSet<A>
where
    A::Item: Eq, {
        type IntoIter = ::smallvec::IntoIter<A>;
        type Item = A::Item;
        fn into_iter(self) -> Self::IntoIter {
            self.elements.into_iter()
        }
    }

impl<A: Array> Clone for SmallSortedSet<A>
where
    A::Item: PartialEq + Eq + Clone,
{
    fn clone(&self) -> SmallSortedSet<A> {
        SmallSortedSet {
            elements: self.elements.clone(),
        }
    }
}

impl<A: Array> fmt::Debug for SmallSortedSet<A>
where
    A::Item: PartialEq + Eq + fmt::Debug,
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        self.elements.fmt(f)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use std::fmt::Write;

    #[test]
    fn test_basic_set() {
        let mut s: SmallSet<[u32; 2]> = SmallSet::new();
        assert!(s.insert(1) == true);
        assert!(s.insert(2) == true);
        assert!(s.insert(2) == false);
        assert!(s.insert(3) == true);
        assert!(s.insert(2) == false);
        assert!(s.insert(3) == false);
        assert!(s.contains(&1));
        assert!(s.contains(&2));
        assert!(s.contains(&3));
        assert!(!s.contains(&4));
        assert!(s.len() == 3);
        assert!(s.iter().map(|r| *r).collect::<Vec<u32>>() == vec![1, 2, 3]);
        s.clear();
        assert!(!s.contains(&1));
    }

    #[test]
    fn test_remove() {
        let mut s: SmallSet<[u32; 2]> = SmallSet::new();
        assert!(s.insert(1) == true);
        assert!(s.insert(2) == true);
        assert!(s.len() == 2);
        assert!(s.contains(&1));
        assert!(s.remove(&1) == true);
        assert!(s.remove(&1) == false);
        assert!(s.len() == 1);
        assert!(!s.contains(&1));
        assert!(s.insert(1) == true);
        assert!(s.iter().map(|r| *r).collect::<Vec<u32>>() == vec![2, 1]);
    }

    #[test]
    fn test_clone() {
        let mut s: SmallSet<[u32; 2]> = SmallSet::new();
        s.insert(1);
        s.insert(2);
        let c = s.clone();
        assert!(c.contains(&1));
        assert!(c.contains(&2));
        assert!(!c.contains(&3));
    }

    #[test]
    fn test_debug() {
        let mut s: SmallSet<[u32; 2]> = SmallSet::new();
        s.insert(1);
        s.insert(2);
        let mut buf = String::new();
        write!(buf, "{:?}", s).unwrap();
        assert!(&buf == "[1, 2]");
    }
}

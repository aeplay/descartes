use std::collections::hash_map::Entry;
use std::hash::Hash;
use N;

#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub struct CellCoords(pub i32, pub i32);

use fnv::{FnvHashMap};
use util::SmallSet;

pub type CellContent<M> = SmallSet<[M; 4]>;

pub trait GridShape {
    fn covered_cell_coords(&self, cell_width: N) -> Vec<CellCoords>;
}

#[derive(Clone)]
pub struct Grid<M: Hash + Eq + Clone> {
    pub cell_width: N,
    cells: FnvHashMap<CellCoords, CellContent<M>>,
    pub min_bounds: CellCoords,
    pub max_bounds: CellCoords,
}

impl<M: Hash + Eq + Clone> Grid<M> {
    pub fn new(cell_width: N) -> Grid<M> {
        Grid {
            cell_width,
            cells: FnvHashMap::default(),
            min_bounds: CellCoords(0, 0),
            max_bounds: CellCoords(0, 0),
        }
    }

    pub fn insert_unchecked<S: GridShape>(&mut self, member: M, shape: &S) {
        for coords in shape.covered_cell_coords(self.cell_width) {
            match self.cells.entry(coords) {
                Entry::Vacant(vacant) => {
                    let mut set = SmallSet::new();
                    set.insert(member.clone());
                    vacant.insert(set);
                }
                Entry::Occupied(mut occupied) => {
                    occupied.into_mut().insert(member.clone());
                }
            }
            self.min_bounds.0 = self.min_bounds.0.min(coords.0);
            self.min_bounds.1 = self.min_bounds.0.min(coords.1);
            self.max_bounds.0 = self.max_bounds.0.max(coords.0);
            self.max_bounds.1 = self.max_bounds.0.max(coords.1);
        }
    }

    pub fn insert_visiting<S: GridShape, F: FnMut(&CellContent<M>)>(
        &mut self,
        member: M,
        shape: &S,
        mut visitor: F,
    ) {
        for coords in shape.covered_cell_coords(self.cell_width) {
            match self.cells.entry(coords) {
                Entry::Vacant(vacant) => {
                    let mut set = SmallSet::new();
                    set.insert(member.clone());
                    vacant.insert(set);
                }
                Entry::Occupied(mut occupied) => {
                    visitor(occupied.get());
                    occupied.into_mut().insert(member.clone());
                }
            }
            self.min_bounds.0 = self.min_bounds.0.min(coords.0);
            self.min_bounds.1 = self.min_bounds.0.min(coords.1);
            self.max_bounds.0 = self.max_bounds.0.max(coords.0);
            self.max_bounds.1 = self.max_bounds.0.max(coords.1);
        }
    }

    pub fn visit<S: GridShape, F: FnMut(&CellContent<M>)>(&self, shape: &S, mut visitor: F) {
        for coords in shape.covered_cell_coords(self.cell_width) {
            if let Some(content) = self.cells.get(&coords) {
                visitor(content);
            }
        }
    }

    pub fn retain<S: GridShape, F: FnMut(&mut M) -> bool>(&mut self, shape: S, mut predicate: F) {
        for coords in shape.covered_cell_coords(self.cell_width) {
            if let Entry::Occupied(mut occupied) = self.cells.entry(coords) {
                let remove = {
                    let set = occupied.get_mut();
                    set.retain(&mut predicate);
                    set.is_empty()
                };

                if remove {
                    occupied.remove();
                }
            }
        }
    }
}

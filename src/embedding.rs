use fnv::{FnvHashMap, FnvHashSet};
use grid::{Grid, GridShape};
use intersect::Intersect;
use line_path::LinePath;
use ordered_float::OrderedFloat;
use segments::LineSegment;
use stable_vec::StableVec;
use std::collections::hash_map::Entry;
use util::SmallSortedSet;
use {P2, V2, VecLike, N};

#[derive(Copy, Clone, Hash, PartialEq, Eq, PartialOrd, Ord)]
pub struct PieceSegmentIndex {
    pub piece_idx: usize,
    pub segment_idx: usize,
}

#[derive(Clone)]
pub struct Embedding<L: Clone> {
    piece_segment_grid: Grid<PieceSegmentIndex>,
    pub pieces: StableVec<(LinePath, L)>,
}

struct PieceIntersection {
    segment_idx: usize,
    along: N,
    position: P2,
}

impl<L: Clone> Embedding<L> {
    pub fn new(cell_width: N) -> Self {
        Embedding {
            piece_segment_grid: Grid::new(cell_width),
            pieces: StableVec::new(),
        }
    }

    pub fn insert(&mut self, new_path: LinePath, label: L) {
        let mut intersections_on_new: Vec<PieceIntersection> = Vec::new();
        let mut intersections_on_others: FnvHashMap<usize, Vec<PieceIntersection>> =
            FnvHashMap::default();

        for (new_segment_idx, new_segment) in new_path.segments().enumerate() {
            let mut seen_other_piece_segments = SmallSortedSet::<[PieceSegmentIndex; 16]>::new();

            self.piece_segment_grid.visit(&new_segment, |cell_content| {
                for other_piece_segment_idx in cell_content.iter() {
                    if !seen_other_piece_segments.contains(other_piece_segment_idx) {
                        let other_segment = self.pieces[other_piece_segment_idx.piece_idx]
                            .0
                            .nth_segment(other_piece_segment_idx.segment_idx);

                        for intersection in (new_segment, other_segment).intersect() {
                            intersections_on_new.push(PieceIntersection {
                                segment_idx: new_segment_idx,
                                along: intersection.along_a,
                                position: intersection.position,
                            });

                            let other_intersection = PieceIntersection {
                                segment_idx: other_piece_segment_idx.segment_idx,
                                along: intersection.along_b,
                                position: intersection.position,
                            };

                            match intersections_on_others.entry(other_piece_segment_idx.piece_idx) {
                                Entry::Vacant(vacant) => {
                                    vacant.insert(vec![other_intersection]);
                                }
                                Entry::Occupied(mut occupied) => {
                                    occupied.into_mut().push(other_intersection);
                                }
                            }
                        }

                        seen_other_piece_segments.insert(*other_piece_segment_idx);
                    }
                }
            })
        }

        for (other_piece_idx, other_intersections) in intersections_on_others.into_iter() {
            let (other_piece, other_label) = self.remove_piece(other_piece_idx);
            self.insert_piece_splits(other_piece, other_intersections, other_label)
        }

        if intersections_on_new.is_empty() {
            self.insert_whole_piece(new_path, label);
        } else {
            self.insert_piece_splits(new_path, intersections_on_new, label);
        }
    }

    fn insert_piece_splits(
        &mut self,
        initial_piece: LinePath,
        mut intersections: Vec<PieceIntersection>,
        label: L,
    ) {
        intersections.sort_unstable_by(|intersection_a, intersection_b| {
            intersection_a
                .segment_idx
                .cmp(&intersection_b.segment_idx)
                .then(OrderedFloat(intersection_a.along).cmp(&OrderedFloat(intersection_b.along)))
        });
        let mut original_points_iter = initial_piece.points.iter().enumerate().peekable();
        let mut intersections_iter = intersections.into_iter().peekable();

        let mut new_point_groups = vec![VecLike::new()];

        loop {
            let take_point = {
                let maybe_next_point_with_idx = original_points_iter.peek();
                let maybe_next_intersection = intersections_iter.peek();

                match (maybe_next_point_with_idx, maybe_next_intersection) {
                    (Some((next_point_idx, _next_point)), Some(next_intersection)) => {
                        if *next_point_idx <= next_intersection.segment_idx {
                            true
                        } else {
                            false
                        }
                    }
                    (Some(_), None) => true,
                    (None, Some(_)) => false,
                    (None, None) => break,
                }
            };

            if take_point {
                new_point_groups
                    .last_mut()
                    .expect("should have last point group")
                    .push(
                        original_points_iter
                            .next()
                            .expect("already peeked point!")
                            .1
                            .clone(),
                    );
            } else {
                let intersection_point = intersections_iter
                    .next()
                    .expect("already peeked intersetion!")
                    .position;
                new_point_groups
                    .last_mut()
                    .expect("should have last point group")
                    .push(intersection_point);
                new_point_groups.push(Some(intersection_point).into_iter().collect());
            }
        }

        for points_group in new_point_groups.into_iter() {
            if let Some(split_piece) = LinePath::new(points_group) {
                self.insert_whole_piece(split_piece, label.clone())
            }
        }
    }

    fn insert_whole_piece(&mut self, path: LinePath, label: L) {
        let piece_idx = self.pieces.next_index();

        for (segment_idx, segment) in path.segments().enumerate() {
            self.piece_segment_grid.insert_unchecked(
                PieceSegmentIndex {
                    piece_idx,
                    segment_idx,
                },
                &segment,
            );
        }

        self.pieces.push((path, label));
    }

    fn remove_piece(&mut self, piece_idx: usize) -> (LinePath, L) {
        let (path, label) = self
            .pieces
            .remove(piece_idx)
            .expect("Tried to remove non-existing piece");

        // TODO: this might redundantly try to remove several times from the same grid cells
        for segment in path.segments() {
            self.piece_segment_grid
                .retain(segment, |piece_segment_idx: &mut PieceSegmentIndex| {
                    piece_segment_idx.piece_idx != piece_idx
                })
        }

        (path, label)
    }

    pub fn query_pieces<S: GridShape>(
        &self,
        shape: &S,
    ) -> Vec<(LineSegment, &L, PieceSegmentIndex)> {
        let mut unique_piece_segment_indices =
            FnvHashSet::with_capacity_and_hasher(100, Default::default());

        self.piece_segment_grid.visit(shape, |cell_content| {
            unique_piece_segment_indices.extend(cell_content.iter().cloned());
        });

        unique_piece_segment_indices
            .into_iter()
            .map(|piece_segment_idx| {
                let (path, label) = &self.pieces[piece_segment_idx.piece_idx];
                (
                    path.nth_segment(piece_segment_idx.segment_idx),
                    label,
                    piece_segment_idx,
                )
            })
            .collect()
    }

    pub fn query_ray_along_x(&self, start_point: P2) -> Vec<(LineSegment, &L, PieceSegmentIndex)> {
        let ray_length = (self.piece_segment_grid.max_bounds.0
            - (start_point.x / self.piece_segment_grid.cell_width) as i32)
            .max(1) as f32 * self.piece_segment_grid.cell_width;
        let ray = LineSegment::new(start_point, start_point + V2::new(ray_length, 0.0));

        self.query_pieces(&ray)
    }

    pub fn map_labels<L2: Clone, M: FnMut(&LinePath, &L, &Self) -> L2>(
        &mut self,
        mut mapper: M,
    ) -> Embedding<L2> {
        let mut new_pieces = StableVec::new();
        new_pieces.grow(self.pieces.capacity());

        for piece_idx in self.pieces.keys() {
            let (piece, old_label) = &self.pieces[piece_idx];
            let new_label = mapper(&piece, &old_label, self);
            new_pieces
                .insert_into_hole(piece_idx, (piece.clone(), new_label))
                .ok()
                .expect("Index in clone should be free!");
        }

        Embedding {
            piece_segment_grid: self.piece_segment_grid.clone(),
            pieces: new_pieces,
        }
    }

    pub fn map_labels_in_place<M: FnMut(&LinePath, &L, &Self) -> L>(&mut self, mut mapper: M) {
        let mut new_pieces = StableVec::new();
        new_pieces.grow(self.pieces.capacity());

        for piece_idx in self.pieces.keys() {
            let (piece, old_label) = &self.pieces[piece_idx];
            let new_label = mapper(&piece, &old_label, self);
            new_pieces
                .insert_into_hole(piece_idx, (piece.clone(), new_label))
                .ok()
                .expect("Index in clone should be free!");
        }

        self.pieces = new_pieces;
    }

    pub fn retain_pieces<P: FnMut(&LinePath, &L, &Self) -> bool>(&mut self, mut predicate: P) {
        for piece_idx in 0..self.pieces.next_index() {
            let (existed, keep) = self
                .pieces
                .get(piece_idx)
                .map(|(path, label)| (true, predicate(path, label, self)))
                .unwrap_or((false, false));

            if existed && !keep {
                self.remove_piece(piece_idx);
            }
        }
    }
}

#[test]
fn embedding_test() {
    let mut embedding = Embedding::new(0.25);

    #[derive(Clone, PartialEq, Eq, Debug)]
    enum Label {
        A,
        B,
    }

    //        |
    //     .--x--- B
    // A --x--'
    //     |
    embedding.insert(
        LinePath::new(vec![
            P2::new(0.0, 0.0),
            P2::new(1.0, 0.0),
            P2::new(1.0, 1.0),
        ]).unwrap(),
        Label::A,
    );
    embedding.insert(
        LinePath::new(vec![
            P2::new(0.5, -0.5),
            P2::new(0.5, 0.5),
            P2::new(1.5, 0.5),
        ]).unwrap(),
        Label::B,
    );

    assert_eq!(
        embedding.pieces.iter().cloned().collect::<Vec<_>>(),
        vec![
            (
                LinePath::new(vec![P2::new(0.0, 0.0), P2::new(0.5, 0.0)]).unwrap(),
                Label::A,
            ),
            (
                LinePath::new(vec![
                    P2::new(0.5, 0.0),
                    P2::new(1.0, 0.0),
                    P2::new(1.0, 0.5),
                ]).unwrap(),
                Label::A,
            ),
            (
                LinePath::new(vec![P2::new(1.0, 0.5), P2::new(1.0, 1.0)]).unwrap(),
                Label::A,
            ),
            (
                LinePath::new(vec![P2::new(0.5, -0.5), P2::new(0.5, 0.0)]).unwrap(),
                Label::B,
            ),
            (
                LinePath::new(vec![
                    P2::new(0.5, 0.0),
                    P2::new(0.5, 0.5),
                    P2::new(1.0, 0.5),
                ]).unwrap(),
                Label::B,
            ),
            (
                LinePath::new(vec![P2::new(1.0, 0.5), P2::new(1.5, 0.5)]).unwrap(),
                Label::B,
            ),
        ]
    )
}

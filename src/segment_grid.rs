use bbox::{BoundingBox, HasBoundingBox};
use fnv::FnvHashSet;
use grid::{CellCoords, Grid, GridShape};
use line_path::LinePath;
use segments::{LineSegment, Segment};
use N;

#[derive(Copy, Clone, PartialEq, Eq, Hash)]
pub struct PathIdx(u32);

impl PathIdx {
    pub fn as_idx(self) -> usize {
        self.0 as usize
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Hash)]
pub struct SegmentIdx(u32);

impl SegmentIdx {
    pub fn as_idx(self) -> usize {
        self.0 as usize
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Hash)]
pub struct PathSegmentIdx {
    pub path_idx: PathIdx,
    pub segment_idx: SegmentIdx,
}

pub struct SegmentGrid {
    grid: Grid<PathSegmentIdx>,
    next_path_idx: usize,
}

impl GridShape for LineSegment {
    fn covered_cell_coords(&self, cell_width: N) -> Vec<CellCoords> {
        let length = self.length();
        let step = cell_width / 2.0;

        if length <= step {
            bbox_unique_cell_coords(self.bounding_box(), cell_width)
                .iter()
                .filter_map(|maybe| maybe.clone())
                .collect()
        } else {
            let start = self.start();
            let direction = self.direction();

            let mut start_along = 0.0;
            let mut end_along = step;

            let start_4_coords = bbox_unique_cell_coords(
                LineSegment::new(
                    start + direction * start_along,
                    start + direction * end_along,
                ).bounding_box(),
                cell_width,
            );
            let mut coords: Vec<_> = start_4_coords
                .iter()
                .filter_map(|maybe| maybe.clone())
                .collect();

            loop {
                start_along += step;
                end_along = (end_along + step).min(length);

                let mut new_4_coords = bbox_unique_cell_coords(
                    LineSegment::new(
                        start + direction * start_along,
                        start + direction * end_along,
                    ).bounding_box(),
                    cell_width,
                );

                {
                    let only_if_new = |new_coord| {
                        if coords[coords.len().saturating_sub(4)..coords.len()].contains(&new_coord)
                        {
                            None
                        } else {
                            Some(new_coord)
                        }
                    };

                    new_4_coords[0] = new_4_coords[0].and_then(only_if_new);
                    new_4_coords[1] = new_4_coords[1].and_then(only_if_new);
                    new_4_coords[2] = new_4_coords[2].and_then(only_if_new);
                    new_4_coords[3] = new_4_coords[3].and_then(only_if_new);
                }

                coords.extend(new_4_coords.iter().filter_map(|maybe| maybe.clone()));

                if start_along + step >= length {
                    break;
                }
            }

            coords
        }
    }
}

fn bbox_unique_cell_coords(bbox: BoundingBox, cell_width: N) -> [Option<CellCoords>; 4] {
    let bottom_left = CellCoords(
        (bbox.min.x / cell_width).floor() as i32,
        (bbox.min.y / cell_width).floor() as i32,
    );
    let top_left = CellCoords(
        (bbox.min.x / cell_width).floor() as i32,
        (bbox.max.y / cell_width).floor() as i32,
    );
    let top_right = CellCoords(
        (bbox.max.x / cell_width).floor() as i32,
        (bbox.max.y / cell_width).floor() as i32,
    );
    let bottom_right = CellCoords(
        (bbox.max.x / cell_width).floor() as i32,
        (bbox.min.y / cell_width).floor() as i32,
    );

    let mut result = [Some(bottom_left), None, None, None];

    // compiler should figure out redundant checks

    if top_left != bottom_left {
        result[1] = Some(top_left);
    }

    if top_right != bottom_left && top_right != top_left {
        result[2] = Some(top_right);
    }

    if bottom_right != bottom_left && bottom_right != top_left && bottom_right != top_right {
        result[3] = Some(bottom_right);
    }

    result
}

#[test]
fn intersection_grid() {
    use P2;
    assert_eq!(
        LineSegment::new(P2::new(0.0, 0.0), P2::new(3.0, 2.0)).covered_cell_coords(0.25),
        vec![
            CellCoords(-1, -1),
            CellCoords(-1, 0),
            CellCoords(0, 0),
            CellCoords(0, -1),
            CellCoords(1, 0),
            CellCoords(1, 1),
            CellCoords(2, 1),
            CellCoords(2, 2),
            CellCoords(3, 2),
            CellCoords(3, 1),
            CellCoords(4, 2),
            CellCoords(4, 3),
            CellCoords(5, 3),
            CellCoords(5, 4),
            CellCoords(6, 4),
            CellCoords(6, 3),
            CellCoords(7, 4),
            CellCoords(7, 5),
            CellCoords(8, 5),
            CellCoords(8, 6),
            CellCoords(9, 6),
            CellCoords(9, 5),
            CellCoords(10, 6),
            CellCoords(10, 7),
            CellCoords(11, 7),
            CellCoords(11, 8),
            CellCoords(12, 8),
            CellCoords(12, 7),
        ]
    );
}

impl SegmentGrid {
    pub fn new(cell_width: N) -> SegmentGrid {
        SegmentGrid {
            grid: Grid::new(cell_width),
            next_path_idx: 0,
        }
    }

    pub fn insert(
        &mut self,
        path: &LinePath,
    ) -> (PathIdx, FnvHashSet<(SegmentIdx, PathSegmentIdx)>) {
        let path_idx = PathIdx(self.next_path_idx as u32);
        self.next_path_idx += 1;

        let mut interactions = FnvHashSet::default();

        for (segment_idx, segment) in path.segments().enumerate() {
            let path_segment_idx = PathSegmentIdx {
                path_idx: path_idx,
                segment_idx: SegmentIdx(segment_idx as u32),
            };

            self.grid
                .insert_visiting(path_segment_idx, &segment, |existing_cell_content| {
                    for other_path_segment_idx in existing_cell_content.iter() {
                        // TODO: allow self-intersections
                        if path_segment_idx.path_idx != other_path_segment_idx.path_idx {
                            interactions
                                .insert((SegmentIdx(segment_idx as u32), *other_path_segment_idx));
                        }
                    }
                });
        }

        (path_idx, interactions)
    }
}

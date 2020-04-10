use arc_line_path::{ArcLinePath, direction_along_line};
use segments::{Segment, ArcSegment};
use {VecLike, P2, V2, N};
use ordered_float::OrderedFloat;

#[cfg_attr(feature = "serde-serialization", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug)]
pub struct Corner {
    position: P2,
    in_direction: Option<V2>,
    out_direction: Option<V2>,
}

impl Corner {
    pub fn new(position: P2, in_direction: Option<V2>, out_direction: Option<V2>) -> Self {
        Corner {
            position,
            in_direction,
            out_direction,
        }
    }
}

#[cfg_attr(feature = "compact_containers", derive(Compact))]
#[cfg_attr(feature = "serde-serialization", derive(Serialize, Deserialize))]
#[derive(Clone, Debug)]
pub struct EditArcLinePath {
    corners: VecLike<Corner>,
    strategy: ResolutionStrategy,
    closed: Closedness,
}

impl EditArcLinePath {
    pub fn new<V>(corners: V, strategy: ResolutionStrategy, closed: Closedness) -> Self
    where
        V: Into<VecLike<Corner>>,
    {
        EditArcLinePath {
            corners: corners.into(),
            strategy,
            closed,
        }
    }
}

#[cfg_attr(feature = "serde-serialization", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug)]
pub enum ResolutionStrategy {
    AssumeSmooth,
    AssumeLines,
}

#[cfg_attr(feature = "serde-serialization", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug)]
pub enum Closedness {
    AlwaysClosed,
    Closable,
    NeverClosed,
}

impl EditArcLinePath {
    fn nth(&self, i: isize) -> Option<Corner> {
        match self.closed {
            Closedness::AlwaysClosed => {
                let len = self.corners.len() as isize;
                let wrapped_i = (i % len + len) % len;
                Some(self.corners[wrapped_i as usize])
            }
            Closedness::NeverClosed => {
                if (i >= 0) {
                    self.corners.get(i as usize).copied()
                } else {
                    None
                }
            }
            Closedness::Closable => unimplemented!(),
        }
    }

    fn resolve_step(&mut self) -> bool {
        let mut made_progress = false;
        match self.strategy {
            ResolutionStrategy::AssumeSmooth => {
                for i in 0..self.corners.len() {
                    if self.corners[i].in_direction.is_none() {
                        if self.corners[i].out_direction.is_some() {
                            self.corners[i].in_direction = self.corners[i].out_direction;
                            made_progress = true;
                        } else if let Some(previous) = self.nth(i as isize - 1) {
                            if let Some(previous_out_direction) = previous.out_direction {
                                if direction_along_line(
                                    previous.position,
                                    previous_out_direction,
                                    self.corners[i].position,
                                ) {
                                    self.corners[i].in_direction = Some(previous_out_direction);
                                    made_progress = true;
                                } else if let Some(induced_in_direction) =
                                    ArcSegment::minor_arc_with_start_direction(
                                        previous.position,
                                        previous_out_direction,
                                        self.corners[i].position,
                                    )
                                    .map(|segment| segment.end_direction())
                                {
                                    self.corners[i].in_direction = Some(induced_in_direction);
                                    made_progress = true;
                                }
                            }
                        }
                    }
                    if self.corners[i].out_direction.is_none() {
                        if self.corners[i].in_direction.is_some() {
                            self.corners[i].out_direction = self.corners[i].in_direction;
                            made_progress = true;
                        } else if let Some(next) = self.nth(i as isize + 1) {
                            if let Some(next_in_direction) = next.in_direction {
                                if direction_along_line(
                                    self.corners[i].position,
                                    next_in_direction,
                                    next.position,
                                ) {
                                    self.corners[i].in_direction = Some(next_in_direction);
                                    made_progress = true;
                                } else if let Some(induced_out_direction) =
                                    ArcSegment::minor_arc_with_start_direction(
                                        next.position,
                                        -1.0 * next_in_direction,
                                        self.corners[i].position,
                                    )
                                    .map(|segment| -1.0 * segment.start_direction())
                                {
                                    self.corners[i].out_direction = Some(induced_out_direction);
                                    made_progress = true;
                                }
                            } else {
                                // TODO: keep this here to set initial direction,
                                //       or do we really expect to be given at least one direction?
                                self.corners[i].out_direction =
                                    Some((next.position - self.corners[i].position).normalize());
                                made_progress = true;
                            }
                        }
                    }
                }
            }
            ResolutionStrategy::AssumeLines => {
                for i in 0..self.corners.len() {
                    if self.corners[i].in_direction.is_none() {
                        if let Some(previous) = self.nth(i as isize - 1) {
                            self.corners[i].in_direction =
                                Some((self.corners[i].position - previous.position).normalize());
                            made_progress = true;
                        }
                    }
                    if self.corners[i].out_direction.is_none() {
                        if let Some(next) = self.nth(i as isize + 1) {
                            self.corners[i].out_direction =
                                Some((next.position - self.corners[i].position).normalize());
                            made_progress = true;
                        }
                    }
                }
            }
        }
        made_progress
    }

    pub fn resolve(&self) -> (Option<ArcLinePath>, EditArcLinePath) {
        let mut work_in_progress = self.clone();

        while work_in_progress.resolve_step() {}

        let mut maybe_arc_line_path: Option<ArcLinePath> = None;

        match self.closed {
            Closedness::AlwaysClosed => {
                work_in_progress.corners.push(work_in_progress.corners[0]);
            }
            _ => {}
        }

        for pair in work_in_progress.corners.windows(2) {
            let maybe_next_segment = match (pair[0].out_direction, pair[1].in_direction) {
                (Some(out_direction), Some(in_direction)) => ArcLinePath::biarc(
                    pair[0].position,
                    out_direction,
                    pair[1].position,
                    in_direction,
                ),
                _ => None,
            };

            maybe_arc_line_path = match (maybe_arc_line_path, maybe_next_segment) {
                (Some(arc_line_path), Some(next_segment)) => {
                    arc_line_path.concat(&next_segment).ok()
                }
                (maybe_arc_line_path, None) => maybe_arc_line_path,
                (None, maybe_next_segment) => maybe_next_segment,
            };
        }

        (maybe_arc_line_path, work_in_progress)
    }

    pub fn move_corner(&mut self, idx: usize, new_position: P2) {
        self.corners[idx].position = new_position;
    }

    pub fn with_corner_moved(&self, idx: usize, new_position: P2) -> EditArcLinePath {
        let mut new = self.clone();
        new.move_corner(idx, new_position);
        new
    }

    pub fn direct_corner(&mut self, idx: usize, new_direction: Option<V2>, in_direction: bool) {
        if in_direction {
            self.corners[idx].in_direction = new_direction;
        } else {
            self.corners[idx].out_direction = new_direction;
        }
    }

    pub fn with_corner_directed(
        &self,
        idx: usize,
        new_direction: Option<V2>,
        in_direction: bool,
    ) -> EditArcLinePath {
        let mut new = self.clone();
        new.direct_corner(idx, new_direction, in_direction);
        new
    }

    pub fn add_corner(&mut self, add_to_end: bool, corner: Corner) {
        if add_to_end {
            self.corners.push(corner);
        } else {
            self.corners.insert(0, corner);
        }
    }

    pub fn with_corner_added(&self, add_to_end: bool, corner: Corner) -> EditArcLinePath {
        let mut new = self.clone();
        new.add_corner(add_to_end, corner);
        new
    }

    pub fn insert_corner(&mut self, corner: Corner) -> bool {
        if let (Some(path), _) = self.resolve() {
            let line_path = path.to_line_path();
            let distance_along_by_corner: Vec<_> = self
                .corners
                .iter()
                .filter_map(|c| line_path.project(c.position))
                .collect();
            if let Some(new_corner_projection) = line_path.project(corner.position) {
                if let Err(idx) = distance_along_by_corner.binary_search_by(|corner_projection| {
                    OrderedFloat(corner_projection.0).cmp(&OrderedFloat(new_corner_projection.0))
                }) {
                    self.corners.insert(idx, corner);
                }
                true
            } else {
                false
            }
        } else {
            false
        }
    }

    pub fn with_corner_inserted(&self, corner: Corner) -> Option<EditArcLinePath> {
        let mut new = self.clone();
        if new.insert_corner(corner) {
            Some(new)
        } else {
            None
        }
    }

    pub fn subsection(&self, start: N, end: N) -> Option<EditArcLinePath> {
        // TODO circular when closed

        if let (Some(arc_line_path), _) = self.resolve() {
            let line_path = arc_line_path.to_line_path();

            let first_corner_position = line_path.along(start);
            let first_corner_direction = line_path.direction_along(start);
            let first_corner = Corner::new(
                first_corner_position,
                Some(first_corner_direction),
                Some(first_corner_direction),
            );

            let last_corner_position = line_path.along(end);
            let last_corner_direction = line_path.direction_along(end);
            let last_corner = Corner::new(
                last_corner_position,
                Some(last_corner_direction),
                Some(last_corner_direction),
            );

            let corners_between = self
                .corners
                .iter()
                .filter(|corner| {
                    if let Some(_) = line_path.project(corner.position) {
                        true
                    } else {
                        false
                    }
                })
                .copied();

            let new_corners: VecLike<_> = Some(first_corner)
                .into_iter()
                .chain(corners_between)
                .chain(Some(last_corner))
                .collect();

            Some(EditArcLinePath::new(
                new_corners,
                self.strategy,
                self.closed,
            ))
        } else {
            None
        }
    }
}

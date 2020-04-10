use intersect::{Intersect, Intersection};
use itertools::Itertools;
use line_path::{ConcatError, LinePath};
use rough_eq::{RoughEq, THICKNESS};
use segments::{
    ArcOrLineSegment, ArcSegment, LineSegment, Segment, MIN_ARC_LENGTH, MIN_LINE_LENGTH,
};
use {P2, V2, VecLike, N};

#[cfg_attr(feature = "serde-serialization", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug)]
pub enum ArcLinePoint {
    Point(P2),
    Apex(P2),
}

impl ArcLinePoint {
    pub fn expect_point(&self) -> Option<P2> {
        if let &Point(p) = self {
            Some(p)
        } else {
            None
        }
    }

    pub fn expect_apex(&self) -> Option<P2> {
        if let &Apex(a) = self {
            Some(a)
        } else {
            None
        }
    }
}

use self::ArcLinePoint::{Apex, Point};

#[cfg_attr(feature = "compact_containers", derive(Compact))]
#[cfg_attr(feature = "serde-serialization", derive(Serialize, Deserialize))]
#[derive(Clone, Debug)]
pub struct ArcLinePath {
    points: VecLike<ArcLinePoint>,
}

const ARC_DIRECTION_TOLERANCE: N = 0.0001;
const CURVE_LINEARIZATION_MAX_ANGLE: N = 0.1;

pub fn direction_along_line(start: P2, start_direction: V2, end: P2) -> bool {
    start_direction.rough_eq_by((end - start).normalize(), ARC_DIRECTION_TOLERANCE)
}

/// Creation
impl ArcLinePath {
    pub fn line(start: P2, end: P2) -> Option<Self> {
        if (end - start).norm() <= MIN_LINE_LENGTH {
            None
        } else {
            Some(ArcLinePath {
                points: vec![Point(start), Point(end)].into(),
            })
        }
    }

    pub fn arc(start: P2, start_direction: V2, end: P2) -> Option<Self> {
        if (end - start).norm() <= MIN_ARC_LENGTH {
            None
        } else if direction_along_line(start, start_direction, end) {
            Self::line(start, end)
        } else {
            if let Some(segment) =
                ArcSegment::minor_arc_with_start_direction(start, start_direction, end)
            {
                Some(ArcLinePath {
                    points: vec![
                        Point(segment.start()),
                        Apex(segment.apex()),
                        Point(segment.end()),
                    ].into(),
                })
            } else {
                Self::line(start, end)
            }
        }
    }

    pub fn biarc(start: P2, start_direction: V2, end: P2, end_direction: V2) -> Option<Self> {
        const MAX_SIMPLE_LINE_LENGTH: N = 0.1;
        const RAY_LENGTH: N = 10_000.0;

        if (end - start).norm() <= THICKNESS {
            None
        } else if (end - start).norm() < MAX_SIMPLE_LINE_LENGTH {
            Self::line(start, end)
        } else {
            let single_arc = Self::arc(start, start_direction, end)?;
            if single_arc
                .end_direction()
                .rough_eq_by(end_direction, ARC_DIRECTION_TOLERANCE)
            {
                Some(single_arc)
            } else {
                let start_ray = LineSegment::new(start, start + RAY_LENGTH * start_direction);
                let end_ray = LineSegment::new(end, end - RAY_LENGTH * end_direction);
                let maybe_linear_intersection = (start_ray, end_ray).intersect().into_iter().find(
                    |intersection| {
                        intersection.along_a < 0.8 * RAY_LENGTH
                            && intersection.along_b < 0.8 * RAY_LENGTH
                    },
                );

                let (connection_position, connection_direction) =
                    if let Some(Intersection { position, .. }) = maybe_linear_intersection {
                        let start_to_intersection_distance = (start - position).norm();
                        let end_to_intersection_distance = (end - position).norm();

                        if start_to_intersection_distance < end_to_intersection_distance {
                            // arc then line
                            (
                                position + start_to_intersection_distance * end_direction,
                                end_direction,
                            )
                        } else {
                            // line then arc
                            (
                                position + end_to_intersection_distance * -start_direction,
                                start_direction,
                            )
                        }
                    } else {
                        // http://www.ryanjuckett.com/programming/biarc-interpolation/
                        let v = end - start;
                        let t = start_direction + end_direction;
                        let same_direction =
                            start_direction.rough_eq_by(end_direction, ARC_DIRECTION_TOLERANCE);
                        let end_orthogonal_of_start = v.dot(&end_direction).rough_eq(0.0);

                        if same_direction && end_orthogonal_of_start {
                            //    __
                            //   /  \
                            //  ^    v    ^
                            //        \__/
                            (
                                P2::from_coordinates((start.coords + end.coords) / 2.0),
                                -start_direction,
                            )
                        } else {
                            let d = if same_direction {
                                v.dot(&v) / (4.0 * v.dot(&end_direction))
                            } else {
                                // magic - I'm pretty sure this can be simplified
                                let v_dot_t = v.dot(&t);
                                (-v_dot_t
                                    + (v_dot_t * v_dot_t
                                        + 2.0
                                            * (1.0 - start_direction.dot(&end_direction))
                                            * v.dot(&v))
                                        .sqrt())
                                    / (2.0 * (1.0 - start_direction.dot(&end_direction)))
                            };

                            let start_offset_point = start + d * start_direction;
                            let end_offset_point = end - d * end_direction;
                            let connection_direction =
                                (end_offset_point - start_offset_point).normalize();

                            (
                                start_offset_point + d * connection_direction,
                                connection_direction,
                            )
                        }
                    };

                match (
                    Self::arc(start, start_direction, connection_position),
                    Self::arc(connection_position, connection_direction, end),
                ) {
                    (Some(first), Some(second)) => first.concat(&second).ok(),
                    (Some(first), None) => Some(first),
                    (None, Some(second)) => Some(second),
                    _ => None,
                }
            }
        }
    }

    pub fn circle(center: P2, radius: N) -> Option<Self> {
        let top = center + V2::new(0.0, radius);
        let right = center + V2::new(radius, 0.0);
        let bottom = center + V2::new(0.0, -radius);
        let left = center + V2::new(-radius, 0.0);
        let top_right_segment = Self::arc(top, V2::new(1.0, 0.0), right)?;
        let bottom_right_segment = Self::arc(right, V2::new(0.0, -1.0), bottom)?;
        let bottom_left_segment = Self::arc(bottom, V2::new(-1.0, 0.0), left)?;
        let top_left_segment = Self::arc(left, V2::new(0.0, 1.0), top)?;
        top_right_segment
            .concat(&bottom_right_segment)
            .and_then(|path| path.concat(&bottom_left_segment))
            .and_then(|path| path.concat(&top_left_segment))
            .ok()
    }
}

/// Inspection
impl ArcLinePath {
    pub fn start(&self) -> P2 {
        self.points[0]
            .expect_point()
            .expect("ArcLinePath start should be a Point")
    }

    pub fn end(&self) -> P2 {
        self.points
            .last()
            .and_then(ArcLinePoint::expect_point)
            .expect("ArcLinePath end should exist and be a Point")
    }

    pub fn length(&self) -> N {
        self.segments().map(|segment| segment.length()).sum()
    }

    pub fn start_direction(&self) -> V2 {
        self.segments().next().unwrap().start_direction()
    }

    pub fn end_direction(&self) -> V2 {
        self.segments().last().unwrap().end_direction()
    }

    pub fn segments<'a>(&'a self) -> impl Iterator<Item = ArcOrLineSegment> + 'a {
        let mut points_iter = self.points.iter();
        let mut start = points_iter
            .next()
            .and_then(ArcLinePoint::expect_point)
            .expect("ArcLinePath should have and start with a Point");

        points_iter.batching(move |iter| match iter.next() {
            Some(&Point(end)) => {
                let segment = ArcOrLineSegment::line_unchecked(start, end);
                start = end;
                Some(segment)
            }
            Some(&Apex(apex)) => {
                if let Some(&Point(end)) = iter.next() {
                    //let segment = ArcOrLineSegment::arc_unchecked(start, apex, end);
                    if let Some(segment) = ArcSegment::new(start, apex, end) {
                        start = end;
                        Some(ArcOrLineSegment::Arc(segment))
                    } else {
                        panic!(
                            "Invalid segment in path: {:?}, seg: {:?} {:?} {:?}",
                            self, start, apex, end
                        )
                    }
                } else {
                    unreachable!("After an Apex there should be at least one more Point")
                }
            }
            None => None,
        })
    }
}

/// Combination/Modification
impl ArcLinePath {
    pub fn concat(&self, other: &Self) -> Result<Self, ConcatError> {
        if self.end().rough_eq(other.start()) {
            let old_len = self.points.len();
            let other_len = other.points.len();
            let concat_path = ArcLinePath {
                points: self
                    .points
                    .iter()
                    .chain(other.points[1..].iter())
                    .cloned()
                    .collect(),
            };
            // if there was an arc at either side of the transition, make sure it's still valid
            if other_len >= 3 {
                if let (Point(start), Apex(apex), Point(end)) = (
                    concat_path.points[old_len - 1],
                    concat_path.points[old_len],
                    concat_path.points[old_len + 1],
                ) {
                    if ArcSegment::new(start, apex, end).is_none() {
                        return Err(ConcatError::CreatedInvalidSegment);
                    }
                }
            }

            Ok(concat_path)
        } else {
            Err(ConcatError::PointsTooFarApart)
        }
    }

    pub fn reverse(&self) -> Self {
        let mut new_points = self.points.clone();
        new_points.reverse();
        ArcLinePath {
            points: new_points
        }
    }

    pub fn to_line_path_with_max_angle(&self, max_angle: N) -> LinePath {
        let points: VecLike<P2> = self
            .segments()
            .flat_map(|segment| segment.subdivisions_without_end(max_angle))
            .chain(Some(self.end()))
            .collect();

        if let Some(path) = LinePath::new(points.clone()) {
            path
        } else {
            panic!(
                "A valid ArcLinePath should always produce a valid LinePath: {:?}, points: {:?}",
                self, points
            )
        }
    }

    pub fn to_line_path(&self) -> LinePath {
        self.to_line_path_with_max_angle(CURVE_LINEARIZATION_MAX_ANGLE)
    }
}

#[test]
fn to_line_path() {
    use PI;
    let curved_path = ArcLinePath::line(P2::new(0.0, 0.0), P2::new(1.0, 0.0))
        .expect("first line should work")
        .concat(
            &ArcLinePath::arc(P2::new(1.0, 0.0), V2::new(1.0, 0.0), P2::new(2.0, 1.0))
                .expect("arc should work"),
        )
        .expect("line>arc concat should work")
        .concat(
            &ArcLinePath::line(P2::new(2.0, 1.0), P2::new(2.0, 2.0))
                .expect("second line should work"),
        )
        .expect("line-arc>line concat should work");

    println!("{:#?}", curved_path.segments().collect::<Vec<_>>());

    assert_rough_eq_by!(
        &LinePath::new(vec![
            P2::new(0.0, 0.0),
            P2::new(1.0, 0.0),
            P2::new(1.5, 0.13397461),
            P2::new(1.8660254, 0.5),
            P2::new(2.0, 1.0),
            P2::new(2.0, 2.0),
        ]).unwrap(),
        &curved_path.to_line_path_with_max_angle(PI / 6.0),
        0.0001
    );
}

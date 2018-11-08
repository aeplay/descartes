use bbox::{BoundingBox, HasBoundingBox};
use closed_line_path::ClosedLinePath;
use intersect::Intersect;
use line_path::LinePath;
use ordered_float::OrderedFloat;
use segments::Segment;
use util::join_within_vec;
use {P2, RoughEq, VecLike, N, PI, THICKNESS};

mod debug;

#[cfg(test)]
mod tests;

#[derive(Debug)]
pub struct UnclosedPathError;

#[derive(PartialEq)]
pub enum AreaLocation {
    Inside,
    Boundary,
    Outside,
}

pub trait PointContainer {
    fn location_of(&self, point: P2) -> AreaLocation;

    fn contains(&self, point: P2) -> bool {
        self.location_of(point) != AreaLocation::Outside
    }
}

/// Represents a filled area bounded by a clockwise boundary.
/// Everything "right of" the boundary is considered "inside"
#[derive(Clone, Debug)]
#[cfg_attr(feature = "compact_containers", derive(Compact))]
#[cfg_attr(feature = "serde-serialization", derive(Serialize, Deserialize))]
pub struct PrimitiveArea {
    pub boundary: ClosedLinePath,
}

impl PrimitiveArea {
    pub fn new(boundary: ClosedLinePath) -> PrimitiveArea {
        PrimitiveArea { boundary }
    }

    pub fn fully_contains(&self, other: &PrimitiveArea) -> bool {
        let n_intersections = (&self.boundary, &other.boundary).intersect().len();

        n_intersections <= 1
            && other
                .boundary
                .path()
                .segments()
                .all(|other_segment| self.contains(other_segment.start()))
    }

    pub fn winding_number(&self, point: P2) -> f32 {
        (self
            .boundary
            .path()
            .segments()
            .map(|segment| segment.winding_angle(point))
            .sum::<f32>() / (2.0 * PI))
            .round()
    }

    pub fn area(&self) -> N {
        // http://mathworld.wolfram.com/PolygonArea.html
        0.5f32 * self.boundary.path().points.windows(2).map(|pair| {
            let (p1, p2) = (pair[0], pair[1]);
            p1.x * p2.y - p2.x * p1.y
        }).sum::<N>()
    }
}

impl PointContainer for PrimitiveArea {
    fn location_of(&self, point: P2) -> AreaLocation {
        if self.boundary.path().includes(point) {
            AreaLocation::Boundary
        } else if self.winding_number(point) == 0.0 {
            AreaLocation::Outside
        } else {
            AreaLocation::Inside
        }
    }
}

impl<'a> RoughEq for &'a PrimitiveArea {
    fn rough_eq_by(&self, other: Self, tolerance: N) -> bool {
        (&self.boundary).rough_eq_by(&other.boundary, tolerance)
    }
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "compact_containers", derive(Compact))]
#[cfg_attr(feature = "serde-serialization", derive(Serialize, Deserialize))]
pub struct Area {
    pub primitives: VecLike<PrimitiveArea>,
    pub cached_bbox: Option<BoundingBox>,
}

impl Area {
    pub fn new(primitives: VecLike<PrimitiveArea>) -> Self {
        Area {
            primitives,
            cached_bbox: None
        }
    }

    pub fn new_simple(boundary: ClosedLinePath) -> Self {
        Area {
            primitives: Some(PrimitiveArea::new(boundary)).into_iter().collect(),
            cached_bbox: None
        }
    }

    pub fn disjoint(&self) -> Vec<Area> {
        // TODO: this is not quite correct yet
        let mut groups = Vec::<VecLike<PrimitiveArea>>::new();

        for primitive in self.primitives.iter().cloned() {
            if let Some(surrounding_group_i) = groups
                .iter()
                .position(|group| group[0].fully_contains(&primitive))
            {
                groups[surrounding_group_i].push(primitive);
            } else if let Some(surrounded_group_i) = groups
                .iter()
                .position(|group| primitive.fully_contains(&group[0]))
            {
                groups[surrounded_group_i].insert(0, primitive);
            } else {
                groups.push(Some(primitive).into_iter().collect());
            }
        }

        groups.into_iter().map(Area::new).collect()
    }

    fn winding_number(&self, point: P2) -> f32 {
        self.primitives
            .iter()
            .map(|primitive| primitive.winding_number(point))
            .sum()
    }

    pub fn area(&self) -> N {
        self.primitives.iter().map(PrimitiveArea::area).sum()
    }
}

impl PointContainer for Area {
    fn location_of(&self, point: P2) -> AreaLocation {
        if self
            .primitives
            .iter()
            .any(|primtive| primtive.boundary.path().includes(point))
        {
            AreaLocation::Boundary
        } else if self.winding_number(point) == 0.0 {
            AreaLocation::Outside
        } else {
            AreaLocation::Inside
        }
    }
}

impl HasBoundingBox for Area {
    fn bounding_box(&self) -> BoundingBox {
        if let Some(cached) = self.cached_bbox {
            cached
        } else {
            let bbox = self
                .primitives
                .iter()
                .flat_map(|primitive_area| {
                    primitive_area
                        .boundary
                        .path()
                        .segments()
                        .map(|segment| segment.bounding_box())
                })
                .collect();
            
            unsafe {
                // we should use a Cell here, but Compact doesn't support that yet
                let self_mut: *mut Area = ::std::mem::transmute(self);
                (&mut *self_mut).cached_bbox = Some(bbox);
            }
            bbox
        }
    }
}

impl<'a> RoughEq for &'a Area {
    fn rough_eq_by(&self, other: Self, tolerance: N) -> bool {
        self.primitives.len() == other.primitives.len()
            && self.primitives.iter().all(|own_primitive| {
                other
                    .primitives
                    .iter()
                    .any(|other_primitive| own_primitive.rough_eq_by(other_primitive, tolerance))
            })
    }
}

use line_path::ConcatError;

#[derive(Debug)]
pub enum AreaError {
    WeldingShouldWork(ConcatError),
    LeftOver(String),
}

impl Area {
    pub fn from_pieces(mut paths: Vec<LinePath>) -> Result<Area, AreaError> {
        // println!(
        //     "PATHS \n{:#?}",
        //     paths
        //         .iter()
        //         .map(|path| format!(
        //             "Path: {:?}",
        //             path.points
        //                 .iter()
        //                 .map(|p| format!("{}", p))
        //                 .collect::<Vec<_>>()
        //         ))
        //         .collect::<Vec<_>>()
        // );

        let mut complete_paths = Vec::<ClosedLinePath>::new();

        let mut combining_tolerance = THICKNESS;

        while !paths.is_empty() && combining_tolerance < 1.0 {
            join_within_vec(&mut paths, |path_a, path_b| {
                if path_b
                    .start()
                    .rough_eq_by(path_a.end(), combining_tolerance)
                {
                    Ok(Some(
                        path_a
                            .concat_weld(&path_b, combining_tolerance)
                            .map_err(AreaError::WeldingShouldWork)?,
                    ))
                } else {
                    Ok(None)
                }
            })?;

            paths.retain(|path| {
                if path.length() < combining_tolerance {
                    false
                } else if let Some(closed) = ClosedLinePath::try_clone_from(path) {
                    complete_paths.push(closed);
                    false
                } else if (path.start() - path.end()).norm() <= combining_tolerance {
                    if let Some(welded_path) =
                        path.with_new_start_and_end(path.start(), path.start())
                    {
                        complete_paths.push(
                            ClosedLinePath::new(welded_path).expect("Welded should be closed"),
                        );
                        false
                    } else {
                        true
                    }
                } else {
                    true
                }
            });

            combining_tolerance *= 2.0;
        }

        if !paths.is_empty() {
            let min_distance = paths
                .iter()
                .map(|other| OrderedFloat((paths[0].start() - other.end()).norm()))
                .min()
                .expect("should have a min");

            return Err(AreaError::LeftOver(format!(
                "Start to closest end: {}\n{}\n\n{}",
                min_distance,
                "SVG MISSING", //self.debug_svg(),
                format!(
                    r#"<path d="{}" stroke="rgba(0, 255, 0, 0.8)"/>"#,
                    paths[0].to_svg()
                )
            )));
        }

        Ok(Area::new(
            complete_paths.into_iter().map(PrimitiveArea::new).collect(),
        ))
    }
}

#[test]
fn area_area_test() {
    assert_rough_eq_by!(Area::new_simple(ClosedLinePath::new(LinePath::new(vec![
        P2::new(0.0, 0.0),
        P2::new(3.0, 0.0),
        P2::new(3.0, 4.0),
        P2::new(0.0, 4.0),
        P2::new(0.0, 0.0),
    ]).unwrap()).unwrap()).area(), 12.0, 0.1)
}
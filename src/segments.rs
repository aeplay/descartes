use angles::{signed_angle_to, WithUniqueOrthogonal};
use rough_eq::THICKNESS;
use {P2, Rotation2, V2, N, PI};

pub const MIN_LINE_LENGTH: N = 0.01;
pub const MIN_ARC_LENGTH: N = MIN_LINE_LENGTH;

pub trait Segment {
    fn start(&self) -> P2;
    fn end(&self) -> P2;
    fn length(&self) -> N;
    fn start_direction(&self) -> V2;
    fn end_direction(&self) -> V2;
    fn subdivisions_without_end(&self, max_angle: N) -> Vec<P2>;
}

#[derive(Copy, Clone, Debug)]
pub struct LineSegment {
    start: P2,
    end: P2,
}

impl LineSegment {
    pub fn new(start: P2, end: P2) -> Self {
        LineSegment { start, end }
    }

    pub fn direction(&self) -> V2 {
        (self.end - self.start).normalize()
    }
}

impl Segment for LineSegment {
    fn start(&self) -> P2 {
        self.start
    }

    fn end(&self) -> P2 {
        self.end
    }

    fn length(&self) -> N {
        (self.start - self.end).norm()
    }

    fn start_direction(&self) -> V2 {
        self.direction()
    }

    fn end_direction(&self) -> V2 {
        self.direction()
    }

    fn subdivisions_without_end(&self, _: N) -> Vec<P2> {
        vec![self.start]
    }
}

impl LineSegment {
    pub fn along(&self, distance: N) -> P2 {
        self.start + distance * self.direction()
    }

    pub fn midpoint(&self) -> P2 {
        P2::from_coordinates((self.start.coords + self.end.coords) / 2.0)
    }

    pub fn project_with_tolerance(&self, point: P2, tolerance: N) -> Option<(N, P2)> {
        if (point - self.start).norm() < tolerance {
            Some((0.0, self.start))
        } else if (point - self.end).norm() < tolerance {
            Some((self.length(), self.end))
        } else {
            let direction = self.direction();
            let line_offset = direction.dot(&(point - self.start));
            if line_offset >= 0.0 && line_offset <= self.length() {
                Some((line_offset, self.start + line_offset * direction))
            } else {
                None
            }
        }
    }

    pub fn project_with_max_distance(
        &self,
        point: P2,
        tolerance: N,
        max_distance: N,
    ) -> Option<(N, P2)> {
        self.project_with_tolerance(point, tolerance)
            .and_then(|(along, projected_point)| {
                if (projected_point - point).norm() <= max_distance {
                    Some((along, projected_point))
                } else {
                    None
                }
            })
    }

    pub fn winding_angle(&self, point: P2) -> N {
        signed_angle_to(self.start - point, self.end - point)
    }

    pub fn side_of(&self, point: P2) -> N {
        self.winding_angle(point).signum()
    }

    pub fn is_point_left_of(&self, point: P2) -> bool {
        self.winding_angle(point) > 0.0
    }

    pub fn is_point_right_of(&self, point: P2) -> bool {
        self.winding_angle(point) < 0.0
    }

    pub fn signed_distance_of(&self, point: P2) -> N {
        let direction_orth = self.direction().orthogonal_left();
        (point - self.start).dot(&direction_orth)
    }
}

use bbox::{BoundingBox, HasBoundingBox};

impl HasBoundingBox for LineSegment {
    fn bounding_box(&self) -> BoundingBox {
        BoundingBox {
            min: P2::new(
                self.start.x.min(self.end.x) - THICKNESS,
                self.start.y.min(self.end.y) - THICKNESS,
            ),
            max: P2::new(
                self.start.x.max(self.end.x) + THICKNESS,
                self.start.y.max(self.end.y) + THICKNESS,
            ),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct ArcSegment {
    start: P2,
    apex: P2,
    end: P2,
}

impl ArcSegment {
    pub fn new(start: P2, apex: P2, end: P2) -> Option<ArcSegment> {
        if (start - apex).norm() < MIN_LINE_LENGTH
            || (end - apex).norm() < MIN_LINE_LENGTH
            || (start - end).norm() < MIN_ARC_LENGTH
        {
            None
        } else {
            let segment = ArcSegment { start, apex, end };
            let center = segment.center();
            if center.x.is_nan() || center.y.is_nan() || center.x.is_infinite() || center.y.is_infinite() {
                None
            } else {
                Some(segment)
            }
        }
    }

    pub fn minor_arc_with_center(start: P2, center: P2, end: P2) -> Option<ArcSegment> {
        let center_to_start = start - center;
        let center_to_end = end - center;
        let radius = center_to_start.norm();

        let sum = center_to_start + center_to_end;

        let apex = if sum.norm() > 0.01 {
            center + sum.normalize() * radius
        } else {
            let signed_angle_span = signed_angle_to(center_to_start, center_to_end);
            let center_to_apex = Rotation2::new(signed_angle_span / 2.0) * center_to_start;
            center + center_to_apex
        };

        // TODO: avoid redundant calculation of center, but still check for validity somehow
        ArcSegment::new(start, apex, end)
    }

    pub fn minor_arc_with_start_direction(
        start: P2,
        start_direction: V2,
        end: P2,
    ) -> Option<ArcSegment> {
        let signed_radius = {
            let half_chord = (end - start) / 2.0;
            half_chord.norm_squared() / start_direction.orthogonal_right().dot(&half_chord)
        };

        let center = start + signed_radius * start_direction.orthogonal_right();

        Self::minor_arc_with_center(start, center, end)
    }

    pub fn apex(&self) -> P2 {
        self.apex
    }

    #[inline]
    pub fn center(&self) -> P2 {
        // https://en.wikipedia.org/wiki/Circumscribed_circle#Circumcenter_coordinates
        let (a_abs, b_abs, c_abs) = (self.start, self.apex, self.end);
        let (b, c) = (b_abs - a_abs, c_abs - a_abs);
        let d_inv = 1.0 / (2.0 * (b.x * c.y - b.y * c.x));
        let (b_norm_sq, c_norm_sq) = (b.norm_squared(), c.norm_squared());
        let center_x = d_inv * (c.y * b_norm_sq - b.y * c_norm_sq);
        let center_y = d_inv * (b.x * c_norm_sq - c.x * b_norm_sq);
        a_abs + V2::new(center_x, center_y)
    }

    pub fn radius(&self) -> N {
        (self.start - self.center()).norm()
    }

    pub fn signed_angle_span(&self) -> N {
        let center = self.center();
        signed_angle_to(self.start - center, self.end - center)
    }

    pub fn is_minor(&self) -> bool {
        self.signed_angle_span() * LineSegment::new(self.start, self.end).side_of(self.apex()) < 0.0
    }
}

impl Segment for ArcSegment {
    fn start(&self) -> P2 {
        self.start
    }

    fn end(&self) -> P2 {
        self.end
    }

    fn start_direction(&self) -> V2 {
        let center = self.center();
        let center_to_start_orth = (self.start - center).orthogonal_right();

        if LineSegment::new(self.start, self.end).is_point_left_of(self.apex) {
            center_to_start_orth
        } else {
            -center_to_start_orth
        }
    }

    fn end_direction(&self) -> V2 {
        let center = self.center();
        let center_to_end_orth = (self.end - center).orthogonal_right();

        if LineSegment::new(self.start, self.end).is_point_left_of(self.apex) {
            center_to_end_orth
        } else {
            -center_to_end_orth
        }
    }

    fn length(&self) -> N {
        let simple_angle_span = self.signed_angle_span().abs();

        let angle_span = if self.is_minor() {
            simple_angle_span
        } else {
            2.0 * PI - simple_angle_span
        };

        self.radius() * angle_span
    }

    fn subdivisions_without_end(&self, max_angle: N) -> Vec<P2> {
        let center = self.center();
        let signed_angle_span = signed_angle_to(self.start - center, self.end - center);

        if signed_angle_span.is_nan() {
            println!("KAPUTT {:?} {:?}", self, center);
        }

        let subdivisions = (signed_angle_span.abs() / max_angle).floor() as usize;
        let subdivision_angle = signed_angle_span / (subdivisions as f32);

        let mut pointer = self.start - center;

        let mut maybe_previous_point: Option<P2> = None;

        (0..subdivisions.max(1))
            .into_iter()
            .filter_map(|_| {
                let point = center + pointer;
                pointer = Rotation2::new(subdivision_angle) * pointer;

                if let Some(previous_point) = maybe_previous_point {
                    if (point - previous_point).norm() > 2.0 * THICKNESS {
                        maybe_previous_point = Some(point);
                        Some(point)
                    } else {
                        None
                    }
                } else {
                    maybe_previous_point = Some(point);
                    Some(point)
                }
            })
            .collect::<Vec<_>>()
    }
}

#[derive(Copy, Clone, Debug)]
pub enum ArcOrLineSegment {
    Line(LineSegment),
    Arc(ArcSegment),
}

impl ArcOrLineSegment {
    pub fn line_unchecked(start: P2, end: P2) -> ArcOrLineSegment {
        ArcOrLineSegment::Line(LineSegment { start, end })
    }

    pub fn arc_unchecked(start: P2, apex: P2, end: P2) -> ArcOrLineSegment {
        ArcOrLineSegment::Arc(ArcSegment { start, apex, end })
    }
}

impl Segment for ArcOrLineSegment {
    fn start(&self) -> P2 {
        match *self {
            ArcOrLineSegment::Line(line) => line.start(),
            ArcOrLineSegment::Arc(arc) => arc.start(),
        }
    }
    fn end(&self) -> P2 {
        match *self {
            ArcOrLineSegment::Line(line) => line.end(),
            ArcOrLineSegment::Arc(arc) => arc.end(),
        }
    }
    fn length(&self) -> N {
        match *self {
            ArcOrLineSegment::Line(line) => line.length(),
            ArcOrLineSegment::Arc(arc) => arc.length(),
        }
    }
    fn start_direction(&self) -> V2 {
        match *self {
            ArcOrLineSegment::Line(line) => line.start_direction(),
            ArcOrLineSegment::Arc(arc) => arc.start_direction(),
        }
    }
    fn end_direction(&self) -> V2 {
        match *self {
            ArcOrLineSegment::Line(line) => line.end_direction(),
            ArcOrLineSegment::Arc(arc) => arc.end_direction(),
        }
    }
    fn subdivisions_without_end(&self, max_angle: N) -> Vec<P2> {
        match *self {
            ArcOrLineSegment::Line(line) => line.subdivisions_without_end(max_angle),
            ArcOrLineSegment::Arc(arc) => arc.subdivisions_without_end(max_angle),
        }
    }
}

#[cfg(test)]
const TOL: f32 = 0.0001;

#[cfg(test)]
use rough_eq::RoughEq;

#[test]
fn minor_arc_test() {
    let o = V2::new(10.0, 5.0);
    let minor_arc = ArcSegment::new(
        P2::new(0.0, 1.0) + o,
        P2::new(-3.0f32.sqrt() / 2.0, 0.5) + o,
        P2::new(-1.0, 0.0) + o,
    ).expect("should be valid");
    assert_rough_eq_by!(minor_arc.center(), P2::new(0.0, 0.0) + o, TOL);
    assert!(minor_arc.is_minor());
    assert_rough_eq_by!(minor_arc.length(), PI / 2.0, TOL);

    let minor_arc_rev = ArcSegment::new(
        P2::new(-1.0, 0.0) + o,
        P2::new(-3.0f32.sqrt() / 2.0, 0.5) + o,
        P2::new(0.0, 1.0) + o,
    ).expect("should be valid");
    assert_rough_eq_by!(minor_arc_rev.center(), P2::new(0.0, 0.0) + o, TOL);
    assert!(minor_arc_rev.is_minor());
    assert_rough_eq_by!(minor_arc_rev.length(), PI / 2.0, TOL);

    let minor_arc_by_center = ArcSegment::minor_arc_with_center(
        P2::new(0.0, 1.0) + o,
        P2::new(0.0, 0.0) + o,
        P2::new(-1.0, 0.0) + o,
    ).expect("should be valid");
    assert_rough_eq_by!(
        minor_arc_by_center.apex(),
        P2::new(-2.0f32.sqrt() / 2.0, 2.0f32.sqrt() / 2.0) + o,
        TOL
    );
    assert_rough_eq_by!(minor_arc_by_center.length(), PI / 2.0, TOL);

    let minor_arc_by_center_rev = ArcSegment::minor_arc_with_center(
        P2::new(-1.0, 0.0) + o,
        P2::new(0.0, 0.0) + o,
        P2::new(0.0, 1.0) + o,
    ).expect("should be valid");
    assert_rough_eq_by!(
        minor_arc_by_center_rev.apex(),
        P2::new(-2.0f32.sqrt() / 2.0, 2.0f32.sqrt() / 2.0) + o,
        TOL
    );
    assert_rough_eq_by!(minor_arc_by_center_rev.length(), PI / 2.0, TOL);

    let minor_arc_by_direction = ArcSegment::minor_arc_with_start_direction(
        P2::new(0.0, 1.0) + o,
        V2::new(-1.0, 0.0),
        P2::new(-1.0, 0.0) + o,
    ).expect("should be valid");
    assert_rough_eq_by!(
        minor_arc_by_direction.apex(),
        P2::new(-2.0f32.sqrt() / 2.0, 2.0f32.sqrt() / 2.0) + o,
        TOL
    );
    assert_rough_eq_by!(minor_arc_by_direction.length(), PI / 2.0, TOL);

    let minor_arc_by_direction_rev = ArcSegment::minor_arc_with_start_direction(
        P2::new(-1.0, 0.0) + o,
        V2::new(0.0, 1.0),
        P2::new(0.0, 1.0) + o,
    ).expect("should be valid");
    assert_rough_eq_by!(
        minor_arc_by_direction_rev.apex(),
        P2::new(-2.0f32.sqrt() / 2.0, 2.0f32.sqrt() / 2.0) + o,
        TOL
    );
    assert_rough_eq_by!(minor_arc_by_direction_rev.length(), PI / 2.0, TOL);
}

#[test]
fn colinear_apex_test() {
    assert!(ArcSegment::new(P2::new(0.0, 0.0), P2::new(1.0, 0.0), P2::new(2.0, 0.0)).is_none());
}

#[test]
fn major_arc_test() {
    let o = V2::new(10.0, 5.0);
    let major_arc = ArcSegment::new(
        P2::new(0.0, -1.0) + o,
        P2::new(-3.0f32.sqrt() / 2.0, 0.5) + o,
        P2::new(1.0, 0.0) + o,
    ).expect("should be valid");
    assert_rough_eq_by!(major_arc.center(), P2::new(0.0, 0.0) + o, TOL);
    assert!(!major_arc.is_minor());
    assert_rough_eq_by!(major_arc.length(), 3.0 * PI / 2.0, TOL);

    let major_arc_rev = ArcSegment::new(
        P2::new(-1.0, 0.0) + o,
        P2::new(-3.0f32.sqrt() / 2.0, 0.5) + o,
        P2::new(0.0, -1.0) + o,
    ).expect("should be valid");
    assert_rough_eq_by!(major_arc_rev.center(), P2::new(0.0, 0.0) + o, TOL);
    assert!(!major_arc_rev.is_minor());
    assert_rough_eq_by!(major_arc_rev.length(), 3.0 * PI / 2.0, TOL);
}

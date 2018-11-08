use areas::{Area, AreaError, PrimitiveArea};
use closed_line_path::ClosedLinePath;
use embedding::Embedding;
use line_path::LinePath;
use rough_eq::{RoughEq, THICKNESS};
use segments::Segment;
use std::hash::Hash;
use util::join_within_vec;
use util::SmallSet;
use bbox::HasBoundingBox;
use {P2, N};

#[derive(Clone, Debug)]
pub struct AreaLabel<L: PartialEq + Eq + Clone + Hash> {
    pub own_right_label: L,
    pub right_labels: SmallSet<[L; 4]>,
    pub left_labels: SmallSet<[L; 4]>,
}

pub struct AreaEmbedding<L: PartialEq + Eq + Clone + Hash> {
    embedding: Embedding<AreaLabel<L>>,
    areas: Vec<(Area, L)>,
    calculated_containment: bool,
}

impl<L: PartialEq + Eq + Clone + Hash + ::std::fmt::Debug> AreaEmbedding<L> {
    pub fn new(cell_width: N) -> Self {
        AreaEmbedding {
            embedding: Embedding::new(cell_width),
            areas: Vec::new(),
            calculated_containment: false,
        }
    }

    pub fn insert(&mut self, area: Area, label: L) {
        for primitive in area.primitives.iter() {
            self.embedding.insert(
                primitive.boundary.0.clone(),
                AreaLabel {
                    own_right_label: label.clone(),
                    left_labels: SmallSet::new(),
                    right_labels: SmallSet::new(),
                },
            );
        }
        self.areas.push((area, label));
        self.calculated_containment = false;
    }

    fn ensure_containment_calculated(&mut self) {
        if self.calculated_containment {
            return;
        }

        let keys = self.embedding.pieces.keys().collect::<Vec<_>>();
        for piece_idx in keys {
            let (midpoint, midpoint_direction, own_label) = {
                let (piece, area_label) = &self.embedding.pieces[piece_idx];
                let half_along = piece.length() / 2.0;
                (
                    piece.along(half_along),
                    piece.direction_along(half_along),
                    area_label.own_right_label.clone(),
                )
            };

            // inspired by http://geomalgorithms.com/a03-_inclusion.html
            for (area, label) in &self.areas {
                enum PieceAreaRelation {
                    CoincidentForward,
                    CoincidentBackward,
                    Inside,
                    Outside
                }

                if label != &own_label {
                    if !area.bounding_box().contains(midpoint) {
                        continue;
                    }

                    let mut winding_number = 0;
                    let mut coincident_relation = None;

                    'area: for primitive in &area.primitives {
                        for segment in primitive.boundary.path().segments() {
                            let signed_distance = segment.signed_distance_of(midpoint);
                            let segment_direction = segment.direction();

                            if signed_distance.abs() < THICKNESS {
                                let line_offset =
                                    segment_direction.dot(&(midpoint - segment.start()));
                                if line_offset >= -THICKNESS && line_offset <= segment.length() + THICKNESS {
                                    // on the segment
                                    if midpoint_direction.dot(&segment_direction) > 0.0 {
                                        coincident_relation =
                                            Some(PieceAreaRelation::CoincidentForward);
                                    } else {
                                        coincident_relation =
                                            Some(PieceAreaRelation::CoincidentBackward);
                                    }
                                    break 'area;
                                }
                            }

                            if segment.start().y <= midpoint.y {
                                // start y <= P.y
                                if segment.end().y > midpoint.y {
                                    // an upward crossing
                                    if signed_distance > 0.0 {
                                        // P left of  edge
                                        winding_number += 1; // have  a valid up intersect
                                    }
                                }
                            } else {
                                // start y > P.y (no test needed)
                                if segment.end().y <= midpoint.y {
                                    // a downward crossing
                                    if signed_distance < 0.0 {
                                        winding_number -= 1; // have  a valid down intersect
                                    }
                                }
                            }
                        }
                    }

                    let relation = coincident_relation.unwrap_or(if winding_number == 0 {
                        PieceAreaRelation::Outside
                    } else {
                        PieceAreaRelation::Inside
                    });

                    let (_, piece_area_label) = &mut self.embedding.pieces[piece_idx];

                    match relation {
                        PieceAreaRelation::Inside => {
                            piece_area_label.right_labels.insert(label.clone());
                            piece_area_label.left_labels.insert(label.clone());
                        }
                        PieceAreaRelation::CoincidentForward => {
                            piece_area_label.right_labels.insert(label.clone());
                        }
                        PieceAreaRelation::CoincidentBackward => {
                            piece_area_label.left_labels.insert(label.clone());
                        }
                        _ => {}
                    }
                }
            }
        }

        self.calculated_containment = true;
    }

    pub fn view<'a>(&'a mut self, inital_filter: AreaFilter<L>) -> AreaEmbeddingView<'a, L>
    where
        L: 'static,
    {
        self.ensure_containment_calculated();
        AreaEmbeddingView {
            area_embedding: self,
            area_filter: inital_filter,
        }
    }
}

pub enum AreaFilter<L: PartialEq + Eq + Clone + Hash> {
    Always,
    Function(Box<Fn(&[L]) -> bool>),
    Not(Box<AreaFilter<L>>),
    And(Box<AreaFilter<L>>, Box<AreaFilter<L>>),
    Or(Box<AreaFilter<L>>, Box<AreaFilter<L>>),
}

impl<L: PartialEq + Eq + Clone + Hash + 'static> AreaFilter<L> {
    fn apply(&self, labels: &[L]) -> bool {
        match self {
            AreaFilter::Always => true,
            AreaFilter::Function(ref filter_fn) => filter_fn(labels),
            AreaFilter::Not(filter) => !filter.apply(labels),
            AreaFilter::And(left, right) => left.apply(labels) && right.apply(labels),
            AreaFilter::Or(left, right) => left.apply(labels) || right.apply(labels),
        }
    }

    pub fn has(label: L) -> Self {
        AreaFilter::Function(Box::new(move |label_set| {
            label_set.contains(&label)
        }))
    }

    pub fn any_of(unioned_labels: Vec<L>) -> Self {
        AreaFilter::Function(Box::new(move |label_set| {
            unioned_labels
                .iter()
                .any(|unioned_label| label_set.contains(unioned_label))
        }))
    }

    pub fn all_of(intersected_labels: Vec<L>) -> Self {
        AreaFilter::Function(Box::new(move |label_set| {
            intersected_labels
                .iter()
                .all(|intersected_label| label_set.contains(intersected_label))
        }))
    }

    pub fn and(self, other: Self) -> Self {
        AreaFilter::And(Box::new(self), Box::new(other))
    }

    pub fn or(self, other: Self) -> Self {
        AreaFilter::Or(Box::new(self), Box::new(other))
    }

    pub fn not(self) -> Self {
        AreaFilter::Not(Box::new(self))
    }
}

pub struct AreaEmbeddingView<'a, L: PartialEq + Eq + Clone + Hash + 'a> {
    area_embedding: &'a AreaEmbedding<L>,
    area_filter: AreaFilter<L>,
}

impl<'a, L: PartialEq + Eq + Clone + Hash + 'static> AreaEmbeddingView<'a, L> {
    pub fn get_unique_pieces(&self) -> Vec<(LinePath, AreaLabel<L>)> {
        let mut pieces = self
            .area_embedding
            .embedding
            .pieces
            .iter()
            .filter_map(|(piece, area_labels)| {
                let left_labels = &area_labels.left_labels;
                let mut right_labels = area_labels.right_labels.clone();
                right_labels.insert(area_labels.own_right_label.clone());

                let filter_applies_to_left = self.area_filter.apply(&left_labels);
                let filter_applies_to_right = self.area_filter.apply(&right_labels);

                if !filter_applies_to_left && filter_applies_to_right {
                    // forward boundary
                    Some((piece.clone(), area_labels.clone()))
                } else if filter_applies_to_left && !filter_applies_to_right {
                    // backward boundary
                    Some((piece.reverse(), area_labels.clone()))
                } else {
                    // inside or outside
                    None
                }
            })
            .collect::<Vec<_>>();

        // only keep unique pieces
        let mut midpoints = vec![];

        pieces.retain(|(piece, _label)| {
            let piece_midpoint = piece.along(piece.length() / 2.0);
            let is_new = midpoints
                .iter()
                .all(|old_midpoint: &P2| (piece_midpoint - old_midpoint).norm() > THICKNESS);
            if is_new {
                midpoints.push(piece_midpoint);
                true
            } else {
                false
            }
        });

        pieces
    }

    pub fn get_area(&self) -> Result<Area, AreaError> {
        let pieces = self
            .get_unique_pieces()
            .into_iter()
            .map(|(piece, _label)| piece)
            .collect::<Vec<_>>();

        Area::from_pieces(pieces)
    }

    pub fn get_areas_with_pieces(
        &self,
    ) -> Result<Vec<(Area, Vec<(LinePath, AreaLabel<L>)>)>, AreaError> {
        let mut wip_boundaries_and_corresponding_pieces: Vec<_> = self
            .get_unique_pieces()
            .into_iter()
            .map(|(piece, label)| (piece.clone(), vec![(piece, label)]))
            .collect();

        let mut complete_paths = Vec::<(ClosedLinePath, Vec<(LinePath, AreaLabel<L>)>)>::new();

        let mut combining_tolerance = THICKNESS;
        const MAX_COMBINING_TOLERANCE: N = 1.0;

        while !wip_boundaries_and_corresponding_pieces.is_empty()
            && combining_tolerance < MAX_COMBINING_TOLERANCE
        {
            join_within_vec(
                &mut wip_boundaries_and_corresponding_pieces,
                |(path_a, pieces_a), (path_b, pieces_b)| {
                    if path_b
                        .start()
                        .rough_eq_by(path_a.end(), combining_tolerance)
                    {
                        Ok(Some((
                            path_a
                                .concat_weld(&path_b, combining_tolerance)
                                .map_err(AreaError::WeldingShouldWork)?,
                            pieces_a.into_iter().chain(pieces_b).cloned().collect(),
                        )))
                    } else {
                        Ok(None)
                    }
                },
            )?;

            wip_boundaries_and_corresponding_pieces.retain(|(path, pieces)| {
                if path.length() < combining_tolerance {
                    false
                } else if let Some(closed) = ClosedLinePath::try_clone_from(path) {
                    complete_paths.push((closed, pieces.clone()));
                    false
                } else if (path.start() - path.end()).norm() <= combining_tolerance {
                    if let Some(welded_path) =
                        path.with_new_start_and_end(path.start(), path.start())
                    {
                        complete_paths.push((
                            ClosedLinePath::new(welded_path).expect("Welded should be closed"),
                            pieces.clone(),
                        ));
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

        // if !wip_boundaries_and_corresponding_pieces.is_empty() {
        //     let min_distance = wip_boundaries_and_corresponding_pieces
        //         .iter()
        //         .map(|other| {
        //             OrderedFloat(
        //                 (wip_boundaries_and_corresponding_pieces[0].0.start() - other.0.end())
        //                     .norm(),
        //             )
        //         })
        //         .min()
        //         .expect("should have a min");

        //     return Err(AreaError::LeftOver(format!(
        //         "Start to closest end: {}\n{}\n\n{}",
        //         min_distance,
        //         "SVG MISSING", //self.debug_svg(),
        //         format!(
        //             r#"<path d="{}" stroke="rgba(0, 255, 0, 0.8)"/>"#,
        //             wip_boundaries_and_corresponding_pieces[0].0.to_svg()
        //         )
        //     )));
        // }

        let mut disjoint_area_groups = complete_paths
            .into_iter()
            .map(|(boundary_path, pieces)| (vec![PrimitiveArea::new(boundary_path)], pieces))
            .collect::<Vec<_>>();

        let _: Result<(), ()> = join_within_vec(
            &mut disjoint_area_groups,
            |(boundaries_a, pieces_a), (boundaries_b, pieces_b)| {
                // TODO: maybe use a faster way to check containment here?
                if boundaries_a[0].fully_contains(&boundaries_b[0]) {
                    Ok(Some((
                        boundaries_a
                            .iter()
                            .chain(boundaries_b.iter())
                            .cloned()
                            .collect(),
                        pieces_a.iter().chain(pieces_b.iter()).cloned().collect(),
                    )))
                } else {
                    Ok(None)
                }
            },
        );

        Ok(disjoint_area_groups
            .into_iter()
            .map(|(primitives_group, pieces)| (Area::new(primitives_group.into()), pieces))
            .collect())
    }
}

#[test]
fn area_embedding_test() {
    use closed_line_path::ClosedLinePath;
    use line_path::LinePath;
    use rough_eq::RoughEq;
    use P2;

    //      _____
    //   __|__   |
    //  |  |__|__|
    //  |_____|
    //

    let area_a = Area::new_simple(
        ClosedLinePath::new(
            LinePath::new(vec![
                P2::new(0.0, 0.0),
                P2::new(0.0, 1.0),
                P2::new(1.0, 1.0),
                P2::new(1.0, 0.0),
                P2::new(0.0, 0.0),
            ]).unwrap(),
        ).unwrap(),
    );
    let area_b = Area::new_simple(
        ClosedLinePath::new(
            LinePath::new(vec![
                P2::new(0.5, 0.5),
                P2::new(0.5, 1.5),
                P2::new(1.5, 1.5),
                P2::new(1.5, 0.5),
                P2::new(0.5, 0.5),
            ]).unwrap(),
        ).unwrap(),
    );

    #[derive(Clone, Hash, Eq, PartialEq, Debug)]
    enum TestLabel {
        A,
        B,
        C,
    }

    let mut embedding = AreaEmbedding::new(0.2);
    embedding.insert(area_a, TestLabel::A);
    embedding.insert(area_b, TestLabel::B);

    {
        let union_area_ab = embedding.view(AreaFilter::any_of(vec![TestLabel::A, TestLabel::B]));

        assert_rough_eq_by!(
            &union_area_ab.get_area().expect("Should be valid area"),
            &Area::new_simple(
                ClosedLinePath::new(
                    LinePath::new(vec![
                        P2::new(0.0, 0.0),
                        P2::new(0.0, 1.0),
                        P2::new(0.5, 1.0),
                        P2::new(0.5, 1.5),
                        P2::new(1.5, 1.5),
                        P2::new(1.5, 0.5),
                        P2::new(1.0, 0.5),
                        P2::new(1.0, 0.0),
                        P2::new(0.0, 0.0),
                    ]).unwrap(),
                ).unwrap()
            ),
            0.01
        );
    }

    //   __.__ __.
    //  |  |  |  |
    //  |__|__|__|
    //

    let area_c = Area::new_simple(
        ClosedLinePath::new(
            LinePath::new(vec![
                P2::new(0.5, 0.0),
                P2::new(0.5, 1.0),
                P2::new(1.5, 1.0),
                P2::new(1.5, 0.0),
                P2::new(0.5, 0.0),
            ]).unwrap(),
        ).unwrap(),
    );

    embedding.insert(area_c, TestLabel::C);

    {
        let union_area_ac = embedding.view(AreaFilter::any_of(vec![TestLabel::A, TestLabel::C]));

        assert_rough_eq_by!(
            &union_area_ac.get_area().expect("Should be valid area"),
            &Area::new_simple(
                ClosedLinePath::new(
                    LinePath::new(vec![
                        P2::new(0.0, 0.0),
                        P2::new(0.0, 1.0),
                        P2::new(0.5, 1.0),
                        P2::new(1.0, 1.0),
                        P2::new(1.5, 1.0),
                        P2::new(1.5, 0.5),
                        P2::new(1.5, 0.0),
                        P2::new(1.0, 0.0),
                        P2::new(0.5, 0.0),
                        P2::new(0.0, 0.0),
                    ]).unwrap(),
                ).unwrap()
            ),
            0.01
        );
    }

    {
        let union_area_abc = embedding.view(AreaFilter::any_of(vec![
            TestLabel::A,
            TestLabel::B,
            TestLabel::C,
        ]));

        assert_rough_eq_by!(
            &union_area_abc.get_area().expect("Should be valid area"),
            &Area::new_simple(
                ClosedLinePath::new(
                    LinePath::new(vec![
                        P2::new(0.0, 0.0),
                        P2::new(0.0, 1.0),
                        P2::new(0.5, 1.0),
                        P2::new(0.5, 1.5),
                        P2::new(1.5, 1.5),
                        P2::new(1.5, 1.0),
                        P2::new(1.5, 0.5),
                        P2::new(1.5, 0.0),
                        P2::new(1.0, 0.0),
                        P2::new(0.5, 0.0),
                        P2::new(0.0, 0.0),
                    ]).unwrap(),
                ).unwrap()
            ),
            0.01
        );
    }

    {
        let a_int_c_minus_b = embedding.view(
            AreaFilter::all_of(vec![TestLabel::A, TestLabel::C])
                .and(AreaFilter::any_of(vec![TestLabel::B]).not()),
        );

        assert_rough_eq_by!(
            &a_int_c_minus_b.get_area().expect("Should be valid area"),
            &Area::new_simple(
                ClosedLinePath::new(
                    LinePath::new(vec![
                        P2::new(0.5, 0.5),
                        P2::new(1.0, 0.5),
                        P2::new(1.0, 0.0),
                        P2::new(0.5, 0.0),
                        P2::new(0.5, 0.5),
                    ]).unwrap(),
                ).unwrap()
            ),
            0.01
        );
    }

    //      _____
    //   __|__ __|
    //  |  |XX|XX|
    //  |__|XX|__|
    //

    {
        let union_of_intersections =
            embedding.view(AreaFilter::Function(Box::new(|labels| labels.len() >= 2)));

        assert_rough_eq_by!(
            &union_of_intersections
                .get_area()
                .expect("Should be valid area"),
            &Area::new_simple(
                ClosedLinePath::new(
                    LinePath::new(vec![
                        P2::new(0.5, 1.0),
                        P2::new(1.0, 1.0),
                        P2::new(1.5, 1.0),
                        P2::new(1.5, 0.5),
                        P2::new(1.0, 0.5),
                        P2::new(1.0, 0.0),
                        P2::new(0.5, 0.0),
                        P2::new(0.5, 0.5),
                        P2::new(0.5, 1.0),
                    ]).unwrap(),
                ).unwrap()
            ),
            0.01
        );

        let (area, pieces) = union_of_intersections
            .get_areas_with_pieces()
            .expect("Should be valid area")
            .into_iter()
            .next()
            .unwrap();

        assert_rough_eq_by!(
            &area,
            &Area::new_simple(
                ClosedLinePath::new(
                    LinePath::new(vec![
                        P2::new(0.5, 1.0),
                        P2::new(1.0, 1.0),
                        P2::new(1.5, 1.0),
                        P2::new(1.5, 0.5),
                        P2::new(1.0, 0.5),
                        P2::new(1.0, 0.0),
                        P2::new(0.5, 0.0),
                        P2::new(0.5, 0.5),
                        P2::new(0.5, 1.0),
                    ]).unwrap(),
                ).unwrap()
            ),
            0.01
        );

        assert_rough_eq_by!(
            &pieces[0].0,
            &LinePath::new(vec![P2::new(0.5, 1.0), P2::new(1.0, 1.0)]).unwrap(),
            0.01
        );

        assert_eq!(&pieces[0].1.left_labels[..], &[TestLabel::B]);
        assert_eq!(&pieces[0].1.right_labels[..], &[TestLabel::B, TestLabel::C]);
        assert_eq!(pieces[0].1.own_right_label, TestLabel::A);

        assert_eq!(&pieces[1].1.left_labels[..], &[TestLabel::B]);
        assert_eq!(&pieces[1].1.right_labels[..], &[TestLabel::B]);
        assert_eq!(pieces[1].1.own_right_label, TestLabel::C);

        assert_eq!(&pieces[2].1.left_labels[..], &[]);
        assert_eq!(&pieces[2].1.right_labels[..], &[TestLabel::C]);
        assert_eq!(pieces[2].1.own_right_label, TestLabel::B);

        assert_eq!(&pieces[3].1.left_labels[..], &[TestLabel::C]);
        assert_eq!(&pieces[3].1.right_labels[..], &[TestLabel::C]);
        assert_eq!(pieces[3].1.own_right_label, TestLabel::B);

        assert_eq!(&pieces[4].1.left_labels[..], &[TestLabel::C]);
        assert_eq!(&pieces[4].1.right_labels[..], &[TestLabel::C]);
        assert_eq!(pieces[4].1.own_right_label, TestLabel::A);

        assert_eq!(&pieces[5].1.left_labels[..], &[]);
        assert_eq!(&pieces[5].1.right_labels[..], &[TestLabel::C]);
        assert_eq!(pieces[5].1.own_right_label, TestLabel::A);

        assert_eq!(&pieces[6].1.left_labels[..], &[TestLabel::A]);
        assert_eq!(&pieces[6].1.right_labels[..], &[TestLabel::A]);
        assert_eq!(pieces[6].1.own_right_label, TestLabel::C);

        assert_eq!(&pieces[7].1.left_labels[..], &[TestLabel::A]);
        assert_eq!(&pieces[7].1.right_labels[..], &[TestLabel::A, TestLabel::C]);
        assert_eq!(pieces[7].1.own_right_label, TestLabel::B);
    }
}


#[test]
fn area_embedding_test_x_coords_smaller_y_coords() {
    use closed_line_path::ClosedLinePath;
    use line_path::LinePath;
    use rough_eq::RoughEq;
    use P2;

    //      _____
    //   __|__   |
    //  |  |__|__|
    //  |_____|
    //

    let area_a = Area::new_simple(
        ClosedLinePath::new(
            LinePath::new(vec![
                P2::new(0.0 - 30.0, 0.0 - 10.0),
                P2::new(0.0 - 30.0, 1.0 - 10.0),
                P2::new(1.0 - 30.0, 1.0 - 10.0),
                P2::new(1.0 - 30.0, 0.0 - 10.0),
                P2::new(0.0 - 30.0, 0.0 - 10.0),
            ]).unwrap(),
        ).unwrap(),
    );
    let area_b = Area::new_simple(
        ClosedLinePath::new(
            LinePath::new(vec![
                P2::new(0.5 - 30.0, 0.5 - 10.0),
                P2::new(0.5 - 30.0, 1.5 - 10.0),
                P2::new(1.5 - 30.0, 1.5 - 10.0),
                P2::new(1.5 - 30.0, 0.5 - 10.0),
                P2::new(0.5 - 30.0, 0.5 - 10.0),
            ]).unwrap(),
        ).unwrap(),
    );

    #[derive(Clone, Hash, Eq, PartialEq, Debug)]
    enum TestLabel {
        A,
        B,
    }

    let mut embedding = AreaEmbedding::new(0.333);
    embedding.insert(area_a, TestLabel::A);
    embedding.insert(area_b, TestLabel::B);

    {
        let union_of_intersections =
            embedding.view(AreaFilter::Function(Box::new(|labels| labels.len() >= 2)));

        assert_rough_eq_by!(
            &union_of_intersections.get_area().expect("Should be valid area"),
            &Area::new_simple(
                ClosedLinePath::new(
                    LinePath::new(vec![
                        P2::new(0.5 - 30.0, 0.5 - 10.0),
                        P2::new(0.5 - 30.0, 1.0 - 10.0),
                        P2::new(1.0 - 30.0, 1.0 - 10.0),
                        P2::new(1.0 - 30.0, 0.5 - 10.0),
                        P2::new(0.5 - 30.0, 0.5 - 10.0),
                    ]).unwrap(),
                ).unwrap()
            ),
            0.01
        );
    }
}
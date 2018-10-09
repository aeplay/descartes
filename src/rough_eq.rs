use {N, P2, V2};

// Thickness radius
pub const THICKNESS: N = 0.001;
const ROUGH_TOLERANCE: N = 0.000_000_1;

pub trait RoughEq: Sized {
    fn rough_eq(&self, other: Self) -> bool {
        self.rough_eq_by(other, ROUGH_TOLERANCE)
    }
    fn rough_eq_by(&self, other: Self, tolerance: N) -> bool;
}

impl RoughEq for N {
    fn rough_eq_by(&self, other: N, tolerance: N) -> bool {
        (self - other).abs() <= tolerance
    }
}

impl RoughEq for P2 {
    fn rough_eq_by(&self, other: P2, tolerance: N) -> bool {
        (*self - other).norm() <= tolerance
    }
}

impl RoughEq for V2 {
    fn rough_eq_by(&self, other: V2, tolerance: N) -> bool {
        (*self - other).norm() <= tolerance
    }
}

#[macro_export]
macro_rules! assert_rough_eq_by {
    ($left:expr , $right:expr, $tol:expr,) => ({
        assert_eq!($left, $right, $tol)
    });
    ($left:expr , $right:expr, $tol:expr) => ({
        match (&($left), &($right)) {
            (left_val, right_val) => {
                if !((*left_val).rough_eq_by(*right_val, $tol)) {
                    panic!("assertion failed: `(left ~= right by {})`\
                          \n\
                          \n{}\
                          \n",
                          $tol,
                          ::pretty_assertions::Comparison::new(left_val, right_val))
                }
            }
        }
    });
}
use super::{N, P2, LinePath};

impl LinePath {
    pub fn from_svg(string: &str) -> Option<Self> {
        let mut tokens = string.split_whitespace();
        let mut points = vec![];

        while let Some(command) = tokens.next() {
            if command == "M" || command == "L" {
                let x: N = tokens
                    .next()
                    .expect("Expected 1st token after M/L")
                    .parse()
                    .expect("Can't parse 1st token after M/L");
                let y: N = tokens
                    .next()
                    .expect("Expected 2nd token after M/L")
                    .parse()
                    .expect("Can't parse 2nd token after M/L");

                points.push(P2::new(x, y));
            } else if command == "Z" {
                let first_point = points[0];
                points.push(first_point)
            }
        }

        Self::new(points.into())
    }

    pub fn to_svg(&self) -> String {
        format!(
            "M {}",
            self.points
                .iter()
                .map(|point| format!("{} {}", point.x, point.y))
                .collect::<Vec<_>>()
                .join(" L ")
        )
    }
}
use {P2, V2, INFINITY, N, NEG_INFINITY};

#[cfg_attr(feature = "serde-serialization", derive(Serialize, Deserialize))]
#[derive(Copy, Clone, Debug)]
pub struct BoundingBox {
    pub min: P2,
    pub max: P2,
}

impl BoundingBox {
    pub fn infinite() -> Self {
        BoundingBox {
            min: P2::new(NEG_INFINITY, NEG_INFINITY),
            max: P2::new(INFINITY, INFINITY),
        }
    }

    pub fn overlaps(&self, other: &BoundingBox) -> bool {
        self.max.x >= other.min.x
            && other.max.x >= self.min.x
            && self.max.y >= other.min.y
            && other.max.y >= self.min.y
    }

    pub fn point(p: P2) -> Self {
        BoundingBox { min: p, max: p }
    }

    pub fn grown_by(&self, offset: N) -> Self {
        BoundingBox {
            min: self.min - V2::new(offset, offset),
            max: self.max + V2::new(offset, offset),
        }
    }

    pub fn contains(&self, point: P2) -> bool {
        self.min.x <= point.x && self.max.x >= point.x && self.min.y <= point.y && self.max.y >= point.y
    }

    pub fn extended_by(&self, other: BoundingBox) -> Self {
        BoundingBox {
            min: P2::new(self.min.x.min(other.min.x), self.min.y.min(other.min.y)),
            max: P2::new(self.max.x.max(other.max.x), self.max.y.max(other.max.y)),
        }
    }
}

impl ::std::iter::FromIterator<BoundingBox> for BoundingBox {
    fn from_iter<T: IntoIterator<Item = BoundingBox>>(collection: T) -> Self {
        let mut iter = collection.into_iter();
        let mut bbox = iter.next().expect("Should have at least one bounding box");

        for next_bbox in iter {
            bbox = bbox.extended_by(next_bbox);
        }

        bbox
    }
}

pub trait HasBoundingBox {
    fn bounding_box(&self) -> BoundingBox;
}

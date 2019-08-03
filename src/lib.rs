use std::ops::{Add, Index, IndexMut};

mod diamond;

pub use diamond::*;

/// Trait for Line-of-sight calculation algorithm.
pub trait LosAlgorithm {
    /// Compute the line-of-sight in `map` starting from `origin`.
    ///
    /// This method takes `&mut self` because `self` might use some internal cache data.
    /// This allow to avoid allocation each time this function is called.
    fn compute_los<M: MapProvider>(&mut self, origin: Coord, vision_range: u32, map: &mut M);
}

pub trait MapProvider {
    /// Return true if the cell at `coords` is blocking the view
    fn is_blocking(&self, coord: Coord) -> bool;

    /// Return the bounds of the map as [min, max]
    fn bounds(&self) -> (Coord, Coord);

    /// Mark the cell at `coord` as visible
    fn mark_as_visible(&mut self, coord: Coord);
}

/// 2D Coordinates
///
/// TOOD: use u32?
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct Coord(pub i32, pub i32);

impl Add<Coord> for Coord {
    type Output = Coord;

    fn add(self, rhs: Coord) -> Self::Output {
        Coord(self.0 + rhs.0, self.1 + rhs.1)
    }
}

impl Add<(i32, i32)> for Coord {
    type Output = Coord;

    fn add(self, rhs: (i32, i32)) -> Self::Output {
        Coord(self.0 + rhs.0, self.1 + rhs.1)
    }
}

/// 2D map implemented as a vector
#[derive(Clone, Debug, Default)]
struct Map<T> {
    height: usize,
    pub inner: Vec<T>,
}

impl<T> Map<T> {
    pub fn new((width, height): (usize, usize), initial_value: T) -> Self
    where
        T: Clone,
    {
        Self {
            height,
            inner: vec![initial_value; width * height],
        }
    }

    #[inline]
    fn assert_in_bounds(&self, index: Coord) {
        debug_assert!(
            index.1 >= 0
                && (index.1 as usize) < self.height
                && index.0 >= 0
                && (index.0 as usize) < self.inner.len() / self.height,
            "coord is out of bounds: {:?}",
            index
        );
    }
}

impl<T> Index<Coord> for Map<T> {
    type Output = T;

    #[inline]
    fn index(&self, index: Coord) -> &Self::Output {
        self.assert_in_bounds(index);

        &self.inner[index.1 as usize * self.height + index.0 as usize]
    }
}

impl<T> IndexMut<Coord> for Map<T> {
    #[inline]
    fn index_mut(&mut self, index: Coord) -> &mut Self::Output {
        self.assert_in_bounds(index);

        &mut self.inner[index.1 as usize * self.height + index.0 as usize]
    }
}

#[cfg(test)]
pub mod tests {
    use std::fmt;

    use crate::{Coord, Map, MapProvider};
    use std::fmt::Write;

    #[derive(Clone)]
    pub struct ArrayMapProvider {
        size: (usize, usize),
        visible: Map<bool>,
        map: Map<bool>,
    }

    impl ArrayMapProvider {
        pub fn new(size: (usize, usize)) -> Self {
            Self {
                size,
                visible: Map::new(size, false),
                map: Map::new(size, false),
            }
        }

        /// Remove marked visible cells
        pub fn reset(&mut self) {
            self.visible.inner.iter_mut().for_each(|c| *c = false);
        }

        pub fn set_wall(&mut self, coord: Coord, value: bool) {
            self.map[coord] = value;
        }

        /// Iterate over visible result map
        pub fn iter_result<'a>(&'a self) -> impl Iterator<Item = (Coord, bool)> + 'a {
            (0..self.size.1).flat_map(move |y| {
                (0..self.size.0).map(move |x| {
                    (
                        Coord(x as i32, y as i32),
                        self.visible[Coord(x as i32, y as i32)],
                    )
                })
            })
        }

        fn in_bounds(&self, coord: Coord) -> bool {
            coord.0 >= 0
                && (coord.0 as usize) < self.size.0
                && coord.1 >= 0
                && (coord.1 as usize) < self.size.1
        }
    }

    impl fmt::Debug for ArrayMapProvider {
        fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
            for y in 0..self.size.1 {
                f.write_char('[')?;
                for x in 0..self.size.0 {
                    match (
                        self.map.inner[y * self.size.1 + x],
                        self.visible.inner[y * self.size.1 + x],
                    ) {
                        (false, false) => f.write_char(' '),
                        (false, true) => f.write_char('.'),
                        (true, false) => f.write_char('x'),
                        (true, true) => f.write_char('X'),
                    }?
                }
                f.write_str("]\n")?;
            }

            Ok(())
        }
    }

    impl MapProvider for ArrayMapProvider {
        fn is_blocking(&self, cell: Coord) -> bool {
            self.map[cell]
        }

        fn bounds(&self) -> (Coord, Coord) {
            (
                Coord(0, 0),
                Coord(self.size.0 as i32 - 1, self.size.1 as i32 - 1),
            )
        }

        fn mark_as_visible(&mut self, cell: Coord) {
            self.visible[cell] = true;
        }
    }
}

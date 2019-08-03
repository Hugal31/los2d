use super::{Coord, LosAlgorithm, Map, MapProvider};

/// Implementation of the Diamond LOS algorithm.
///
/// See <> for more information,
/// and https://github.com/libtcod/libtcod/blob/master/src/libtcod/fov_diamond_raycasting.c#L149
/// for an example implementation
#[derive(Clone, Debug)]
pub struct DiamondLos {
    origin: Coord,
    max_view_range: u32,
    cache: Map<CellData>,
}

impl DiamondLos {
    pub fn new(max_view_range: u32) -> Self {
        DiamondLos {
            origin: Default::default(),
            max_view_range,
            cache: Map::new(
                (
                    max_view_range as usize * 2 + 1,
                    max_view_range as usize * 2 + 1,
                ),
                CellData::default(),
            ),
        }
    }

    fn propagate_from<M: MapProvider>(&mut self, offset: Coord, map: &M) {
        if !Self::is_in_bounds(self.origin + offset, map) {
            return;
        }

        if !self.get_data(offset).ignore {
            if offset.0 >= 0 {
                self.apply_ray(offset + Coord(1, 0), offset, map);
            }
            if offset.1 >= 0 {
                self.apply_ray(offset + Coord(0, 1), offset, map);
            }
            if offset.0 <= 0 {
                self.apply_ray(offset + Coord(-1, 0), offset, map);
            }
            if offset.1 <= 0 {
                self.apply_ray(offset + Coord(0, -1), offset, map);
            }
        }
    }

    fn apply_ray<M: MapProvider>(&mut self, offset: Coord, input: Coord, map: &M) {
        if !Self::is_in_bounds(self.origin + offset, map) {
            return;
        }
        let origin = self.origin;
        let input_data = self.get_data(input).clone();
        let mut self_data = self.get_data_mut(offset);

        if input_data.obs != (0, 0) {
            if offset.0 != input.0 {
                // We move on X axis
                // Not sure about this condition, copied from source (see DiamondLos doc)
                if input_data.err.0 > 0
                    && (self_data.obs.0 == 0 || input_data.err.1 <= 0 && input_data.obs.1 > 0)
                {
                    self_data.obs = input_data.obs;
                    self_data.err = (
                        input_data.err.0 - input_data.obs.1,
                        input_data.err.1 + input_data.obs.1,
                    );
                }
            } else {
                // We moved on Y axis
                // Not sure about this condition, copied from source (see DiamondLos doc)
                if input_data.err.1 > 0
                    && (self_data.obs.1 == 0 || input_data.err.0 <= 0 && input_data.obs.0 > 0)
                {
                    self_data.obs = input_data.obs;
                    self_data.err = (
                        input_data.err.0 + input_data.obs.0,
                        input_data.err.1 - input_data.obs.0,
                    );
                }
            }
        }

        self_data.ignore = (!self_data.visited || self_data.ignore) && input_data.is_obstacle();

        if !self_data.ignore && map.is_blocking(origin + offset) {
            self_data.obs = (offset.0.abs(), offset.1.abs());
            self_data.err = self_data.obs;
        }

        self_data.visited = true;
    }

    fn get_data(&self, offset: Coord) -> &CellData {
        &self.cache[Coord(
            offset.0 + self.max_view_range as i32,
            offset.1 + self.max_view_range as i32,
        )]
    }

    fn get_data_mut(&mut self, offset: Coord) -> &mut CellData {
        &mut self.cache[Coord(
            offset.0 + self.max_view_range as i32,
            offset.1 + self.max_view_range as i32,
        )]
    }

    fn is_in_bounds<M: MapProvider>(cell: Coord, map: &M) -> bool {
        let bounds = map.bounds();

        cell.0 >= (bounds.0).0
            && cell.0 <= (bounds.1).0
            && cell.1 >= (bounds.0).1
            && cell.1 <= (bounds.1).1
    }

    fn reset(&mut self) {
        self.cache
            .inner
            .iter_mut()
            .for_each(|d| *d = Default::default());
    }
}

impl LosAlgorithm for DiamondLos {
    fn compute_los<M: MapProvider>(&mut self, origin: Coord, vision_range: u32, map: &mut M) {
        if vision_range > self.max_view_range {
            self.max_view_range = vision_range;
            self.cache = Map::new(
                (vision_range as usize * 2 + 1, vision_range as usize * 2 + 1),
                CellData::default(),
            )
        }

        self.origin = origin;

        map.mark_as_visible(origin);
        {
            let zero = Coord(0, 0);
            let origin_data = self.get_data_mut(zero);
            origin_data.visited = true;
            self.propagate_from(zero, map);
        }

        for distance in 1..vision_range as i32 {
            let mut offset_x = distance;
            let mut offset_y = 0;

            // Turn clockwise starting from the east
            while offset_x != 0 {
                self.propagate_from(Coord(offset_x, offset_y), map);
                offset_x -= 1;
                offset_y += 1;
            }
            while offset_y != 0 {
                self.propagate_from(Coord(offset_x, offset_y), map);
                offset_x -= 1;
                offset_y -= 1;
            }
            while offset_x != 0 {
                self.propagate_from(Coord(offset_x, offset_y), map);
                offset_x += 1;
                offset_y -= 1;
            }
            while offset_y != 0 {
                self.propagate_from(Coord(offset_x, offset_y), map);
                offset_x += 1;
                offset_y += 1;
            }
        }

        for y in -(vision_range as i32)..=vision_range as i32 {
            for x in -(vision_range as i32)..=vision_range as i32 {
                let offset = Coord(x, y);
                let map_coord = offset + origin;
                if Self::is_in_bounds(map_coord, map) {
                    let data = self.get_data(offset);
                    if data.is_visible() {
                        map.mark_as_visible(map_coord)
                    }
                }
            }
        }

        self.reset();
    }
}

#[derive(Clone, Debug, Default)]
struct CellData {
    obs: (i32, i32),
    err: (i32, i32),
    ignore: bool,
    visited: bool,
}

impl CellData {
    fn is_visible(&self) -> bool {
        self.visited && !self.ignore && (!self.is_obstacle() || self.is_wall())
    }

    fn is_wall(&self) -> bool {
        self.err == self.obs
    }

    fn is_obstacle(&self) -> bool {
        self.err.0 != 0 && self.err.0 <= self.obs.0 || self.err.1 != 0 && self.err.1 <= self.obs.1
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::*;

    #[test]
    fn test_empty() {
        let mut map = ArrayMapProvider::new((5, 5));
        let mut alg = DiamondLos::new(5);
        alg.compute_los(Coord(2, 2), 10, &mut map);

        let map_str = format!("{:?}", map);
        let expected_str = "\
[.....]
[.....]
[.....]
[.....]
[.....]
";
        assert_eq!(map_str, expected_str);
    }

    #[test]
    fn test_vision_field() {
        let mut map = ArrayMapProvider::new((5, 5));
        let mut alg = DiamondLos::new(4);
        alg.compute_los(Coord(1, 0), 4, &mut map);

        let map_str = format!("{:?}", map);
        let expected_str = "\
[.....]
[.....]
[.... ]
[...  ]
[ .   ]
";
        assert_eq!(map_str, expected_str);
    }

    #[test]
    fn test_vision_walls_aligned() {
        let mut map = ArrayMapProvider::new((5, 5));
        map.set_wall(Coord(2, 0), true);
        map.set_wall(Coord(3, 0), true);
        map.set_wall(Coord(0, 2), true);
        let mut alg = DiamondLos::new(5);
        alg.compute_los(Coord(0, 0), 10, &mut map);

        let map_str = format!("{:?}", map);
        let expected_str = "\
[..Xx ]
[.....]
[X....]
[ ....]
[ ....]
";
        assert_eq!(map_str, expected_str);
    }

    #[test]
    fn test_vision_walls() {
        let mut map = ArrayMapProvider::new((5, 5));
        map.set_wall(Coord(3, 1), true);
        map.set_wall(Coord(2, 2), true);
        let mut alg = DiamondLos::new(5);
        alg.compute_los(Coord(0, 0), 10, &mut map);

        let map_str = format!("{:?}", map);
        let expected_str = "\
[.....]
[...X ]
[..X  ]
[.... ]
[.....]
";
        assert_eq!(map_str, expected_str);
    }

}

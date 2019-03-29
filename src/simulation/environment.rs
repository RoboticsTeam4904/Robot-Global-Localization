extern crate bitvec;

use bitvec::BitVec;
use rand::prelude::*;
use crate::utility::clamp;

use std::fmt;

/// A one dimensional environment that is filled simply with either a true or false in every space
pub struct BinaryEnvironment {
    pub map: BitVec,
    pub size: usize,
    pub robot_position: usize,
}

impl BinaryEnvironment {
    pub fn new(size: usize, map_percentage_true: f64) -> Self {
        let mut map = BitVec::with_capacity(size);
        let mut rng = thread_rng();
        for _ in 0..size {
            map.push(rng.gen_bool(map_percentage_true));
        }
        Self {
            map,
            size,
            robot_position: thread_rng().gen_range(0, size),
        }
    }

    pub fn move_robot(&mut self, dist: isize) -> isize {
        let previous_position = self.robot_position as isize;
        let new_position = clamp(
            self.robot_position as isize + dist,
            0isize,
            Some(self.size as isize),
        );
        self.robot_position = new_position as usize;
        new_position - previous_position
    }
}

impl fmt::Display for BinaryEnvironment {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        const TILE_SIZE: usize = 1;
        let true_tile = "◾".repeat(TILE_SIZE);
        let false_tile = "◽".repeat(TILE_SIZE);
        let map = self
            .map
            .iter()
            .map(|bit| {
                if bit {
                    true_tile.clone()
                } else {
                    false_tile.clone()
                }
            })
            .collect::<String>();
        let mut robot = " ".repeat((self.size - 1) * TILE_SIZE);
        robot.insert(self.robot_position * TILE_SIZE + TILE_SIZE / 2, '🤖');
        write!(f, "{}\n{}", map, robot)
    }
}

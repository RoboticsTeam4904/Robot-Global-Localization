use serde::Deserialize;

use crate::{
    ai::localization::DeathCondition,
    utility::{Point, Pose},
};

#[derive(Debug, Deserialize)]
pub struct TegraConfig {
    pub render: RenderConfig,
    pub mcl: MCLConfig,
}

#[derive(Debug, Deserialize)]
pub struct RenderConfig {
    pub window_size: (f64, f64),
    pub render_map: bool,
    pub render_scan: bool,
    pub render_prediction: bool,
    pub map_scale: f64,
    pub map_offset: Point,
}

#[derive(Debug, Deserialize)]
pub struct MCLConfig {
    pub max_particle_count: usize,
    pub death_condition: DeathCondition,
    pub variant: MCLVariant,
}

#[derive(Debug, Deserialize)]
pub enum MCLVariant {
    Adaptive {
        weight_sum_threshold: f64,
    },
    KLD {
        min_particle_count: usize,
        error_bound: f64,      // ε
        error_confidence: f64, // δ
        bin_size: Pose,        // ∆
    },
}

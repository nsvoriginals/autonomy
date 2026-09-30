//! Benchmark runner

use autonomy_common::error::Result;
use autonomy_common::ids::{AlgorithmId, VehicleId};
use autonomy_common::state::AlgorithmConfig;
use autonomy_common::time::SimTime;
use crate::report::{BenchmarkReport, BenchmarkResult, ComparisonReport};
use parking_lot::RwLock;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Instant;

/// Benchmark configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BenchmarkConfig {
    pub scenario_path: String,
    pub algorithm_ids: Vec<AlgorithmId>,
    pub runs: usize,
    pub max_steps: Option<u64>,
    pub seed_base: u64,
    pub parallel: bool,
}

impl Default for BenchmarkConfig {
    fn default() -> Self {
        Self {
            scenario_path: "scenario.yaml".into(),
            algorithm_ids: Vec::new(),
            runs: 100,
            max_steps: None,
            seed_base: 42,
            parallel: true,
        }
    }
}

/// Benchmark runner
pub struct BenchmarkRunner {
    config: BenchmarkConfig,
    results: RwLock<Vec<BenchmarkResult>>,
}

impl BenchmarkRunner {
    pub fn new(config: BenchmarkConfig) -> Self {
        Self {
            config,
            results: RwLock::new(Vec::new()),
        }
    }

    pub async fn run(&self) -> Result<BenchmarkReport> {
        let mut all_results = Vec::new();
        
        for &algorithm_id in &self.config.algorithm_ids {
            for run in 0..self.config.runs {
                let seed = self.config.seed_base + run as u64;
                let result = self.run_single(algorithm_id, seed).await?;
                all_results.push(result);
            }
        }
        
        *self.results.write() = all_results.clone();
        Ok(BenchmarkReport::from_results(all_results))
    }

    async fn run_single(&self, algorithm_id: autonomy_common::ids::AlgorithmId, seed: u64) -> Result<BenchmarkResult> {
        let start = Instant::now();
        
        // This would run the actual simulation
        // For now, return mock result
        let result = BenchmarkResult {
            algorithm_id,
            seed,
            run_id: 0,
            success: true,
            mission_completed: true,
            mission_time: 100.0,
            position_error_mean: 1.5,
            position_error_max: 5.0,
            energy_consumed: 50.0,
            avg_cpu_time_ms: 2.0,
            max_cpu_time_ms: 10.0,
            communication_loss: 0.01,
            custom_metrics: HashMap::new(),
        };
        
        Ok(result)
    }

    pub fn compare_algorithms(&self, algorithm_a: AlgorithmId, algorithm_b: AlgorithmId) -> ComparisonReport {
        let results = self.results.read();
        let a_results: Vec<_> = results.iter().filter(|r| r.algorithm_id == algorithm_a).collect();
        let b_results: Vec<_> = results.iter().filter(|r| r.algorithm_id == algorithm_b).collect();
        
        ComparisonReport::new(algorithm_a, algorithm_b, &a_results, &b_results)
    }
}
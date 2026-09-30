//! Benchmark reports

use autonomy_common::ids::AlgorithmId;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Single benchmark result
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BenchmarkResult {
    pub algorithm_id: AlgorithmId,
    pub seed: u64,
    pub run_id: usize,
    pub success: bool,
    pub mission_completed: bool,
    pub mission_time: f64,
    pub position_error_mean: f64,
    pub position_error_max: f64,
    pub energy_consumed: f64,
    pub avg_cpu_time_ms: f64,
    pub max_cpu_time_ms: f64,
    pub communication_loss: f64,
    pub custom_metrics: HashMap<String, f64>,
}

/// Aggregated benchmark report
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BenchmarkReport {
    pub algorithm_id: AlgorithmId,
    pub total_runs: usize,
    pub successful_runs: usize,
    pub success_rate: f64,
    pub mission_completion_rate: f64,
    pub mission_time: StatSummary,
    pub position_error_mean: StatSummary,
    pub position_error_max: StatSummary,
    pub energy_consumed: StatSummary,
    pub avg_cpu_time_ms: StatSummary,
    pub max_cpu_time_ms: StatSummary,
    pub communication_loss: StatSummary,
    pub custom_metrics: HashMap<String, StatSummary>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StatSummary {
    pub count: usize,
    pub mean: f64,
    pub std_dev: f64,
    pub min: f64,
    pub max: f64,
    pub median: f64,
    pub p95: f64,
    pub p99: f64,
}

impl BenchmarkReport {
    pub fn from_results(results: Vec<BenchmarkResult>) -> Self {
        if results.is_empty() {
            return Self::default();
        }
        
        let algorithm_id = results[0].algorithm_id;
        let total_runs = results.len();
        let successful_runs = results.iter().filter(|r| r.success).count();
        let mission_completed = results.iter().filter(|r| r.mission_completed).count();
        
        let mission_times: Vec<f64> = results.iter().map(|r| r.mission_time).collect();
        let pos_errors_mean: Vec<f64> = results.iter().map(|r| r.position_error_mean).collect();
        let pos_errors_max: Vec<f64> = results.iter().map(|r| r.position_error_max).collect();
        let energy: Vec<f64> = results.iter().map(|r| r.energy_consumed).collect();
        let cpu_avg: Vec<f64> = results.iter().map(|r| r.avg_cpu_time_ms).collect();
        let cpu_max: Vec<f64> = results.iter().map(|r| r.max_cpu_time_ms).collect();
        let comm_loss: Vec<f64> = results.iter().map(|r| r.communication_loss).collect();
        
        Self {
            algorithm_id,
            total_runs,
            successful_runs,
            success_rate: successful_runs as f64 / total_runs as f64,
            mission_completion_rate: mission_completed as f64 / total_runs as f64,
            mission_time: StatSummary::from_slice(&mission_times),
            position_error_mean: StatSummary::from_slice(&pos_errors_mean),
            position_error_max: StatSummary::from_slice(&pos_errors_max),
            energy_consumed: StatSummary::from_slice(&energy),
            avg_cpu_time_ms: StatSummary::from_slice(&cpu_avg),
            max_cpu_time_ms: StatSummary::from_slice(&cpu_max),
            communication_loss: StatSummary::from_slice(&comm_loss),
            custom_metrics: HashMap::new(),
        }
    }
}

impl StatSummary {
    pub fn from_slice(data: &[f64]) -> Self {
        if data.is_empty() {
            return Self::default();
        }
        
        let mut sorted = data.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        
        let count = sorted.len();
        let mean = sorted.iter().sum::<f64>() / count as f64;
        let variance = sorted.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / count as f64;
        let std_dev = variance.sqrt();
        
        Self {
            count,
            mean,
            std_dev,
            min: sorted[0],
            max: sorted[count - 1],
            median: sorted[count / 2],
            p95: sorted[(count as f64 * 0.95) as usize],
            p99: sorted[(count as f64 * 0.99) as usize],
        }
    }
}

impl Default for StatSummary {
    fn default() -> Self {
        Self {
            count: 0,
            mean: 0.0,
            std_dev: 0.0,
            min: 0.0,
            max: 0.0,
            median: 0.0,
            p95: 0.0,
            p99: 0.0,
        }
    }
}

impl Default for BenchmarkReport {
    fn default() -> Self {
        Self {
            algorithm_id: AlgorithmId::nil(),
            total_runs: 0,
            successful_runs: 0,
            success_rate: 0.0,
            mission_completion_rate: 0.0,
            mission_time: StatSummary::default(),
            position_error_mean: StatSummary::default(),
            position_error_max: StatSummary::default(),
            energy_consumed: StatSummary::default(),
            avg_cpu_time_ms: StatSummary::default(),
            max_cpu_time_ms: StatSummary::default(),
            communication_loss: StatSummary::default(),
            custom_metrics: HashMap::new(),
        }
    }
}

/// Comparison report between two algorithms
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ComparisonReport {
    pub algorithm_a: AlgorithmId,
    pub algorithm_b: AlgorithmId,
    pub sample_size_a: usize,
    pub sample_size_b: usize,
    pub mission_time: ComparisonStat,
    pub position_error_mean: ComparisonStat,
    pub position_error_max: ComparisonStat,
    pub energy_consumed: ComparisonStat,
    pub avg_cpu_time_ms: ComparisonStat,
    pub success_rate_diff: f64,
    pub conclusion: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ComparisonStat {
    pub mean_a: f64,
    pub mean_b: f64,
    pub diff: f64,
    pub diff_percent: f64,
    pub p_value: Option<f64>,
    pub significant: bool,
}

impl ComparisonReport {
    pub fn new(
        algorithm_a: AlgorithmId,
        algorithm_b: AlgorithmId,
        results_a: &[&BenchmarkResult],
        results_b: &[&BenchmarkResult],
    ) -> Self {
        let mission_time_a: Vec<f64> = results_a.iter().map(|r| r.mission_time).collect();
        let mission_time_b: Vec<f64> = results_b.iter().map(|r| r.mission_time).collect();
        let pos_err_a: Vec<f64> = results_a.iter().map(|r| r.position_error_mean).collect();
        let pos_err_b: Vec<f64> = results_b.iter().map(|r| r.position_error_mean).collect();
        let pos_max_a: Vec<f64> = results_a.iter().map(|r| r.position_error_max).collect();
        let pos_max_b: Vec<f64> = results_b.iter().map(|r| r.position_error_max).collect();
        let energy_a: Vec<f64> = results_a.iter().map(|r| r.energy_consumed).collect();
        let energy_b: Vec<f64> = results_b.iter().map(|r| r.energy_consumed).collect();
        let cpu_a: Vec<f64> = results_a.iter().map(|r| r.avg_cpu_time_ms).collect();
        let cpu_b: Vec<f64> = results_b.iter().map(|r| r.avg_cpu_time_ms).collect();
        
        let success_rate_a = results_a.iter().filter(|r| r.success).count() as f64 / results_a.len() as f64;
        let success_rate_b = results_b.iter().filter(|r| r.success).count() as f64 / results_b.len() as f64;
        
        Self {
            algorithm_a,
            algorithm_b,
            sample_size_a: results_a.len(),
            sample_size_b: results_b.len(),
            mission_time: compare_stat(&mission_time_a, &mission_time_b),
            position_error_mean: compare_stat(&pos_err_a, &pos_err_b),
            position_error_max: compare_stat(&pos_max_a, &pos_max_b),
            energy_consumed: compare_stat(&energy_a, &energy_b),
            avg_cpu_time_ms: compare_stat(&cpu_a, &cpu_b),
            success_rate_diff: success_rate_a - success_rate_b,
            conclusion: generate_conclusion(success_rate_a, success_rate_b, &mission_time_a, &mission_time_b),
        }
    }
}

fn compare_stat(a: &[f64], b: &[f64]) -> ComparisonStat {
    let mean_a = a.iter().sum::<f64>() / a.len() as f64;
    let mean_b = b.iter().sum::<f64>() / b.len() as f64;
    let diff = mean_b - mean_a;
    let diff_percent = if mean_a != 0.0 { (diff / mean_a) * 100.0 } else { 0.0 };
    
    ComparisonStat {
        mean_a,
        mean_b,
        diff,
        diff_percent,
        p_value: None, // Would need t-test
        significant: diff.abs() > 0.05 * mean_a.max(mean_b), // Simple heuristic
    }
}

fn generate_conclusion(success_a: f64, success_b: f64, time_a: &[f64], time_b: &[f64]) -> String {
    let mut parts = Vec::new();
    
    if (success_a - success_b).abs() > 0.1 {
        if success_a > success_b {
            parts.push("Algorithm A has higher success rate");
        } else {
            parts.push("Algorithm B has higher success rate");
        }
    }
    
    let mean_a = time_a.iter().sum::<f64>() / time_a.len() as f64;
    let mean_b = time_b.iter().sum::<f64>() / time_b.len() as f64;
    
    if (mean_a - mean_b).abs() > 0.1 * mean_a.max(mean_b) {
        if mean_a < mean_b {
            parts.push("Algorithm A completes missions faster");
        } else {
            parts.push("Algorithm B completes missions faster");
        }
    }
    
    if parts.is_empty() {
        "Algorithms perform similarly".to_string()
    } else {
        parts.join("; ")
    }
}
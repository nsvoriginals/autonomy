//! WASM execution metrics

use autonomy_common::time::SimTime;
use parking_lot::RwLock;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Instant;

/// WASM execution metrics
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct WasmMetrics {
    pub total_invocations: u64,
    pub successful_invocations: u64,
    pub failed_invocations: u64,
    pub total_execution_time_us: u64,
    pub max_execution_time_us: u64,
    pub avg_execution_time_us: f64,
    pub total_memory_used: u64,
    pub max_memory_used: u64,
    pub traps: u64,
    pub ooms: u64,
    pub timeouts: u64,
    pub host_function_calls: u64,
    pub last_invocation: Option<SimTime>,
}

impl WasmMetrics {
    pub fn record_invocation(&mut self, execution_time_us: u64, success: bool) {
        self.total_invocations += 1;
        if success {
            self.successful_invocations += 1;
        } else {
            self.failed_invocations += 1;
        }
        
        self.total_execution_time_us += execution_time_us;
        self.max_execution_time_us = self.max_execution_time_us.max(execution_time_us);
        self.avg_execution_time_us = self.total_execution_time_us as f64 / self.total_invocations as f64;
    }

    pub fn record_trap(&mut self) {
        self.traps += 1;
    }

    pub fn record_oom(&mut self) {
        self.ooms += 1;
    }

    pub fn record_timeout(&mut self) {
        self.timeouts += 1;
    }

    pub fn record_host_call(&mut self) {
        self.host_function_calls += 1;
    }

    pub fn update_memory(&mut self, current: u64, peak: u64) {
        self.total_memory_used = current;
        self.max_memory_used = self.max_memory_used.max(peak);
    }
}

/// Per-algorithm metrics collector
pub struct WasmMetricsCollector {
    metrics: RwLock<HashMap<autonomy_common::ids::AlgorithmId, WasmMetrics>>,
    global: RwLock<WasmMetrics>,
}

impl WasmMetricsCollector {
    pub fn new() -> Self {
        Self {
            metrics: RwLock::new(HashMap::new()),
            global: RwLock::new(WasmMetrics::default()),
        }
    }

    pub fn record(&self, algorithm_id: autonomy_common::ids::AlgorithmId, execution_time_us: u64, success: bool) {
        let mut global = self.global.write();
        global.record_invocation(execution_time_us, success);
        global.last_invocation = Some(autonomy_common::time::SimTime::ZERO); // Would use actual time
        
        let mut metrics = self.metrics.write();
        let entry = metrics.entry(algorithm_id).or_default();
        entry.record_invocation(execution_time_us, success);
    }

    pub fn record_trap(&self, algorithm_id: autonomy_common::ids::AlgorithmId) {
        self.global.write().record_trap();
        self.metrics.write().entry(algorithm_id).or_default().record_trap();
    }

    pub fn record_oom(&self, algorithm_id: autonomy_common::ids::AlgorithmId) {
        self.global.write().record_oom();
        self.metrics.write().entry(algorithm_id).or_default().record_oom();
    }

    pub fn record_timeout(&self, algorithm_id: autonomy_common::ids::AlgorithmId) {
        self.global.write().record_timeout();
        self.metrics.write().entry(algorithm_id).or_default().record_timeout();
    }

    pub fn record_host_call(&self, algorithm_id: autonomy_common::ids::AlgorithmId) {
        self.global.write().record_host_call();
        self.metrics.write().entry(algorithm_id).or_default().record_host_call();
    }

    pub fn get_metrics(&self, algorithm_id: autonomy_common::ids::AlgorithmId) -> WasmMetrics {
        self.metrics.read().get(&algorithm_id).cloned().unwrap_or_default()
    }

    pub fn get_global_metrics(&self) -> WasmMetrics {
        self.global.read().clone()
    }

    pub fn get_all_metrics(&self) -> HashMap<autonomy_common::ids::AlgorithmId, WasmMetrics> {
        self.metrics.read().clone()
    }
}

impl Default for WasmMetricsCollector {
    fn default() -> Self {
        Self::new()
    }
}

/// Execution timer for measuring WASM execution time
pub struct ExecutionTimer {
    start: Instant,
}

impl ExecutionTimer {
    pub fn start() -> Self {
        Self { start: Instant::now() }
    }

    pub fn elapsed_us(&self) -> u64 {
        self.start.elapsed().as_micros() as u64
    }
}
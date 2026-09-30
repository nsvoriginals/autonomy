//! Host functions for WASM modules

use autonomy_common::error::Result;
use autonomy_common::frames::Vec3;
use autonomy_common::ids::{VehicleId, AlgorithmId};
use autonomy_common::state::FleetRequest;
use autonomy_common::time::SimTime;
use parking_lot::RwLock;
use std::collections::HashMap;
use std::sync::Arc;

/// Host context provided to WASM modules
pub struct HostContext {
    pub vehicle_id: VehicleId,
    pub algorithm_id: AlgorithmId,
    pub current_time: SimTime,
    pub rng_state: RwLock<u64>,
    pub metrics: RwLock<HashMap<String, f64>>,
    pub log_buffer: RwLock<Vec<LogEntry>>,
    pub fleet_requests: RwLock<Vec<FleetRequest>>,
}

#[derive(Clone, Debug)]
pub struct LogEntry {
    pub level: super::abi::LogLevel,
    pub message: String,
    pub timestamp: SimTime,
}

impl HostContext {
    pub fn new(vehicle_id: VehicleId, algorithm_id: AlgorithmId) -> Self {
        Self {
            vehicle_id,
            algorithm_id,
            current_time: SimTime::ZERO,
            rng_state: RwLock::new(0),
            metrics: RwLock::new(HashMap::new()),
            log_buffer: RwLock::new(Vec::new()),
            fleet_requests: RwLock::new(Vec::new()),
        }
    }

    pub fn update_time(&self, time: SimTime) {
        // In a real implementation, this would update a shared time
    }

    /// Host function: Get current simulation time
    pub fn get_time(&self) -> SimTime {
        self.current_time
    }

    /// Host function: Log a message
    pub fn log(&self, level: super::abi::LogLevel, message: &str) {
        self.log_buffer.write().push(LogEntry {
            level,
            message: message.to_string(),
            timestamp: self.current_time,
        });
    }

    /// Host function: Record a metric
    pub fn record_metric(&self, name: &str, value: f64) {
        self.metrics.write().insert(name.to_string(), value);
    }

    /// Host function: Get deterministic random number
    pub fn random(&self) -> u64 {
        let mut state = self.rng_state.write();
        // Simple deterministic RNG
        *state = state.wrapping_mul(0x5DEECE66D).wrapping_add(0xB);
        *state
    }

    /// Host function: Request fleet action
    pub fn request_fleet_action(&self, request_type: &str, data: &[u8]) -> Result<u32> {
        let request = FleetRequest::Custom {
            request_type: request_type.to_string(),
            data: data.to_vec(),
        };
        self.fleet_requests.write().push(request);
        Ok(0)
    }

    pub fn get_logs(&self) -> Vec<LogEntry> {
        self.log_buffer.read().clone()
    }

    pub fn get_metrics(&self) -> HashMap<String, f64> {
        self.metrics.read().clone()
    }

    pub fn take_fleet_requests(&self) -> Vec<FleetRequest> {
        std::mem::take(&mut *self.fleet_requests.write())
    }
}

/// Host function registry for WASM
pub struct HostFunctionRegistry {
    contexts: HashMap<AlgorithmId, Arc<HostContext>>,
}

impl HostFunctionRegistry {
    pub fn new() -> Self {
        Self {
            contexts: HashMap::new(),
        }
    }

    pub fn register(&mut self, algorithm_id: AlgorithmId, vehicle_id: VehicleId) -> Arc<HostContext> {
        let ctx = Arc::new(HostContext::new(vehicle_id, algorithm_id));
        self.contexts.insert(algorithm_id, ctx.clone());
        ctx
    }

    pub fn get_context(&self, algorithm_id: AlgorithmId) -> Option<Arc<HostContext>> {
        self.contexts.get(&algorithm_id).cloned()
    }

    pub fn remove_context(&mut self, algorithm_id: AlgorithmId) {
        self.contexts.remove(&algorithm_id);
    }
}

impl Default for HostFunctionRegistry {
    fn default() -> Self {
        Self::new()
    }
}
//! WASM ABI (Application Binary Interface)

use autonomy_common::frames::{Pose, Vec3, LinearVelocity, AngularVelocity, LinearAcceleration};
use autonomy_common::ids::{VehicleId, AlgorithmId, MissionId, TaskId};
use autonomy_common::state::{EstimatedState, SensorMeasurements, MissionState, MissionExecutionState, AlgorithmConfig, AlgorithmOutput, Trajectory, FleetRequest, MissionTransition, EnvironmentSnapshot, NeighborState};
use autonomy_common::time::SimTime;
use serde::{Deserialize, Serialize};

/// WASM ABI version
pub const ABI_VERSION: u32 = 1;

/// Algorithm input (host -> WASM)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AlgorithmInputV1 {
    pub version: u32,
    pub timestamp: SimTime,
    pub vehicle_id: VehicleId,
    pub estimated_state: EstimatedState,
    pub sensor_measurements: SensorMeasurements,
    pub neighbors: Vec<NeighborState>,
    pub mission_state: MissionState,
    pub environment: EnvironmentSnapshot,
    pub config: AlgorithmConfig,
}

/// Algorithm output (WASM -> host)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AlgorithmOutputV1 {
    pub version: u32,
    pub desired_trajectory: Option<Trajectory>,
    pub desired_velocity: Option<Vec3>,
    pub desired_heading: Option<f64>,
    pub mission_transition: Option<MissionTransition>,
    pub fleet_requests: Vec<FleetRequest>,
    pub custom_data: Vec<u8>,
}

/// Host functions available to WASM modules
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum HostFunction {
    /// Get current simulation time
    GetTime,
    /// Log a message
    Log,
    /// Record a metric
    RecordMetric,
    /// Get deterministic random number
    Random,
    /// Request fleet action
    RequestFleetAction,
}

/// Host function signatures
pub mod host_functions {
    use super::*;
    
    // fn get_time() -> SimTime
    pub const GET_TIME: &str = "host_get_time";
    
    // fn log(level: u32, message_ptr: *const u8, message_len: usize)
    pub const LOG: &str = "host_log";
    
    // fn record_metric(name_ptr: *const u8, name_len: usize, value: f64)
    pub const RECORD_METRIC: &str = "host_record_metric";
    
    // fn random() -> u64
    pub const RANDOM: &str = "host_random";
    
    // fn request_fleet_action(request_type_ptr: *const u8, request_type_len: usize, data_ptr: *const u8, data_len: usize) -> u32
    pub const REQUEST_FLEET_ACTION: &str = "host_request_fleet_action";
}

/// Algorithm entry point (WASM -> host)
/// 
/// Expected signature:
/// fn algorithm_tick(input_ptr: *const u8, input_len: usize, output_ptr: *mut u8, output_len: *mut usize) -> i32
pub const ALGORITHM_TICK: &str = "algorithm_tick";

/// Algorithm initialization (WASM -> host)
/// 
/// Expected signature:
/// fn algorithm_init(config_ptr: *const u8, config_len: usize) -> i32
pub const ALGORITHM_INIT: &str = "algorithm_init";

/// Algorithm cleanup (WASM -> host)
/// 
/// Expected signature:
/// fn algorithm_cleanup()
pub const ALGORITHM_CLEANUP: &str = "algorithm_cleanup";

/// Memory layout for WASM
pub mod memory {
    /// Input buffer offset
    pub const INPUT_BUFFER: u32 = 0x1000;
    /// Output buffer offset
    pub const OUTPUT_BUFFER: u32 = 0x2000;
    /// Max input size
    pub const MAX_INPUT_SIZE: u32 = 64 * 1024;
    /// Max output size
    pub const MAX_OUTPUT_SIZE: u32 = 64 * 1024;
}

/// Error codes
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(i32)]
pub enum AlgorithmError {
    Success = 0,
    InvalidInput = -1,
    InvalidOutput = -2,
    ExecutionTimeout = -3,
    MemoryError = -4,
    HostFunctionError = -5,
    NotInitialized = -6,
    InvalidVersion = -7,
}

/// Log levels
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u32)]
pub enum LogLevel {
    Trace = 0,
    Debug = 1,
    Info = 2,
    Warn = 3,
    Error = 4,
}

/// Metric types
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u32)]
pub enum MetricType {
    Counter = 0,
    Gauge = 1,
    Histogram = 2,
}

/// Serialization format for WASM messages
pub fn serialize_input(input: &AlgorithmInputV1) -> Vec<u8> {
    postcard::to_stdvec(input).unwrap()
}

pub fn deserialize_input(data: &[u8]) -> Result<AlgorithmInputV1, postcard::Error> {
    postcard::from_bytes(data)
}

pub fn serialize_output(output: &AlgorithmOutputV1) -> Vec<u8> {
    postcard::to_stdvec(output).unwrap()
}

pub fn deserialize_output(data: &[u8]) -> Result<AlgorithmOutputV1, postcard::Error> {
    postcard::from_bytes(data)
}
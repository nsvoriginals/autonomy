//! Structured telemetry types and event definitions

use crate::ids::{MissionId, TaskId, VehicleId};
use crate::state::{BatteryState, HealthStatus, SensorMeasurement};
use crate::time::SimTime;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Telemetry event types
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum TelemetryEvent {
    VehicleState(VehicleStateEvent),
    SensorMeasurement(SensorMeasurement),
    MissionEvent(MissionEvent),
    HealthEvent(HealthEvent),
    NetworkEvent(NetworkEvent),
    AlgorithmEvent(AlgorithmEvent),
    FleetEvent(FleetEvent),
    SimulationEvent(SimulationEvent),
    Custom(CustomEvent),
}

/// Vehicle state telemetry
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VehicleStateEvent {
    pub time: SimTime,
    pub vehicle_id: VehicleId,
    pub pose: crate::frames::Pose,
    pub velocity: crate::frames::LinearVelocity,
    pub acceleration: crate::frames::LinearAcceleration,
    pub battery: BatteryState,
    pub health: HealthStatus,
    pub control_inputs: crate::state::ControlInputs,
}

/// Mission events
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MissionEvent {
    pub time: SimTime,
    pub mission_id: MissionId,
    pub vehicle_id: VehicleId,
    pub task_id: Option<TaskId>,
    pub event_type: MissionEventType,
    pub details: HashMap<String, serde_json::Value>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MissionEventType {
    MissionStarted,
    MissionCompleted,
    MissionFailed,
    MissionAborted,
    TaskStarted,
    TaskCompleted,
    TaskFailed,
    TaskReassigned,
    WaypointReached,
    WaypointMissed,
    HoldStarted,
    HoldCompleted,
    ReturnToBase,
    Landed,
}

/// Health events
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HealthEvent {
    pub time: SimTime,
    pub vehicle_id: VehicleId,
    pub previous_health: HealthStatus,
    pub current_health: HealthStatus,
    pub reason: String,
    pub affected_systems: Vec<String>,
}

/// Network events
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NetworkEvent {
    pub time: SimTime,
    pub source: VehicleId,
    pub destination: Option<VehicleId>,
    pub event_type: NetworkEventType,
    pub latency_ms: Option<f64>,
    pub packet_size: Option<u32>,
    pub link_quality: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum NetworkEventType {
    Connected,
    Disconnected,
    PacketSent,
    PacketReceived,
    PacketLost,
    PacketCorrupted,
    BandwidthChanged,
    LatencySpike,
    PartitionDetected,
    PartitionHealed,
}

/// Algorithm execution events
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AlgorithmEvent {
    pub time: SimTime,
    pub vehicle_id: VehicleId,
    pub algorithm_id: crate::ids::AlgorithmId,
    pub event_type: AlgorithmEventType,
    pub execution_time_us: u64,
    pub memory_used: Option<u64>,
    pub output_summary: Option<String>,
    pub error: Option<String>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum AlgorithmEventType {
    ModuleLoaded,
    ModuleUnloaded,
    ModuleRestarted,
    TickStarted,
    TickCompleted,
    TickTimeout,
    Trap,
    Oom,
    InvalidOutput,
}

/// Fleet events
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FleetEvent {
    pub time: SimTime,
    pub event_type: FleetEventType,
    pub vehicle_ids: Vec<VehicleId>,
    pub details: HashMap<String, serde_json::Value>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum FleetEventType {
    VehicleRegistered,
    VehicleUnregistered,
    VehicleHeartbeatLost,
    VehicleHeartbeatRestored,
    TaskAllocated,
    TaskReallocated,
    FormationChanged,
    ConvoyFormed,
    ConvoyBroken,
    EmergencyStop,
    FleetModeChanged,
}

/// Simulation control events
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SimulationEvent {
    pub time: SimTime,
    pub event_type: SimulationEventType,
    pub details: HashMap<String, serde_json::Value>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SimulationEventType {
    Started,
    Paused,
    Resumed,
    Stepped,
    Reset,
    TimeScaleChanged,
    ScenarioLoaded,
    ScenarioCompleted,
    RecordingStarted,
    RecordingStopped,
    ReplayStarted,
    ReplayCompleted,
}

/// Custom user-defined event
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CustomEvent {
    pub time: SimTime,
    pub event_name: String,
    pub vehicle_id: Option<VehicleId>,
    pub payload: Vec<u8>,
}

/// Telemetry stream configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TelemetryConfig {
    pub enabled: bool,
    pub output_format: TelemetryFormat,
    pub output_path: Option<String>,
    pub buffer_size: usize,
    pub flush_interval_ms: u64,
    pub event_filters: Vec<EventFilter>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TelemetryFormat {
    Json,
    JsonLines,
    Postcard,
    Bincode,
    Csv,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EventFilter {
    pub event_types: Vec<String>,
    pub vehicle_ids: Vec<VehicleId>,
    pub min_time: Option<SimTime>,
    pub max_time: Option<SimTime>,
}

impl Default for TelemetryConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            output_format: TelemetryFormat::JsonLines,
            output_path: None,
            buffer_size: 10000,
            flush_interval_ms: 100,
            event_filters: Vec::new(),
        }
    }
}

/// Telemetry metrics aggregation
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TelemetryMetrics {
    pub total_events: u64,
    pub events_by_type: HashMap<String, u64>,
    pub events_by_vehicle: HashMap<VehicleId, u64>,
    pub bytes_written: u64,
    pub buffer_overflows: u64,
    pub flush_count: u64,
    pub avg_flush_time_us: f64,
}
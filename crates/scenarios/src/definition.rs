//! Scenario definition

use autonomy_common::frames::Vec3;
use autonomy_common::ids::{MissionId, TaskId, VehicleId, ZoneId};
use autonomy_common::state::{MissionExecutionState};
use autonomy_common::time::SimTime;
use autonomy_environment::zones::{ZoneType, RestrictionLevel, SurfaceType, ConnectorType, CompletionCriteria};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Scenario definition
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Scenario {
    pub metadata: ScenarioMetadata,
    pub environment: EnvironmentConfig,
    pub vehicles: Vec<VehicleConfig>,
    pub zones: Vec<ZoneConfig>,
    pub missions: Vec<MissionConfig>,
    pub failures: Vec<FailureConfig>,
    pub network: NetworkConfig,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ScenarioMetadata {
    pub name: String,
    pub description: String,
    pub seed: u64,
    pub duration: f64,
    pub version: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EnvironmentConfig {
    pub terrain: autonomy_environment::terrain::TerrainType,
    pub weather: autonomy_environment::weather::WeatherState,
    pub time_of_day: f64,
    pub gravity: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VehicleConfig {
    pub id: VehicleId,
    pub vehicle_type: autonomy_common::ids::VehicleType,
    pub callsign: String,
    pub position: Vec3,
    pub velocity: Vec3,
    pub orientation: f64, // Yaw
    pub battery: f64, // 0.0 to 1.0
    pub autonomy_config: AutonomyConfig,
    pub sensors: Vec<SensorConfig>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AutonomyConfig {
    pub algorithm_id: autonomy_common::ids::AlgorithmId,
    pub wasm_module: Option<String>,
    pub parameters: HashMap<String, serde_json::Value>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SensorConfig {
    pub sensor_type: autonomy_common::ids::SensorType,
    pub position: Vec3,
    pub orientation: Vec3,
    pub update_rate: f64,
    pub noise_params: autonomy_sensors::suite::NoiseParams,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ZoneConfig {
    pub id: ZoneId,
    pub zone_type: ZoneType,
    pub name: String,
    pub center: Vec3,
    pub radius: f64,
    pub height: f64,
    pub properties: HashMap<String, serde_json::Value>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MissionConfig {
    pub id: MissionId,
    pub mission_type: String,
    pub assigned_vehicles: Vec<VehicleId>,
    pub tasks: Vec<TaskConfig>,
    pub parameters: HashMap<String, serde_json::Value>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TaskConfig {
    pub id: TaskId,
    pub task_type: String,
    pub position: Option<Vec3>,
    pub parameters: HashMap<String, serde_json::Value>,
    pub priority: u8,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FailureConfig {
    pub time: f64,
    pub target: VehicleId,
    pub failure_type: FailureType,
    pub parameters: HashMap<String, serde_json::Value>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FailureType {
    GpsLoss,
    ImuBias,
    SensorDropout,
    MotorFailure,
    BatteryDegradation,
    CommunicationLoss,
    ActuatorDegradation,
    Custom,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NetworkConfig {
    pub max_range: f64,
    pub default_latency_ms: f64,
    pub default_loss_rate: f64,
    pub partitions: Vec<NetworkPartition>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NetworkPartition {
    pub start_time: f64,
    pub duration: f64,
    pub affected_vehicles: Vec<VehicleId>,
}

impl Default for Scenario {
    fn default() -> Self {
        Self {
            metadata: ScenarioMetadata {
                name: "default".into(),
                description: "Default scenario".into(),
                seed: 42,
                duration: 300.0,
                version: "0.1".into(),
            },
            environment: EnvironmentConfig {
                terrain: autonomy_environment::terrain::TerrainType::Flat { height: 0.0 },
                weather: autonomy_environment::weather::WeatherState::clear(),
                time_of_day: 12.0,
                gravity: 9.80665,
            },
            vehicles: Vec::new(),
            zones: Vec::new(),
            missions: Vec::new(),
            failures: Vec::new(),
            network: NetworkConfig {
                max_range: 1000.0,
                default_latency_ms: 50.0,
                default_loss_rate: 0.01,
                partitions: Vec::new(),
            },
        }
    }
}
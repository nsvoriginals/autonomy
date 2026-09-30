//! Extended vehicle traits

use autonomy_common::error::Result;
use autonomy_common::frames::{LinearAcceleration, LinearVelocity, Pose, Vec3};
use autonomy_common::ids::{SensorId, VehicleId, VehicleType};
use autonomy_common::state::{AlgorithmInput, AlgorithmOutput, BatteryState, ControlInputs, EstimatedState, HealthStatus, MissionState, SensorMeasurements, VehicleState};
use autonomy_common::time::SimTime;
use autonomy_common::traits::{ActuatorSet, AutonomyModule, Controller, Sensor, SensorSuite, SimModule, StateEstimator};

/// Extended vehicle trait with lifecycle methods
pub trait VehicleExt: Vehicle {
    fn initialize(&mut self, config: &VehicleConfig) -> Result<()>;
    fn shutdown(&mut self) -> Result<()>;
    fn pre_step(&mut self, time: SimTime, dt: f64) -> Result<()>;
    fn post_step(&mut self, time: SimTime, dt: f64) -> Result<()>;
    fn handle_event(&mut self, event: &autonomy_common::telemetry::TelemetryEvent) -> Result<()>;
}

/// Vehicle configuration
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct VehicleConfig {
    pub vehicle_id: VehicleId,
    pub vehicle_type: VehicleType,
    pub mass: f64,
    pub dimensions: VehicleDimensions,
    pub battery: BatteryConfig,
    pub sensors: Vec<SensorConfig>,
    pub actuators: ActuatorConfig,
    pub estimator: EstimatorConfig,
    pub planner: PlannerConfig,
    pub controller: ControllerConfig,
    pub autonomy: AutonomyConfig,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct VehicleDimensions {
    pub length: f64,
    pub width: f64,
    pub height: f64,
    pub rotor_radius: Option<f64>,
    pub wheelbase: Option<f64>,
    pub track_width: Option<f64>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct BatteryConfig {
    pub capacity_wh: f64,
    pub voltage: f64,
    pub max_current: f64,
    pub cell_count: u8,
    pub low_voltage_threshold: f64,
    pub critical_voltage_threshold: f64,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SensorConfig {
    pub sensor_id: SensorId,
    pub sensor_type: autonomy_common::ids::SensorType,
    pub update_rate: f64,
    pub noise_params: NoiseParams,
    pub position: Vec3,
    pub orientation: Vec3, // Euler angles
    pub enabled: bool,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct NoiseParams {
    pub gaussian_std: f64,
    pub bias: f64,
    pub dropout_prob: f64,
    pub latency_ms: f64,
    pub latency_jitter_ms: f64,
}

impl Default for NoiseParams {
    fn default() -> Self {
        Self {
            gaussian_std: 0.01,
            bias: 0.0,
            dropout_prob: 0.0,
            latency_ms: 1.0,
            latency_jitter_ms: 0.1,
        }
    }
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ActuatorConfig {
    pub motor_count: usize,
    pub max_thrust_per_motor: f64,
    pub max_torque_per_motor: f64,
    pub motor_time_constant: f64,
    pub max_rpm: f64,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct EstimatorConfig {
    pub estimator_type: autonomy_common::state::EstimatorType,
    pub process_noise: f64,
    pub measurement_noise: f64,
    pub initial_covariance: f64,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PlannerConfig {
    pub planner_type: String,
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub lookahead_distance: f64,
    pub replan_interval: f64,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ControllerConfig {
    pub controller_type: String,
    pub position_gains: [f64; 3],
    pub velocity_gains: [f64; 3],
    pub attitude_gains: [f64; 3],
    pub max_tilt: f64,
    pub max_velocity: f64,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct AutonomyConfig {
    pub algorithm_id: autonomy_common::ids::AlgorithmId,
    pub wasm_module: Option<String>,
    pub parameters: std::collections::HashMap<String, serde_json::Value>,
    pub tick_budget_us: u64,
}

/// Vehicle state accessor for read-only access
pub trait VehicleStateAccess {
    fn state_read(&self) -> &VehicleState;
    fn state_write(&mut self) -> &mut VehicleState;
}

/// Vehicle with physics integration
pub trait VehiclePhysics: Vehicle {
    fn physics_handle(&self) -> Option<autonomy_physics::bodies::BodyHandle>;
    fn sync_from_physics(&mut self, pose: Pose, linear_vel: LinearVelocity, angular_vel: Vec3, linear_accel: LinearAcceleration);
    fn sync_to_physics(&self) -> (Pose, LinearVelocity, Vec3, LinearAcceleration);
}

/// Vehicle with health monitoring
pub trait VehicleHealth: Vehicle {
    fn update_health(&mut self, dt: f64);
    fn check_battery(&self) -> BatteryStatus;
    fn check_sensors(&self) -> SensorHealth;
    fn check_actuators(&self) -> ActuatorHealth;
    fn check_communication(&self) -> CommHealth;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum BatteryStatus {
    Full,
    Good,
    Low,
    Critical,
    Empty,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum SensorHealth {
    AllNominal,
    Degraded,
    Failed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ActuatorHealth {
    AllNominal,
    Degraded,
    Failed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum CommHealth {
    Connected,
    Degraded,
    Disconnected,
}

/// Vehicle telemetry provider
pub trait VehicleTelemetry: Vehicle {
    fn emit_telemetry(&self, time: SimTime) -> Vec<autonomy_common::telemetry::TelemetryEvent>;
}
//! Common traits for the simulation system

use crate::error::Result;
use crate::ids::{SensorId, VehicleId};
use crate::state::{AlgorithmInput, AlgorithmOutput, ControlInputs, EstimatedState, SensorMeasurements, VehicleState};
use crate::time::SimTime;

/// Simulation module trait - implemented by all major subsystems
pub trait SimModule: Send + Sync {
    fn name(&self) -> &'static str;
    fn initialize(&mut self, _config: &ModuleConfig) -> Result<()> { Ok(()) }
    fn step(&mut self, _time: SimTime, _dt: f64) -> Result<()> { Ok(()) }
    fn shutdown(&mut self) -> Result<()> { Ok(()) }
}

/// Module configuration
#[derive(Clone, Debug, Default)]
pub struct ModuleConfig {
    pub parameters: std::collections::HashMap<String, serde_json::Value>,
}

/// Vehicle abstraction
pub trait Vehicle: Send + Sync {
    fn id(&self) -> VehicleId;
    fn vehicle_type(&self) -> crate::ids::VehicleType;
    fn state(&self) -> &VehicleState;
    fn state_mut(&mut self) -> &mut VehicleState;
    fn sensors(&self) -> &dyn SensorSuite;
    fn actuators(&self) -> &dyn ActuatorSet;
    fn estimator(&self) -> &dyn StateEstimator;
    fn planner(&self) -> &dyn Planner;
    fn controller(&self) -> &dyn Controller;
    fn autonomy(&self) -> &dyn AutonomyModule;
    fn health(&self) -> crate::state::HealthStatus;
    fn step(&mut self, dt: f64, estimation: &EstimatedState) -> Result<()>;
}

/// Sensor suite (collection of sensors)
pub trait SensorSuite: Send + Sync {
    fn sensors(&self) -> Vec<SensorId>;
    fn get_sensor(&self, id: SensorId) -> Option<&dyn Sensor>;
    fn update_all(&mut self, time: SimTime, vehicle_state: &VehicleState) -> Result<SensorMeasurements>;
}

/// Individual sensor
pub trait Sensor: Send + Sync {
    fn id(&self) -> SensorId;
    fn sensor_type(&self) -> crate::ids::SensorType;
    fn update_rate(&self) -> f64;
    fn measure(&mut self, time: SimTime, vehicle_state: &VehicleState) -> Result<Option<crate::state::SensorMeasurement>>;
    fn inject_failure(&mut self, failure: SensorFailure);
    fn clear_failure(&mut self);
    fn is_failed(&self) -> bool;
}

/// Sensor failure types
#[derive(Clone, Debug)]
pub enum SensorFailure {
    CompleteLoss,
    IntermittentLoss { probability: f64 },
    Bias { offset: crate::frames::Vec3 },
    NoiseIncrease { multiplier: f64 },
    LatencyIncrease { additional_ms: f64 },
    Custom(String),
}

/// Actuator set
pub trait ActuatorSet: Send + Sync {
    fn apply_controls(&mut self, inputs: ControlInputs) -> Result<()>;
    fn get_motor_states(&self) -> &[crate::state::MotorState];
    fn set_motor_command(&mut self, index: usize, command: f64) -> Result<()>;
    fn num_actuators(&self) -> usize;
}

/// State estimator
pub trait StateEstimator: Send + Sync {
    fn estimate(&mut self, time: SimTime, measurements: &SensorMeasurements) -> Result<EstimatedState>;
    fn get_estimate(&self) -> &EstimatedState;
    fn reset(&mut self);
    fn estimator_type(&self) -> crate::state::EstimatorType;
}

/// Planner
pub trait Planner: Send + Sync {
    fn plan(&mut self, state: &EstimatedState, mission: &crate::state::MissionState) -> Result<crate::state::Trajectory>;
    fn replan(&mut self, state: &EstimatedState, reason: &str) -> Result<crate::state::Trajectory>;
    fn get_current_plan(&self) -> Option<&crate::state::Trajectory>;
}

/// Controller
pub trait Controller: Send + Sync {
    fn compute_control(&mut self, state: &EstimatedState, trajectory: &crate::state::Trajectory) -> Result<ControlInputs>;
    fn reset(&mut self);
}

/// Autonomy module (can be WASM or native)
pub trait AutonomyModule: Send + Sync {
    fn tick(&mut self, input: &AlgorithmInput) -> Result<AlgorithmOutput>;
    fn reset(&mut self);
    fn algorithm_id(&self) -> crate::ids::AlgorithmId;
}

/// Network interface for vehicle communication
pub trait NetworkInterface: Send + Sync {
    fn send(&self, destination: VehicleId, data: &[u8]) -> Result<()>;
    fn receive(&mut self) -> Result<Vec<(VehicleId, Vec<u8>)>>;
    fn link_quality(&self, peer: VehicleId) -> f64;
    fn is_connected(&self, peer: VehicleId) -> bool;
}

/// Event bus for decoupled communication
pub trait EventBus: Send + Sync {
    fn publish(&self, event: crate::telemetry::TelemetryEvent);
    fn subscribe(&self, event_type: &str) -> Box<dyn EventSubscription>;
}

/// Event subscription
pub trait EventSubscription: Send + Sync {
    fn next(&mut self) -> Option<crate::telemetry::TelemetryEvent>;
    fn try_next(&mut self) -> Option<crate::telemetry::TelemetryEvent>;
}

/// Recording interface
pub trait Recorder: Send + Sync {
    fn record(&mut self, event: &crate::telemetry::TelemetryEvent) -> Result<()>;
    fn flush(&mut self) -> Result<()>;
}

/// Replay interface
pub trait Replayer: Send + Sync {
    fn next_event(&mut self) -> Result<Option<crate::telemetry::TelemetryEvent>>;
    fn seek(&mut self, time: SimTime) -> Result<()>;
    fn current_time(&self) -> SimTime;
}

/// Configuration provider
pub trait ConfigProvider: Send + Sync {
    fn get<T: serde::de::DeserializeOwned>(&self, key: &str) -> Result<T>;
    fn get_or<T: serde::de::DeserializeOwned>(&self, key: &str, default: T) -> T;
    fn set<T: serde::Serialize>(&mut self, key: &str, value: &T) -> Result<()>;
}

/// Time source (for testing)
pub trait TimeSource: Send + Sync {
    fn now(&self) -> SimTime;
    fn real_time(&self) -> std::time::Instant;
}
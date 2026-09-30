//! Core state types for vehicles, sensors, and estimation

use crate::frames::{AngularVelocity, LinearAcceleration, LinearVelocity, Pose, Vec3, deserialize_opt_vec3, deserialize_vec3, deserialize_vec3_vec, serialize_opt_vec3, serialize_vec3, serialize_vec3_vec};
use crate::ids::{AlgorithmId, SensorId, SensorType, VehicleId, VehicleType, ZoneId};
use crate::time::SimTime;
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;
use std::collections::HashMap;

/// Vehicle state at a specific simulation time
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VehicleState {
    pub time: SimTime,
    pub vehicle_id: VehicleId,
    pub vehicle_type: VehicleType,
    pub pose: Pose,
    pub linear_velocity: LinearVelocity,
    pub angular_velocity: AngularVelocity,
    pub linear_acceleration: LinearAcceleration,
    pub battery: BatteryState,
    pub health: HealthStatus,
    pub motor_states: SmallVec<[MotorState; 8]>,
    pub control_inputs: ControlInputs,
}

impl VehicleState {
    pub fn new(vehicle_id: VehicleId, vehicle_type: VehicleType) -> Self {
        Self {
            time: SimTime::ZERO,
            vehicle_id,
            vehicle_type,
            pose: Pose::identity(),
            linear_velocity: LinearVelocity(Vec3::zeros()),
            angular_velocity: AngularVelocity(Vec3::zeros()),
            linear_acceleration: LinearAcceleration(Vec3::zeros()),
            battery: BatteryState::default(),
            health: HealthStatus::Nominal,
            motor_states: SmallVec::new(),
            control_inputs: ControlInputs::default(),
        }
    }
}

/// Battery state
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct BatteryState {
    pub voltage: f64,
    pub current: f64,
    pub charge_remaining: f64, // 0.0 to 1.0
    pub capacity_wh: f64,
    pub temperature: f64,
    pub health: f64, // 0.0 to 1.0
}

impl Default for BatteryState {
    fn default() -> Self {
        Self {
            voltage: 22.2, // 6S LiPo
            current: 0.0,
            charge_remaining: 1.0,
            capacity_wh: 100.0,
            temperature: 25.0,
            health: 1.0,
        }
    }
}

/// Vehicle health status
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum HealthStatus {
    Nominal,
    Degraded,
    Critical,
    Failed,
}

/// Motor/actuator state
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotorState {
    pub rpm: f64,
    pub thrust: f64,
    pub torque: f64,
    pub temperature: f64,
    pub voltage: f64,
    pub current: f64,
    pub health: f64,
}

/// Control inputs to vehicle
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ControlInputs {
    pub collective_thrust: f64, // 0.0 to 1.0
    pub roll_torque: f64,
    pub pitch_torque: f64,
    pub yaw_torque: f64,
    pub throttle: f64,      // UGV
    pub steering: f64,      // UGV
    pub brake: f64,         // UGV
}

/// Estimated state from state estimator
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EstimatedState {
    pub time: SimTime,
    pub vehicle_id: VehicleId,
    pub pose: Pose,
    pub linear_velocity: LinearVelocity,
    pub angular_velocity: AngularVelocity,
    pub linear_acceleration: LinearAcceleration,
    pub covariance: StateCovariance,
    pub estimator_type: EstimatorType,
    pub gps_fixed: bool,
    #[serde(serialize_with = "serialize_opt_vec3", deserialize_with = "deserialize_opt_vec3")]
    pub innovation: Option<Vec3>,
}

/// State covariance matrix (6x6 for pose + velocity)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StateCovariance {
    pub pos: [[f64; 3]; 3],
    pub vel: [[f64; 3]; 3],
    pub pos_vel: [[f64; 3]; 3],
}

impl StateCovariance {
    pub fn new() -> Self {
        Self {
            pos: [[0.0; 3]; 3],
            vel: [[0.0; 3]; 3],
            pos_vel: [[0.0; 3]; 3],
        }
    }

    pub fn diagonal(pos_var: f64, vel_var: f64) -> Self {
        let mut cov = Self::new();
        for i in 0..3 {
            cov.pos[i][i] = pos_var;
            cov.vel[i][i] = vel_var;
        }
        cov
    }

    pub fn position_std(&self) -> Vec3 {
        Vec3::new(
            self.pos[0][0].sqrt(),
            self.pos[1][1].sqrt(),
            self.pos[2][2].sqrt(),
        )
    }

    pub fn velocity_std(&self) -> Vec3 {
        Vec3::new(
            self.vel[0][0].sqrt(),
            self.vel[1][1].sqrt(),
            self.vel[2][2].sqrt(),
        )
    }
}

impl Default for StateCovariance {
    fn default() -> Self {
        Self::new()
    }
}

/// Estimator type for debugging/telemetry
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EstimatorType {
    DeadReckoning,
    ComplementaryFilter,
    Ekf,
    Ukf,
    External,
}

/// Raw sensor measurement
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SensorMeasurement {
    pub time: SimTime,
    pub sensor_id: SensorId,
    pub sensor_type: SensorType,
    pub vehicle_id: VehicleId,
    pub data: SensorData,
    pub latency: f64, // seconds
    pub noise_applied: bool,
}

/// Sensor data variants
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum SensorData {
    Imu {
        #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
        accel: Vec3,
        #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
        gyro: Vec3,
    },
    Gps {
        #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
        position: Vec3,
        #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
        velocity: Vec3,
        hdop: f64,
        vdop: f64,
        satellites: u8,
        fix_type: GpsFixType,
    },
    Camera {
        image_id: u64,
        width: u32,
        height: u32,
        exposure: f64,
    },
    Depth {
        #[serde(serialize_with = "serialize_vec3_vec", deserialize_with = "deserialize_vec3_vec")]
        points: Vec<Vec3>,
        intensity: Vec<f32>,
    },
    Lidar {
        #[serde(serialize_with = "serialize_vec3_vec", deserialize_with = "deserialize_vec3_vec")]
        points: Vec<Vec3>,
        intensities: Vec<f32>,
    },
    Barometer {
        pressure: f64,
        altitude: f64,
        temperature: f64,
    },
    Magnetometer {
        #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
        field: Vec3,
    },
    Custom {
        type_name: String,
        data: Vec<u8>,
    },
}

/// GPS fix type
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum GpsFixType {
    NoFix,
    Fix2D,
    Fix3D,
    DGps,
    RtkFloat,
    RtkFixed,
}

/// Aggregated sensor measurements for a vehicle
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct SensorMeasurements {
    pub imu: Option<SensorMeasurement>,
    pub gps: Option<SensorMeasurement>,
    pub camera: Vec<SensorMeasurement>,
    pub depth: Vec<SensorMeasurement>,
    pub lidar: Vec<SensorMeasurement>,
    pub barometer: Option<SensorMeasurement>,
    pub magnetometer: Option<SensorMeasurement>,
    pub custom: Vec<SensorMeasurement>,
}

impl SensorMeasurements {
    pub fn iter(&self) -> impl Iterator<Item = &SensorMeasurement> {
        self.imu
            .iter()
            .chain(self.gps.iter())
            .chain(self.camera.iter())
            .chain(self.depth.iter())
            .chain(self.lidar.iter())
            .chain(self.barometer.iter())
            .chain(self.magnetometer.iter())
            .chain(self.custom.iter())
    }

    pub fn get_latest(&self, sensor_type: SensorType) -> Option<&SensorMeasurement> {
        match sensor_type {
            SensorType::Imu => self.imu.as_ref(),
            SensorType::Gps => self.gps.as_ref(),
            SensorType::Camera => self.camera.last(),
            SensorType::Depth => self.depth.last(),
            SensorType::Lidar => self.lidar.last(),
            SensorType::Barometer => self.barometer.as_ref(),
            SensorType::Magnetometer => self.magnetometer.as_ref(),
            SensorType::Custom(_) => self.custom.last(),
        }
    }
}

/// Mission state for a vehicle
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MissionState {
    pub mission_id: crate::ids::MissionId,
    pub current_task: Option<crate::ids::TaskId>,
    pub task_progress: f64,
    pub waypoint_index: usize,
    pub state: MissionExecutionState,
    pub parameters: HashMap<String, serde_json::Value>,
}

/// Mission execution state machine
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MissionExecutionState {
    Planned,
    Assigned,
    Executing,
    Paused,
    Completed,
    Failed,
    Aborted,
}

/// Neighbor vehicle state (for coordination)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NeighborState {
    pub vehicle_id: VehicleId,
    pub vehicle_type: VehicleType,
    pub pose: Pose,
    pub velocity: LinearVelocity,
    pub estimated_state: Option<EstimatedState>,
    pub distance: f64,
    #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
    pub relative_velocity: Vec3,
    pub communication_quality: f64, // 0.0 to 1.0
    pub last_update: SimTime,
}

/// Environment snapshot for autonomy algorithms
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EnvironmentSnapshot {
    pub time: SimTime,
    #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
    pub wind: Vec3,
    pub gravity: f64,
    #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
    pub magnetic_field: Vec3,
    pub no_fly_zones: Vec<ZoneId>,
    pub obstacles: Vec<Obstacle>,
    pub terrain_config: Option<TerrainConfig>,
}

/// Terrain configuration for serialization
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TerrainConfig {
    pub terrain_type: String,
    pub seed: u64,
    pub max_height: f64,
}

/// Obstacle representation
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Obstacle {
    pub id: ZoneId,
    #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
    pub center: Vec3,
    pub radius: f64,
    pub height: f64,
    pub obstacle_type: ObstacleType,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ObstacleType {
    Building,
    Tree,
    Terrain,
    Vehicle,
    Dynamic,
    RestrictedZone,
}

/// Algorithm configuration (serializable)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AlgorithmConfig {
    pub algorithm_id: AlgorithmId,
    pub parameters: HashMap<String, serde_json::Value>,
}

impl Default for AlgorithmConfig {
    fn default() -> Self {
        Self {
            algorithm_id: AlgorithmId::nil(),
            parameters: HashMap::new(),
        }
    }
}

/// Algorithm output to vehicle controller
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AlgorithmOutput {
    pub time: SimTime,
    pub vehicle_id: VehicleId,
    pub desired_trajectory: Option<Trajectory>,
    #[serde(serialize_with = "serialize_opt_vec3", deserialize_with = "deserialize_opt_vec3")]
    pub desired_velocity: Option<Vec3>,
    pub desired_heading: Option<f64>,
    pub mission_transition: Option<MissionTransition>,
    pub fleet_requests: Vec<FleetRequest>,
    pub custom_data: Vec<u8>,
}

/// Trajectory representation
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Trajectory {
    pub waypoints: Vec<TrajectoryWaypoint>,
    pub start_time: SimTime,
    pub duration: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TrajectoryWaypoint {
    #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
    pub position: Vec3,
    #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
    pub velocity: Vec3,
    #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
    pub acceleration: Vec3,
    pub yaw: f64,
    pub yaw_rate: f64,
    pub time_from_start: f64,
}

/// Mission state transition request
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MissionTransition {
    pub from: MissionExecutionState,
    pub to: MissionExecutionState,
    pub reason: String,
}

/// Fleet-level request from algorithm
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum FleetRequest {
    TaskRequest {
        task_type: String,
        priority: u8,
        #[serde(serialize_with = "serialize_opt_vec3", deserialize_with = "deserialize_opt_vec3")]
        location: Option<Vec3>,
    },
    CoordinationRequest {
        target_vehicle: VehicleId,
        request_type: String,
        data: Vec<u8>,
    },
    StatusReport {
        status: VehicleStatusReport,
    },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VehicleStatusReport {
    pub vehicle_id: VehicleId,
    pub health: HealthStatus,
    pub battery: BatteryState,
    pub mission_state: MissionState,
    pub estimator_quality: f64,
}

/// Algorithm input from simulation to WASM module
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AlgorithmInput {
    pub version: u32,
    pub time: SimTime,
    pub vehicle_id: VehicleId,
    pub estimated_state: EstimatedState,
    pub sensor_measurements: SensorMeasurements,
    pub neighbors: Vec<NeighborState>,
    pub mission_state: MissionState,
    pub environment: EnvironmentSnapshot,
    pub config: AlgorithmConfig,
}

impl Default for AlgorithmInput {
    fn default() -> Self {
        Self {
            version: 1,
            time: SimTime::ZERO,
            vehicle_id: VehicleId::nil(),
            estimated_state: EstimatedState {
                time: SimTime::ZERO,
                vehicle_id: VehicleId::nil(),
                pose: Pose::identity(),
                linear_velocity: LinearVelocity(Vec3::zeros()),
                angular_velocity: AngularVelocity(Vec3::zeros()),
                linear_acceleration: LinearAcceleration(Vec3::zeros()),
                covariance: StateCovariance::new(),
                estimator_type: EstimatorType::DeadReckoning,
                gps_fixed: false,
                innovation: None,
            },
            sensor_measurements: SensorMeasurements::default(),
            neighbors: Vec::new(),
            mission_state: MissionState {
                mission_id: crate::ids::MissionId::nil(),
                current_task: None,
                task_progress: 0.0,
                waypoint_index: 0,
                state: MissionExecutionState::Planned,
                parameters: HashMap::new(),
            },
            environment: EnvironmentSnapshot {
                time: SimTime::ZERO,
                wind: Vec3::zeros(),
                gravity: GRAVITY,
                magnetic_field: Vec3::new(0.0, 0.0, -50.0),
                no_fly_zones: Vec::new(),
                obstacles: Vec::new(),
                terrain_config: None,
            },
            config: AlgorithmConfig::default(),
        }
    }
}

const GRAVITY: f64 = 9.80665;
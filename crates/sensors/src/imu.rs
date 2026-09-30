//! IMU sensor model

use autonomy_common::frames::{Vec3, LinearAcceleration, AngularVelocity, Pose};
use autonomy_common::ids::{SensorId, SensorType};
use autonomy_common::rng::SimRng;
use autonomy_common::state::{SensorMeasurement, SensorData};
use autonomy_common::time::SimTime;
use autonomy_common::traits::{Sensor, SensorFailure};
use autonomy_environment::World;
use nalgebra::UnitQuaternion;
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// IMU configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ImuConfig {
    pub update_rate: f64,
    pub accelerometer_noise: f64,
    pub gyroscope_noise: f64,
    pub accelerometer_bias: Vec3,
    pub gyroscope_bias: Vec3,
    pub accelerometer_random_walk: f64,
    pub gyroscope_random_walk: f64,
    pub latency_ms: f64,
    pub latency_jitter_ms: f64,
}

impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            update_rate: 200.0,
            accelerometer_noise: 0.01,
            gyroscope_noise: 0.001,
            accelerometer_bias: Vec3::zeros(),
            gyroscope_bias: Vec3::zeros(),
            accelerometer_random_walk: 0.001,
            gyroscope_random_walk: 0.0001,
            latency_ms: 2.0,
            latency_jitter_ms: 0.5,
        }
    }
}

/// IMU sensor
pub struct ImuSensor {
    id: SensorId,
    config: ImuConfig,
    pose: Pose, // Body to sensor transform
    last_update: SimTime,
    failure: RwLock<Option<SensorFailure>>,
    bias_accel: RwLock<Vec3>,
    bias_gyro: RwLock<Vec3>,
    rng: RwLock<SimRng>,
}

impl ImuSensor {
    pub fn new(id: SensorId, config: ImuConfig, pose: Pose, rng: SimRng) -> Self {
        Self {
            id,
            config,
            pose,
            last_update: SimTime::ZERO,
            failure: RwLock::new(None),
            bias_accel: RwLock::new(config.accelerometer_bias),
            bias_gyro: RwLock::new(config.gyroscope_bias),
            rng: RwLock::new(rng),
        }
    }

    pub fn update_biases(&self, dt: f64) {
        use autonomy_common::rng::sample_normal;
        let mut rng = self.rng.write();
        
        let mut accel_bias = self.bias_accel.write();
        let mut gyro_bias = self.bias_gyro.write();
        
        // Random walk
        *accel_bias += Vec3::new(
            sample_normal(&mut rng, 0.0, self.config.accelerometer_random_walk * dt.sqrt()),
            sample_normal(&mut rng, 0.0, self.config.accelerometer_random_walk * dt.sqrt()),
            sample_normal(&mut rng, 0.0, self.config.accelerometer_random_walk * dt.sqrt()),
        );
        
        *gyro_bias += Vec3::new(
            sample_normal(&mut rng, 0.0, self.config.gyroscope_random_walk * dt.sqrt()),
            sample_normal(&mut rng, 0.0, self.config.gyroscope_random_walk * dt.sqrt()),
            sample_normal(&mut rng, 0.0, self.config.gyroscope_random_walk * dt.sqrt()),
        );
    }
}

impl Sensor for ImuSensor {
    fn id(&self) -> SensorId {
        self.id
    }

    fn sensor_type(&self) -> SensorType {
        SensorType::Imu
    }

    fn update_rate(&self) -> f64 {
        self.config.update_rate
    }

    fn measure(&mut self, time: SimTime, vehicle_state: &autonomy_common::state::VehicleState) -> autonomy_common::error::Result<Option<SensorMeasurement>> {
        // Check if it's time to update
        let dt = (time - self.last_update).as_seconds();
        if dt < 1.0 / self.config.update_rate {
            return Ok(None);
        }

        // Check failure
        if let Some(failure) = self.failure.read().as_ref() {
            if matches!(failure, SensorFailure::CompleteLoss) {
                return Ok(None);
            }
            if matches!(failure, SensorFailure::IntermittentLoss { probability }) {
                let mut rng = self.rng.write();
                if autonomy_common::rng::sample_bool(&mut rng, *probability) {
                    return Ok(None);
                }
            }
        }

        self.last_update = time;
        self.update_biases(dt);

        // Get ground truth
        let true_accel = vehicle_state.linear_acceleration.0;
        let true_gyro = vehicle_state.angular_velocity.0;

        // Transform to sensor frame
        let sensor_accel = self.pose.orientation.inverse() * true_accel;
        let sensor_gyro = self.pose.orientation.inverse() * true_gyro;

        // Add gravity in sensor frame
        let gravity_sensor = self.pose.orientation.inverse() * Vec3::new(0.0, -9.80665, 0.0);
        let measured_accel = sensor_accel + gravity_sensor;

        // Add noise and bias
        let mut rng = self.rng.write();
        let mut measured_accel = measured_accel;
        let mut measured_gyro = sensor_gyro;

        let failure = self.failure.read();
        
        let noise_mult = failure.as_ref()
            .and_then(|f| if let SensorFailure::NoiseIncrease { multiplier } = f { Some(*multiplier) } else { None })
            .unwrap_or(1.0);

        // Accelerometer noise
        measured_accel += Vec3::new(
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.accelerometer_noise * noise_mult),
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.accelerometer_noise * noise_mult),
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.accelerometer_noise * noise_mult),
        );
        measured_accel += *self.bias_accel.read();

        // Gyroscope noise
        measured_gyro += Vec3::new(
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.gyroscope_noise * noise_mult),
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.gyroscope_noise * noise_mult),
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.gyroscope_noise * noise_mult),
        );
        measured_gyro += *self.bias_gyro.read();

        // Apply latency
        let latency = self.config.latency_ms / 1000.0;
        let jitter = autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.latency_jitter_ms / 1000.0);
        let total_latency = (latency + jitter).max(0.0);

        Ok(Some(SensorMeasurement {
            time: time - SimTime::from_seconds(total_latency),
            sensor_id: self.id,
            sensor_type: SensorType::Imu,
            vehicle_id: vehicle_state.vehicle_id,
            data: SensorData::Imu {
                accel: measured_accel,
                gyro: measured_gyro,
            },
            latency: total_latency,
            noise_applied: true,
        }))
    }

    fn inject_failure(&mut self, failure: SensorFailure) {
        *self.failure.write() = Some(failure);
    }

    fn clear_failure(&mut self) {
        *self.failure.write() = None;
    }

    fn is_failed(&self) -> bool {
        self.failure.read().is_some()
    }
}

/// IMU sensor module for simulation
pub struct ImuModule {
    sensors: Vec<Arc<parking_lot::RwLock<ImuSensor>>>,
}

impl ImuModule {
    pub fn new() -> Self {
        Self { sensors: Vec::new() }
    }

    pub fn add_sensor(&mut self, sensor: ImuSensor) {
        self.sensors.push(Arc::new(parking_lot::RwLock::new(sensor)));
    }
}

impl autonomy_common::traits::SimModule for ImuModule {
    fn name(&self) -> &'static str {
        "imu"
    }
}
//! Magnetometer sensor model

use autonomy_common::frames::{Vec3, Pose};
use autonomy_common::ids::{SensorId, SensorType};
use autonomy_common::rng::SimRng;
use autonomy_common::state::{SensorMeasurement, SensorData};
use autonomy_common::time::SimTime;
use autonomy_common::traits::{Sensor, SensorFailure};
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// Magnetometer configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MagnetometerConfig {
    pub update_rate: f64,
    pub noise: f64,
    pub bias: Vec3,
    pub hard_iron: Vec3,
    pub soft_iron: Vec3, // Diagonal of 3x3 matrix
    pub latency_ms: f64,
    pub latency_jitter_ms: f64,
}

impl Default for MagnetometerConfig {
    fn default() -> Self {
        Self {
            update_rate: 50.0,
            noise: 0.1, // µT
            bias: Vec3::zeros(),
            hard_iron: Vec3::zeros(),
            soft_iron: Vec3::new(1.0, 1.0, 1.0),
            latency_ms: 10.0,
            latency_jitter_ms: 2.0,
        }
    }
}

/// Magnetometer sensor
pub struct MagnetometerSensor {
    id: SensorId,
    config: MagnetometerConfig,
    pose: Pose,
    last_update: SimTime,
    failure: RwLock<Option<SensorFailure>>,
    rng: RwLock<SimRng>,
}

impl MagnetometerSensor {
    pub fn new(id: SensorId, config: MagnetometerConfig, pose: Pose, rng: SimRng) -> Self {
        Self {
            id,
            config,
            pose,
            last_update: SimTime::ZERO,
            failure: RwLock::new(None),
            rng: RwLock::new(rng),
        }
    }
}

impl Sensor for MagnetometerSensor {
    fn id(&self) -> SensorId {
        self.id
    }

    fn sensor_type(&self) -> SensorType {
        SensorType::Magnetometer
    }

    fn update_rate(&self) -> f64 {
        self.config.update_rate
    }

    fn measure(&mut self, time: SimTime, vehicle_state: &autonomy_common::state::VehicleState) -> autonomy_common::error::Result<Option<SensorMeasurement>> {
        let dt = (time - self.last_update).as_seconds();
        if dt < 1.0 / self.config.update_rate {
            return Ok(None);
        }

        let mut rng = self.rng.write();
        
        // Check failure
        if let Some(failure) = self.failure.read().as_ref() {
            if matches!(failure, SensorFailure::CompleteLoss) {
                return Ok(None);
            }
            if matches!(failure, SensorFailure::IntermittentLoss { probability }) {
                if autonomy_common::rng::sample_bool(&mut rng, *probability) {
                    return Ok(None);
                }
            }
        }

        self.last_update = time;

        // Get Earth magnetic field in world frame
        let world_field = vehicle_state.pose.orientation * Vec3::new(20.0, 0.0, -45.0); // Example field

        // Transform to sensor frame
        let sensor_field = self.pose.orientation.inverse() * world_field;

        // Apply hard/soft iron distortion
        let distorted = Vec3::new(
            sensor_field.x * self.config.soft_iron.x + self.config.hard_iron.x,
            sensor_field.y * self.config.soft_iron.y + self.config.hard_iron.y,
            sensor_field.z * self.config.soft_iron.z + self.config.hard_iron.z,
        );

        // Add noise and bias
        let failure = self.failure.read();
        let noise_mult = failure.as_ref()
            .and_then(|f| if let SensorFailure::NoiseIncrease { multiplier } = f { Some(*multiplier) } else { None })
            .unwrap_or(1.0);

        let measured = distorted + self.config.bias + Vec3::new(
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.noise * noise_mult),
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.noise * noise_mult),
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.noise * noise_mult),
        );

        // Apply latency
        let latency = self.config.latency_ms / 1000.0;
        let jitter = autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.latency_jitter_ms / 1000.0);
        let total_latency = (latency + jitter).max(0.0);

        Ok(Some(SensorMeasurement {
            time: time - SimTime::from_seconds(total_latency),
            sensor_id: self.id,
            sensor_type: SensorType::Magnetometer,
            vehicle_id: vehicle_state.vehicle_id,
            data: SensorData::Magnetometer {
                field: measured,
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

/// Magnetometer sensor module
pub struct MagnetometerModule {
    sensors: Vec<Arc<parking_lot::RwLock<MagnetometerSensor>>>,
}

impl MagnetometerModule {
    pub fn new() -> Self {
        Self { sensors: Vec::new() }
    }

    pub fn add_sensor(&mut self, sensor: MagnetometerSensor) {
        self.sensors.push(Arc::new(parking_lot::RwLock::new(sensor)));
    }
}

impl autonomy_common::traits::SimModule for MagnetometerModule {
    fn name(&self) -> &'static str {
        "magnetometer"
    }
}
//! Barometer sensor model

use autonomy_common::frames::{Vec3, Pose};
use autonomy_common::ids::{SensorId, SensorType};
use autonomy_common::rng::SimRng;
use autonomy_common::state::{SensorMeasurement, SensorData};
use autonomy_common::time::SimTime;
use autonomy_common::traits::{Sensor, SensorFailure};
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// Barometer configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BarometerConfig {
    pub update_rate: f64,
    pub pressure_noise: f64,
    pub pressure_bias: f64,
    pub temperature_noise: f64,
    pub latency_ms: f64,
    pub latency_jitter_ms: f64,
}

impl Default for BarometerConfig {
    fn default() -> Self {
        Self {
            update_rate: 50.0,
            pressure_noise: 1.0, // Pa
            pressure_bias: 0.0,
            temperature_noise: 0.1,
            latency_ms: 10.0,
            latency_jitter_ms: 2.0,
        }
    }
}

/// Barometer sensor
pub struct BarometerSensor {
    id: SensorId,
    config: BarometerConfig,
    pose: Pose,
    last_update: SimTime,
    failure: RwLock<Option<SensorFailure>>,
    rng: RwLock<SimRng>,
}

impl BarometerSensor {
    pub fn new(id: SensorId, config: BarometerConfig, pose: Pose, rng: SimRng) -> Self {
        Self {
            id,
            config,
            pose,
            last_update: SimTime::ZERO,
            failure: RwLock::new(None),
            rng: RwLock::new(rng),
        }
    }

    /// Pressure to altitude conversion (standard atmosphere)
    pub fn pressure_to_altitude(pressure: f64, sea_level_pressure: f64) -> f64 {
        44330.0 * (1.0 - (pressure / sea_level_pressure).powf(0.1903))
    }

    /// Altitude to pressure conversion
    pub fn altitude_to_pressure(altitude: f64, sea_level_pressure: f64) -> f64 {
        sea_level_pressure * (1.0 - altitude / 44330.0).powf(5.255)
    }
}

impl Sensor for BarometerSensor {
    fn id(&self) -> SensorId {
        self.id
    }

    fn sensor_type(&self) -> SensorType {
        SensorType::Barometer
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

        // Get altitude from vehicle position
        let altitude = vehicle_state.pose.position.y;

        // Standard atmosphere
        let sea_level_pressure = 101325.0; // Pa
        let temperature = 288.15 - 0.0065 * altitude; // K

        // True pressure
        let true_pressure = Self::altitude_to_pressure(altitude, sea_level_pressure);

        // Add noise and bias
        let failure = self.failure.read();
        let noise_mult = failure.as_ref()
            .and_then(|f| if let SensorFailure::NoiseIncrease { multiplier } = f { Some(*multiplier) } else { None })
            .unwrap_or(1.0);

        let measured_pressure = true_pressure + self.config.pressure_bias + 
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.pressure_noise * noise_mult);
        let measured_temperature = temperature + 
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.temperature_noise);

        // Calculate altitude from measured pressure
        let measured_altitude = Self::pressure_to_altitude(measured_pressure, sea_level_pressure);

        // Apply latency
        let latency = self.config.latency_ms / 1000.0;
        let jitter = autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.latency_jitter_ms / 1000.0);
        let total_latency = (latency + jitter).max(0.0);

        Ok(Some(SensorMeasurement {
            time: time - SimTime::from_seconds(total_latency),
            sensor_id: self.id,
            sensor_type: SensorType::Barometer,
            vehicle_id: vehicle_state.vehicle_id,
            data: SensorData::Barometer {
                pressure: measured_pressure,
                altitude: measured_altitude,
                temperature: measured_temperature,
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

/// Barometer sensor module
pub struct BarometerModule {
    sensors: Vec<Arc<parking_lot::RwLock<BarometerSensor>>>,
}

impl BarometerModule {
    pub fn new() -> Self {
        Self { sensors: Vec::new() }
    }

    pub fn add_sensor(&mut self, sensor: BarometerSensor) {
        self.sensors.push(Arc::new(parking_lot::RwLock::new(sensor)));
    }
}

impl autonomy_common::traits::SimModule for BarometerModule {
    fn name(&self) -> &'static str {
        "barometer"
    }
}
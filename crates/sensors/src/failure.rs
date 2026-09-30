//! Sensor failure injection

use autonomy_common::frames::Vec3;
use autonomy_common::ids::{SensorId, SensorType};
use autonomy_common::rng::SimRng;
use autonomy_common::traits::{Sensor, SensorFailure};
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

/// Failure injection manager
pub struct FailureInjector {
    failures: HashMap<SensorId, ActiveFailure>,
    rng: RwLock<SimRng>,
}

#[derive(Clone, Debug)]
struct ActiveFailure {
    failure: SensorFailure,
    start_time: f64,
    duration: Option<f64>,
    next_intermittent: f64,
}

impl FailureInjector {
    pub fn new(rng: SimRng) -> Self {
        Self {
            failures: HashMap::new(),
            rng: RwLock::new(rng),
        }
    }

    pub fn inject(&mut self, sensor_id: SensorId, failure: SensorFailure, start_time: f64, duration: Option<f64>) {
        let next_intermittent = if let SensorFailure::IntermittentLoss { .. } = &failure {
            start_time + 1.0
        } else {
            0.0
        };
        
        self.failures.insert(sensor_id, ActiveFailure {
            failure,
            start_time,
            duration,
            next_intermittent,
        });
    }

    pub fn remove(&mut self, sensor_id: SensorId) {
        self.failures.remove(&sensor_id);
    }

    pub fn update(&mut self, current_time: f64) {
        let mut to_remove = Vec::new();
        
        for (id, failure) in &mut self.failures {
            // Check duration
            if let Some(dur) = failure.duration {
                if current_time - failure.start_time >= dur {
                    to_remove.push(*id);
                    continue;
                }
            }
            
            // Update intermittent failures
            if let SensorFailure::IntermittentLoss { .. } = &failure.failure {
                if current_time >= failure.next_intermittent {
                    failure.next_intermittent = current_time + 1.0; // Check every second
                }
            }
        }
        
        for id in to_remove {
            self.failures.remove(&id);
        }
    }

    pub fn apply_failure(&self, sensor_id: SensorId, measurement: &mut autonomy_common::state::SensorMeasurement, current_time: f64) -> bool {
        if let Some(active) = self.failures.get(&sensor_id) {
            match &active.failure {
                SensorFailure::CompleteLoss => {
                    return false; // Indicate measurement should be dropped
                }
                SensorFailure::IntermittentLoss { probability } => {
                    let mut rng = self.rng.write();
                    if autonomy_common::rng::sample_bool(&mut rng, *probability) {
                        return false;
                    }
                }
                SensorFailure::Bias { offset } => {
                    if let autonomy_common::state::SensorData::Imu { accel, gyro } = &mut measurement.data {
                        *accel += *offset;
                    }
                    if let autonomy_common::state::SensorData::Gps { position, .. } = &mut measurement.data {
                        *position += *offset;
                    }
                }
                SensorFailure::NoiseIncrease { multiplier } => {
                    // Handled in sensor measure function
                }
                SensorFailure::LatencyIncrease { additional_ms } => {
                    measurement.latency += additional_ms / 1000.0;
                }
                SensorFailure::Custom(_) => {}
            }
        }
        true
    }

    pub fn is_failed(&self, sensor_id: SensorId) -> bool {
        self.failures.contains_key(&sensor_id)
    }

    pub fn get_failure(&self, sensor_id: SensorId) -> Option<&SensorFailure> {
        self.failures.get(&sensor_id).map(|a| &a.failure)
    }
}

/// Predefined failure scenarios
pub struct FailureScenarios;

impl FailureScenarios {
    /// GPS denial scenario
    pub fn gps_denial(start_time: f64, duration: f64) -> (SensorId, SensorFailure, f64, Option<f64>) {
        // Would need actual sensor ID
        (SensorId::nil(), SensorFailure::CompleteLoss, start_time, Some(duration))
    }

    /// GPS intermittent scenario
    pub fn gps_intermittent(start_time: f64, probability: f64) -> (SensorId, SensorFailure, f64, Option<f64>) {
        (SensorId::nil(), SensorFailure::IntermittentLoss { probability }, start_time, None)
    }

    /// IMU bias scenario
    pub fn imu_bias(start_time: f64, bias: Vec3, duration: f64) -> (SensorId, SensorFailure, f64, Option<f64>) {
        (SensorId::nil(), SensorFailure::Bias { offset: bias }, start_time, Some(duration))
    }

    /// Sensor degradation scenario
    pub fn sensor_degradation(start_time: f64, noise_multiplier: f64, duration: f64) -> (SensorId, SensorFailure, f64, Option<f64>) {
        (SensorId::nil(), SensorFailure::NoiseIncrease { multiplier: noise_multiplier }, start_time, Some(duration))
    }

    /// Complete sensor failure
    pub fn sensor_failure(start_time: f64) -> (SensorId, SensorFailure, f64, Option<f64>) {
        (SensorId::nil(), SensorFailure::CompleteLoss, start_time, None)
    }
}

/// Failure injection schedule for scenarios
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FailureSchedule {
    pub failures: Vec<ScheduledFailure>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ScheduledFailure {
    pub sensor_id: SensorId,
    pub sensor_type: SensorType,
    pub failure: SensorFailure,
    pub start_time: f64,
    pub duration: Option<f64>,
    pub description: String,
}

impl FailureSchedule {
    pub fn new() -> Self {
        Self { failures: Vec::new() }
    }

    pub fn add_failure(&mut self, failure: ScheduledFailure) {
        self.failures.push(failure);
    }

    pub fn apply(&self, injector: &mut FailureInjector, current_time: f64) {
        for failure in &self.failures {
            if current_time >= failure.start_time 
                && failure.duration.map_or(true, |d| current_time - failure.start_time < d) {
                injector.inject(failure.sensor_id, failure.failure.clone(), failure.start_time, failure.duration);
            }
        }
    }
}

/// Common failure configurations
pub mod presets {
    use super::*;

    pub fn gps_outage_30s(sensor_id: SensorId, start: f64) -> ScheduledFailure {
        ScheduledFailure {
            sensor_id,
            sensor_type: SensorType::Gps,
            failure: SensorFailure::CompleteLoss,
            start_time: start,
            duration: Some(30.0),
            description: "GPS outage for 30 seconds".into(),
        }
    }

    pub fn gps_intermittent_50(sensor_id: SensorId, start: f64) -> ScheduledFailure {
        ScheduledFailure {
            sensor_id,
            sensor_type: SensorType::Gps,
            failure: SensorFailure::IntermittentLoss { probability: 0.5 },
            start_time: start,
            duration: Some(60.0),
            description: "GPS 50% dropout for 60 seconds".into(),
        }
    }

    pub fn imu_bias_x(sensor_id: SensorId, start: f64, bias: f64) -> ScheduledFailure {
        ScheduledFailure {
            sensor_id,
            sensor_type: SensorType::Imu,
            failure: SensorFailure::Bias { offset: Vec3::new(bias, 0.0, 0.0) },
            start_time: start,
            duration: Some(120.0),
            description: format!("IMU X-axis bias of {} m/s^2", bias),
        }
    }

    pub fn lidar_dropout(sensor_id: SensorId, start: f64, duration: f64) -> ScheduledFailure {
        ScheduledFailure {
            sensor_id,
            sensor_type: SensorType::Lidar,
            failure: SensorFailure::CompleteLoss,
            start_time: start,
            duration: Some(duration),
            description: format!("LiDAR dropout for {} seconds", duration),
        }
    }

    pub fn camera_degraded(sensor_id: SensorId, start: f64, noise_mult: f64, duration: f64) -> ScheduledFailure {
        ScheduledFailure {
            sensor_id,
            sensor_type: SensorType::Camera,
            failure: SensorFailure::NoiseIncrease { multiplier: noise_mult },
            start_time: start,
            duration: Some(duration),
            description: format!("Camera noise increased {}x for {} seconds", noise_mult, duration),
        }
    }
}
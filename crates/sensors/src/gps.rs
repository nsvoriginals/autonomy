//! GPS/GNSS sensor model

use autonomy_common::frames::{Vec3, LinearVelocity, Pose};
use autonomy_common::ids::{SensorId, SensorType};
use autonomy_common::rng::SimRng;
use autonomy_common::state::{SensorMeasurement, SensorData, GpsFixType};
use autonomy_common::time::SimTime;
use autonomy_common::traits::{Sensor, SensorFailure};
use autonomy_environment::World;
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// GPS configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GpsConfig {
    pub update_rate: f64,
    pub position_noise: f64,
    pub velocity_noise: f64,
    pub position_bias: Vec3,
    pub velocity_bias: Vec3,
    pub min_satellites: u8,
    pub max_satellites: u8,
    pub latency_ms: f64,
    pub latency_jitter_ms: f64,
    pub dropout_probability: f64,
    pub outage_duration: f64, // seconds
}

impl Default for GpsConfig {
    fn default() -> Self {
        Self {
            update_rate: 10.0,
            position_noise: 1.5,
            velocity_noise: 0.1,
            position_bias: Vec3::zeros(),
            velocity_bias: Vec3::zeros(),
            min_satellites: 6,
            max_satellites: 12,
            latency_ms: 50.0,
            latency_jitter_ms: 10.0,
            dropout_probability: 0.0,
            outage_duration: 0.0,
        }
    }
}

/// GPS sensor
pub struct GpsSensor {
    id: SensorId,
    config: GpsConfig,
    pose: Pose,
    last_update: SimTime,
    failure: RwLock<Option<SensorFailure>>,
    outage_timer: RwLock<f64>,
    rng: RwLock<SimRng>,
}

impl GpsSensor {
    pub fn new(id: SensorId, config: GpsConfig, pose: Pose, rng: SimRng) -> Self {
        Self {
            id,
            config,
            pose,
            last_update: SimTime::ZERO,
            failure: RwLock::new(None),
            outage_timer: RwLock::new(0.0),
            rng: RwLock::new(rng),
        }
    }
}

impl Sensor for GpsSensor {
    fn id(&self) -> SensorId {
        self.id
    }

    fn sensor_type(&self) -> SensorType {
        SensorType::Gps
    }

    fn update_rate(&self) -> f64 {
        self.config.update_rate
    }

    fn measure(&mut self, time: SimTime, vehicle_state: &autonomy_common::state::VehicleState) -> autonomy_common::error::Result<Option<SensorMeasurement>> {
        let dt = (time - self.last_update).as_seconds();
        if dt < 1.0 / self.config.update_rate {
            return Ok(None);
        }

        // Check failure
        let mut outage_timer = self.outage_timer.write();
        let mut rng = self.rng.write();
        
        if let Some(failure) = self.failure.read().as_ref() {
            match failure {
                SensorFailure::CompleteLoss => {
                    return Ok(None);
                }
                SensorFailure::IntermittentLoss { probability } => {
                    if autonomy_common::rng::sample_bool(&mut rng, *probability) {
                        return Ok(None);
                    }
                }
                _ => {}
            }
        }

        // Handle GPS outage
        if self.config.outage_duration > 0.0 {
            *outage_timer += dt;
            if *outage_timer < self.config.outage_duration {
                return Ok(None);
            }
        }

        self.last_update = time;

        // Get ground truth
        let true_pos = vehicle_state.pose.position;
        let true_vel = vehicle_state.linear_velocity.0;

        // Transform to sensor frame
        let sensor_pos = self.pose.orientation.inverse() * true_pos + self.pose.position;
        let sensor_vel = self.pose.orientation.inverse() * true_vel;

        // Add noise and bias
        let failure = self.failure.read();
        let noise_mult = failure.as_ref()
            .and_then(|f| if let SensorFailure::NoiseIncrease { multiplier } = f { Some(*multiplier) } else { None })
            .unwrap_or(1.0);

        let mut measured_pos = sensor_pos + self.config.position_bias;
        let mut measured_vel = sensor_vel + self.config.velocity_bias;

        measured_pos += Vec3::new(
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.position_noise * noise_mult),
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.position_noise * noise_mult),
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.position_noise * noise_mult * 1.5), // Worse vertical
        );

        measured_vel += Vec3::new(
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.velocity_noise * noise_mult),
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.velocity_noise * noise_mult),
            autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.velocity_noise * noise_mult),
        );

        // Simulate satellite count
        let satellites = self.config.min_satellites + 
            (autonomy_common::rng::sample_uniform(&mut rng, 0.0, 1.0) * 
            (self.config.max_satellites - self.config.min_satellites) as f64) as u8;

        // Determine fix type based on satellites and conditions
        let fix_type = if satellites >= 4 {
            if satellites >= 8 { GpsFixType::RtkFixed } else { GpsFixType::Fix3D }
        } else if satellites >= 3 {
            GpsFixType::Fix2D
        } else {
            GpsFixType::NoFix
        };

        // HDOP/VDOP based on satellite geometry (simplified)
        let hdop = 1.0 + 2.0 / (satellites as f64).max(1.0);
        let vdop = hdop * 1.5;

        // Apply latency
        let latency = self.config.latency_ms / 1000.0;
        let jitter = autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.latency_jitter_ms / 1000.0);
        let total_latency = (latency + jitter).max(0.0);

        Ok(Some(SensorMeasurement {
            time: time - SimTime::from_seconds(total_latency),
            sensor_id: self.id,
            sensor_type: SensorType::Gps,
            vehicle_id: vehicle_state.vehicle_id,
            data: SensorData::Gps {
                position: measured_pos,
                velocity: measured_vel,
                hdop,
                vdop,
                satellites,
                fix_type,
            },
            latency: total_latency,
            noise_applied: true,
        }))
    }

    fn inject_failure(&mut self, failure: SensorFailure) {
        *self.failure.write() = Some(failure);
        if matches!(failure, SensorFailure::CompleteLoss) {
            *self.outage_timer.write() = 0.0;
        }
    }

    fn clear_failure(&mut self) {
        *self.failure.write() = None;
        *self.outage_timer.write() = 0.0;
    }

    fn is_failed(&self) -> bool {
        self.failure.read().is_some()
    }
}

/// GPS sensor module
pub struct GpsModule {
    sensors: Vec<Arc<parking_lot::RwLock<GpsSensor>>>,
}

impl GpsModule {
    pub fn new() -> Self {
        Self { sensors: Vec::new() }
    }

    pub fn add_sensor(&mut self, sensor: GpsSensor) {
        self.sensors.push(Arc::new(parking_lot::RwLock::new(sensor)));
    }
}

impl autonomy_common::traits::SimModule for GpsModule {
    fn name(&self) -> &'static str {
        "gps"
    }
}
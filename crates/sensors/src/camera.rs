//! Camera sensor model

use autonomy_common::frames::{Vec3, Pose};
use autonomy_common::ids::{SensorId, SensorType};
use autonomy_common::rng::SimRng;
use autonomy_common::state::{SensorMeasurement, SensorData};
use autonomy_common::time::SimTime;
use autonomy_common::traits::{Sensor, SensorFailure};
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// Camera configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CameraConfig {
    pub update_rate: f64,
    pub width: u32,
    pub height: u32,
    pub hfov: f64, // Horizontal field of view in radians
    pub vfov: f64, // Vertical field of view in radians
    pub exposure_time: f64,
    pub gain: f64,
    pub noise_std: f64,
    pub latency_ms: f64,
    pub latency_jitter_ms: f64,
    pub dropout_probability: f64,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            update_rate: 30.0,
            width: 640,
            height: 480,
            hfov: 1.047, // 60 degrees
            vfov: 0.785, // 45 degrees
            exposure_time: 0.01,
            gain: 1.0,
            noise_std: 5.0, // Digital noise
            latency_ms: 33.0,
            latency_jitter_ms: 5.0,
            dropout_probability: 0.0,
        }
    }
}

/// Camera sensor
pub struct CameraSensor {
    id: SensorId,
    config: CameraConfig,
    pose: Pose,
    last_update: SimTime,
    failure: RwLock<Option<SensorFailure>>,
    frame_counter: RwLock<u64>,
    rng: RwLock<SimRng>,
}

impl CameraSensor {
    pub fn new(id: SensorId, config: CameraConfig, pose: Pose, rng: SimRng) -> Self {
        Self {
            id,
            config,
            pose,
            last_update: SimTime::ZERO,
            failure: RwLock::new(None),
            frame_counter: RwLock::new(0),
            rng: RwLock::new(rng),
        }
    }

    pub fn projection_matrix(&self) -> nalgebra::Matrix4<f64> {
        let fx = (self.config.width as f64) / (2.0 * (self.config.hfov / 2.0).tan());
        let fy = (self.config.height as f64) / (2.0 * (self.config.vfov / 2.0).tan());
        let cx = self.config.width as f64 / 2.0;
        let cy = self.config.height as f64 / 2.0;

        nalgebra::Matrix4::new(
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        )
    }

    pub fn project_point(&self, point: Vec3) -> Option<(f64, f64)> {
        // Transform point to camera frame
        let local = self.pose.inverse().transform_point(point);
        
        // Check if in front of camera
        if local.z <= 0.0 {
            return None;
        }

        // Perspective projection
        let x = local.x / local.z * (self.config.hfov / 2.0).tan() * (self.config.width as f64 / 2.0) + self.config.width as f64 / 2.0;
        let y = local.y / local.z * (self.config.vfov / 2.0).tan() * (self.config.height as f64 / 2.0) + self.config.height as f64 / 2.0;

        // Check if in image bounds
        if x >= 0.0 && x < self.config.width as f64 && y >= 0.0 && y < self.config.height as f64 {
            Some((x, y))
        } else {
            None
        }
    }

    pub fn frustum_corners(&self, distance: f64) -> [Vec3; 4] {
        let hw = distance * (self.config.hfov / 2.0).tan();
        let hh = distance * (self.config.vfov / 2.0).tan();
        
        let corners_local = [
            Vec3::new(-hw, -hh, distance),
            Vec3::new(hw, -hh, distance),
            Vec3::new(hw, hh, distance),
            Vec3::new(-hw, hh, distance),
        ];

        corners_local.map(|c| self.pose.transform_point(c))
    }
}

impl Sensor for CameraSensor {
    fn id(&self) -> SensorId {
        self.id
    }

    fn sensor_type(&self) -> SensorType {
        SensorType::Camera
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
        let frame_id = {
            let mut counter = self.frame_counter.write();
            *counter += 1;
            *counter
        };

        // Apply latency
        let latency = self.config.latency_ms / 1000.0;
        let jitter = autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.latency_jitter_ms / 1000.0);
        let total_latency = (latency + jitter).max(0.0);

        Ok(Some(SensorMeasurement {
            time: time - SimTime::from_seconds(total_latency),
            sensor_id: self.id,
            sensor_type: SensorType::Camera,
            vehicle_id: vehicle_state.vehicle_id,
            data: SensorData::Camera {
                image_id: frame_id,
                width: self.config.width,
                height: self.config.height,
                exposure: self.config.exposure_time,
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

/// Camera sensor module
pub struct CameraModule {
    sensors: Vec<Arc<parking_lot::RwLock<CameraSensor>>>,
}

impl CameraModule {
    pub fn new() -> Self {
        Self { sensors: Vec::new() }
    }

    pub fn add_sensor(&mut self, sensor: CameraSensor) {
        self.sensors.push(Arc::new(parking_lot::RwLock::new(sensor)));
    }
}

impl autonomy_common::traits::SimModule for CameraModule {
    fn name(&self) -> &'static str {
        "camera"
    }
}
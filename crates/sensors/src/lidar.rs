//! LiDAR sensor model

use autonomy_common::frames::{Vec3, Pose};
use autonomy_common::ids::{SensorId, SensorType};
use autonomy_common::rng::SimRng;
use autonomy_common::state::{SensorMeasurement, SensorData};
use autonomy_common::time::SimTime;
use autonomy_common::traits::{Sensor, SensorFailure};
use autonomy_environment::World;
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// LiDAR configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LidarConfig {
    pub update_rate: f64,
    pub channels: usize,
    pub points_per_channel: usize,
    pub vertical_fov: f64, // radians
    pub horizontal_fov: f64, // radians
    pub min_range: f64,
    pub max_range: f64,
    pub range_noise: f64,
    pub intensity_noise: f64,
    pub dropout_probability: f64,
    pub latency_ms: f64,
    pub latency_jitter_ms: f64,
}

impl Default for LidarConfig {
    fn default() -> Self {
        Self {
            update_rate: 20.0,
            channels: 64,
            points_per_channel: 512,
            vertical_fov: 0.733, // ~42 degrees
            horizontal_fov: 6.283, // 360 degrees
            min_range: 0.5,
            max_range: 200.0,
            range_noise: 0.02,
            intensity_noise: 10.0,
            dropout_probability: 0.0,
            latency_ms: 50.0,
            latency_jitter_ms: 5.0,
        }
    }
}

/// LiDAR sensor
pub struct LidarSensor {
    id: SensorId,
    config: LidarConfig,
    pose: Pose,
    last_update: SimTime,
    failure: RwLock<Option<SensorFailure>>,
    rng: RwLock<SimRng>,
}

impl LidarSensor {
    pub fn new(id: SensorId, config: LidarConfig, pose: Pose, rng: SimRng) -> Self {
        Self {
            id,
            config,
            pose,
            last_update: SimTime::ZERO,
            failure: RwLock::new(None),
            rng: RwLock::new(rng),
        }
    }

    fn generate_rays(&self) -> Vec<Vec3> {
        let mut rays = Vec::with_capacity(self.config.channels * self.config.points_per_channel);
        
        let v_step = self.config.vertical_fov / (self.config.channels - 1).max(1) as f64;
        let h_step = self.config.horizontal_fov / self.config.points_per_channel as f64;
        
        let v_start = -self.config.vertical_fov / 2.0;
        
        for c in 0..self.config.channels {
            let v_angle = v_start + c as f64 * v_step;
            let cos_v = v_angle.cos();
            let sin_v = v_angle.sin();
            
            for p in 0..self.config.points_per_channel {
                let h_angle = p as f64 * h_step;
                let cos_h = h_angle.cos();
                let sin_h = h_angle.sin();
                
                // Ray in sensor frame (forward is +X, up is +Z for typical LiDAR)
                rays.push(Vec3::new(cos_v * cos_h, sin_v, cos_v * sin_h));
            }
        }
        
        rays
    }

    fn cast_ray(&self, world: &World, origin: Vec3, direction: Vec3, rng: &mut SimRng) -> Option<(Vec3, f32)> {
        // Simplified ray casting - would use proper collision detection
        // For now, return simulated hits based on terrain
        let terrain_height = world.height_at(origin);
        
        // Check if ray hits ground
        if direction.y < 0.0 {
            let t = (terrain_height - origin.y) / direction.y;
            if t > 0.0 && t < self.config.max_range {
                let hit = origin + direction * t;
                let distance = t + autonomy_common::rng::sample_normal(rng, 0.0, self.config.range_noise);
                let intensity = (1000.0 / (1.0 + distance)).max(0.0) as f32;
                return Some((hit, intensity));
            }
        }
        
        // Check obstacles (simplified)
        for zone in world.zones().iter_zones() {
            if zone.zone_type == autonomy_environment::zones::ZoneType::Obstacle 
                || zone.zone_type == autonomy_environment::zones::ZoneType::Building {
                // Simplified sphere intersection
                let oc = origin - zone.center;
                let a = direction.dot(&direction);
                let b = 2.0 * oc.dot(&direction);
                let c = oc.dot(&oc) - zone.radius * zone.radius;
                let disc = b * b - 4.0 * a * c;
                
                if disc >= 0.0 {
                    let t = (-b - disc.sqrt()) / (2.0 * a);
                    if t > 0.0 && t < self.config.max_range {
                        let hit = origin + direction * t;
                        let distance = t + autonomy_common::rng::sample_normal(rng, 0.0, self.config.range_noise);
                        let intensity = (500.0 / (1.0 + distance)).max(0.0) as f32;
                        return Some((hit, intensity));
                    }
                }
            }
        }
        
        None
    }
}

impl Sensor for LidarSensor {
    fn id(&self) -> SensorId {
        self.id
    }

    fn sensor_type(&self) -> SensorType {
        SensorType::Lidar
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

        // In a real implementation, we'd need access to the world
        // For now, generate simulated point cloud
        let rays = self.generate_rays();
        let mut points = Vec::new();
        let mut intensities = Vec::new();
        
        let sensor_origin = self.pose.transform_point(Vec3::zeros());
        
        for ray in rays {
            let world_ray = self.pose.orientation * ray;
            
            // Simulate some hits
            if autonomy_common::rng::sample_bool(&mut rng, 0.3) {
                let distance = autonomy_common::rng::sample_uniform(&mut rng, self.config.min_range, self.config.max_range);
                let hit = sensor_origin + world_ray * distance;
                let noise = autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.range_noise);
                let noisy_hit = hit + world_ray * noise;
                
                points.push(noisy_hit);
                intensities.push((1000.0 / (1.0 + distance)).max(0.0) as f32);
            }
        }

        // Apply latency
        let latency = self.config.latency_ms / 1000.0;
        let jitter = autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.latency_jitter_ms / 1000.0);
        let total_latency = (latency + jitter).max(0.0);

        Ok(Some(SensorMeasurement {
            time: time - SimTime::from_seconds(total_latency),
            sensor_id: self.id,
            sensor_type: SensorType::Lidar,
            vehicle_id: vehicle_state.vehicle_id,
            data: SensorData::Lidar {
                points,
                intensities,
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

/// LiDAR sensor module
pub struct LidarModule {
    sensors: Vec<Arc<parking_lot::RwLock<LidarSensor>>>,
}

impl LidarModule {
    pub fn new() -> Self {
        Self { sensors: Vec::new() }
    }

    pub fn add_sensor(&mut self, sensor: LidarSensor) {
        self.sensors.push(Arc::new(parking_lot::RwLock::new(sensor)));
    }
}

impl autonomy_common::traits::SimModule for LidarModule {
    fn name(&self) -> &'static str {
        "lidar"
    }
}
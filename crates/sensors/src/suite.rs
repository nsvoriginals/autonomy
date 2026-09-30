//! Sensor suite - collection of sensors on a vehicle

use autonomy_common::error::Result;
use autonomy_common::frames::Pose;
use autonomy_common::ids::{SensorId, SensorType, VehicleId};
use autonomy_common::rng::{derive_rng, seeded_rng};
use autonomy_common::state::{SensorMeasurement, SensorMeasurements};
use autonomy_common::time::SimTime;
use autonomy_common::traits::{Sensor, SensorSuite};
use crate::{ImuSensor, ImuConfig, GpsSensor, GpsConfig, CameraSensor, CameraConfig, LidarSensor, LidarConfig, BarometerSensor, BarometerConfig, MagnetometerSensor, MagnetometerConfig};
use parking_lot::RwLock;
use smallvec::SmallVec;
use std::collections::HashMap;
use std::sync::Arc;

/// Sensor suite configuration
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SensorSuiteConfig {
    pub imu: Option<ImuConfig>,
    pub gps: Option<GpsConfig>,
    pub cameras: Vec<CameraConfig>,
    pub lidars: Vec<LidarConfig>,
    pub barometer: Option<BarometerConfig>,
    pub magnetometer: Option<MagnetometerConfig>,
}

impl Default for SensorSuiteConfig {
    fn default() -> Self {
        Self {
            imu: Some(ImuConfig::default()),
            gps: Some(GpsConfig::default()),
            cameras: vec![CameraConfig::default()],
            lidars: vec![],
            barometer: Some(BarometerConfig::default()),
            magnetometer: Some(MagnetometerConfig::default()),
        }
    }
}

/// Sensor suite implementation
pub struct SensorSuiteImpl {
    vehicle_id: VehicleId,
    sensors: HashMap<SensorId, Arc<RwLock<dyn Sensor>>>,
    imu_id: Option<SensorId>,
    gps_id: Option<SensorId>,
    camera_ids: Vec<SensorId>,
    lidar_ids: Vec<SensorId>,
    barometer_id: Option<SensorId>,
    magnetometer_id: Option<SensorId>,
}

impl SensorSuiteImpl {
    pub fn new(vehicle_id: VehicleId, config: SensorSuiteConfig, base_rng: seeded_rng) -> Self {
        let mut sensors = HashMap::new();
        let mut rng = base_rng;
        
        let mut imu_id = None;
        let mut gps_id = None;
        let mut camera_ids = Vec::new();
        let mut lidar_ids = Vec::new();
        let mut barometer_id = None;
        let mut magnetometer_id = None;

        // IMU
        if let Some(imu_config) = config.imu {
            let id = SensorId::new();
            let pose = Pose::identity();
            let sensor = ImuSensor::new(id, imu_config, pose, derive_rng());
            sensors.insert(id, Arc::new(RwLock::new(sensor)));
            imu_id = Some(id);
        }

        // GPS
        if let Some(gps_config) = config.gps {
            let id = SensorId::new();
            let pose = Pose::identity();
            let sensor = GpsSensor::new(id, gps_config, pose, derive_rng());
            sensors.insert(id, Arc::new(RwLock::new(sensor)));
            gps_id = Some(id);
        }

        // Cameras
        for cam_config in config.cameras {
            let id = SensorId::new();
            let pose = Pose::identity();
            let sensor = CameraSensor::new(id, cam_config, pose, derive_rng());
            sensors.insert(id, Arc::new(RwLock::new(sensor)));
            camera_ids.push(id);
        }

        // LiDARs
        for lidar_config in config.lidars {
            let id = SensorId::new();
            let pose = Pose::identity();
            let sensor = LidarSensor::new(id, lidar_config, pose, derive_rng());
            sensors.insert(id, Arc::new(RwLock::new(sensor)));
            lidar_ids.push(id);
        }

        // Barometer
        if let Some(baro_config) = config.barometer {
            let id = SensorId::new();
            let pose = Pose::identity();
            let sensor = BarometerSensor::new(id, baro_config, pose, derive_rng());
            sensors.insert(id, Arc::new(RwLock::new(sensor)));
            barometer_id = Some(id);
        }

        // Magnetometer
        if let Some(mag_config) = config.magnetometer {
            let id = SensorId::new();
            let pose = Pose::identity();
            let sensor = MagnetometerSensor::new(id, mag_config, pose, derive_rng());
            sensors.insert(id, Arc::new(RwLock::new(sensor)));
            magnetometer_id = Some(id);
        }

        Self {
            vehicle_id,
            sensors,
            imu_id,
            gps_id,
            camera_ids,
            lidar_ids,
            barometer_id,
            magnetometer_id,
        }
    }

    pub fn get_sensor(&self, id: SensorId) -> Option<Arc<RwLock<dyn Sensor>>> {
        self.sensors.get(&id).cloned()
    }

    pub fn imu(&self) -> Option<Arc<RwLock<dyn Sensor>>> {
        self.imu_id.and_then(|id| self.sensors.get(&id).cloned())
    }

    pub fn gps(&self) -> Option<Arc<RwLock<dyn Sensor>>> {
        self.gps_id.and_then(|id| self.sensors.get(&id).cloned())
    }

    pub fn cameras(&self) -> Vec<Arc<RwLock<dyn Sensor>>> {
        self.camera_ids.iter().filter_map(|id| self.sensors.get(id).cloned()).collect()
    }

    pub fn lidars(&self) -> Vec<Arc<RwLock<dyn Sensor>>> {
        self.lidar_ids.iter().filter_map(|id| self.sensors.get(id).cloned()).collect()
    }

    pub fn barometer(&self) -> Option<Arc<RwLock<dyn Sensor>>> {
        self.barometer_id.and_then(|id| self.sensors.get(&id).cloned())
    }

    pub fn magnetometer(&self) -> Option<Arc<RwLock<dyn Sensor>>> {
        self.magnetometer_id.and_then(|id| self.sensors.get(&id).cloned())
    }
}

impl SensorSuite for SensorSuiteImpl {
    fn sensors(&self) -> Vec<SensorId> {
        self.sensors.keys().copied().collect()
    }

    fn get_sensor(&self, id: SensorId) -> Option<&dyn Sensor> {
        // Can't return reference to RwLock guard
        None
    }

    fn update_all(&mut self, time: SimTime, vehicle_state: &autonomy_common::state::VehicleState) -> Result<SensorMeasurements> {
        let mut measurements = SensorMeasurements::default();
        
        for (id, sensor) in &self.sensors {
            let mut sensor = sensor.write();
            if let Some(measurement) = sensor.measure(time, vehicle_state)? {
                match measurement.sensor_type {
                    SensorType::Imu => measurements.imu = Some(measurement),
                    SensorType::Gps => measurements.gps = Some(measurement),
                    SensorType::Camera => measurements.camera.push(measurement),
                    SensorType::Lidar => measurements.lidar.push(measurement),
                    SensorType::Barometer => measurements.barometer = Some(measurement),
                    SensorType::Magnetometer => measurements.magnetometer = Some(measurement),
                    SensorType::Custom(_) => measurements.custom.push(measurement),
                    _ => measurements.custom.push(measurement),
                }
            }
        }
        
        Ok(measurements)
    }
}
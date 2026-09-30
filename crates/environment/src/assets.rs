//! 3D asset management for rendering

use autonomy_common::frames::Vec3;
use autonomy_common::ids::ZoneId;
use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::PathBuf;

/// Asset handle for 3D models
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AssetHandle {
    pub path: PathBuf,
    pub scale: Vec3,
    pub rotation: Vec3, // Euler angles in degrees
}

/// Predefined asset library
pub struct AssetLibrary {
    assets: HashMap<String, AssetHandle>,
    loaded: HashMap<String, Handle<Scene>>,
}

impl AssetLibrary {
    pub fn new() -> Self {
        Self {
            assets: HashMap::new(),
            loaded: HashMap::new(),
        }
    }

    pub fn register(&mut self, name: impl Into<String>, handle: AssetHandle) {
        self.assets.insert(name.into(), handle);
    }

    pub fn get(&self, name: &str) -> Option<&AssetHandle> {
        self.assets.get(name)
    }

    pub fn load_all(&mut self, asset_server: &AssetServer) {
        for (name, handle) in &self.assets {
            let scene: Handle<Scene> = asset_server.load(&handle.path);
            self.loaded.insert(name.clone(), scene);
        }
    }

    pub fn get_scene(&self, name: &str) -> Option<Handle<Scene>> {
        self.loaded.get(name).cloned()
    }
}

impl Default for AssetLibrary {
    fn default() -> Self {
        let mut lib = Self::new();
        
        // Default vehicle models
        lib.register("uav_quad", AssetHandle {
            path: "models/uav_quad.glb".into(),
            scale: Vec3::splat(1.0),
            rotation: Vec3::new(-90.0, 0.0, 0.0),
        });
        
        lib.register("uav_fixed_wing", AssetHandle {
            path: "models/uav_fixed_wing.glb".into(),
            scale: Vec3::splat(1.0),
            rotation: Vec3::new(-90.0, 0.0, 0.0),
        });
        
        lib.register("ugv_wheeled", AssetHandle {
            path: "models/ugv_wheeled.glb".into(),
            scale: Vec3::splat(1.0),
            rotation: Vec3::new(-90.0, 0.0, 0.0),
        });
        
        lib.register("ugv_tracked", AssetHandle {
            path: "models/ugv_tracked.glb".into(),
            scale: Vec3::splat(1.0),
            rotation: Vec3::new(-90.0, 0.0, 0.0),
        });

        // Environment assets
        lib.register("landing_pad", AssetHandle {
            path: "models/landing_pad.glb".into(),
            scale: Vec3::splat(1.0),
            rotation: Vec3::new(-90.0, 0.0, 0.0),
        });
        
        lib.register("charging_station", AssetHandle {
            path: "models/charging_station.glb".into(),
            scale: Vec3::splat(1.0),
            rotation: Vec3::new(-90.0, 0.0, 0.0),
        });
        
        lib.register("comm_tower", AssetHandle {
            path: "models/comm_tower.glb".into(),
            scale: Vec3::splat(1.0),
            rotation: Vec3::new(-90.0, 0.0, 0.0),
        });
        
        lib.register("building_small", AssetHandle {
            path: "models/building_small.glb".into(),
            scale: Vec3::splat(1.0),
            rotation: Vec3::new(-90.0, 0.0, 0.0),
        });
        
        lib.register("tree_pine", AssetHandle {
            path: "models/tree_pine.glb".into(),
            scale: Vec3::splat(1.0),
            rotation: Vec3::new(-90.0, 0.0, 0.0),
        });

        lib
    }
}

/// Vehicle visual configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VehicleVisualConfig {
    pub model: String,
    pub scale: Vec3,
    pub color: Color,
    pub show_trajectory: bool,
    pub show_velocity: bool,
    pub show_sensor_frustums: bool,
    pub show_comm_range: bool,
}

impl Default for VehicleVisualConfig {
    fn default() -> Self {
        Self {
            model: "uav_quad".into(),
            scale: Vec3::splat(1.0),
            color: Color::srgb(0.2, 0.6, 1.0),
            show_trajectory: true,
            show_velocity: false,
            show_sensor_frustums: false,
            show_comm_range: false,
        }
    }
}

/// Zone visual configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ZoneVisualConfig {
    pub color: Color,
    pub wireframe: bool,
    pub height: f64,
    pub pulse: bool,
}

impl Default for ZoneVisualConfig {
    fn default() -> Self {
        Self {
            color: Color::srgba(1.0, 1.0, 0.0, 0.3),
            wireframe: true,
            height: 10.0,
            pulse: false,
        }
    }
}

/// Default visual configs by zone type
pub fn default_zone_visual(zone_type: crate::zones::ZoneType) -> ZoneVisualConfig {
    use crate::zones::ZoneType;
    match zone_type {
        ZoneType::LandingZone => ZoneVisualConfig {
            color: Color::srgba(0.0, 1.0, 0.0, 0.3),
            wireframe: false,
            height: 0.5,
            pulse: false,
        },
        ZoneType::ChargingZone => ZoneVisualConfig {
            color: Color::srgba(0.0, 0.5, 1.0, 0.4),
            wireframe: false,
            height: 3.0,
            pulse: true,
        },
        ZoneType::RestrictedZone => ZoneVisualConfig {
            color: Color::srgba(1.0, 0.0, 0.0, 0.3),
            wireframe: true,
            height: 50.0,
            pulse: false,
        },
        ZoneType::MissionArea => ZoneVisualConfig {
            color: Color::srgba(0.0, 1.0, 1.0, 0.2),
            wireframe: true,
            height: 5.0,
            pulse: false,
        },
        ZoneType::CommunicationNode => ZoneVisualConfig {
            color: Color::srgba(1.0, 0.5, 0.0, 0.3),
            wireframe: true,
            height: 20.0,
            pulse: true,
        },
        ZoneType::Obstacle | ZoneType::Building => ZoneVisualConfig {
            color: Color::srgba(0.5, 0.5, 0.5, 0.5),
            wireframe: false,
            height: 10.0,
            pulse: false,
        },
        _ => ZoneVisualConfig::default(),
    }
}

/// Default vehicle visual by type
pub fn default_vehicle_visual(vehicle_type: autonomy_common::ids::VehicleType) -> VehicleVisualConfig {
    use autonomy_common::ids::VehicleType;
    match vehicle_type {
        VehicleType::Uav => VehicleVisualConfig {
            model: "uav_quad".into(),
            scale: Vec3::splat(1.0),
            color: Color::srgb(0.2, 0.6, 1.0),
            ..Default::default()
        },
        VehicleType::Ugv => VehicleVisualConfig {
            model: "ugv_wheeled".into(),
            scale: Vec3::splat(1.0),
            color: Color::srgb(0.6, 0.4, 0.2),
            ..Default::default()
        },
        VehicleType::BaseStation => VehicleVisualConfig {
            model: "comm_tower".into(),
            scale: Vec3::splat(2.0),
            color: Color::srgb(1.0, 0.8, 0.0),
            ..Default::default()
        },
        VehicleType::Unknown => VehicleVisualConfig::default(),
    }
}

/// Environment rendering resources
#[derive(Resource, Clone, Default)]
pub struct EnvironmentAssets {
    pub library: AssetLibrary,
    pub vehicle_configs: HashMap<autonomy_common::ids::VehicleId, VehicleVisualConfig>,
    pub zone_configs: HashMap<ZoneId, ZoneVisualConfig>,
}

impl EnvironmentAssets {
    pub fn new() -> Self {
        Self {
            library: AssetLibrary::default(),
            vehicle_configs: HashMap::new(),
            zone_configs: HashMap::new(),
        }
    }

    pub fn set_vehicle_config(&mut self, vehicle_id: autonomy_common::ids::VehicleId, config: VehicleVisualConfig) {
        self.vehicle_configs.insert(vehicle_id, config);
    }

    pub fn get_vehicle_config(&self, vehicle_id: autonomy_common::ids::VehicleId) -> VehicleVisualConfig {
        self.vehicle_configs.get(&vehicle_id).cloned()
            .unwrap_or_else(|| default_vehicle_visual(autonomy_common::ids::VehicleType::Unknown))
    }

    pub fn set_zone_config(&mut self, zone_id: ZoneId, config: ZoneVisualConfig) {
        self.zone_configs.insert(zone_id, config);
    }

    pub fn get_zone_config(&self, zone_id: ZoneId) -> ZoneVisualConfig {
        self.zone_configs.get(&zone_id).cloned().unwrap_or_default()
    }
}
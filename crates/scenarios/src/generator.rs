//! Scenario generator

use autonomy_common::frames::Vec3;
use autonomy_common::ids::{MissionId, TaskId, VehicleId, ZoneId, VehicleType};
use autonomy_common::rng::{seeded_rng, SimRng};
use autonomy_common::time::SimTime;
use crate::definition::{Scenario, ScenarioMetadata, EnvironmentConfig, VehicleConfig, ZoneConfig, MissionConfig, TaskConfig, FailureConfig, NetworkConfig, NetworkPartition, AutonomyConfig, SensorConfig};
use autonomy_environment::terrain::TerrainType;
use autonomy_environment::weather::WeatherState;
use autonomy_environment::zones::ZoneType;
use crate::definition::FailureType;
use serde_json;
use std::collections::HashMap;

/// Scenario generator
pub struct ScenarioGenerator {
    rng: SimRng,
}

impl ScenarioGenerator {
    pub fn new(seed: u64) -> Self {
        Self {
            rng: seeded_rng(seed),
        }
    }

    pub fn generate(&mut self, template: &ScenarioTemplate) -> Scenario {
        let mut scenario = Scenario::default();
        scenario.metadata = ScenarioMetadata {
            name: template.name.clone(),
            description: template.description.clone(),
            seed: self.rng.gen(),
            duration: template.duration,
            version: "0.1".into(),
        };

        // Generate environment
        scenario.environment = self.generate_environment(&template.environment);

        // Generate vehicles
        scenario.vehicles = self.generate_vehicles(&template.vehicles);

        // Generate zones
        scenario.zones = self.generate_zones(&template.zones);

        // Generate missions
        scenario.missions = self.generate_missions(&template.missions, &scenario.vehicles);

        // Generate failures
        scenario.failures = self.generate_failures(&template.failures, &scenario.vehicles);

        // Generate network config
        scenario.network = template.network.clone().unwrap_or_default();

        scenario
    }

    fn generate_environment(&mut self, template: &EnvironmentTemplate) -> EnvironmentConfig {
        EnvironmentConfig {
            terrain: template.terrain.clone().unwrap_or_else(|| TerrainType::Flat { height: 0.0 }),
            weather: template.weather.clone().unwrap_or_else(|| WeatherState::clear()),
            time_of_day: template.time_of_day.unwrap_or(12.0),
            gravity: template.gravity.unwrap_or(9.80665),
        }
    }

    fn generate_vehicles(&mut self, template: &VehicleTemplate) -> Vec<VehicleConfig> {
        let mut vehicles = Vec::new();
        
        for i in 0..template.count {
            let vtype = template.vehicle_type.unwrap_or(VehicleType::Uav);
            let id = match vtype {
                VehicleType::Uav => VehicleId::uav(i),
                VehicleType::Ugv => VehicleId::ugv(i),
                _ => VehicleId::new(),
            };

            let pos = Vec3::new(
                autonomy_common::rng::sample_uniform(&mut self.rng, -500.0, 500.0),
                template.altitude.unwrap_or(50.0),
                autonomy_common::rng::sample_uniform(&mut self.rng, -500.0, 500.0),
            );

            vehicles.push(VehicleConfig {
                id,
                vehicle_type: vtype,
                callsign: format!("{:?}-{:03}", vtype, i),
                position: pos,
                velocity: Vec3::zeros(),
                orientation: autonomy_common::rng::sample_uniform(&mut self.rng, 0.0, 2.0 * std::f64::consts::PI),
                battery: autonomy_common::rng::sample_uniform(&mut self.rng, 0.5, 1.0),
                autonomy_config: AutonomyConfig {
                    algorithm_id: autonomy_common::ids::AlgorithmId::new(),
                    wasm_module: template.wasm_module.clone(),
                    parameters: HashMap::new(),
                },
                sensors: template.sensors.clone().unwrap_or_default(),
            });
        }

        vehicles
    }

    fn generate_zones(&mut self, template: &ZoneTemplate) -> Vec<ZoneConfig> {
        let mut zones = Vec::new();

        for i in 0..template.landing_zones {
            zones.push(ZoneConfig {
                id: ZoneId::new(),
                zone_type: ZoneType::LandingZone,
                name: format!("LZ-{:03}", i),
                center: Vec3::new(
                    autonomy_common::rng::sample_uniform(&mut self.rng, -800.0, 800.0),
                    0.0,
                    autonomy_common::rng::sample_uniform(&mut self.rng, -800.0, 800.0),
                ),
                radius: 10.0,
                height: 5.0,
                properties: HashMap::new(),
            });
        }

        for i in 0..template.charging_zones {
            zones.push(ZoneConfig {
                id: ZoneId::new(),
                zone_type: ZoneType::ChargingZone,
                name: format!("CHG-{:03}", i),
                center: Vec3::new(
                    autonomy_common::rng::sample_uniform(&mut self.rng, -800.0, 800.0),
                    0.0,
                    autonomy_common::rng::sample_uniform(&mut self.rng, -800.0, 800.0),
                ),
                radius: 5.0,
                height: 3.0,
                properties: HashMap::new(),
            });
        }

        zones
    }

    fn generate_missions(&mut self, template: &MissionTemplate, vehicles: &[VehicleConfig]) -> Vec<MissionConfig> {
        let mut missions = Vec::new();

        for i in 0..template.count {
            let assigned: Vec<VehicleId> = vehicles.iter()
                .filter(|v| v.vehicle_type == template.vehicle_type.unwrap_or(VehicleType::Uav))
                .map(|v| v.id)
                .collect();

            missions.push(MissionConfig {
                id: MissionId::new(),
                mission_type: template.mission_type.clone().unwrap_or("waypoint".into()),
                assigned_vehicles: assigned,
                tasks: vec![TaskConfig {
                    id: TaskId::new(),
                    task_type: "navigate".into(),
                    position: Some(Vec3::new(
                        autonomy_common::rng::sample_uniform(&mut self.rng, -800.0, 800.0),
                        template.altitude.unwrap_or(50.0),
                        autonomy_common::rng::sample_uniform(&mut self.rng, -800.0, 800.0),
                    )),
                    parameters: HashMap::new(),
                    priority: 1,
                }],
                parameters: HashMap::new(),
            });
        }

        missions
    }

    fn generate_failures(&mut self, template: &FailureTemplate, vehicles: &[VehicleConfig]) -> Vec<FailureConfig> {
        let mut failures = Vec::new();

        for failure_spec in &template.failures {
            let target = if failure_spec.target == "random" {
                vehicles[autonomy_common::rng::sample_uniform(&mut self.rng, 0.0, vehicles.len() as f64) as usize].id
            } else {
                // Find by callsign
                vehicles.iter().find(|v| v.callsign == failure_spec.target).map(|v| v.id).unwrap_or(vehicles[0].id)
            };

            failures.push(FailureConfig {
                time: failure_spec.time,
                target,
                failure_type: failure_spec.failure_type,
                parameters: failure_spec.parameters.clone(),
            });
        }

        failures
    }
}

/// Template for generating scenarios
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ScenarioTemplate {
    pub name: String,
    pub description: String,
    pub duration: f64,
    pub environment: EnvironmentTemplate,
    pub vehicles: VehicleTemplate,
    pub zones: ZoneTemplate,
    pub missions: MissionTemplate,
    pub failures: FailureTemplate,
    pub network: Option<NetworkConfig>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct EnvironmentTemplate {
    pub terrain: Option<TerrainType>,
    pub weather: Option<WeatherState>,
    pub time_of_day: Option<f64>,
    pub gravity: Option<f64>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct VehicleTemplate {
    pub count: usize,
    pub vehicle_type: Option<VehicleType>,
    pub altitude: Option<f64>,
    pub wasm_module: Option<String>,
    pub sensors: Option<Vec<SensorConfig>>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct ZoneTemplate {
    pub landing_zones: usize,
    pub charging_zones: usize,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct MissionTemplate {
    pub count: usize,
    pub mission_type: Option<String>,
    pub vehicle_type: Option<VehicleType>,
    pub altitude: Option<f64>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct FailureTemplate {
    pub failures: Vec<FailureSpec>,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct FailureSpec {
    pub time: f64,
    pub target: String, // "random" or callsign
    pub failure_type: FailureType,
    pub parameters: HashMap<String, serde_json::Value>,
}
//! Physics simulation orchestration

use autonomy_common::frames::Vec3;
use autonomy_common::ids::{VehicleId, VehicleType};
use autonomy_common::time::SimTime;
use autonomy_common::traits::SimModule;
use rapier3d::prelude::*;

use crate::bodies::{BodyConfig, BodyHandle, PhysicsBody, uav_body_config, ugv_body_config};
use crate::collision::{ColliderConfig, uav_collider_config, ugv_collider_config};
use crate::rapier_backend::{PhysicsModule, RapierBackend};

/// Physics simulation configuration
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PhysicsSimulationConfig {
    pub gravity: Vec3,
    pub integration_method: crate::integration::IntegrationMethod,
    pub max_substeps: u32,
    pub solver_iterations: usize,
    pub ccd_enabled: bool,
}

impl Default for PhysicsSimulationConfig {
    fn default() -> Self {
        Self {
            gravity: Vec3::new(0.0, -9.80665, 0.0),
            integration_method: crate::integration::IntegrationMethod::SemiImplicitEuler,
            max_substeps: 4,
            solver_iterations: 10,
            ccd_enabled: true,
        }
    }
}

/// Main physics simulation
pub struct PhysicsSimulation {
    module: PhysicsModule,
    config: PhysicsSimulationConfig,
    vehicle_configs: std::collections::HashMap<VehicleId, VehiclePhysicsConfig>,
}

#[derive(Clone, Debug)]
pub struct VehiclePhysicsConfig {
    pub body_config: BodyConfig,
    pub collider_config: ColliderConfig,
}

impl PhysicsSimulation {
    pub fn new(config: PhysicsSimulationConfig) -> Self {
        let mut module = PhysicsModule::new(config.gravity);
        module.backend_mut().set_integration_method(config.integration_method);
        
        Self {
            module,
            config,
            vehicle_configs: std::collections::HashMap::new(),
        }
    }

    pub fn add_uav(&mut self, vehicle_id: VehicleId, mass: f64, arm_length: f64, pose: autonomy_common::frames::Pose) -> BodyHandle {
        let body_config = uav_body_config(mass, arm_length);
        let collider_config = uav_collider_config(arm_length, 0.1);
        
        self.vehicle_configs.insert(vehicle_id, VehiclePhysicsConfig {
            body_config: body_config.clone(),
            collider_config: collider_config.clone(),
        });
        
        self.module.add_vehicle(vehicle_id, body_config, pose, collider_config)
    }

    pub fn add_ugv(&mut self, vehicle_id: VehicleId, mass: f64, length: f64, width: f64, height: f64, pose: autonomy_common::frames::Pose) -> BodyHandle {
        let body_config = ugv_body_config(mass, length, width);
        let collider_config = ugv_collider_config(length, width, height);
        
        self.vehicle_configs.insert(vehicle_id, VehiclePhysicsConfig {
            body_config: body_config.clone(),
            collider_config: collider_config.clone(),
        });
        
        self.module.add_vehicle(vehicle_id, body_config, pose, collider_config)
    }

    pub fn remove_vehicle(&mut self, vehicle_id: VehicleId) -> Option<PhysicsBody> {
        self.vehicle_configs.remove(&vehicle_id);
        self.module.remove_vehicle(vehicle_id)
    }

    pub fn apply_thrust(&mut self, vehicle_id: VehicleId, thrust: Vec3) {
        if let Some(handle) = self.module.backend().get_body_by_vehicle(vehicle_id) {
            self.module.backend_mut().apply_force(handle, thrust, "thrust");
        }
    }

    pub fn apply_torque(&mut self, vehicle_id: VehicleId, torque: Vec3) {
        if let Some(handle) = self.module.backend().get_body_by_vehicle(vehicle_id) {
            self.module.backend_mut().apply_torque(handle, torque, "torque");
        }
    }

    pub fn apply_motor_forces(&mut self, vehicle_id: VehicleId, motor_forces: &[f64], motor_positions: &[Vec3]) {
        if let Some(handle) = self.module.backend().get_body_by_vehicle(vehicle_id) {
            for (force, pos) in motor_forces.iter().zip(motor_positions.iter()) {
                if *force > 0.0 {
                    self.module.backend_mut().apply_force_at_point(
                        handle, 
                        Vec3::new(0.0, *force, 0.0), 
                        *pos, 
                        "motor"
                    );
                }
            }
        }
    }

    pub fn get_vehicle_state(&self, vehicle_id: VehicleId) -> Option<crate::rapier_backend::PhysicsState> {
        if let Some(handle) = self.module.backend().get_body_by_vehicle(vehicle_id) {
            self.module.backend().get_state(handle)
        } else {
            None
        }
    }

    pub fn set_vehicle_pose(&mut self, vehicle_id: VehicleId, pose: autonomy_common::frames::Pose) {
        if let Some(handle) = self.module.backend().get_body_by_vehicle(vehicle_id) {
            self.module.backend_mut().set_pose(handle, pose);
        }
    }

    pub fn set_vehicle_velocity(&mut self, vehicle_id: VehicleId, linear: Vec3, angular: Vec3) {
        if let Some(handle) = self.module.backend().get_body_by_vehicle(vehicle_id) {
            self.module.backend_mut().set_velocity(handle, linear, angular);
        }
    }

    pub fn ray_cast(&self, origin: Vec3, direction: Vec3, max_distance: f64) -> Option<(BodyHandle, f64)> {
        self.module.backend().ray_cast(origin, direction, max_distance)
    }

    pub fn collision_events(&self) -> &[crate::collision::CollisionEvent] {
        self.module.backend().collision_events()
    }

    pub fn step_count(&self) -> u64 {
        self.module.backend().step_count()
    }
}

impl SimModule for PhysicsSimulation {
    fn name(&self) -> &'static str {
        "physics_simulation"
    }

    fn step(&mut self, _time: SimTime, dt: f64) -> autonomy_common::error::Result<()> {
        self.module.step(_time, dt)?;
        Ok(())
    }
}

/// Vehicle type presets
pub fn default_uav_config(mass: f64, arm_length: f64) -> VehiclePhysicsConfig {
    VehiclePhysicsConfig {
        body_config: uav_body_config(mass, arm_length),
        collider_config: uav_collider_config(arm_length, 0.1),
    }
}

pub fn default_ugv_config(mass: f64, length: f64, width: f64, height: f64) -> VehiclePhysicsConfig {
    VehiclePhysicsConfig {
        body_config: ugv_body_config(mass, length, width),
        collider_config: ugv_collider_config(length, width, height),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_physics_simulation() {
        let mut sim = PhysicsSimulation::new(PhysicsSimulationConfig::default());
        
        let pose = autonomy_common::frames::Pose::from_translation(Vec3::new(0.0, 10.0, 0.0));
        sim.add_uav(VehicleId::new(), 1.0, 0.3, pose);
        
        // Let it fall
        for _ in 0..60 {
            sim.module.step(SimTime::ZERO, 1.0 / 60.0).unwrap();
        }
        
        // Check it fell
        let vehicles: Vec<_> = sim.module.backend().vehicle_to_body.keys().copied().collect();
        if let Some(vid) = vehicles.first() {
            let state = sim.get_vehicle_state(*vid).unwrap();
            assert!(state.pose.position.y < 10.0);
        }
    }
}
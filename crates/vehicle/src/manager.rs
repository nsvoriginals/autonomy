//! Vehicle manager for coordinating multiple vehicles

use autonomy_common::error::Result;
use autonomy_common::frames::{LinearAcceleration, LinearVelocity, Pose, Vec3};
use autonomy_common::ids::{VehicleId, VehicleType};
use autonomy_common::state::{AlgorithmInput, AlgorithmOutput, BatteryState, ControlInputs, EstimatedState, HealthStatus, MissionState, SensorMeasurements, VehicleState};
use autonomy_common::time::SimTime;
use autonomy_common::traits::{ActuatorSet, AutonomyModule, Controller, SensorSuite, SimModule, StateEstimator, Vehicle};
use crate::registry::{VehicleRegistry, VehicleMetadata, VehicleStatus, CommunicationState};
use std::collections::HashMap;
use std::sync::Arc;

/// Managed vehicle wrapper
pub struct ManagedVehicle {
    vehicle: Arc<dyn Vehicle>,
    metadata: VehicleMetadata,
}

impl ManagedVehicle {
    pub fn new(vehicle: Arc<dyn Vehicle>, metadata: VehicleMetadata) -> Self {
        Self { vehicle, metadata }
    }

    pub fn vehicle(&self) -> &Arc<dyn Vehicle> {
        &self.vehicle
    }

    pub fn metadata(&self) -> &VehicleMetadata {
        &self.metadata
    }

    pub fn metadata_mut(&mut self) -> &mut VehicleMetadata {
        &mut self.metadata
    }
}

/// Vehicle manager
pub struct VehicleManager {
    registry: Arc<VehicleRegistry>,
    vehicles: HashMap<VehicleId, ManagedVehicle>,
    update_order: Vec<VehicleId>,
}

impl VehicleManager {
    pub fn new(registry: Arc<VehicleRegistry>) -> Self {
        Self {
            registry,
            vehicles: HashMap::new(),
            update_order: Vec::new(),
        }
    }

    pub fn add_vehicle(&mut self, vehicle: Arc<dyn Vehicle>, metadata: VehicleMetadata) -> Result<()> {
        let id = vehicle.id();
        self.registry.register(metadata.clone())?;
        self.vehicles.insert(id, ManagedVehicle::new(vehicle, metadata));
        self.update_order.push(id);
        Ok(())
    }

    pub fn remove_vehicle(&mut self, id: VehicleId) -> Option<ManagedVehicle> {
        self.registry.unregister(id).ok()?;
        self.update_order.retain(|&v| v != id);
        self.vehicles.remove(&id)
    }

    pub fn get_vehicle(&self, id: VehicleId) -> Option<&ManagedVehicle> {
        self.vehicles.get(&id)
    }

    pub fn get_vehicle_mut(&mut self, id: VehicleId) -> Option<&mut ManagedVehicle> {
        self.vehicles.get_mut(&id)
    }

    pub fn all_vehicles(&self) -> Vec<&ManagedVehicle> {
        self.vehicles.values().collect()
    }

    pub fn vehicles_by_type(&self, vtype: VehicleType) -> Vec<&ManagedVehicle> {
        self.vehicles.values().filter(|v| v.vehicle.vehicle_type() == vtype).collect()
    }

    pub fn vehicles_by_status(&self, status: VehicleStatus) -> Vec<&ManagedVehicle> {
        self.vehicles.values().filter(|v| v.metadata.status == status).collect()
    }

    pub fn update_all(&mut self, time: SimTime, dt: f64) -> Result<()> {
        // Update in deterministic order
        for id in self.update_order.clone() {
            if let Some(managed) = self.vehicles.get_mut(&id) {
                let estimation = managed.vehicle.estimator().get_estimate().clone();
                managed.vehicle.step(dt, &estimation)?;
                
                // Update registry
                let state = managed.vehicle.state_read();
                self.registry.update_position(id, state.pose.position)?;
                self.registry.update_battery(id, state.battery)?;
                self.registry.update_health(id, state.health)?;
                self.registry.heartbeat(id, time)?;
            }
        }
        Ok(())
    }

    pub fn step_vehicle(&mut self, id: VehicleId, dt: f64, estimation: &EstimatedState) -> Result<()> {
        if let Some(managed) = self.vehicles.get_mut(&id) {
            managed.vehicle.step(dt, estimation)?;
            
            let state = managed.vehicle.state_read();
            self.registry.update_position(id, state.pose.position)?;
            self.registry.update_battery(id, state.battery)?;
            self.registry.update_health(id, state.health)?;
        }
        Ok(())
    }

    pub fn count(&self) -> usize {
        self.vehicles.len()
    }

    pub fn uav_count(&self) -> usize {
        self.vehicles.values().filter(|v| v.vehicle.vehicle_type() == VehicleType::Uav).count()
    }

    pub fn ugv_count(&self) -> usize {
        self.vehicles.values().filter(|v| v.vehicle.vehicle_type() == VehicleType::Ugv).count()
    }
}

/// Vehicle manager module for simulation
pub struct VehicleManagerModule {
    manager: VehicleManager,
}

impl VehicleManagerModule {
    pub fn new(registry: Arc<VehicleRegistry>) -> Self {
        Self {
            manager: VehicleManager::new(registry),
        }
    }

    pub fn manager(&self) -> &VehicleManager {
        &self.manager
    }

    pub fn manager_mut(&mut self) -> &mut VehicleManager {
        &mut self.manager
    }
}

impl SimModule for VehicleManagerModule {
    fn name(&self) -> &'static str {
        "vehicle_manager"
    }

    fn step(&mut self, time: SimTime, dt: f64) -> Result<()> {
        self.manager.update_all(time, dt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::registry::VehicleRegistry;

    #[test]
    fn test_vehicle_manager() {
        let registry = Arc::new(VehicleRegistry::new());
        let mut manager = VehicleManager::new(registry.clone());
        
        // Would need a concrete vehicle implementation to test fully
        assert_eq!(manager.count(), 0);
    }
}
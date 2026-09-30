//! Fleet coordinator

use autonomy_common::error::Result;
use autonomy_common::frames::Vec3;
use autonomy_common::ids::{MissionId, TaskId, VehicleId, VehicleType};
use autonomy_common::state::{BatteryState, HealthStatus, MissionExecutionState, MissionState};
use autonomy_common::time::SimTime;
use autonomy_common::traits::SimModule;
use crate::task_allocator::{TaskAllocator, AllocationStrategy};
use crate::mission_manager::MissionManager;
use dashmap::DashMap;
use parking_lot::RwLock;
use std::collections::HashMap;
use std::sync::Arc;

/// Fleet coordinator - main fleet management entity
pub struct FleetCoordinator {
    vehicle_registry: Arc<crate::vehicle::registry::VehicleRegistry>,
    mission_manager: Arc<RwLock<MissionManager>>,
    task_allocator: Arc<RwLock<TaskAllocator>>,
    vehicle_positions: DashMap<VehicleId, Vec3>,
    vehicle_missions: DashMap<VehicleId, MissionId>,
    vehicle_tasks: DashMap<VehicleId, TaskId>,
    formation_assignments: DashMap<MissionId, FormationAssignment>,
    strategy: AllocationStrategy,
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct FormationAssignment {
    pub mission_id: MissionId,
    pub leader: VehicleId,
    pub followers: Vec<VehicleId>,
    pub formation_type: FormationType,
    pub spacing: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FormationType {
    Line,
    Vee,
    Echelon,
    Diamond,
    Circle,
    Custom,
}

impl FleetCoordinator {
    pub fn new(registry: Arc<crate::vehicle::registry::VehicleRegistry>) -> Self {
        Self {
            vehicle_registry: registry.clone(),
            mission_manager: Arc::new(RwLock::new(MissionManager::new())),
            task_allocator: Arc::new(RwLock::new(TaskAllocator::new(AllocationStrategy::Greedy))),
            vehicle_positions: DashMap::new(),
            vehicle_missions: DashMap::new(),
            vehicle_tasks: DashMap::new(),
            formation_assignments: DashMap::new(),
            strategy: AllocationStrategy::Greedy,
        }
    }

    pub fn register_vehicle(&self, vehicle_id: VehicleId, position: Vec3) -> Result<()> {
        self.vehicle_positions.insert(vehicle_id, position);
        Ok(())
    }

    pub fn unregister_vehicle(&self, vehicle_id: VehicleId) -> Result<()> {
        self.vehicle_positions.remove(&vehicle_id);
        self.vehicle_missions.remove(&vehicle_id);
        self.vehicle_tasks.remove(&vehicle_id);
        Ok(())
    }

    pub fn update_position(&self, vehicle_id: VehicleId, position: Vec3) {
        self.vehicle_positions.insert(vehicle_id, position);
    }

    pub fn assign_mission(&self, mission_id: MissionId, vehicle_ids: Vec<VehicleId>) -> Result<()> {
        let mut mm = self.mission_manager.write();
        mm.assign_mission(mission_id, vehicle_ids.clone())?;
        
        for vid in vehicle_ids {
            self.vehicle_missions.insert(vid, mission_id);
        }
        Ok(())
    }

    pub fn create_formation(&self, mission_id: MissionId, leader: VehicleId, followers: Vec<VehicleId>, formation: FormationType, spacing: f64) {
        self.formation_assignments.insert(mission_id, FormationAssignment {
            mission_id,
            leader,
            followers,
            formation_type: formation,
            spacing,
        });
    }

    pub fn get_formation(&self, mission_id: MissionId) -> Option<FormationAssignment> {
        self.formation_assignments.get(&mission_id).map(|v| v.clone())
    }

    pub fn get_vehicle_mission(&self, vehicle_id: VehicleId) -> Option<MissionId> {
        self.vehicle_missions.get(&vehicle_id).map(|v| *v)
    }

    pub fn get_vehicle_position(&self, vehicle_id: VehicleId) -> Option<Vec3> {
        self.vehicle_positions.get(&vehicle_id).map(|v| *v)
    }

    pub fn get_all_positions(&self) -> HashMap<VehicleId, Vec3> {
        self.vehicle_positions.iter().map(|v| (*v.key(), *v.value())).collect()
    }

    pub fn get_available_vehicles(&self, vtype: Option<VehicleType>) -> Vec<VehicleId> {
        self.vehicle_registry.get_ready_vehicles(vtype)
            .into_iter()
            .map(|v| v.id)
            .collect()
    }

    pub fn set_allocation_strategy(&mut self, strategy: AllocationStrategy) {
        self.strategy = strategy;
        self.task_allocator.write().set_strategy(strategy);
    }
}

impl SimModule for FleetCoordinator {
    fn name(&self) -> &'static str {
        "fleet_coordinator"
    }

    fn step(&mut self, time: SimTime, dt: f64) -> Result<()> {
        // Update mission states
        self.mission_manager.write().update(time, dt)?;
        
        // Check for failed vehicles and reallocate tasks
        let failed_vehicles: Vec<VehicleId> = self.vehicle_registry.all()
            .into_iter()
            .filter(|v| v.health == HealthStatus::Failed || v.communication_state == crate::vehicle::registry::CommunicationState::Disconnected)
            .map(|v| v.id)
            .collect();
        
        for vid in failed_vehicles {
            if let Some(task_id) = self.vehicle_tasks.get(&vid).map(|v| *v) {
                self.task_allocator.write().reallocate_task(task_id, &self.get_available_vehicles(None))?;
                self.vehicle_tasks.remove(&vid);
            }
        }
        
        Ok(())
    }
}
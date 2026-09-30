//! Mission manager

use autonomy_common::error::Result;
use autonomy_common::ids::{MissionId, TaskId, VehicleId};
use autonomy_common::state::{MissionExecutionState, MissionState};
use autonomy_common::time::SimTime;
use dashmap::DashMap;
use parking_lot::RwLock;
use std::collections::HashMap;
use std::sync::Arc;

/// Mission manager
pub struct MissionManager {
    missions: DashMap<MissionId, MissionInfo>,
    vehicle_missions: DashMap<VehicleId, MissionId>,
}

#[derive(Clone, Debug)]
pub struct MissionInfo {
    pub id: MissionId,
    pub state: MissionExecutionState,
    pub tasks: Vec<TaskId>,
    pub assigned_vehicles: Vec<VehicleId>,
    pub start_time: Option<SimTime>,
    pub end_time: Option<SimTime>,
    pub parameters: HashMap<String, serde_json::Value>,
}

impl MissionManager {
    pub fn new() -> Self {
        Self {
            missions: DashMap::new(),
            vehicle_missions: DashMap::new(),
        }
    }

    pub fn create_mission(&self, id: MissionId, tasks: Vec<TaskId>, parameters: HashMap<String, serde_json::Value>) -> MissionInfo {
        let info = MissionInfo {
            id,
            state: MissionExecutionState::Planned,
            tasks,
            assigned_vehicles: Vec::new(),
            start_time: None,
            end_time: None,
            parameters,
        };
        self.missions.insert(id, info.clone());
        info
    }

    pub fn assign_mission(&self, mission_id: MissionId, vehicles: Vec<VehicleId>) -> Result<()> {
        let mut mission = self.missions.get_mut(&mission_id).ok_or_else(|| autonomy_common::error::AutonomyError::Fleet("Mission not found".into()))?;
        
        for vid in &vehicles {
            self.vehicle_missions.insert(*vid, mission_id);
        }
        
        mission.assigned_vehicles = vehicles;
        mission.state = MissionExecutionState::Assigned;
        
        Ok(())
    }

    pub fn start_mission(&self, mission_id: MissionId, time: SimTime) -> Result<()> {
        let mut mission = self.missions.get_mut(&mission_id).ok_or_else(|| autonomy_common::error::AutonomyError::Fleet("Mission not found".into()))?;
        mission.state = MissionExecutionState::Executing;
        mission.start_time = Some(time);
        Ok(())
    }

    pub fn pause_mission(&self, mission_id: MissionId) -> Result<()> {
        let mut mission = self.missions.get_mut(&mission_id).ok_or_else(|| autonomy_common::error::AutonomyError::Fleet("Mission not found".into()))?;
        mission.state = MissionExecutionState::Paused;
        Ok(())
    }

    pub fn resume_mission(&self, mission_id: MissionId) -> Result<()> {
        let mut mission = self.missions.get_mut(&mission_id).ok_or_else(|| autonomy_common::error::AutonomyError::Fleet("Mission not found".into()))?;
        mission.state = MissionExecutionState::Executing;
        Ok(())
    }

    pub fn complete_mission(&self, mission_id: MissionId, time: SimTime) -> Result<()> {
        let mut mission = self.missions.get_mut(&mission_id).ok_or_else(|| autonomy_common::error::AutonomyError::Fleet("Mission not found".into()))?;
        mission.state = MissionExecutionState::Completed;
        mission.end_time = Some(time);
        
        // Clear vehicle assignments
        for vid in &mission.assigned_vehicles {
            self.vehicle_missions.remove(vid);
        }
        
        Ok(())
    }

    pub fn fail_mission(&self, mission_id: MissionId, time: SimTime) -> Result<()> {
        let mut mission = self.missions.get_mut(&mission_id).ok_or_else(|| autonomy_common::error::AutonomyError::Fleet("Mission not found".into()))?;
        mission.state = MissionExecutionState::Failed;
        mission.end_time = Some(time);
        Ok(())
    }

    pub fn get_mission(&self, id: MissionId) -> Option<MissionInfo> {
        self.missions.get(&id).map(|v| v.clone())
    }

    pub fn get_vehicle_mission(&self, vehicle_id: VehicleId) -> Option<MissionId> {
        self.vehicle_missions.get(&vehicle_id).map(|v| *v)
    }

    pub fn get_missions_by_state(&self, state: MissionExecutionState) -> Vec<MissionInfo> {
        self.missions.iter()
            .filter(|m| m.state == state)
            .map(|m| m.clone())
            .collect()
    }

    pub fn update(&self, time: SimTime, dt: f64) -> Result<()> {
        // Check for mission timeouts, etc.
        Ok(())
    }
}

impl Default for MissionManager {
    fn default() -> Self {
        Self::new()
    }
}
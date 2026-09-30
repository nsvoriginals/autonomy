//! Vehicle registry for fleet management

use autonomy_common::frames::Vec3;
use autonomy_common::ids::{VehicleId, VehicleType};
use autonomy_common::state::{BatteryState, HealthStatus};
use autonomy_common::time::SimTime;
use dashmap::DashMap;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;

/// Vehicle metadata in the registry
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VehicleMetadata {
    pub id: VehicleId,
    pub vehicle_type: VehicleType,
    pub callsign: String,
    pub capabilities: VehicleCapabilities,
    pub status: VehicleStatus,
    pub position: Vec3,
    pub battery: BatteryState,
    pub health: HealthStatus,
    pub last_heartbeat: SimTime,
    pub communication_state: CommunicationState,
    pub registered_at: SimTime,
    pub metadata: HashMap<String, serde_json::Value>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct VehicleCapabilities {
    pub can_fly: bool,
    pub can_drive: bool,
    pub max_altitude: f64,
    pub max_speed: f64,
    pub endurance_seconds: f64,
    pub payload_capacity_kg: f64,
    pub sensor_types: Vec<autonomy_common::ids::SensorType>,
    pub communication_range: f64,
    pub autonomous: bool,
    pub can_charge_others: bool,
    pub can_relay_comms: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum VehicleStatus {
    Unregistered,
    Registered,
    Initializing,
    Ready,
    Active,
    Mission,
    Returning,
    Landing,
    Charging,
    Maintenance,
    Emergency,
    Lost,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CommunicationState {
    Connected,
    Degraded,
    Disconnected,
    Unknown,
}

/// Vehicle registry
pub struct VehicleRegistry {
    vehicles: DashMap<VehicleId, VehicleMetadata>,
    by_type: DashMap<VehicleType, Vec<VehicleId>>,
    by_status: DashMap<VehicleStatus, Vec<VehicleId>>,
}

impl VehicleRegistry {
    pub fn new() -> Self {
        Self {
            vehicles: DashMap::new(),
            by_type: DashMap::new(),
            by_status: DashMap::new(),
        }
    }

    pub fn register(&self, metadata: VehicleMetadata) -> Result<(), RegistryError> {
        let id = metadata.id;
        let vtype = metadata.vehicle_type;
        let status = metadata.status;
        
        if self.vehicles.contains_key(&id) {
            return Err(RegistryError::AlreadyRegistered(id));
        }
        
        self.vehicles.insert(id, metadata);
        self.by_type.entry(vtype).or_default().push(id);
        self.by_status.entry(status).or_default().push(id);
        
        Ok(())
    }

    pub fn unregister(&self, id: VehicleId) -> Result<VehicleMetadata, RegistryError> {
        let metadata = self.vehicles.remove(&id).ok_or(RegistryError::NotFound(id))?.1;
        
        if let Some(mut vec) = self.by_type.get_mut(&metadata.vehicle_type) {
            vec.retain(|&v| v != id);
        }
        if let Some(mut vec) = self.by_status.get_mut(&metadata.status) {
            vec.retain(|&v| v != id);
        }
        
        Ok(metadata)
    }

    pub fn update_status(&self, id: VehicleId, status: VehicleStatus) -> Result<(), RegistryError> {
        let mut metadata = self.vehicles.get_mut(&id).ok_or(RegistryError::NotFound(id))?;
        
        let old_status = metadata.status;
        if old_status != status {
            if let Some(mut vec) = self.by_status.get_mut(&old_status) {
                vec.retain(|&v| v != id);
            }
            self.by_status.entry(status).or_default().push(id);
            metadata.status = status;
        }
        
        Ok(())
    }

    pub fn update_position(&self, id: VehicleId, position: Vec3) -> Result<(), RegistryError> {
        let mut metadata = self.vehicles.get_mut(&id).ok_or(RegistryError::NotFound(id))?;
        metadata.position = position;
        Ok(())
    }

    pub fn update_battery(&self, id: VehicleId, battery: BatteryState) -> Result<(), RegistryError> {
        let mut metadata = self.vehicles.get_mut(&id).ok_or(RegistryError::NotFound(id))?;
        metadata.battery = battery;
        Ok(())
    }

    pub fn update_health(&self, id: VehicleId, health: HealthStatus) -> Result<(), RegistryError> {
        let mut metadata = self.vehicles.get_mut(&id).ok_or(RegistryError::NotFound(id))?;
        metadata.health = health;
        Ok(())
    }

    pub fn heartbeat(&self, id: VehicleId, time: SimTime) -> Result<(), RegistryError> {
        let mut metadata = self.vehicles.get_mut(&id).ok_or(RegistryError::NotFound(id))?;
        metadata.last_heartbeat = time;
        Ok(())
    }

    pub fn update_communication(&self, id: VehicleId, state: CommunicationState) -> Result<(), RegistryError> {
        let mut metadata = self.vehicles.get_mut(&id).ok_or(RegistryError::NotFound(id))?;
        metadata.communication_state = state;
        Ok(())
    }

    pub fn get(&self, id: VehicleId) -> Option<VehicleMetadata> {
        self.vehicles.get(&id).map(|v| v.clone())
    }

    pub fn get_by_type(&self, vtype: VehicleType) -> Vec<VehicleMetadata> {
        self.by_type.get(&vtype)
            .map(|ids| ids.iter().filter_map(|id| self.vehicles.get(id).map(|v| v.clone())).collect())
            .unwrap_or_default()
    }

    pub fn get_by_status(&self, status: VehicleStatus) -> Vec<VehicleMetadata> {
        self.by_status.get(&status)
            .map(|ids| ids.iter().filter_map(|id| self.vehicles.get(id).map(|v| v.clone())).collect())
            .unwrap_or_default()
    }

    pub fn all(&self) -> Vec<VehicleMetadata> {
        self.vehicles.iter().map(|v| v.clone()).collect()
    }

    pub fn count(&self) -> usize {
        self.vehicles.len()
    }

    pub fn count_by_type(&self, vtype: VehicleType) -> usize {
        self.by_type.get(&vtype).map(|v| v.len()).unwrap_or(0)
    }

    pub fn count_by_status(&self, status: VehicleStatus) -> usize {
        self.by_status.get(&status).map(|v| v.len()).unwrap_or(0)
    }

    pub fn get_ready_vehicles(&self, vtype: Option<VehicleType>) -> Vec<VehicleMetadata> {
        let mut result = Vec::new();
        
        for entry in self.vehicles.iter() {
            let meta = entry.value();
            if meta.status == VehicleStatus::Ready || meta.status == VehicleStatus::Active {
                if vtype.map_or(true, |t| t == meta.vehicle_type) {
                    result.push(meta.clone());
                }
            }
        }
        
        result
    }

    pub fn get_vehicles_needing_charge(&self, threshold: f64) -> Vec<VehicleMetadata> {
        self.vehicles.iter()
            .filter_map(|v| {
                let meta = v.value();
                if meta.battery.charge_remaining < threshold {
                    Some(meta.clone())
                } else {
                    None
                }
            })
            .collect()
    }

    pub fn get_lost_vehicles(&self, timeout: SimTime) -> Vec<VehicleMetadata> {
        let now = SimTime::ZERO; // Would be current sim time
        self.vehicles.iter()
            .filter_map(|v| {
                let meta = v.value();
                if meta.last_heartbeat < now - timeout && meta.communication_state != CommunicationState::Connected {
                    Some(meta.clone())
                } else {
                    None
                }
            })
            .collect()
    }
}

impl Default for VehicleRegistry {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, thiserror::Error)]
pub enum RegistryError {
    #[error("Vehicle {0} already registered")]
    AlreadyRegistered(VehicleId),
    #[error("Vehicle {0} not found")]
    NotFound(VehicleId),
    #[error("Invalid vehicle type")]
    InvalidType,
}

/// Vehicle registry module for simulation
pub struct RegistryModule {
    registry: Arc<VehicleRegistry>,
}

impl RegistryModule {
    pub fn new() -> Self {
        Self {
            registry: Arc::new(VehicleRegistry::new()),
        }
    }

    pub fn registry(&self) -> Arc<VehicleRegistry> {
        self.registry.clone()
    }
}

impl autonomy_common::traits::SimModule for RegistryModule {
    fn name(&self) -> &'static str {
        "registry"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_registry() {
        let reg = VehicleRegistry::new();
        let id = VehicleId::new();
        
        let meta = VehicleMetadata {
            id,
            vehicle_type: VehicleType::Uav,
            callsign: "UAV-001".into(),
            capabilities: VehicleCapabilities::default(),
            status: VehicleStatus::Registered,
            position: Vec3::zeros(),
            battery: BatteryState::default(),
            health: HealthStatus::Nominal,
            last_heartbeat: SimTime::ZERO,
            communication_state: CommunicationState::Connected,
            registered_at: SimTime::ZERO,
            metadata: HashMap::new(),
        };
        
        reg.register(meta).unwrap();
        assert_eq!(reg.count(), 1);
        
        let found = reg.get(id).unwrap();
        assert_eq!(found.callsign, "UAV-001");
        
        reg.update_status(id, VehicleStatus::Ready).unwrap();
        assert_eq!(reg.count_by_status(VehicleStatus::Ready), 1);
        
        let unreg = reg.unregister(id).unwrap();
        assert_eq!(unreg.id, id);
        assert_eq!(reg.count(), 0);
    }
}
//! Zone definitions for the environment

use autonomy_common::frames::Vec3;
use autonomy_common::ids::ZoneId;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Zone types in the environment
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ZoneType {
    LandingZone,
    ChargingZone,
    RestrictedZone,
    MissionArea,
    CommunicationNode,
    Obstacle,
    Building,
    Road,
    Water,
    Forest,
    Custom(String),
}

/// Zone definition
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Zone {
    pub id: ZoneId,
    pub zone_type: ZoneType,
    pub name: String,
    pub center: Vec3,
    pub radius: f64,
    pub height: f64,
    pub properties: HashMap<String, serde_json::Value>,
    pub active: bool,
}

impl Zone {
    pub fn new(zone_type: ZoneType, name: impl Into<String>, center: Vec3, radius: f64, height: f64) -> Self {
        Self {
            id: ZoneId::new(),
            zone_type,
            name: name.into(),
            center,
            radius,
            height,
            properties: HashMap::new(),
            active: true,
        }
    }

    pub fn contains(&self, position: Vec3) -> bool {
        if !self.active {
            return false;
        }
        let d = position - self.center;
        let horizontal_dist = (d.x * d.x + d.z * d.z).sqrt();
        horizontal_dist <= self.radius && d.y.abs() <= self.height * 0.5
    }

    pub fn distance_to(&self, position: Vec3) -> f64 {
        let d = position - self.center;
        let horizontal_dist = (d.x * d.x + d.z * d.z).sqrt();
        horizontal_dist - self.radius
    }

    pub fn with_property(mut self, key: impl Into<String>, value: serde_json::Value) -> Self {
        self.properties.insert(key.into(), value);
        self
    }

    pub fn with_id(mut self, id: ZoneId) -> Self {
        self.id = id;
        self
    }
}

/// Landing zone with additional properties
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LandingZone {
    pub zone: Zone,
    pub max_vehicle_size: f64,
    pub max_weight: f64,
    pub surface_type: SurfaceType,
    pub orientation: Option<f64>, // Preferred heading
    pub occupied_by: Option<autonomy_common::ids::VehicleId>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SurfaceType {
    Concrete,
    Asphalt,
    Grass,
    Dirt,
    Pad,
    Water,
}

impl LandingZone {
    pub fn new(center: Vec3, radius: f64) -> Self {
        let zone = Zone::new(ZoneType::LandingZone, "Landing Zone", center, radius, 5.0);
        Self {
            zone,
            max_vehicle_size: 2.0,
            max_weight: 100.0,
            surface_type: SurfaceType::Pad,
            orientation: None,
            occupied_by: None,
        }
    }

    pub fn can_land(&self, vehicle_size: f64, vehicle_weight: f64) -> bool {
        self.occupied_by.is_none() && vehicle_size <= self.max_vehicle_size && vehicle_weight <= self.max_weight
    }
}

/// Charging zone
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ChargingZone {
    pub zone: Zone,
    pub power_kw: f64,
    pub voltage: f64,
    pub connector_type: ConnectorType,
    pub occupied_by: Option<autonomy_common::ids::VehicleId>,
    pub queue: Vec<autonomy_common::ids::VehicleId>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConnectorType {
    Type1,
    Type2,
    CCS,
    CHAdeMO,
    Tesla,
    Wireless,
    Custom(u16),
}

impl ChargingZone {
    pub fn new(center: Vec3, radius: f64, power_kw: f64) -> Self {
        let zone = Zone::new(ZoneType::ChargingZone, "Charging Station", center, radius, 3.0);
        Self {
            zone,
            power_kw,
            voltage: 400.0,
            connector_type: ConnectorType::CCS,
            occupied_by: None,
            queue: Vec::new(),
        }
    }
}

/// Restricted zone (no-fly, no-drive)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RestrictedZone {
    pub zone: Zone,
    pub restriction_level: RestrictionLevel,
    pub applies_to: Vec<autonomy_common::ids::VehicleType>,
    pub time_restrictions: Option<TimeRestriction>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RestrictionLevel {
    Warning,      // Warn but allow
    Soft,         // Penalize in planning
    Hard,         // Block completely
    Emergency,    // Immediate divert
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TimeRestriction {
    pub start_hour: u8,
    pub end_hour: u8,
    pub days: Vec<u8>, // 0=Sunday, 1=Monday, etc.
}

impl RestrictedZone {
    pub fn new(center: Vec3, radius: f64, height: f64, level: RestrictionLevel) -> Self {
        let zone = Zone::new(ZoneType::RestrictedZone, "Restricted Zone", center, radius, height);
        Self {
            zone,
            restriction_level: level,
            applies_to: Vec::new(),
            time_restrictions: None,
        }
    }

    pub fn is_restricted(&self, vehicle_type: autonomy_common::ids::VehicleType, time: f64) -> bool {
        if !self.applies_to.is_empty() && !self.applies_to.contains(&vehicle_type) {
            return false;
        }

        if let Some(tr) = &self.time_restrictions {
            // Simplified time check - would need proper time of day
            let hour = (time / 3600.0) as u8 % 24;
            if hour < tr.start_hour || hour >= tr.end_hour {
                return false;
            }
        }

        true
    }
}

/// Mission area
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MissionArea {
    pub zone: Zone,
    pub mission_type: String,
    pub priority: u8,
    pub required_capabilities: Vec<String>,
    pub completion_criteria: CompletionCriteria,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CompletionCriteria {
    Coverage { percentage: f64 },
    Waypoints { count: usize },
    Duration { seconds: f64 },
    Custom(String),
}

/// Communication node
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CommunicationNode {
    pub zone: Zone,
    pub node_type: CommNodeType,
    pub range: f64,
    pub bandwidth_mbps: f64,
    pub latency_ms: f64,
    pub connected_vehicles: Vec<autonomy_common::ids::VehicleId>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CommNodeType {
    BaseStation,
    Repeater,
    Satellite,
    MeshNode,
    Gcs, // Ground Control Station
}

/// Zone manager for the environment
pub struct ZoneManager {
    zones: HashMap<ZoneId, Zone>,
    landing_zones: HashMap<ZoneId, LandingZone>,
    charging_zones: HashMap<ZoneId, ChargingZone>,
    restricted_zones: HashMap<ZoneId, RestrictedZone>,
    mission_areas: HashMap<ZoneId, MissionArea>,
    comm_nodes: HashMap<ZoneId, CommunicationNode>,
}

impl ZoneManager {
    pub fn new() -> Self {
        Self {
            zones: HashMap::new(),
            landing_zones: HashMap::new(),
            charging_zones: HashMap::new(),
            restricted_zones: HashMap::new(),
            mission_areas: HashMap::new(),
            comm_nodes: HashMap::new(),
        }
    }

    pub fn add_zone(&mut self, zone: Zone) {
        self.zones.insert(zone.id, zone);
    }

    pub fn add_landing_zone(&mut self, zone: LandingZone) {
        self.zones.insert(zone.zone.id, zone.zone.clone());
        self.landing_zones.insert(zone.zone.id, zone);
    }

    pub fn add_charging_zone(&mut self, zone: ChargingZone) {
        self.zones.insert(zone.zone.id, zone.zone.clone());
        self.charging_zones.insert(zone.zone.id, zone);
    }

    pub fn add_restricted_zone(&mut self, zone: RestrictedZone) {
        self.zones.insert(zone.zone.id, zone.zone.clone());
        self.restricted_zones.insert(zone.zone.id, zone);
    }

    pub fn add_mission_area(&mut self, zone: MissionArea) {
        self.zones.insert(zone.zone.id, zone.zone.clone());
        self.mission_areas.insert(zone.zone.id, zone);
    }

    pub fn add_comm_node(&mut self, node: CommunicationNode) {
        self.zones.insert(node.zone.id, node.zone.clone());
        self.comm_nodes.insert(node.zone.id, node);
    }

    pub fn get_zone(&self, id: ZoneId) -> Option<&Zone> {
        self.zones.get(&id)
    }

    pub fn get_landing_zone(&self, id: ZoneId) -> Option<&LandingZone> {
        self.landing_zones.get(&id)
    }

    pub fn get_charging_zone(&self, id: ZoneId) -> Option<&ChargingZone> {
        self.charging_zones.get(&id)
    }

    pub fn get_restricted_zone(&self, id: ZoneId) -> Option<&RestrictedZone> {
        self.restricted_zones.get(&id)
    }

    pub fn get_mission_area(&self, id: ZoneId) -> Option<&MissionArea> {
        self.mission_areas.get(&id)
    }

    pub fn get_comm_node(&self, id: ZoneId) -> Option<&CommunicationNode> {
        self.comm_nodes.get(&id)
    }

    pub fn zones_of_type(&self, zone_type: ZoneType) -> Vec<&Zone> {
        self.zones.values().filter(|z| z.zone_type == zone_type).collect()
    }

    pub fn zones_at(&self, position: Vec3) -> Vec<&Zone> {
        self.zones.values().filter(|z| z.contains(position)).collect()
    }

    pub fn nearest_landing_zone(&self, position: Vec3, vehicle_size: f64, vehicle_weight: f64) -> Option<&LandingZone> {
        self.landing_zones.values()
            .filter(|z| z.can_land(vehicle_size, vehicle_weight))
            .min_by(|a, b| {
                let da = a.zone.distance_to(position);
                let db = b.zone.distance_to(position);
                da.partial_cmp(&db).unwrap()
            })
    }

    pub fn nearest_charging_zone(&self, position: Vec3) -> Option<&ChargingZone> {
        self.charging_zones.values()
            .filter(|z| z.zone.active && z.occupied_by.is_none())
            .min_by(|a, b| {
                let da = a.zone.distance_to(position);
                let db = b.zone.distance_to(position);
                da.partial_cmp(&db).unwrap()
            })
    }

    pub fn is_restricted(&self, position: Vec3, vehicle_type: autonomy_common::ids::VehicleType, time: f64) -> bool {
        self.restricted_zones.values()
            .any(|z| z.zone.contains(position) && z.is_restricted(vehicle_type, time))
    }

    pub fn iter_zones(&self) -> impl Iterator<Item = &Zone> {
        self.zones.values()
    }
}

impl Default for ZoneManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zone_contains() {
        let zone = Zone::new(ZoneType::LandingZone, "Test", Vec3::zeros(), 10.0, 5.0);
        assert!(zone.contains(Vec3::new(5.0, 0.0, 0.0)));
        assert!(zone.contains(Vec3::new(0.0, 2.0, 0.0)));
        assert!(!zone.contains(Vec3::new(15.0, 0.0, 0.0)));
        assert!(!zone.contains(Vec3::new(0.0, 10.0, 0.0)));
    }

    #[test]
    fn test_zone_manager() {
        let mut mgr = ZoneManager::new();
        let lz = LandingZone::new(Vec3::new(100.0, 0.0, 0.0), 5.0);
        mgr.add_landing_zone(lz.clone());

        let found = mgr.nearest_landing_zone(Vec3::new(90.0, 0.0, 0.0), 1.0, 10.0);
        assert!(found.is_some());
        assert_eq!(found.unwrap().zone.id, lz.zone.id);
    }
}
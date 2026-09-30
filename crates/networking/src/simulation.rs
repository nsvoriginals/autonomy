//! Network simulation module

use autonomy_common::error::Result;
use autonomy_common::frames::Vec3;
use autonomy_common::ids::VehicleId;
use autonomy_common::time::SimTime;
use autonomy_common::traits::SimModule;
use crate::topology::NetworkTopology;
use parking_lot::RwLock;
use std::sync::Arc;

/// Network simulation
pub struct NetworkSimulation {
    topology: Arc<RwLock<NetworkTopology>>,
    default_config: crate::link::LinkConfig,
    max_range: f64,
}

impl NetworkSimulation {
    pub fn new(rng: autonomy_common::rng::SimRng, max_range: f64) -> Self {
        Self {
            topology: Arc::new(RwLock::new(NetworkTopology::new(rng))),
            default_config: crate::link::LinkConfig::default(),
            max_range,
        }
    }

    pub fn topology(&self) -> Arc<RwLock<NetworkTopology>> {
        self.topology.clone()
    }

    pub fn add_vehicle(&self, vehicle_id: VehicleId, position: Vec3, node_type: crate::topology::NodeType) {
        let mut topo = self.topology.write();
        topo.add_node(crate::topology::NetworkNode {
            id: vehicle_id,
            position,
            node_type,
            max_range: self.max_range,
        });
    }

    pub fn remove_vehicle(&self, vehicle_id: VehicleId) {
        let mut topo = self.topology.write();
        topo.remove_node(vehicle_id);
    }

    pub fn update_position(&self, vehicle_id: VehicleId, position: Vec3) {
        let mut topo = self.topology.write();
        topo.update_position(vehicle_id, position);
    }

    pub fn send(&self, source: VehicleId, dest: VehicleId, data: Vec<u8>, time: SimTime) -> Result<()> {
        let topo = self.topology.read();
        if let Some(link) = topo.get_link(source, dest) {
            link.write().send(time, data)?;
        } else {
            // Try to create link dynamically
            drop(topo);
            let mut topo = self.topology.write();
            topo.create_link(source, dest, self.default_config.clone());
            topo.get_link(source, dest).unwrap().write().send(time, data)?;
        }
        Ok(())
    }

    pub fn receive(&self, dest: VehicleId, current_time: SimTime) -> HashMap<VehicleId, Vec<Vec<u8>>> {
        let topo = self.topology.read();
        let mut received = HashMap::new();
        
        for neighbor in topo.get_connected_nodes(dest) {
            if let Some(link) = topo.get_link(neighbor, dest) {
                let packets = link.write().receive(current_time);
                if !packets.is_empty() {
                    received.insert(neighbor, packets);
                }
            }
        }
        
        received
    }

    pub fn update_quality(&self) {
        self.topology.write().update_quality();
    }

    pub fn get_stats(&self) -> HashMap<(VehicleId, VehicleId), crate::link::LinkStats> {
        self.topology.read().get_stats()
    }
}

use std::collections::HashMap;

impl SimModule for NetworkSimulation {
    fn name(&self) -> &'static str {
        "network"
    }

    fn step(&mut self, _time: SimTime, dt: f64) -> Result<()> {
        self.update_quality();
        Ok(())
    }
}
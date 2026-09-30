//! Network topology management

use autonomy_common::frames::Vec3;
use autonomy_common::ids::{VehicleId, NetworkNodeId};
use autonomy_common::rng::SimRng;
use crate::link::{NetworkLink, LinkConfig, LinkStats};
use parking_lot::RwLock;
use petgraph::graph::{NodeIndex, Graph};
use std::collections::HashMap;
use std::sync::Arc;

/// Network topology
pub struct NetworkTopology {
    graph: Graph<NetworkNode, NetworkEdge>,
    node_indices: HashMap<VehicleId, NodeIndex>,
    links: HashMap<(VehicleId, VehicleId), Arc<RwLock<NetworkLink>>>,
    rng: RwLock<SimRng>,
}

#[derive(Clone, Debug)]
pub struct NetworkNode {
    pub id: VehicleId,
    pub position: Vec3,
    pub node_type: NodeType,
    pub max_range: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NodeType {
    Vehicle,
    BaseStation,
    Repeater,
    Satellite,
}

#[derive(Clone, Debug)]
pub struct NetworkEdge {
    pub link_id: (VehicleId, VehicleId),
    pub quality: f64,
}

impl NetworkTopology {
    pub fn new(rng: SimRng) -> Self {
        Self {
            graph: Graph::new(),
            node_indices: HashMap::new(),
            links: HashMap::new(),
            rng: RwLock::new(rng),
        }
    }

    pub fn add_node(&mut self, node: NetworkNode) {
        let idx = self.graph.add_node(node.clone());
        self.node_indices.insert(node.id, idx);
    }

    pub fn remove_node(&mut self, id: VehicleId) {
        if let Some(idx) = self.node_indices.remove(&id) {
            self.graph.remove_node(idx);
        }
    }

    pub fn update_position(&mut self, id: VehicleId, position: Vec3) {
        if let Some(&idx) = self.node_indices.get(&id) {
            if let Some(node) = self.graph.node_weight_mut(idx) {
                node.position = position;
            }
        }
    }

    pub fn create_link(&mut self, source: VehicleId, dest: VehicleId, config: LinkConfig) -> Arc<RwLock<NetworkLink>> {
        let mut rng = self.rng.write();
        let link = Arc::new(RwLock::new(NetworkLink::new(source, dest, config, autonomy_common::rng::derive_rng())));
        
        self.links.insert((source, dest), link.clone());
        
        // Add edge to graph
        if let (Some(&src_idx), Some(&dst_idx)) = (self.node_indices.get(&source), self.node_indices.get(&dest)) {
            self.graph.add_edge(src_idx, dst_idx, NetworkEdge {
                link_id: (source, dest),
                quality: 1.0,
            });
        }
        
        link
    }

    pub fn remove_link(&mut self, source: VehicleId, dest: VehicleId) {
        self.links.remove(&(source, dest));
        
        // Remove edges
        let edges_to_remove: Vec<_> = self.graph.edge_indices()
            .filter(|&e| {
                if let Some(edge) = self.graph.edge_weight(e) {
                    edge.link_id == (source, dest)
                } else {
                    false
                }
            })
            .collect();
        
        for e in edges_to_remove {
            self.graph.remove_edge(e);
        }
    }

    pub fn get_link(&self, source: VehicleId, dest: VehicleId) -> Option<Arc<RwLock<NetworkLink>>> {
        self.links.get(&(source, dest)).cloned()
    }

    pub fn get_all_links(&self) -> Vec<Arc<RwLock<NetworkLink>>> {
        self.links.values().cloned().collect()
    }

    pub fn update_quality(&mut self) {
        // Update edge qualities based on positions
        for edge_idx in self.graph.edge_indices() {
            if let Some((source, dest)) = self.graph.edge_endpoints(edge_idx) {
                if let (Some(src_node), Some(dst_node)) = (self.graph.node_weight(source), self.graph.node_weight(dest)) {
                    let quality = crate::link::estimate_link_quality(src_node.position, dst_node.position, src_node.max_range);
                    if let Some(edge) = self.graph.edge_weight_mut(edge_idx) {
                        edge.quality = quality;
                    }
                    
                    // Update link config if quality is very low
                    if quality < 0.1 {
                        if let Some(link) = self.links.get(&(src_node.id, dst_node.id)) {
                            let mut link = link.write();
                            link.update_config(crate::link::LinkConfig {
                                packet_loss_rate: 0.5,
                                latency_ms: 5000.0,
                                ..Default::default()
                            });
                        }
                    }
                }
            }
        }
    }

    pub fn get_connected_nodes(&self, id: VehicleId) -> Vec<VehicleId> {
        if let Some(&idx) = self.node_indices.get(&id) {
            self.graph.neighbors(idx)
                .filter_map(|n| self.graph.node_weight(n).map(|node| node.id))
                .collect()
        } else {
            Vec::new()
        }
    }

    pub fn find_path(&self, source: VehicleId, dest: VehicleId) -> Option<Vec<VehicleId>> {
        let src_idx = self.node_indices.get(&source)?;
        let dst_idx = self.node_indices.get(&dest)?;
        
        let result = petgraph::algo::astar(
            &self.graph,
            *src_idx,
            |n| n == *dst_idx,
            |e| 1.0 - e.weight().quality,
            |_| 0.0,
        );
        
        result.map(|(_, path)| {
            path.iter().filter_map(|idx| self.graph.node_weight(*idx).map(|n| n.id)).collect()
        })
    }

    pub fn get_stats(&self) -> HashMap<(VehicleId, VehicleId), LinkStats> {
        let mut stats = HashMap::new();
        for ((src, dst), link) in &self.links {
            stats.insert((*src, *dst), link.read().get_stats());
        }
        stats
    }
}
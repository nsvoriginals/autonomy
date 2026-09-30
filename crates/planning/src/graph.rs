//! Graph-based planning

use autonomy_common::frames::Vec3;
use autonomy_common::ids::{ZoneId, VehicleId};
use petgraph::graph::{NodeIndex, Graph};
use petgraph::algo::astar;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Navigation graph for global planning
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NavigationGraph {
    pub graph: Graph<NavNode, NavEdge>,
    pub node_positions: HashMap<NodeIndex, Vec3>,
    pub spatial_index: HashMap<ZoneId, NodeIndex>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NavNode {
    pub id: ZoneId,
    pub position: Vec3,
    pub node_type: NavNodeType,
    pub cost: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NavNodeType {
    Waypoint,
    LandingZone,
    ChargingZone,
    MissionArea,
    Obstacle,
    Restricted,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NavEdge {
    pub cost: f64,
    pub distance: f64,
    pub edge_type: NavEdgeType,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NavEdgeType {
    Direct,
    Corridor,
    Airway,
    Road,
}

impl NavigationGraph {
    pub fn new() -> Self {
        Self {
            graph: Graph::new(),
            node_positions: HashMap::new(),
            spatial_index: HashMap::new(),
        }
    }

    pub fn add_node(&mut self, node: NavNode) -> NodeIndex {
        let idx = self.graph.add_node(node.clone());
        self.node_positions.insert(idx, node.position);
        self.spatial_index.insert(node.id, idx);
        idx
    }

    pub fn add_edge(&mut self, from: ZoneId, to: ZoneId, edge: NavEdge) {
        if let (Some(&from_idx), Some(&to_idx)) = (self.spatial_index.get(&from), self.spatial_index.get(&to)) {
            self.graph.add_edge(from_idx, to_idx, edge);
        }
    }

    pub fn find_path(&self, start: ZoneId, goal: ZoneId) -> Option<Vec<ZoneId>> {
        let start_idx = self.spatial_index.get(&start)?;
        let goal_idx = self.spatial_index.get(&goal)?;
        
        let result = astar(
            &self.graph,
            *start_idx,
            |n| n == *goal_idx,
            |e| e.weight().cost,
            |n| {
                let pos = self.node_positions.get(&n)?;
                let goal_pos = self.node_positions.get(goal_idx)?;
                Some((pos - goal_pos).magnitude())
            },
        );
        
        result.map(|(_, path)| {
            path.iter().map(|idx| self.graph[*idx].id).collect()
        })
    }

    pub fn get_node_position(&self, id: ZoneId) -> Option<Vec3> {
        self.spatial_index.get(&id).and_then(|idx| self.node_positions.get(idx)).copied()
    }
}

impl Default for NavigationGraph {
    fn default() -> Self {
        Self::new()
    }
}

/// Graph builder from environment
pub struct GraphBuilder;

impl GraphBuilder {
    pub fn build_from_environment(
        zones: &autonomy_environment::ZoneManager,
        vehicle_type: autonomy_common::ids::VehicleType,
    ) -> NavigationGraph {
        let mut graph = NavigationGraph::new();
        
        // Add all non-restricted zones as nodes
        for zone in zones.iter_zones() {
            if zone.zone_type == autonomy_environment::zones::ZoneType::RestrictedZone {
                continue;
            }
            
            let node_type = match zone.zone_type {
                autonomy_environment::zones::ZoneType::LandingZone => NavNodeType::LandingZone,
                autonomy_environment::zones::ZoneType::ChargingZone => NavNodeType::ChargingZone,
                autonomy_environment::zones::ZoneType::MissionArea => NavNodeType::MissionArea,
                autonomy_environment::zones::ZoneType::Obstacle | 
                autonomy_environment::zones::ZoneType::Building => NavNodeType::Obstacle,
                _ => NavNodeType::Waypoint,
            };
            
            let node = NavNode {
                id: zone.id,
                position: zone.center,
                node_type,
                cost: 1.0,
            };
            
            graph.add_node(node);
        }
        
        // Connect nearby nodes (within communication range)
        let nodes: Vec<_> = graph.graph.node_indices().collect();
        for i in 0..nodes.len() {
            for j in i+1..nodes.len() {
                let pos_i = graph.node_positions[&nodes[i]];
                let pos_j = graph.node_positions[&nodes[j]];
                let dist = (pos_i - pos_j).magnitude();
                
                if dist < 500.0 { // Max connection distance
                    // Check line of sight (simplified)
                    let edge = NavEdge {
                        cost: dist,
                        distance: dist,
                        edge_type: NavEdgeType::Direct,
                    };
                    
                    let from_id = graph.graph[nodes[i]].id;
                    let to_id = graph.graph[nodes[j]].id;
                    graph.add_edge(from_id, to_id, edge);
                    graph.add_edge(to_id, from_id, edge);
                }
            }
        }
        
        graph
    }
}
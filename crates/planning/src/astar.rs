//! A* pathfinding

use autonomy_common::frames::Vec3;
use autonomy_common::ids::{ZoneId, VehicleId};
use petgraph::graph::{NodeIndex, Graph};
use petgraph::algo::astar;
use std::collections::HashMap;

/// A* planner for grid-based or graph-based planning
pub struct AStarPlanner {
    graph: Graph<NavNode, f64>,
    node_positions: HashMap<NodeIndex, Vec3>,
    zone_to_node: HashMap<ZoneId, NodeIndex>,
}

#[derive(Clone, Debug)]
struct NavNode {
    id: ZoneId,
    position: Vec3,
    cost: f64,
}

impl AStarPlanner {
    pub fn new() -> Self {
        Self {
            graph: Graph::new(),
            node_positions: HashMap::new(),
            zone_to_node: HashMap::new(),
        }
    }

    pub fn build_grid(&mut self, bounds: (Vec3, Vec3), resolution: f64, obstacles: &[Vec3]) {
        let (min, max) = bounds;
        let width = ((max.x - min.x) / resolution).ceil() as usize;
        let height = ((max.z - min.z) / resolution).ceil() as usize;
        
        let mut nodes = HashMap::new();
        
        for y in 0..height {
            for x in 0..width {
                let pos = Vec3::new(
                    min.x + x as f64 * resolution,
                    min.y,
                    min.z + y as f64 * resolution,
                );
                
                // Check if in obstacle
                let mut blocked = false;
                for obs in obstacles {
                    if (pos - *obs).magnitude() < resolution * 1.5 {
                        blocked = true;
                        break;
                    }
                }
                
                if !blocked {
                    let node = NavNode {
                        id: ZoneId::new(),
                        position: pos,
                        cost: 1.0,
                    };
                    let idx = self.graph.add_node(node);
                    nodes.insert((x, y), idx);
                    self.node_positions.insert(idx, pos);
                }
            }
        }
        
        // Add edges
        for y in 0..height {
            for x in 0..width {
                if let Some(&idx) = nodes.get(&(x, y)) {
                    // 8-connected neighbors
                    for dx in -1i32..=1 {
                        for dy in -1i32..=1 {
                            if dx == 0 && dy == 0 { continue; }
                            
                            let nx = x as i32 + dx;
                            let ny = y as i32 + dy;
                            
                            if nx >= 0 && nx < width as i32 && ny >= 0 && ny < height as i32 {
                                if let Some(&nidx) = nodes.get(&(nx as usize, ny as usize)) {
                                    let dist = (self.node_positions[&idx] - self.node_positions[&nidx]).magnitude();
                                    self.graph.add_edge(idx, nidx, dist);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    pub fn plan(&self, start: Vec3, goal: Vec3) -> Option<Vec<Vec3>> {
        // Find nearest nodes
        let start_idx = self.nearest_node(start)?;
        let goal_idx = self.nearest_node(goal)?;
        
        let result = astar(
            &self.graph,
            start_idx,
            |n| n == goal_idx,
            |e| *e.weight(),
            |n| {
                let pos = self.node_positions.get(&n)?;
                Some((pos - self.node_positions[&goal_idx]).magnitude())
            },
        );
        
        result.map(|(_, path)| {
            path.iter().map(|idx| self.node_positions[idx]).collect()
        })
    }

    fn nearest_node(&self, pos: Vec3) -> Option<NodeIndex> {
        self.node_positions.iter()
            .min_by(|(_, a), (_, b)| {
                let da = (pos - **a).magnitude();
                let db = (pos - **b).magnitude();
                da.partial_cmp(&db).unwrap()
            })
            .map(|(idx, _)| *idx)
    }
}

impl Default for AStarPlanner {
    fn default() -> Self {
        Self::new()
    }
}
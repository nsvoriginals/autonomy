//! Planner trait and factory

use autonomy_common::error::Result;
use autonomy_common::frames::{Pose, Vec3};
use autonomy_common::state::{EstimatedState, MissionState, Trajectory};
use autonomy_common::time::SimTime;

/// Planner trait
pub trait Planner: Send + Sync {
    fn plan(&mut self, state: &EstimatedState, mission: &MissionState) -> Result<Trajectory>;
    fn replan(&mut self, state: &EstimatedState, reason: &str) -> Result<Trajectory>;
    fn get_current_plan(&self) -> Option<&Trajectory>;
    fn planner_type(&self) -> &'static str;
}

/// Planner types
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PlannerType {
    Waypoint,
    Graph,
    AStar,
    Hybrid,
}

/// Planner configuration
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct PlannerConfig {
    pub planner_type: PlannerType,
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub replan_interval: f64,
    pub lookahead_distance: f64,
}

impl Default for PlannerConfig {
    fn default() -> Self {
        Self {
            planner_type: PlannerType::Waypoint,
            max_velocity: 15.0,
            max_acceleration: 3.0,
            replan_interval: 5.0,
            lookahead_distance: 20.0,
        }
    }
}

/// Planner factory
pub fn create_planner(config: &PlannerConfig) -> Box<dyn Planner> {
    match config.planner_type {
        PlannerType::Waypoint => Box::new(crate::waypoint::WaypointPlanner::new()),
        PlannerType::Graph => Box::new(crate::graph::NavigationGraph::new()),
        PlannerType::AStar => Box::new(crate::astar::AStarPlanner::new()),
        PlannerType::Hybrid => Box::new(HybridPlanner::new(config)),
    }
}

/// Hybrid planner combining global and local planning
pub struct HybridPlanner {
    global: crate::waypoint::WaypointPlanner,
    local: crate::local::LocalPlanner,
    config: PlannerConfig,
    current_trajectory: Option<Trajectory>,
    last_replan: SimTime,
}

impl HybridPlanner {
    pub fn new(config: &PlannerConfig) -> Self {
        Self {
            global: crate::waypoint::WaypointPlanner::new(),
            local: crate::local::LocalPlanner::new(),
            config: config.clone(),
            current_trajectory: None,
            last_replan: SimTime::ZERO,
        }
    }
}

impl Planner for HybridPlanner {
    fn plan(&mut self, state: &EstimatedState, mission: &MissionState) -> Result<Trajectory> {
        // Global plan
        let global_traj = self.global.generate_trajectory(&state.pose, state.linear_velocity.0.magnitude());
        
        // Local refinement would go here
        self.current_trajectory = Some(global_traj.clone());
        self.last_replan = state.time;
        
        Ok(global_traj)
    }

    fn replan(&mut self, state: &EstimatedState, reason: &str) -> Result<Trajectory> {
        self.plan(state, &MissionState::default())
    }

    fn get_current_plan(&self) -> Option<&Trajectory> {
        self.current_trajectory.as_ref()
    }

    fn planner_type(&self) -> &'static str {
        "hybrid"
    }
}
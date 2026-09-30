//! Waypoint-based planning

use autonomy_common::frames::{Pose, Vec3, UnitQuaternion};
use autonomy_common::ids::{MissionId, TaskId, VehicleId};
use autonomy_common::state::{MissionState, MissionExecutionState, Trajectory, TrajectoryWaypoint};
use autonomy_common::time::SimTime;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

/// Waypoint mission planner
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WaypointPlanner {
    pub waypoints: VecDeque<Waypoint>,
    pub current_waypoint: Option<Waypoint>,
    pub lookahead_distance: f64,
    pub waypoint_radius: f64,
    pub max_velocity: f64,
    pub max_acceleration: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Waypoint {
    pub position: Vec3,
    pub velocity: f64,
    pub heading: Option<f64>,
    pub hold_time: f64,
    pub radius: f64,
    pub action: WaypointAction,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum WaypointAction {
    None,
    TakePhoto,
    Scan,
    Wait,
    Land,
    Takeoff,
    Charge,
}

impl WaypointPlanner {
    pub fn new() -> Self {
        Self {
            waypoints: VecDeque::new(),
            current_waypoint: None,
            lookahead_distance: 10.0,
            waypoint_radius: 5.0,
            max_velocity: 15.0,
            max_acceleration: 3.0,
        }
    }

    pub fn add_waypoint(&mut self, waypoint: Waypoint) {
        self.waypoints.push_back(waypoint);
    }

    pub fn set_waypoints(&mut self, waypoints: Vec<Waypoint>) {
        self.waypoints = waypoints.into();
    }

    pub fn clear(&mut self) {
        self.waypoints.clear();
        self.current_waypoint = None;
    }

    pub fn update(&mut self, pose: &Pose, velocity: f64) -> Option<WaypointAction> {
        if let Some(current) = self.current_waypoint.clone() {
            let distance = (pose.position - current.position).magnitude();
            
            if distance < current.radius.max(self.waypoint_radius) {
                // Waypoint reached
                let action = current.action;
                
                // Advance to next waypoint
                self.current_waypoint = self.waypoints.pop_front();
                return Some(action);
            }
        } else if let Some(next) = self.waypoints.pop_front() {
            self.current_waypoint = Some(next);
        }

        None
    }

    pub fn generate_trajectory(&self, current_pose: &Pose, current_vel: f64) -> Trajectory {
        let mut waypoints = Vec::new();
        let mut time_accum = 0.0;
        let mut prev_pos = current_pose.position;
        let mut prev_vel = current_vel;

        if let Some(current) = &self.current_waypoint {
            let dist = (current.position - prev_pos).magnitude();
            let time = if prev_vel > 0.1 { dist / prev_vel } else { dist / self.max_velocity };
            
            waypoints.push(TrajectoryWaypoint {
                position: current.position,
                velocity: (current.position - prev_pos).normalize() * current.velocity.min(self.max_velocity),
                acceleration: Vec3::zeros(),
                yaw: current.heading.unwrap_or_else(|| {
                    let dir = current.position - prev_pos;
                    dir.z.atan2(dir.x)
                }),
                yaw_rate: 0.0,
                time_from_start: time_accum + time,
            });
            
            time_accum += time;
            prev_pos = current.position;
            prev_vel = current.velocity;
        }

        for wp in &self.waypoints {
            let dist = (wp.position - prev_pos).magnitude();
            let time = dist / wp.velocity.min(self.max_velocity);
            
            waypoints.push(TrajectoryWaypoint {
                position: wp.position,
                velocity: (wp.position - prev_pos).normalize() * wp.velocity.min(self.max_velocity),
                acceleration: Vec3::zeros(),
                yaw: wp.heading.unwrap_or_else(|| {
                    let dir = wp.position - prev_pos;
                    dir.z.atan2(dir.x)
                }),
                yaw_rate: 0.0,
                time_from_start: time_accum + time,
            });
            
            time_accum += time;
            prev_pos = wp.position;
            prev_vel = wp.velocity;
        }

        Trajectory {
            waypoints,
            start_time: SimTime::ZERO,
            duration: time_accum,
        }
    }

    pub fn is_complete(&self) -> bool {
        self.waypoints.is_empty() && self.current_waypoint.is_none()
    }

    pub fn remaining_waypoints(&self) -> usize {
        self.waypoints.len() + if self.current_waypoint.is_some() { 1 } else { 0 }
    }
}

impl Default for WaypointPlanner {
    fn default() -> Self {
        Self::new()
    }
}

/// Mission planner that creates waypoint missions from high-level tasks
pub struct MissionPlanner {
    pub vehicle_id: VehicleId,
    pub mission_id: MissionId,
}

impl MissionPlanner {
    pub fn new(vehicle_id: VehicleId, mission_id: MissionId) -> Self {
        Self { vehicle_id, mission_id }
    }

    pub fn plan_area_coverage(&self, area: &autonomy_environment::zones::MissionArea, spacing: f64, altitude: f64) -> Vec<Waypoint> {
        let center = area.zone.center;
        let radius = area.zone.radius;
        
        // Generate lawnmower pattern
        let mut waypoints = Vec::new();
        let rows = ((2.0 * radius) / spacing).ceil() as i32;
        
        for i in 0..rows {
            let y = center.z - radius + i as f64 * spacing;
            let row_width = (radius * radius - (y - center.z) * (y - center.z)).sqrt();
            
            let x_start = center.x - row_width;
            let x_end = center.x + row_width;
            
            if i % 2 == 0 {
                waypoints.push(Waypoint {
                    position: Vec3::new(x_start, altitude, y),
                    velocity: 10.0,
                    heading: Some(0.0),
                    hold_time: 0.0,
                    radius: 5.0,
                    action: WaypointAction::Scan,
                });
                waypoints.push(Waypoint {
                    position: Vec3::new(x_end, altitude, y),
                    velocity: 10.0,
                    heading: Some(std::f64::consts::PI),
                    hold_time: 0.0,
                    radius: 5.0,
                    action: WaypointAction::Scan,
                });
            } else {
                waypoints.push(Waypoint {
                    position: Vec3::new(x_end, altitude, y),
                    velocity: 10.0,
                    heading: Some(std::f64::consts::PI),
                    hold_time: 0.0,
                    radius: 5.0,
                    action: WaypointAction::Scan,
                });
                waypoints.push(Waypoint {
                    position: Vec3::new(x_start, altitude, y),
                    velocity: 10.0,
                    heading: Some(0.0),
                    hold_time: 0.0,
                    radius: 5.0,
                    action: WaypointAction::Scan,
                });
            }
        }
        
        waypoints
    }

    pub fn plan_waypoint_mission(&self, waypoints: Vec<Vec3>, altitude: f64, velocity: f64) -> Vec<Waypoint> {
        waypoints.into_iter().map(|pos| Waypoint {
            position: Vec3::new(pos.x, altitude, pos.z),
            velocity,
            heading: None,
            hold_time: 0.0,
            radius: 5.0,
            action: WaypointAction::None,
        }).collect()
    }

    pub fn plan_return_to_base(&self, base_position: Vec3, altitude: f64) -> Vec<Waypoint> {
        vec![Waypoint {
            position: Vec3::new(base_position.x, altitude, base_position.z),
            velocity: 10.0,
            heading: None,
            hold_time: 0.0,
            radius: 10.0,
            action: WaypointAction::Land,
        }]
    }
}
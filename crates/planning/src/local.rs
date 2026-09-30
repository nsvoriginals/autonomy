//! Local planning and obstacle avoidance

use autonomy_common::frames::{Pose, Vec3};
use autonomy_common::state::TrajectoryWaypoint;
use autonomy_environment::World;

/// Local planner for obstacle avoidance
pub struct LocalPlanner {
    pub max_velocity: f64,
    pub max_acceleration: f64,
    pub safety_radius: f64,
    pub prediction_horizon: f64,
    pub dt: f64,
}

impl LocalPlanner {
    pub fn new() -> Self {
        Self {
            max_velocity: 15.0,
            max_acceleration: 3.0,
            safety_radius: 5.0,
            prediction_horizon: 3.0,
            dt: 0.1,
        }
    }

    pub fn plan(&self, current: &Pose, velocity: Vec3, goal: Vec3, world: &World) -> Vec<TrajectoryWaypoint> {
        // Simple potential field approach
        let to_goal = goal - current.position;
        let dist_to_goal = to_goal.magnitude();
        
        if dist_to_goal < 1.0 {
            return vec![];
        }

        let mut waypoints = Vec::new();
        let mut pos = current.position;
        let mut vel = velocity;
        let mut time = 0.0;

        for _ in 0..(self.prediction_horizon / self.dt) as usize {
            // Attractive force to goal
            let goal_force = to_goal.normalize() * self.max_acceleration;
            
            // Repulsive forces from obstacles
            let mut avoid_force = Vec3::zeros();
            
            // Check terrain
            let terrain_height = world.height_at(pos);
            if pos.y - terrain_height < self.safety_radius {
                avoid_force += Vec3::new(0.0, self.max_acceleration * 2.0, 0.0);
            }
            
            // Check zones
            for zone in world.zones().iter_zones() {
                if zone.zone_type == autonomy_environment::zones::ZoneType::Obstacle
                    || zone.zone_type == autonomy_environment::zones::ZoneType::Building
                    || zone.zone_type == autonomy_environment::zones::ZoneType::RestrictedZone {
                    
                    let diff = pos - zone.center;
                    let dist = diff.magnitude();
                    if dist < zone.radius + self.safety_radius && dist > 0.1 {
                        avoid_force += diff.normalize() * self.max_acceleration * (zone.radius + self.safety_radius - dist) / self.safety_radius;
                    }
                }
            }
            
            // Total force
            let total_force = goal_force + avoid_force;
            let accel = total_force.clamp_magnitude(self.max_acceleration);
            
            // Update velocity and position
            vel += accel * self.dt;
            let speed = vel.magnitude();
            if speed > self.max_velocity {
                vel = vel.normalize() * self.max_velocity;
            }
            
            pos += vel * self.dt;
            time += self.dt;
            
            waypoints.push(TrajectoryWaypoint {
                position: pos,
                velocity: vel,
                acceleration: accel,
                yaw: vel.z.atan2(vel.x),
                yaw_rate: 0.0,
                time_from_start: time,
            });
            
            if (pos - goal).magnitude() < 1.0 {
                break;
            }
        }

        waypoints
    }
}

impl Default for LocalPlanner {
    fn default() -> Self {
        Self::new()
    }
}
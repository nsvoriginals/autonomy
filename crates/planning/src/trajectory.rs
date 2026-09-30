//! Trajectory representation and utilities

use autonomy_common::frames::{Pose, Vec3};
use autonomy_common::state::{Trajectory, TrajectoryWaypoint};
use autonomy_common::time::SimTime;
use serde::{Deserialize, Serialize};

impl Trajectory {
    pub fn new(waypoints: Vec<TrajectoryWaypoint>) -> Self {
        let duration = waypoints.last().map(|w| w.time_from_start).unwrap_or(0.0);
        Self {
            waypoints,
            start_time: SimTime::ZERO,
            duration,
        }
    }

    pub fn interpolate(&self, time: f64) -> Option<(Vec3, Vec3, Vec3, f64)> {
        if self.waypoints.is_empty() {
            return None;
        }

        if time <= self.waypoints[0].time_from_start {
            let wp = &self.waypoints[0];
            return Some((wp.position, wp.velocity, wp.acceleration, wp.yaw));
        }

        if time >= self.duration {
            let wp = self.waypoints.last().unwrap();
            return Some((wp.position, wp.velocity, wp.acceleration, wp.yaw));
        }

        // Find segment
        for i in 0..self.waypoints.len() - 1 {
            let wp0 = &self.waypoints[i];
            let wp1 = &self.waypoints[i + 1];
            
            if time >= wp0.time_from_start && time <= wp1.time_from_start {
                let t = (time - wp0.time_from_start) / (wp1.time_from_start - wp0.time_from_start);
                let t = t.clamp(0.0, 1.0);
                
                let pos = wp0.position + (wp1.position - wp0.position) * t;
                let vel = wp0.velocity + (wp1.velocity - wp0.velocity) * t;
                let accel = wp0.acceleration + (wp1.acceleration - wp0.acceleration) * t;
                let yaw = wp0.yaw + (wp1.yaw - wp0.yaw) * t;
                
                return Some((pos, vel, accel, yaw));
            }
        }

        None
    }

    pub fn sample(&self, num_samples: usize) -> Vec<(Vec3, Vec3, f64)> {
        let mut samples = Vec::new();
        for i in 0..num_samples {
            let t = self.duration * (i as f64 / (num_samples - 1).max(1) as f64);
            if let Some((pos, vel, _, yaw)) = self.interpolate(t) {
                samples.push((pos, vel, yaw));
            }
        }
        samples
    }

    pub fn trim(&self, start_time: f64, end_time: f64) -> Trajectory {
        let waypoints: Vec<TrajectoryWaypoint> = self.waypoints.iter()
            .filter(|wp| wp.time_from_start >= start_time && wp.time_from_start <= end_time)
            .cloned()
            .collect();
        
        Trajectory::new(waypoints)
    }

    pub fn append(&mut self, other: Trajectory) {
        let time_offset = self.duration;
        for mut wp in other.waypoints {
            wp.time_from_start += time_offset;
            self.waypoints.push(wp);
        }
        self.duration += other.duration;
    }
}

/// Trajectory builder for easy construction
pub struct TrajectoryBuilder {
    waypoints: Vec<TrajectoryWaypoint>,
    current_time: f64,
    current_pos: Vec3,
    current_vel: Vec3,
}

impl TrajectoryBuilder {
    pub fn new(start_pos: Vec3, start_vel: Vec3) -> Self {
        Self {
            waypoints: Vec::new(),
            current_time: 0.0,
            current_pos: start_pos,
            current_vel: start_vel,
        }
    }

    pub fn add_waypoint(&mut self, pos: Vec3, vel: f64, accel: f64) -> &mut Self {
        let dist = (pos - self.current_pos).magnitude();
        let time = if self.current_vel.magnitude() > 0.1 {
            dist / self.current_vel.magnitude()
        } else {
            dist / vel.max(0.1)
        };
        
        self.current_time += time;
        self.current_pos = pos;
        self.current_vel = (pos - self.waypoints.last().map(|w| w.position).unwrap_or(self.current_pos)).normalize() * vel;
        
        self.waypoints.push(TrajectoryWaypoint {
            position: pos,
            velocity: self.current_vel,
            acceleration: Vec3::zeros(),
            yaw: self.current_vel.z.atan2(self.current_vel.x),
            yaw_rate: 0.0,
            time_from_start: self.current_time,
        });
        
        self
    }

    pub fn build(self) -> Trajectory {
        Trajectory::new(self.waypoints)
    }
}
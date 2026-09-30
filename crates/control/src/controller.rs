//! Controller trait and implementations

use autonomy_common::error::Result;
use autonomy_common::frames::{LinearVelocity, Pose, Vec3};
use autonomy_common::state::{ControlInputs, EstimatedState, Trajectory};
use autonomy_common::time::SimTime;

/// Controller trait
pub trait Controller: Send + Sync {
    fn compute_control(&mut self, state: &EstimatedState, trajectory: &Trajectory) -> Result<ControlInputs>;
    fn reset(&mut self);
    fn controller_type(&self) -> &'static str;
}

/// UAV position controller
pub struct UavPositionController {
    pub cascaded_pid: crate::pid::CascadedPidController,
    pub max_tilt: f64,
    pub max_velocity: f64,
    pub max_thrust: f64,
}

impl UavPositionController {
    pub fn new() -> Self {
        Self {
            cascaded_pid: crate::pid::CascadedPidController::new(),
            max_tilt: 0.5, // ~30 degrees
            max_velocity: 15.0,
            max_thrust: 1.5,
        }
    }
}

impl Controller for UavPositionController {
    fn compute_control(&mut self, state: &EstimatedState, trajectory: &Trajectory) -> Result<ControlInputs> {
        // Get desired state from trajectory
        let (desired_pos, desired_vel, _, desired_yaw) = trajectory.interpolate(
            (state.time - trajectory.start_time).as_seconds()
        ).unwrap_or((state.pose.position, state.linear_velocity.0, Vec3::zeros(), 0.0));

        // Position error
        let pos_error = desired_pos - state.pose.position;
        
        // Velocity error
        let vel_error = desired_vel - state.linear_velocity.0;
        
        // Attitude error (simplified)
        let (_, _, current_yaw) = state.pose.orientation.euler_angles();
        let yaw_error = autonomy_common::math::wrap_angle(desired_yaw - current_yaw);
        
        // Attitude error from desired acceleration
        let att_error = Vec3::zeros(); // Would compute from desired acceleration
        
        let dt = 1.0 / 60.0;
        
        let (torque, yaw_rate) = self.cascaded_pid.update(pos_error, vel_error, att_error, yaw_error, dt);
        
        // Convert to control inputs
        let collective_thrust = (torque.y + 9.80665).clamp(0.0, self.max_thrust) / self.max_thrust;
        let roll_torque = torque.x.clamp(-1.0, 1.0);
        let pitch_torque = torque.z.clamp(-1.0, 1.0);
        let yaw_torque = yaw_rate.clamp(-1.0, 1.0);
        
        Ok(ControlInputs {
            collective_thrust,
            roll_torque,
            pitch_torque,
            yaw_torque,
            throttle: 0.0,
            steering: 0.0,
            brake: 0.0,
        })
    }

    fn reset(&mut self) {
        self.cascaded_pid.reset();
    }

    fn controller_type(&self) -> &'static str {
        "uav_position"
    }
}

/// UGV controller
pub struct UgvController {
    pub speed_pid: crate::pid::PidController,
    pub steering_pid: crate::pid::PidController,
    pub max_speed: f64,
    pub max_steering: f64,
}

impl UgvController {
    pub fn new() -> Self {
        Self {
            speed_pid: crate::pid::PidController::new(1.0, 0.1, 0.05).with_limits(10.0, 1.0),
            steering_pid: crate::pid::PidController::new(2.0, 0.0, 0.2).with_limits(1.0, 1.0),
            max_speed: 10.0,
            max_steering: 0.5,
        }
    }
}

impl Controller for UgvController {
    fn compute_control(&mut self, state: &EstimatedState, trajectory: &Trajectory) -> Result<ControlInputs> {
        let (desired_pos, desired_vel, _, desired_yaw) = trajectory.interpolate(
            (state.time - trajectory.start_time).as_seconds()
        ).unwrap_or((state.pose.position, state.linear_velocity.0, Vec3::zeros(), 0.0));

        // Speed control
        let speed_error = desired_vel.magnitude() - state.linear_velocity.0.magnitude();
        let throttle = self.speed_pid.update(speed_error, 1.0 / 60.0).clamp(-1.0, 1.0);
        
        // Steering control
        let (_, _, current_yaw) = state.pose.orientation.euler_angles();
        let heading_error = autonomy_common::math::wrap_angle(desired_yaw - current_yaw);
        let steering = self.steering_pid.update(heading_error, 1.0 / 60.0).clamp(-1.0, 1.0);
        
        let brake = if throttle < 0.0 { -throttle } else { 0.0 };
        
        Ok(ControlInputs {
            collective_thrust: 0.0,
            roll_torque: 0.0,
            pitch_torque: 0.0,
            yaw_torque: 0.0,
            throttle: throttle.max(0.0),
            steering,
            brake,
        })
    }

    fn reset(&mut self) {
        self.speed_pid.reset();
        self.steering_pid.reset();
    }

    fn controller_type(&self) -> &'static str {
        "ugv"
    }
}

/// Controller factory
pub fn create_controller(vehicle_type: autonomy_common::ids::VehicleType) -> Box<dyn Controller> {
    match vehicle_type {
        autonomy_common::ids::VehicleType::Uav => Box::new(UavPositionController::new()),
        autonomy_common::ids::VehicleType::Ugv => Box::new(UgvController::new()),
        _ => Box::new(UavPositionController::new()),
    }
}
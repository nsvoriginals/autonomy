//! PID controller implementations

use autonomy_common::frames::Vec3;
use autonomy_common::math::PidController;
use serde::{Deserialize, Serialize};

/// 3D PID controller for position control
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Pid3dController {
    pub x: PidController,
    pub y: PidController,
    pub z: PidController,
}

impl Pid3dController {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            x: PidController::new(kp, ki, kd),
            y: PidController::new(kp, ki, kd),
            z: PidController::new(kp, ki, kd),
        }
    }

    pub fn new_per_axis(kp: [f64; 3], ki: [f64; 3], kd: [f64; 3]) -> Self {
        Self {
            x: PidController::new(kp[0], ki[0], kd[0]),
            y: PidController::new(kp[1], ki[1], kd[1]),
            z: PidController::new(kp[2], ki[2], kd[2]),
        }
    }

    pub fn update(&mut self, error: Vec3, dt: f64) -> Vec3 {
        Vec3::new(
            self.x.update(error.x, dt),
            self.y.update(error.y, dt),
            self.z.update(error.z, dt),
        )
    }

    pub fn reset(&mut self) {
        self.x.reset();
        self.y.reset();
        self.z.reset();
    }

    pub fn set_limits(&mut self, integral_limit: f64, output_limit: f64) {
        self.x = self.x.clone().with_limits(integral_limit, output_limit);
        self.y = self.y.clone().with_limits(integral_limit, output_limit);
        self.z = self.z.clone().with_limits(integral_limit, output_limit);
    }
}

/// Cascaded PID controller (position -> velocity -> attitude)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CascadedPidController {
    pub position: Pid3dController,
    pub velocity: Pid3dController,
    pub attitude: Pid3dController,
    pub yaw: PidController,
}

impl CascadedPidController {
    pub fn new() -> Self {
        Self {
            position: Pid3dController::new_per_axis(
                [1.0, 1.0, 2.0], [0.1, 0.1, 0.2], [0.5, 0.5, 1.0]
            ),
            velocity: Pid3dController::new_per_axis(
                [2.0, 2.0, 5.0], [0.5, 0.5, 1.0], [0.1, 0.1, 0.2]
            ),
            attitude: Pid3dController::new_per_axis(
                [4.0, 4.0, 4.0], [0.0, 0.0, 0.0], [0.5, 0.5, 0.5]
            ),
            yaw: PidController::new(2.0, 0.0, 0.5),
        }
    }

    pub fn update(&mut self, pos_error: Vec3, vel_error: Vec3, att_error: Vec3, yaw_error: f64, dt: f64) -> (Vec3, f64) {
        // Outer loop: position -> velocity setpoint
        let vel_sp = self.position.update(pos_error, dt);
        
        // Middle loop: velocity -> acceleration/thrust
        let vel_err = vel_sp - vel_error;
        let accel_sp = self.velocity.update(vel_err, dt);
        
        // Inner loop: attitude -> motor commands
        let thrust = accel_sp.y; // Z-axis thrust
        let roll = -accel_sp.x / 9.80665; // Simplified
        let pitch = accel_sp.z / 9.80665;
        
        let att_error = Vec3::new(roll, pitch, att_error.z);
        let torque = self.attitude.update(att_error, dt);
        let yaw_rate = self.yaw.update(yaw_error, dt);
        
        (torque, yaw_rate)
    }

    pub fn reset(&mut self) {
        self.position.reset();
        self.velocity.reset();
        self.attitude.reset();
        self.yaw.reset();
    }
}

impl Default for CascadedPidController {
    fn default() -> Self {
        Self::new()
    }
}
//! Common math utilities

use nalgebra::{Vector3, UnitQuaternion};
use std::f64::consts::PI;

pub const DEG_TO_RAD: f64 = PI / 180.0;
pub const RAD_TO_DEG: f64 = 180.0 / PI;
pub const GRAVITY: f64 = 9.80665;

/// Clamp value to range
pub fn clamp(value: f64, min: f64, max: f64) -> f64 {
    value.max(min).min(max)
}

/// Wrap angle to [-pi, pi]
pub fn wrap_angle(angle: f64) -> f64 {
    let mut a = angle % (2.0 * PI);
    if a > PI {
        a -= 2.0 * PI;
    } else if a < -PI {
        a += 2.0 * PI;
    }
    a
}

/// Wrap angle to [0, 2*pi]
pub fn wrap_angle_positive(angle: f64) -> f64 {
    let mut a = angle % (2.0 * PI);
    if a < 0.0 {
        a += 2.0 * PI;
    }
    a
}

/// Shortest angular difference (a - b) wrapped to [-pi, pi]
pub fn angle_diff(a: f64, b: f64) -> f64 {
    wrap_angle(a - b)
}

/// Linear interpolation
pub fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t.clamp(0.0, 1.0)
}

/// Spherical linear interpolation for quaternions
pub fn slerp(a: UnitQuaternion<f64>, b: UnitQuaternion<f64>, t: f64) -> UnitQuaternion<f64> {
    a.slerp(&b, t.clamp(0.0, 1.0))
}

/// Linear interpolation for vectors
pub fn lerp_vec3(a: Vector3<f64>, b: Vector3<f64>, t: f64) -> Vector3<f64> {
    a + (b - a) * t.clamp(0.0, 1.0)
}

/// Deadband function
pub fn deadband(value: f64, threshold: f64) -> f64 {
    if value.abs() < threshold {
        0.0
    } else {
        value
    }
}

/// Rate limiter
pub fn rate_limit(current: f64, target: f64, max_rate: f64, dt: f64) -> f64 {
    let max_change = max_rate * dt;
    let diff = target - current;
    if diff.abs() <= max_change {
        target
    } else {
        current + max_change * diff.signum()
    }
}

/// First-order low-pass filter
pub fn low_pass_filter(current: f64, input: f64, tau: f64, dt: f64) -> f64 {
    let alpha = dt / (tau + dt);
    current + alpha * (input - current)
}

/// Exponential moving average
pub fn ema(current: f64, input: f64, alpha: f64) -> f64 {
    current * (1.0 - alpha) + input * alpha
}

/// PID controller output
pub struct PidController {
    kp: f64,
    ki: f64,
    kd: f64,
    integral: f64,
    prev_error: f64,
    integral_limit: f64,
    output_limit: f64,
}

impl PidController {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: 0.0,
            integral_limit: f64::INFINITY,
            output_limit: f64::INFINITY,
        }
    }

    pub fn with_limits(mut self, integral_limit: f64, output_limit: f64) -> Self {
        self.integral_limit = integral_limit;
        self.output_limit = output_limit;
        self
    }

    pub fn update(&mut self, error: f64, dt: f64) -> f64 {
        self.integral = clamp(self.integral + error * dt, -self.integral_limit, self.integral_limit);
        let derivative = (error - self.prev_error) / dt;
        self.prev_error = error;

        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        clamp(output, -self.output_limit, self.output_limit)
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }
}

/// 2D PID for position control
pub struct Pid2d {
    x: PidController,
    y: PidController,
}

impl Pid2d {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            x: PidController::new(kp, ki, kd),
            y: PidController::new(kp, ki, kd),
        }
    }

    pub fn update(&mut self, error: Vector3<f64>, dt: f64) -> Vector3<f64> {
        Vector3::new(self.x.update(error.x, dt), self.y.update(error.y, dt), 0.0)
    }

    pub fn reset(&mut self) {
        self.x.reset();
        self.y.reset();
    }
}

/// 3D PID for position control
pub struct Pid3d {
    x: PidController,
    y: PidController,
    z: PidController,
}

impl Pid3d {
    pub fn new(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            x: PidController::new(kp, ki, kd),
            y: PidController::new(kp, ki, kd),
            z: PidController::new(kp, ki, kd),
        }
    }

    pub fn update(&mut self, error: Vector3<f64>, dt: f64) -> Vector3<f64> {
        Vector3::new(
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
}

/// Quaternion from yaw angle (Z-up)
pub fn quat_from_yaw(yaw: f64) -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw)
}

/// Yaw from quaternion (Z-up)
pub fn yaw_from_quat(q: UnitQuaternion<f64>) -> f64 {
    let (_, _, yaw) = q.euler_angles();
    yaw
}

/// Rotate vector by yaw only
pub fn rotate_by_yaw(v: Vector3<f64>, yaw: f64) -> Vector3<f64> {
    let c = yaw.cos();
    let s = yaw.sin();
    Vector3::new(c * v.x - s * v.y, s * v.x + c * v.y, v.z)
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::UnitQuaternion;

    #[test]
    fn test_wrap_angle() {
        assert!((wrap_angle(3.0 * PI) - PI).abs() < 1e-10);
        assert!((wrap_angle(-3.0 * PI) + PI).abs() < 1e-10);
        assert!((wrap_angle(0.0) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_angle_diff() {
        assert!((angle_diff(0.1, -0.1) - 0.2).abs() < 1e-10);
        assert!((angle_diff(PI, -PI) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_pid() {
        let mut pid = PidController::new(1.0, 0.1, 0.05);
        let output = pid.update(1.0, 0.01);
        assert!(output > 0.0);
    }

    #[test]
    fn test_quat_yaw() {
        let yaw = 1.5;
        let q = quat_from_yaw(yaw);
        let y = yaw_from_quat(q);
        assert!((y - yaw).abs() < 1e-10);
    }
}
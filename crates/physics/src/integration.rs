//! Numerical integration methods for physics

use autonomy_common::frames::{LinearAcceleration, LinearVelocity, Pose, Vec3};
use nalgebra::{UnitQuaternion, Vector3};

/// Integration method
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum IntegrationMethod {
    Euler,
    SemiImplicitEuler,
    VelocityVerlet,
    RungeKutta4,
}

/// Physics state for integration
#[derive(Clone, Debug)]
pub struct PhysicsState {
    pub pose: Pose,
    pub linear_velocity: LinearVelocity,
    pub angular_velocity: Vec3,
    pub linear_acceleration: LinearAcceleration,
    pub angular_acceleration: Vec3,
    pub mass: f64,
    pub inertia_tensor: nalgebra::Matrix3<f64>,
    pub inverse_inertia_tensor: nalgebra::Matrix3<f64>,
}

impl PhysicsState {
    pub fn new(mass: f64, inertia: Vec3) -> Self {
        let inertia_tensor = nalgebra::Matrix3::from_diagonal(&Vector3::new(inertia.x, inertia.y, inertia.z));
        let inverse_inertia_tensor = inertia_tensor.try_inverse().unwrap_or(nalgebra::Matrix3::zeros());
        
        Self {
            pose: Pose::identity(),
            linear_velocity: LinearVelocity(Vec3::zeros()),
            angular_velocity: Vec3::zeros(),
            linear_acceleration: LinearAcceleration(Vec3::zeros()),
            angular_acceleration: Vec3::zeros(),
            mass,
            inertia_tensor,
            inverse_inertia_tensor,
        }
    }
}

/// Integrate physics state
pub fn integrate(
    state: &mut PhysicsState,
    forces: Vec3,
    torques: Vec3,
    dt: f64,
    method: IntegrationMethod,
) {
    match method {
        IntegrationMethod::Euler => integrate_euler(state, forces, torques, dt),
        IntegrationMethod::SemiImplicitEuler => integrate_semi_implicit_euler(state, forces, torques, dt),
        IntegrationMethod::VelocityVerlet => integrate_velocity_verlet(state, forces, torques, dt),
        IntegrationMethod::RungeKutta4 => integrate_rk4(state, forces, torques, dt),
    }
}

/// Explicit Euler integration
fn integrate_euler(state: &mut PhysicsState, forces: Vec3, torques: Vec3, dt: f64) {
    // Linear
    state.linear_acceleration = LinearAcceleration(forces / state.mass);
    state.linear_velocity.0 += state.linear_acceleration.0 * dt;
    state.pose.position += state.linear_velocity.0 * dt;

    // Angular
    state.angular_acceleration = state.inverse_inertia_tensor * torques;
    state.angular_velocity += state.angular_acceleration * dt;
    
    // Update orientation
    let delta_angle = state.angular_velocity * dt;
    let delta_rot = UnitQuaternion::from_scaled_axis(delta_angle);
    state.pose.orientation = delta_rot * state.pose.orientation;
}

/// Semi-implicit Euler (symplectic Euler)
fn integrate_semi_implicit_euler(state: &mut PhysicsState, forces: Vec3, torques: Vec3, dt: f64) {
    // Linear: update velocity first, then position
    state.linear_acceleration = LinearAcceleration(forces / state.mass);
    state.linear_velocity.0 += state.linear_acceleration.0 * dt;
    state.pose.position += state.linear_velocity.0 * dt;

    // Angular: update velocity first, then orientation
    state.angular_acceleration = state.inverse_inertia_tensor * torques;
    state.angular_velocity += state.angular_acceleration * dt;
    
    let delta_angle = state.angular_velocity * dt;
    let delta_rot = UnitQuaternion::from_scaled_axis(delta_angle);
    state.pose.orientation = delta_rot * state.pose.orientation;
}

/// Velocity Verlet integration
fn integrate_velocity_verlet(state: &mut PhysicsState, forces: Vec3, torques: Vec3, dt: f64) {
    let half_dt = dt * 0.5;
    
    // Linear: v(t + dt/2) = v(t) + a(t) * dt/2
    let accel = forces / state.mass;
    state.linear_velocity.0 += accel * half_dt;
    state.pose.position += state.linear_velocity.0 * dt;
    
    // Angular: omega(t + dt/2) = omega(t) + alpha(t) * dt/2
    let alpha = state.inverse_inertia_tensor * torques;
    state.angular_velocity += alpha * half_dt;
    
    let delta_angle = state.angular_velocity * dt;
    let delta_rot = UnitQuaternion::from_scaled_axis(delta_angle);
    state.pose.orientation = delta_rot * state.pose.orientation;
    
    // v(t + dt) = v(t + dt/2) + a(t + dt) * dt/2
    // Note: In practice, we'd need new forces at t+dt
    // For now, use same acceleration
    state.linear_velocity.0 += accel * half_dt;
    state.angular_velocity += alpha * half_dt;
    
    state.linear_acceleration = LinearAcceleration(accel);
    state.angular_acceleration = alpha;
}

/// Runge-Kutta 4th order integration
fn integrate_rk4(state: &mut PhysicsState, forces: Vec3, torques: Vec3, dt: f64) {
    // RK4 for linear motion
    let k1v = forces / state.mass;
    let k1x = state.linear_velocity.0;
    
    let k2v = forces / state.mass; // Constant forces
    let k2x = state.linear_velocity.0 + k1v * half_dt;
    
    let k3v = forces / state.mass;
    let k3x = state.linear_velocity.0 + k2v * half_dt;
    
    let k4v = forces / state.mass;
    let k4x = state.linear_velocity.0 + k3v * dt;
    
    state.linear_velocity.0 += (k1v + 2.0 * k2v + 2.0 * k3v + k4v) * (dt / 6.0);
    state.pose.position += (k1x + 2.0 * k2x + 2.0 * k3x + k4x) * (dt / 6.0);
    state.linear_acceleration = LinearAcceleration(k4v);
    
    // RK4 for angular motion (simplified - assumes constant torque)
    let half_dt = dt * 0.5;
    let k1w = state.inverse_inertia_tensor * torques;
    let k1o = state.angular_velocity;
    
    let k2w = state.inverse_inertia_tensor * torques;
    let k2o = state.angular_velocity + k1w * half_dt;
    
    let k3w = state.inverse_inertia_tensor * torques;
    let k3o = state.angular_velocity + k2w * half_dt;
    
    let k4w = state.inverse_inertia_tensor * torques;
    let k4o = state.angular_velocity + k3w * dt;
    
    state.angular_velocity += (k1w + 2.0 * k2w + 2.0 * k3w + k4w) * (dt / 6.0);
    state.angular_acceleration = k4w;
    
    // Orientation update using average angular velocity
    let avg_omega = (k1o + 2.0 * k2o + 2.0 * k3o + k4o) / 6.0;
    let delta_angle = avg_omega * dt;
    let delta_rot = UnitQuaternion::from_scaled_axis(delta_angle);
    state.pose.orientation = delta_rot * state.pose.orientation;
}

const fn half_dt(dt: f64) -> f64 {
    dt * 0.5
}

/// Quaternion derivative from angular velocity
pub fn quaternion_derivative(q: UnitQuaternion<f64>, omega: Vec3) -> UnitQuaternion<f64> {
    let omega_quat = UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(0.0, omega.x, omega.y, omega.z));
    (0.5 * omega_quat * q).into()
}

/// Apply damping
pub fn apply_damping(state: &mut PhysicsState, linear_damping: f64, angular_damping: f64, dt: f64) {
    let linear_factor = (-linear_damping * dt).exp();
    let angular_factor = (-angular_damping * dt).exp();
    
    state.linear_velocity.0 *= linear_factor;
    state.angular_velocity *= angular_factor;
}

/// Clamp maximum velocities
pub fn clamp_velocities(state: &mut PhysicsState, max_linear: f64, max_angular: f64) {
    let linear_speed = state.linear_velocity.0.magnitude();
    if linear_speed > max_linear {
        state.linear_velocity.0 *= max_linear / linear_speed;
    }
    
    let angular_speed = state.angular_velocity.magnitude();
    if angular_speed > max_angular {
        state.angular_velocity *= max_angular / angular_speed;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_euler_integration() {
        let mut state = PhysicsState::new(1.0, Vec3::new(1.0, 1.0, 1.0));
        let dt = 0.01;
        
        integrate(&mut state, Vec3::new(10.0, 0.0, 0.0), Vec3::zeros(), dt, IntegrationMethod::Euler);
        
        // v = a * dt = 10 * 0.01 = 0.1
        assert!((state.linear_velocity.0.x - 0.1).abs() < 1e-10);
        // x = v * dt = 0.1 * 0.01 = 0.001
        assert!((state.pose.position.x - 0.001).abs() < 1e-10);
    }

    #[test]
    fn test_semi_implicit_euler() {
        let mut state = PhysicsState::new(1.0, Vec3::new(1.0, 1.0, 1.0));
        let dt = 0.01;
        
        integrate(&mut state, Vec3::new(10.0, 0.0, 0.0), Vec3::zeros(), dt, IntegrationMethod::SemiImplicitEuler);
        
        // Same as Euler for constant forces
        assert!((state.linear_velocity.0.x - 0.1).abs() < 1e-10);
        assert!((state.pose.position.x - 0.001).abs() < 1e-10);
    }

    #[test]
    fn test_damping() {
        let mut state = PhysicsState::new(1.0, Vec3::new(1.0, 1.0, 1.0));
        state.linear_velocity = LinearVelocity(Vec3::new(10.0, 0.0, 0.0));
        state.angular_velocity = Vec3::new(0.0, 10.0, 0.0);
        
        apply_damping(&mut state, 1.0, 1.0, 1.0);
        
        // After 1 second with damping=1, velocity should be 1/e of original
        let expected = 10.0 * std::f64::consts::E.powf(-1.0);
        assert!((state.linear_velocity.0.x - expected).abs() < 0.01);
    }
}
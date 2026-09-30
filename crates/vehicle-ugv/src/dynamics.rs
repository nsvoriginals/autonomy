//! UGV wheeled/tracked dynamics model

use autonomy_common::frames::{LinearAcceleration, LinearVelocity, Pose, Vec3, AngularVelocity};
use autonomy_common::ids::VehicleId;
use autonomy_common::state::{BatteryState, ControlInputs, HealthStatus, MotorState, VehicleState};
use autonomy_common::time::SimTime;
use autonomy_common::traits::ActuatorSet;
use autonomy_physics::bodies::BodyHandle;
use autonomy_physics::forces::gravity_force;
use nalgebra::Vector3;
use parking_lot::RwLock;
use smallvec::SmallVec;
use std::sync::Arc;

/// UGV configuration
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct UgvConfig {
    pub mass: f64,
    pub length: f64,
    pub width: f64,
    pub height: f64,
    pub wheelbase: f64,
    pub track_width: f64,
    pub wheel_radius: f64,
    pub wheel_count: usize,
    pub max_motor_torque: f64,
    pub max_brake_torque: f64,
    pub max_steering_angle: f64,
    pub motor_time_constant: f64,
    pub friction_coefficient: f64,
    pub rolling_resistance: f64,
    pub drag_coefficient: f64,
    pub frontal_area: f64,
    pub battery_capacity_wh: f64,
    pub drivetrain: DrivetrainType,
}

impl Default for UgvConfig {
    fn default() -> Self {
        Self {
            mass: 50.0,
            length: 1.0,
            width: 0.6,
            height: 0.4,
            wheelbase: 0.6,
            track_width: 0.5,
            wheel_radius: 0.1,
            wheel_count: 4,
            max_motor_torque: 10.0,
            max_brake_torque: 20.0,
            max_steering_angle: 0.5,
            motor_time_constant: 0.1,
            friction_coefficient: 0.8,
            rolling_resistance: 0.02,
            drag_coefficient: 0.5,
            frontal_area: 0.3,
            battery_capacity_wh: 200.0,
            drivetrain: DrivetrainType::FourWheelDrive,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum DrivetrainType {
    TwoWheelDrive,
    FourWheelDrive,
    Tracked,
    SkidSteer,
}

/// UGV dynamics state
#[derive(Clone, Debug)]
pub struct UgvDynamics {
    config: UgvConfig,
    wheel_speeds: Vec<f64>,
    wheel_torques: Vec<f64>,
    steering_angles: Vec<f64>,
    battery: BatteryState,
    longitudinal_force: f64,
    lateral_force: f64,
    yaw_moment: f64,
    power_consumption: f64,
    slip_ratios: Vec<f64>,
}

impl UgvDynamics {
    pub fn new(config: UgvConfig) -> Self {
        let battery = BatteryState {
            capacity_wh: config.battery_capacity_wh,
            ..Default::default()
        };
        
        Self {
            config: config.clone(),
            wheel_speeds: vec![0.0; config.wheel_count],
            wheel_torques: vec![0.0; config.wheel_count],
            steering_angles: vec![0.0; config.wheel_count],
            battery,
            longitudinal_force: 0.0,
            lateral_force: 0.0,
            yaw_moment: 0.0,
            power_consumption: 0.0,
            slip_ratios: vec![0.0; config.wheel_count],
        }
    }

    pub fn step(&mut self, dt: f64, controls: &ControlInputs, pose: &Pose, linear_vel: &LinearVelocity, angular_vel: &AngularVelocity) -> (Vec3, Vec3) {
        // Update steering
        self.update_steering(dt, controls);
        
        // Compute wheel torques
        self.update_wheel_torques(dt, controls);
        
        // Compute tire forces (simplified pacejka-like model)
        self.compute_tire_forces(pose, linear_vel, angular_vel);
        
        // Aerodynamic drag
        let speed = linear_vel.0.magnitude();
        let drag_force = -linear_vel.0.normalize() * 0.5 * 1.225 * speed * speed * self.config.drag_coefficient * self.config.frontal_area;
        
        // Rolling resistance
        let rolling_resistance = -linear_vel.0.normalize() * self.config.mass * 9.80665 * self.config.rolling_resistance;
        
        // Total forces in body frame
        let body_force = Vec3::new(self.lateral_force, 0.0, self.longitudinal_force);
        let world_force = pose.orientation * body_force + drag_force + rolling_resistance + gravity_force(self.config.mass, 9.80665);
        
        // Yaw moment
        let total_torque = Vec3::new(0.0, self.yaw_moment, 0.0);
        
        // Update battery
        self.update_battery(dt);
        
        (world_force, total_torque)
    }

    fn update_steering(&mut self, dt: f64, controls: &ControlInputs) {
        let target_steering = controls.steering * self.config.max_steering_angle;
        let steering_rate = 2.0; // rad/s
        
        for i in 0..self.config.wheel_count {
            let error = target_steering - self.steering_angles[i];
            let max_change = steering_rate * dt;
            if error.abs() <= max_change {
                self.steering_angles[i] = target_steering;
            } else {
                self.steering_angles[i] += max_change * error.signum();
            }
        }
    }

    fn update_wheel_torques(&mut self, dt: f64, controls: &ControlInputs) {
        let throttle = controls.throttle.clamp(-1.0, 1.0);
        let brake = controls.brake.clamp(0.0, 1.0);
        
        for i in 0..self.config.wheel_count {
            let target_torque = throttle * self.config.max_motor_torque;
            
            // Apply brake
            if brake > 0.0 {
                let brake_torque = -self.wheel_speeds[i].signum() * brake * self.config.max_brake_torque;
                self.wheel_torques[i] = target_torque + brake_torque;
            } else {
                self.wheel_torques[i] = target_torque;
            }
            
            // Motor dynamics
            let torque_error = self.wheel_torques[i] - self.wheel_torques[i]; // Would need target vs actual
            self.wheel_torques[i] += torque_error * (dt / self.config.motor_time_constant);
        }
    }

    fn compute_tire_forces(&mut self, pose: &Pose, linear_vel: &LinearVelocity, angular_vel: &AngularVelocity) {
        // Simplified tire model
        // Transform velocity to body frame
        let v_body = pose.orientation.inverse() * linear_vel.0;
        let omega = angular_vel.0.y; // Yaw rate
        
        self.longitudinal_force = 0.0;
        self.lateral_force = 0.0;
        self.yaw_moment = 0.0;
        
        let half_wheelbase = self.config.wheelbase * 0.5;
        let half_track = self.config.track_width * 0.5;
        
        for i in 0..self.config.wheel_count {
            // Wheel position relative to CG
            let (x, z) = match i {
                0 => (-half_wheelbase, -half_track), // Front-left
                1 => (-half_wheelbase, half_track),  // Front-right
                2 => (half_wheelbase, -half_track),  // Rear-left
                3 => (half_wheelbase, half_track),   // Rear-right
                _ => (0.0, 0.0),
            };
            
            // Wheel velocity at contact patch
            let wheel_vel = v_body + Vec3::new(-omega * z, 0.0, omega * x);
            
            // Steering angle
            let steer = self.steering_angles[i];
            
            // Slip angle
            let slip_angle = if wheel_vel.z.abs() > 0.1 {
                ((wheel_vel.x).atan2(wheel_vel.z) - steer).clamp(-1.5, 1.5)
            } else {
                0.0
            };
            
            // Slip ratio
            let wheel_speed = self.wheel_speeds[i] * self.config.wheel_radius;
            let slip_ratio = if wheel_vel.z.abs() > 0.1 {
                (wheel_speed - wheel_vel.z) / wheel_vel.z.abs()
            } else {
                0.0
            };
            self.slip_ratios[i] = slip_ratio;
            
            // Normal force (simplified weight distribution)
            let normal_force = self.config.mass * 9.80665 / self.config.wheel_count as f64;
            
            // Lateral force (simplified Pacejka)
            let mu = self.config.friction_coefficient;
            let cornering_stiffness = 10000.0;
            let lateral_force = -normal_force * mu * (slip_angle * cornering_stiffness).tanh();
            
            // Longitudinal force
            let longitudinal_force = normal_force * mu * (slip_ratio * 10.0).tanh();
            
            // Transform to body frame
            let cs = steer.cos();
            let sn = steer.sin();
            
            let fx = longitudinal_force * cs - lateral_force * sn;
            let fz = longitudinal_force * sn + lateral_force * cs;
            
            self.longitudinal_force += fx;
            self.lateral_force += fz;
            
            // Yaw moment
            self.yaw_moment += fx * z - fz * x;
        }
    }

    fn update_battery(&mut self, dt: f64) {
        let voltage = self.battery.voltage;
        let total_power: f64 = self.wheel_torques.iter()
            .zip(self.wheel_speeds.iter())
            .map(|(&torque, &speed)| torque * speed)
            .sum();
        
        self.power_consumption = total_power;
        let current = total_power / voltage;
        
        self.battery.current = current;
        self.battery.charge_remaining -= (current * dt / 3600.0) / (self.config.battery_capacity_wh / voltage);
        self.battery.charge_remaining = self.battery.charge_remaining.clamp(0.0, 1.0);
        self.battery.voltage = voltage * (0.9 + 0.1 * self.battery.charge_remaining);
    }

    pub fn motor_states(&self) -> SmallVec<[MotorState; 8]> {
        self.wheel_torques.iter()
            .zip(self.wheel_speeds.iter())
            .map(|(&torque, &speed)| MotorState {
                rpm: speed * 60.0 / (2.0 * std::f64::consts::PI),
                thrust: 0.0,
                torque,
                temperature: 25.0 + torque.abs() * 0.5,
                voltage: self.battery.voltage,
                current: torque * speed / self.battery.voltage,
                health: 1.0,
            })
            .collect()
    }

    pub fn wheel_speeds(&self) -> &[f64] {
        &self.wheel_speeds
    }

    pub fn slip_ratios(&self) -> &[f64] {
        &self.slip_ratios
    }

    pub fn battery(&self) -> &BatteryState {
        &self.battery
    }

    pub fn config(&self) -> &UgvConfig {
        &self.config
    }
}

/// UGV actuator set
pub struct UgvActuators {
    dynamics: RwLock<UgvDynamics>,
}

impl UgvActuators {
    pub fn new(config: UgvConfig) -> Self {
        Self {
            dynamics: RwLock::new(UgvDynamics::new(config)),
        }
    }

    pub fn dynamics(&self) -> &RwLock<UgvDynamics> {
        &self.dynamics
    }
}

impl ActuatorSet for UgvActuators {
    fn apply_controls(&mut self, inputs: ControlInputs) -> autonomy_common::error::Result<()> {
        Ok(())
    }

    fn get_motor_states(&self) -> &[MotorState] {
        unimplemented!("Use dynamics().read().motor_states()")
    }

    fn set_motor_command(&mut self, index: usize, command: f64) -> autonomy_common::error::Result<()> {
        Ok(())
    }

    fn num_actuators(&self) -> usize {
        self.dynamics.read().config.wheel_count
    }
}
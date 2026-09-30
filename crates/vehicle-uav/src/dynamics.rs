//! UAV multirotor dynamics model

use autonomy_common::frames::{LinearAcceleration, LinearVelocity, Pose, Vec3, AngularVelocity};
use autonomy_common::ids::VehicleId;
use autonomy_common::state::{BatteryState, ControlInputs, HealthStatus, MotorState, VehicleState};
use autonomy_common::time::SimTime;
use autonomy_common::traits::ActuatorSet;
use autonomy_physics::bodies::BodyHandle;
use autonomy_physics::forces::{AerodynamicForces, ForceAccumulator, GroundEffect, gravity_force};
use nalgebra::Vector3;
use parking_lot::RwLock;
use smallvec::SmallVec;
use std::sync::Arc;

/// UAV configuration
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct UavConfig {
    pub mass: f64,
    pub arm_length: f64,
    pub rotor_radius: f64,
    pub rotor_count: usize,
    pub max_thrust_per_rotor: f64,
    pub max_torque_per_rotor: f64,
    pub motor_time_constant: f64,
    pub max_rpm: f64,
    pub battery_capacity_wh: f64,
    pub drag_coefficient: f64,
    pub body_drag: Vec3,
    pub ground_effect: bool,
}

impl Default for UavConfig {
    fn default() -> Self {
        Self {
            mass: 1.5,
            arm_length: 0.3,
            rotor_radius: 0.15,
            rotor_count: 4,
            max_thrust_per_rotor: 5.0,
            max_torque_per_rotor: 0.1,
            motor_time_constant: 0.05,
            max_rpm: 10000.0,
            battery_capacity_wh: 100.0,
            drag_coefficient: 0.1,
            body_drag: Vec3::new(0.1, 0.1, 0.1),
            ground_effect: true,
        }
    }
}

/// UAV dynamics state
#[derive(Clone, Debug)]
pub struct UavDynamics {
    config: UavConfig,
    motor_rpms: Vec<f64>,
    motor_thrusts: Vec<f64>,
    motor_torques: Vec<f64>,
    battery: BatteryState,
    aerodynamics: AerodynamicForces,
    ground_effect: GroundEffect,
    total_thrust: f64,
    total_torque: Vec3,
    power_consumption: f64,
}

impl UavDynamics {
    pub fn new(config: UavConfig) -> Self {
        let aerodynamics = AerodynamicForces::new(1.225, config.drag_coefficient, config.arm_length * config.arm_length);
        let ground_effect = GroundEffect::new(config.rotor_radius);
        let battery = BatteryState {
            capacity_wh: config.battery_capacity_wh,
            ..Default::default()
        };
        
        Self {
            config: config.clone(),
            motor_rpms: vec![0.0; config.rotor_count],
            motor_thrusts: vec![0.0; config.rotor_count],
            motor_torques: vec![0.0; config.rotor_count],
            battery,
            aerodynamics,
            ground_effect,
            total_thrust: 0.0,
            total_torque: Vec3::zeros(),
            power_consumption: 0.0,
        }
    }

    pub fn step(&mut self, dt: f64, controls: &ControlInputs, pose: &Pose, linear_vel: &LinearVelocity, angular_vel: &AngularVelocity) -> (Vec3, Vec3) {
        // Compute motor commands
        let motor_commands = self.mixing_matrix(controls);
        
        // Update motor dynamics
        for i in 0..self.config.rotor_count {
            let target_rpm = motor_commands[i] * self.config.max_rpm;
            let rpm_error = target_rpm - self.motor_rpms[i];
            self.motor_rpms[i] += rpm_error * (dt / self.config.motor_time_constant);
            self.motor_rpms[i] = self.motor_rpms[i].clamp(0.0, self.config.max_rpm);
            
            // Thrust proportional to RPM^2
            let thrust_coeff = self.config.max_thrust_per_rotor / (self.config.max_rpm * self.config.max_rpm);
            self.motor_thrusts[i] = thrust_coeff * self.motor_rpms[i] * self.motor_rpms[i];
            
            // Torque proportional to thrust
            let torque_coeff = self.config.max_torque_per_rotor / self.config.max_thrust_per_rotor;
            self.motor_torques[i] = self.motor_thrusts[i] * torque_coeff * (if i % 2 == 0 { 1.0 } else { -1.0 });
        }
        
        // Compute total thrust and torque
        self.total_thrust = self.motor_thrusts.iter().sum();
        self.total_torque = Vec3::zeros();
        
        // Body torques from differential thrust
        let arm = self.config.arm_length;
        for i in 0..self.config.rotor_count {
            let angle = (i as f64) * 2.0 * std::f64::consts::PI / (self.config.rotor_count as f64);
            let pos = Vec3::new(angle.cos() * arm, 0.0, angle.sin() * arm);
            let force = Vec3::new(0.0, self.motor_thrusts[i], 0.0);
            self.total_torque += pos.cross(&force);
        }
        
        // Add motor reaction torques
        let reaction_torque: f64 = self.motor_torques.iter().sum();
        self.total_torque += Vec3::new(0.0, reaction_torque, 0.0);
        
        // Apply ground effect
        let height_above_ground = pose.position.y.max(0.0);
        let ge_factor = if self.config.ground_effect {
            self.ground_effect.thrust_multiplier(height_above_ground)
        } else {
            1.0
        };
        self.total_thrust *= ge_factor;
        
        // Aerodynamic forces
        let (drag_force, drag_torque) = self.aerodynamics.compute_body_drag(linear_vel.0, angular_vel.0);
        
        // Gravity
        let gravity = gravity_force(self.config.mass, 9.80665);
        
        // Total forces and torques in body frame
        let body_thrust = Vec3::new(0.0, self.total_thrust, 0.0);
        
        // Transform to world frame
        let world_thrust = pose.orientation * body_thrust;
        let world_drag = drag_force; // Simplified
        let world_gravity = gravity;
        
        let total_force = world_thrust + world_drag + world_gravity;
        let total_torque = self.total_torque + drag_torque;
        
        // Update battery
        self.update_battery(dt);
        
        (total_force, total_torque)
    }

    fn mixing_matrix(&self, controls: &ControlInputs) -> Vec<f64> {
        // Quadcopter mixing matrix
        // [thrust, roll, pitch, yaw] -> motor commands
        let n = self.config.rotor_count;
        let mut commands = vec![0.0; n];
        
        if n == 4 {
            // Standard quad X configuration
            commands[0] = controls.collective_thrust - controls.pitch_torque + controls.roll_torque - controls.yaw_torque; // Front-right (CW)
            commands[1] = controls.collective_thrust + controls.pitch_torque + controls.roll_torque + controls.yaw_torque; // Rear-right (CCW)
            commands[2] = controls.collective_thrust + controls.pitch_torque - controls.roll_torque - controls.yaw_torque; // Rear-left (CW)
            commands[3] = controls.collective_thrust - controls.pitch_torque - controls.roll_torque + controls.yaw_torque; // Front-left (CCW)
        } else if n == 6 {
            // Hexacopter
            for i in 0..6 {
                let angle = (i as f64) * std::f64::consts::PI / 3.0;
                commands[i] = controls.collective_thrust 
                    + controls.pitch_torque * angle.sin() 
                    - controls.roll_torque * angle.cos()
                    + controls.yaw_torque * (if i % 2 == 0 { 1.0 } else { -1.0 });
            }
        } else if n == 8 {
            // Octocopter
            for i in 0..8 {
                let angle = (i as f64) * std::f64::consts::PI / 4.0;
                commands[i] = controls.collective_thrust 
                    + controls.pitch_torque * angle.sin() 
                    - controls.roll_torque * angle.cos()
                    + controls.yaw_torque * (if i % 2 == 0 { 1.0 } else { -1.0 });
            }
        }
        
        // Clamp
        for cmd in &mut commands {
            *cmd = cmd.clamp(0.0, 1.0);
        }
        
        commands
    }

    fn update_battery(&mut self, dt: f64) {
        // Power consumption estimate
        let voltage = self.battery.voltage;
        let total_power: f64 = self.motor_thrusts.iter()
            .zip(self.motor_rpms.iter())
            .map(|(&thrust, &rpm)| thrust * rpm / 1000.0) // Simplified
            .sum();
        
        self.power_consumption = total_power;
        let current = total_power / voltage;
        
        self.battery.current = current;
        self.battery.charge_remaining -= (current * dt / 3600.0) / (self.config.battery_capacity_wh / voltage);
        self.battery.charge_remaining = self.battery.charge_remaining.clamp(0.0, 1.0);
        
        // Voltage sag
        self.battery.voltage = voltage * (0.9 + 0.1 * self.battery.charge_remaining);
    }

    pub fn motor_states(&self) -> SmallVec<[MotorState; 8]> {
        self.motor_rpms.iter()
            .zip(self.motor_thrusts.iter())
            .zip(self.motor_torques.iter())
            .map(|((&rpm, &thrust), &torque)| MotorState {
                rpm,
                thrust,
                torque,
                temperature: 25.0 + rpm / 1000.0,
                voltage: self.battery.voltage,
                current: thrust * rpm / 1000.0 / self.battery.voltage,
                health: 1.0,
            })
            .collect()
    }

    pub fn battery(&self) -> &BatteryState {
        &self.battery
    }

    pub fn battery_mut(&mut self) -> &mut BatteryState {
        &mut self.battery
    }

    pub fn total_thrust(&self) -> f64 {
        self.total_thrust
    }

    pub fn total_torque(&self) -> Vec3 {
        self.total_torque
    }

    pub fn power_consumption(&self) -> f64 {
        self.power_consumption
    }

    pub fn config(&self) -> &UavConfig {
        &self.config
    }
}

/// UAV actuator set
pub struct UavActuators {
    dynamics: RwLock<UavDynamics>,
}

impl UavActuators {
    pub fn new(config: UavConfig) -> Self {
        Self {
            dynamics: RwLock::new(UavDynamics::new(config)),
        }
    }

    pub fn dynamics(&self) -> &RwLock<UavDynamics> {
        &self.dynamics
    }
}

impl ActuatorSet for UavActuators {
    fn apply_controls(&mut self, inputs: ControlInputs) -> autonomy_common::error::Result<()> {
        // Controls are applied during dynamics step
        // Store for next physics step
        Ok(())
    }

    fn get_motor_states(&self) -> &[MotorState] {
        // This is a limitation - we can't return a reference to the RwLock read guard
        // In practice, we'd use a different pattern
        unimplemented!("Use dynamics().read().motor_states()")
    }

    fn set_motor_command(&mut self, index: usize, command: f64) -> autonomy_common::error::Result<()> {
        // Direct motor control for testing
        Ok(())
    }

    fn num_actuators(&self) -> usize {
        self.dynamics.read().config.rotor_count
    }
}

/// UAV vehicle state accessor
pub struct UavVehicleState {
    state: RwLock<VehicleState>,
}

impl UavVehicleState {
    pub fn new(vehicle_id: VehicleId) -> Self {
        let mut state = VehicleState::new(vehicle_id, autonomy_common::ids::VehicleType::Uav);
        state.battery.capacity_wh = 100.0;
        Self {
            state: RwLock::new(state),
        }
    }
}
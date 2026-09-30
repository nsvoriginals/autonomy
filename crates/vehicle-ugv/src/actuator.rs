//! UGV actuator implementations

use autonomy_common::error::Result;
use autonomy_common::state::{ControlInputs, MotorState};
use autonomy_common::traits::ActuatorSet;
use crate::dynamics::{UgvActuators, UgvConfig};
use parking_lot::RwLock;
use smallvec::SmallVec;
use std::sync::Arc;

/// High-level UGV actuator interface
pub struct UgvActuatorInterface {
    actuators: Arc<UgvActuators>,
    pending_controls: RwLock<ControlInputs>,
}

impl UgvActuatorInterface {
    pub fn new(config: UgvConfig) -> Self {
        Self {
            actuators: Arc::new(UgvActuators::new(config)),
            pending_controls: RwLock::new(ControlInputs::default()),
        }
    }

    pub fn actuators(&self) -> &Arc<UgvActuators> {
        &self.actuators
    }

    pub fn set_controls(&self, controls: ControlInputs) {
        *self.pending_controls.write() = controls;
    }

    pub fn get_controls(&self) -> ControlInputs {
        *self.pending_controls.read()
    }
}

impl ActuatorSet for UgvActuatorInterface {
    fn apply_controls(&mut self, inputs: ControlInputs) -> Result<()> {
        *self.pending_controls.write() = inputs;
        Ok(())
    }

    fn get_motor_states(&self) -> &[MotorState] {
        unimplemented!("Use actuators().dynamics().read().motor_states()")
    }

    fn set_motor_command(&mut self, index: usize, command: f64) -> Result<()> {
        Ok(())
    }

    fn num_actuators(&self) -> usize {
        self.actuators.dynamics.read().config.wheel_count
    }
}

/// Motor controller for UGV drive motors
pub struct DriveMotorController {
    pub target_torque: f64,
    pub current_torque: f64,
    pub time_constant: f64,
    pub max_torque: f64,
    pub max_speed: f64,
    pub current_speed: f64,
}

impl DriveMotorController {
    pub fn new(time_constant: f64, max_torque: f64, max_speed: f64) -> Self {
        Self {
            target_torque: 0.0,
            current_torque: 0.0,
            time_constant,
            max_torque,
            max_speed,
            current_speed: 0.0,
        }
    }

    pub fn step(&mut self, dt: f64) -> (f64, f64) {
        let error = self.target_torque - self.current_torque;
        self.current_torque += error * (dt / self.time_constant);
        self.current_torque = self.current_torque.clamp(-self.max_torque, self.max_torque);
        
        // Simplified speed model
        let accel = self.current_torque / 1.0; // Assume inertia = 1
        self.current_speed += accel * dt;
        self.current_speed = self.current_speed.clamp(-self.max_speed, self.max_speed);
        
        (self.current_torque, self.current_speed)
    }

    pub fn set_command(&mut self, command: f64) {
        self.target_torque = command.clamp(-1.0, 1.0) * self.max_torque;
    }
}

/// Steering actuator
pub struct SteeringActuator {
    pub target_angle: f64,
    pub current_angle: f64,
    pub max_angle: f64,
    pub max_rate: f64,
}

impl SteeringActuator {
    pub fn new(max_angle: f64, max_rate: f64) -> Self {
        Self {
            target_angle: 0.0,
            current_angle: 0.0,
            max_angle,
            max_rate,
        }
    }

    pub fn step(&mut self, dt: f64) -> f64 {
        let error = self.target_angle - self.current_angle;
        let max_change = self.max_rate * dt;
        
        if error.abs() <= max_change {
            self.current_angle = self.target_angle;
        } else {
            self.current_angle += max_change * error.signum();
        }
        
        self.current_angle
    }

    pub fn set_command(&mut self, command: f64) {
        self.target_angle = command.clamp(-1.0, 1.0) * self.max_angle;
    }
}

/// Brake actuator
pub struct BrakeActuator {
    pub target_pressure: f64,
    pub current_pressure: f64,
    pub max_pressure: f64,
    pub response_time: f64,
}

impl BrakeActuator {
    pub fn new(max_pressure: f64, response_time: f64) -> Self {
        Self {
            target_pressure: 0.0,
            current_pressure: 0.0,
            max_pressure,
            response_time,
        }
    }

    pub fn step(&mut self, dt: f64) -> f64 {
        let error = self.target_pressure - self.current_pressure;
        let max_change = (self.max_pressure / self.response_time) * dt;
        
        if error.abs() <= max_change {
            self.current_pressure = self.target_pressure;
        } else {
            self.current_pressure += max_change * error.signum();
        }
        
        self.current_pressure.clamp(0.0, self.max_pressure)
    }

    pub fn set_command(&mut self, command: f64) {
        self.target_pressure = command.clamp(0.0, 1.0) * self.max_pressure;
    }
}
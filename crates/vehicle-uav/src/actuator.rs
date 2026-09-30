//! UAV actuator implementations

use autonomy_common::error::Result;
use autonomy_common::state::{ControlInputs, MotorState};
use autonomy_common::traits::ActuatorSet;
use crate::dynamics::{UavActuators, UavConfig};
use parking_lot::RwLock;
use smallvec::SmallVec;
use std::sync::Arc;

/// High-level UAV actuator interface
pub struct UavActuatorInterface {
    actuators: Arc<UavActuators>,
    pending_controls: RwLock<ControlInputs>,
}

impl UavActuatorInterface {
    pub fn new(config: UavConfig) -> Self {
        Self {
            actuators: Arc::new(UavActuators::new(config)),
            pending_controls: RwLock::new(ControlInputs::default()),
        }
    }

    pub fn actuators(&self) -> &Arc<UavActuators> {
        &self.actuators
    }

    pub fn set_controls(&self, controls: ControlInputs) {
        *self.pending_controls.write() = controls;
    }

    pub fn get_controls(&self) -> ControlInputs {
        *self.pending_controls.read()
    }
}

impl ActuatorSet for UavActuatorInterface {
    fn apply_controls(&mut self, inputs: ControlInputs) -> Result<()> {
        *self.pending_controls.write() = inputs;
        Ok(())
    }

    fn get_motor_states(&self) -> &[MotorState] {
        // In practice, this would be implemented differently
        unimplemented!("Use actuators().dynamics().read().motor_states()")
    }

    fn set_motor_command(&mut self, index: usize, command: f64) -> Result<()> {
        // Direct motor control
        Ok(())
    }

    fn num_actuators(&self) -> usize {
        self.actuators.dynamics.read().config.rotor_count
    }
}

/// Motor controller for individual motors
pub struct MotorController {
    pub target_rpm: f64,
    pub current_rpm: f64,
    pub time_constant: f64,
    pub max_rpm: f64,
    pub thrust_coefficient: f64,
    pub torque_coefficient: f64,
}

impl MotorController {
    pub fn new(time_constant: f64, max_rpm: f64, max_thrust: f64, max_torque: f64) -> Self {
        Self {
            target_rpm: 0.0,
            current_rpm: 0.0,
            time_constant,
            max_rpm,
            thrust_coefficient: max_thrust / (max_rpm * max_rpm),
            torque_coefficient: max_torque / max_thrust,
        }
    }

    pub fn step(&mut self, dt: f64) -> (f64, f64) {
        let error = self.target_rpm - self.current_rpm;
        self.current_rpm += error * (dt / self.time_constant);
        self.current_rpm = self.current_rpm.clamp(0.0, self.max_rpm);
        
        let thrust = self.thrust_coefficient * self.current_rpm * self.current_rpm;
        let torque = thrust * self.torque_coefficient;
        
        (thrust, torque)
    }

    pub fn set_command(&mut self, command: f64) {
        self.target_rpm = (command.clamp(0.0, 1.0)) * self.max_rpm;
    }
}

/// ESC (Electronic Speed Controller) model
pub struct EscModel {
    pub motor: MotorController,
    pub voltage: f64,
    pub current_limit: f64,
    pub temperature: f64,
    pub max_temperature: f64,
}

impl EscModel {
    pub fn new(motor: MotorController, voltage: f64) -> Self {
        Self {
            motor,
            voltage,
            current_limit: 50.0,
            temperature: 25.0,
            max_temperature: 100.0,
        }
    }

    pub fn step(&mut self, dt: f64) -> MotorState {
        let (thrust, torque) = self.motor.step(dt);
        let current = thrust * self.motor.current_rpm / 1000.0 / self.voltage;
        
        // Thermal model
        let power_loss = current * current * 0.01; // Simplified
        self.temperature += power_loss * dt * 0.1;
        self.temperature = self.temperature.min(self.max_temperature);
        
        MotorState {
            rpm: self.motor.current_rpm,
            thrust,
            torque,
            temperature: self.temperature,
            voltage: self.voltage,
            current,
            health: if self.temperature > self.max_temperature * 0.9 { 0.5 } else { 1.0 },
        }
    }
}
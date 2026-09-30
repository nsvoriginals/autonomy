//! Base vehicle implementation

use autonomy_common::error::Result;
use autonomy_common::frames::{LinearAcceleration, LinearVelocity, Pose, Vec3};
use autonomy_common::ids::{SensorId, VehicleId, VehicleType};
use autonomy_common::state::{AlgorithmInput, AlgorithmOutput, BatteryState, ControlInputs, EstimatedState, HealthStatus, MissionState, SensorMeasurements, VehicleState};
use autonomy_common::time::SimTime;
use autonomy_common::traits::{ActuatorSet, AutonomyModule, Controller, Sensor, SensorSuite, SimModule, StateEstimator, Vehicle};
use parking_lot::RwLock;
use smallvec::SmallVec;
use std::sync::Arc;

/// Base vehicle implementation
pub struct BaseVehicle {
    id: VehicleId,
    vehicle_type: VehicleType,
    state: RwLock<VehicleState>,
    sensors: Arc<dyn SensorSuite>,
    actuators: Arc<dyn ActuatorSet>,
    estimator: Arc<dyn StateEstimator>,
    planner: Arc<dyn autonomy_planning::Planner>,
    controller: Arc<dyn Controller>,
    autonomy: Arc<dyn AutonomyModule>,
    physics_handle: Option<autonomy_physics::bodies::BodyHandle>,
}

impl BaseVehicle {
    pub fn new(
        id: VehicleId,
        vehicle_type: VehicleType,
        sensors: Arc<dyn SensorSuite>,
        actuators: Arc<dyn ActuatorSet>,
        estimator: Arc<dyn StateEstimator>,
        planner: Arc<dyn autonomy_planning::Planner>,
        controller: Arc<dyn Controller>,
        autonomy: Arc<dyn AutonomyModule>,
    ) -> Self {
        let mut state = VehicleState::new(id, vehicle_type);
        state.pose = Pose::identity();
        
        Self {
            id,
            vehicle_type,
            state: RwLock::new(state),
            sensors,
            actuators,
            estimator,
            planner,
            controller,
            autonomy,
            physics_handle: None,
        }
    }

    pub fn set_physics_handle(&mut self, handle: autonomy_physics::bodies::BodyHandle) {
        self.physics_handle = Some(handle);
    }

    pub fn physics_handle(&self) -> Option<autonomy_physics::bodies::BodyHandle> {
        self.physics_handle
    }

    pub fn update_from_physics(&self, pose: Pose, linear_vel: LinearVelocity, angular_vel: Vec3, linear_accel: LinearAcceleration) {
        let mut state = self.state.write();
        state.pose = pose;
        state.linear_velocity = linear_vel;
        state.angular_velocity = autonomy_common::frames::AngularVelocity(angular_vel);
        state.linear_acceleration = linear_accel;
        state.time = SimTime::ZERO; // Will be updated by simulation
    }

    pub fn apply_control(&self, inputs: ControlInputs) -> Result<()> {
        self.actuators.apply_controls(inputs)
    }

    pub fn get_motor_states(&self) -> SmallVec<[autonomy_common::state::MotorState; 8]> {
        self.actuators.get_motor_states().into()
    }
}

impl Vehicle for BaseVehicle {
    fn id(&self) -> VehicleId {
        self.id
    }

    fn vehicle_type(&self) -> VehicleType {
        self.vehicle_type
    }

    fn state(&self) -> &VehicleState {
        // This is a limitation - we can't return a reference to RwLock read guard
        // In practice, we'd use a different pattern
        unimplemented!("Use state_read() or state_write()")
    }

    fn sensors(&self) -> &dyn SensorSuite {
        self.sensors.as_ref()
    }

    fn actuators(&self) -> &dyn ActuatorSet {
        self.actuators.as_ref()
    }

    fn estimator(&self) -> &dyn StateEstimator {
        self.estimator.as_ref()
    }

    fn planner(&self) -> &dyn autonomy_planning::Planner {
        self.planner.as_ref()
    }

    fn controller(&self) -> &dyn Controller {
        self.controller.as_ref()
    }

    fn autonomy(&self) -> &dyn AutonomyModule {
        self.autonomy.as_ref()
    }

    fn health(&self) -> HealthStatus {
        self.state.read().health
    }

    fn step(&mut self, dt: f64, estimation: &EstimatedState) -> Result<()> {
        // Update sensors
        let measurements = self.sensors.update_all(estimation.time, &self.state.read().clone())?;
        
        // Update estimator
        let estimated = self.estimator.estimate(estimation.time, &measurements)?;
        
        // Run autonomy
        let algorithm_input = AlgorithmInput {
            time: estimation.time,
            vehicle_id: self.id,
            estimated_state: estimated.clone(),
            sensor_measurements: measurements,
            neighbors: Vec::new(), // Filled by fleet
            mission_state: MissionState {
                mission_id: autonomy_common::ids::MissionId::nil(),
                current_task: None,
                task_progress: 0.0,
                waypoint_index: 0,
                state: autonomy_common::state::MissionExecutionState::Executing,
                parameters: Default::default(),
            },
            environment: autonomy_common::state::EnvironmentSnapshot {
                time: estimation.time,
                wind: Vec3::zeros(),
                gravity: 9.80665,
                magnetic_field: Vec3::zeros(),
                no_fly_zones: Vec::new(),
                obstacles: Vec::new(),
                terrain_height: None,
            },
            config: autonomy_common::state::AlgorithmConfig::default(),
        };
        
        let output = self.autonomy.tick(&algorithm_input)?;
        
        // Plan trajectory
        let trajectory = if let Some(traj) = output.desired_trajectory {
            traj
        } else {
            self.planner.plan(&estimated, &MissionState::default())?
        };
        
        // Compute control
        let control = self.controller.compute_control(&estimated, &trajectory)?;
        
        // Apply control
        self.actuators.apply_controls(control)?;
        
        // Update state
        let mut state = self.state.write();
        state.control_inputs = control;
        state.time = estimation.time;
        
        Ok(())
    }
}

/// Vehicle builder for constructing vehicles
pub struct VehicleBuilder {
    id: Option<VehicleId>,
    vehicle_type: VehicleType,
    sensors: Option<Arc<dyn SensorSuite>>,
    actuators: Option<Arc<dyn ActuatorSet>>,
    estimator: Option<Arc<dyn StateEstimator>>,
    planner: Option<Arc<dyn autonomy_planning::Planner>>,
    controller: Option<Arc<dyn Controller>>,
    autonomy: Option<Arc<dyn AutonomyModule>>,
}

impl VehicleBuilder {
    pub fn new() -> Self {
        Self {
            id: None,
            vehicle_type: VehicleType::Unknown,
            sensors: None,
            actuators: None,
            estimator: None,
            planner: None,
            controller: None,
            autonomy: None,
        }
    }

    pub fn id(mut self, id: VehicleId) -> Self {
        self.id = Some(id);
        self
    }

    pub fn vehicle_type(mut self, vtype: VehicleType) -> Self {
        self.vehicle_type = vtype;
        self
    }

    pub fn sensors(mut self, sensors: Arc<dyn SensorSuite>) -> Self {
        self.sensors = Some(sensors);
        self
    }

    pub fn actuators(mut self, actuators: Arc<dyn ActuatorSet>) -> Self {
        self.actuators = Some(actuators);
        self
    }

    pub fn estimator(mut self, estimator: Arc<dyn StateEstimator>) -> Self {
        self.estimator = Some(estimator);
        self
    }

    pub fn planner(mut self, planner: Arc<dyn autonomy_planning::Planner>) -> Self {
        self.planner = Some(planner);
        self
    }

    pub fn controller(mut self, controller: Arc<dyn Controller>) -> Self {
        self.controller = Some(controller);
        self
    }

    pub fn autonomy(mut self, autonomy: Arc<dyn AutonomyModule>) -> Self {
        self.autonomy = Some(autonomy);
        self
    }

    pub fn build(self) -> Result<BaseVehicle> {
        Ok(BaseVehicle::new(
            self.id.unwrap_or_default(),
            self.vehicle_type,
            self.sensors.ok_or_else(|| autonomy_common::error::AutonomyError::Vehicle("Sensors required".into()))?,
            self.actuators.ok_or_else(|| autonomy_common::error::AutonomyError::Vehicle("Actuators required".into()))?,
            self.estimator.ok_or_else(|| autonomy_common::error::AutonomyError::Vehicle("Estimator required".into()))?,
            self.planner.ok_or_else(|| autonomy_common::error::AutonomyError::Vehicle("Planner required".into()))?,
            self.controller.ok_or_else(|| autonomy_common::error::AutonomyError::Vehicle("Controller required".into()))?,
            self.autonomy.ok_or_else(|| autonomy_common::error::AutonomyError::Vehicle("Autonomy required".into()))?,
        ))
    }
}

impl Default for VehicleBuilder {
    fn default() -> Self {
        Self::new()
    }
}
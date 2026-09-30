//! State machine infrastructure for missions, vehicles, and fleet

use std::collections::HashMap;
use std::fmt::Debug;

/// Generic state machine trait
pub trait StateMachine: Send + Sync {
    type State: Clone + Debug + PartialEq + Eq + Hash + Send + Sync + 'static;
    type Event: Clone + Debug + Send + Sync + 'static;
    type Context: Send + Sync;

    fn current_state(&self) -> &Self::State;
    fn transition(&mut self, event: Self::Event, context: &mut Self::Context) -> TransitionResult<Self::State>;
    fn can_transition(&self, from: &Self::State, to: &Self::State) -> bool;
}

/// Result of a state transition
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TransitionResult<S> {
    Transitioned(S),
    NoChange,
    InvalidTransition { from: S, to: S },
    GuardFailed { from: S, reason: String },
}

/// Simple state machine implementation
pub struct SimpleStateMachine<S, E, C> {
    current: S,
    transitions: HashMap<(S, E), S>,
    guards: HashMap<(S, E), Box<dyn Fn(&C) -> bool + Send + Sync>>,
}

impl<S, E, C> SimpleStateMachine<S, E, C>
where
    S: Clone + Debug + PartialEq + Eq + Hash + Send + Sync + 'static,
    E: Clone + Debug + PartialEq + Eq + Hash + Send + Sync + 'static,
    C: Send + Sync,
{
    pub fn new(initial: S) -> Self {
        Self {
            current: initial,
            transitions: HashMap::new(),
            guards: HashMap::new(),
        }
    }

    pub fn add_transition(&mut self, from: S, event: E, to: S) {
        self.transitions.insert((from, event), to);
    }

    pub fn add_guarded_transition<F>(&mut self, from: S, event: E, to: S, guard: F)
    where
        F: Fn(&C) -> bool + Send + Sync + 'static,
    {
        self.transitions.insert((from.clone(), event.clone()), to);
        self.guards.insert((from, event), Box::new(guard));
    }

    pub fn current(&self) -> &S {
        &self.current
    }
}

impl<S, E, C> StateMachine for SimpleStateMachine<S, E, C>
where
    S: Clone + Debug + PartialEq + Eq + Hash + Send + Sync + 'static,
    E: Clone + Debug + PartialEq + Eq + Hash + Send + Sync + 'static,
    C: Send + Sync,
{
    type State = S;
    type Event = E;
    type Context = C;

    fn current_state(&self) -> &Self::State {
        &self.current
    }

    fn transition(&mut self, event: Self::Event, context: &mut Self::Context) -> TransitionResult<Self::State> {
        let from = self.current.clone();
        
        if let Some(to) = self.transitions.get(&(from.clone(), event.clone())) {
            // Check guard if present
            if let Some(guard) = self.guards.get(&(from.clone(), event)) {
                if !guard(context) {
                    return TransitionResult::GuardFailed { from, reason: "Guard condition failed".into() };
                }
            }

            if self.can_transition(&from, to) {
                self.current = to.clone();
                return TransitionResult::Transitioned(to.clone());
            } else {
                return TransitionResult::InvalidTransition { from, to: to.clone() };
            }
        }

        TransitionResult::NoChange
    }

    fn can_transition(&self, _from: &Self::State, _to: &Self::State) -> bool {
        true // Override in subtypes for custom logic
    }
}

/// Mission state machine
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum MissionState {
    Planned,
    Assigned,
    Executing,
    Paused,
    Completed,
    Failed,
    Aborted,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum MissionEvent {
    Assign,
    Start,
    Pause,
    Resume,
    Complete,
    Fail,
    Abort,
    Reassign,
}

pub type MissionStateMachine = SimpleStateMachine<MissionState, MissionEvent, MissionContext>;

#[derive(Clone, Debug, Default)]
pub struct MissionContext {
    pub vehicle_id: Option<crate::ids::VehicleId>,
    pub battery_level: f64,
    pub communication_ok: bool,
    pub task_progress: f64,
}

impl MissionStateMachine {
    pub fn new() -> Self {
        let mut sm = Self::new(MissionState::Planned);
        
        // Planned -> Assigned
        sm.add_transition(MissionState::Planned, MissionEvent::Assign, MissionState::Assigned);
        
        // Assigned -> Executing
        sm.add_transition(MissionState::Assigned, MissionEvent::Start, MissionState::Executing);
        
        // Executing -> Paused
        sm.add_transition(MissionState::Executing, MissionEvent::Pause, MissionState::Paused);
        
        // Paused -> Executing
        sm.add_transition(MissionState::Paused, MissionEvent::Resume, MissionState::Executing);
        
        // Executing -> Completed
        sm.add_transition(MissionState::Executing, MissionEvent::Complete, MissionState::Completed);
        
        // Any -> Failed
        sm.add_transition(MissionState::Planned, MissionEvent::Fail, MissionState::Failed);
        sm.add_transition(MissionState::Assigned, MissionEvent::Fail, MissionState::Failed);
        sm.add_transition(MissionState::Executing, MissionEvent::Fail, MissionState::Failed);
        sm.add_transition(MissionState::Paused, MissionEvent::Fail, MissionState::Failed);
        
        // Any -> Aborted
        sm.add_transition(MissionState::Planned, MissionEvent::Abort, MissionState::Aborted);
        sm.add_transition(MissionState::Assigned, MissionEvent::Abort, MissionState::Aborted);
        sm.add_transition(MissionState::Executing, MissionEvent::Abort, MissionState::Aborted);
        sm.add_transition(MissionState::Paused, MissionEvent::Abort, MissionState::Aborted);
        
        // Reassign from failed/aborted back to assigned
        sm.add_transition(MissionState::Failed, MissionEvent::Reassign, MissionState::Assigned);
        sm.add_transition(MissionState::Aborted, MissionEvent::Reassign, MissionState::Assigned);

        sm
    }
}

/// Vehicle health state machine
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum VehicleHealthState {
    Nominal,
    Degraded,
    Critical,
    Failed,
    Recovering,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum HealthEvent {
    Degrade,
    Critical,
    Fail,
    Recover,
    Restore,
}

pub type VehicleHealthStateMachine = SimpleStateMachine<VehicleHealthState, HealthEvent, HealthContext>;

#[derive(Clone, Debug, Default)]
pub struct HealthContext {
    pub battery_critical: bool,
    pub sensor_failures: u32,
    pub actuator_failures: u32,
    pub communication_lost: bool,
    pub autonomy_errors: u32,
}

impl VehicleHealthStateMachine {
    pub fn new() -> Self {
        let mut sm = Self::new(VehicleHealthState::Nominal);
        
        sm.add_guarded_transition(
            VehicleHealthState::Nominal, 
            HealthEvent::Degrade, 
            VehicleHealthState::Degraded,
            |ctx| ctx.sensor_failures > 0 || ctx.battery_critical || ctx.autonomy_errors > 0
        );
        
        sm.add_guarded_transition(
            VehicleHealthState::Degraded, 
            HealthEvent::Critical, 
            VehicleHealthState::Critical,
            |ctx| ctx.actuator_failures > 0 || ctx.communication_lost
        );
        
        sm.add_guarded_transition(
            VehicleHealthState::Critical, 
            HealthEvent::Fail, 
            VehicleHealthState::Failed,
            |ctx| ctx.actuator_failures > 1 || ctx.battery_critical
        );
        
        sm.add_guarded_transition(
            VehicleHealthState::Failed, 
            HealthEvent::Recover, 
            VehicleHealthState::Recovering,
            |ctx| !ctx.battery_critical && ctx.actuator_failures == 0
        );
        
        sm.add_transition(VehicleHealthState::Recovering, HealthEvent::Restore, VehicleHealthState::Nominal);
        sm.add_transition(VehicleHealthState::Degraded, HealthEvent::Restore, VehicleHealthState::Nominal);
        sm.add_transition(VehicleHealthState::Critical, HealthEvent::Recover, VehicleHealthState::Recovering);

        sm
    }
}

/// Communication state machine
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum CommState {
    Connected,
    Degraded,
    Disconnected,
    Reconnecting,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum CommEvent {
    QualityDrop,
    QualityRestore,
    Disconnect,
    ReconnectAttempt,
    ReconnectSuccess,
    ReconnectFailed,
}

pub type CommStateMachine = SimpleStateMachine<CommState, CommEvent, CommContext>;

#[derive(Clone, Debug, Default)]
pub struct CommContext {
    pub packet_loss_rate: f64,
    pub latency_ms: f64,
    pub consecutive_failures: u32,
    pub max_retries: u32,
}

impl CommStateMachine {
    pub fn new() -> Self {
        let mut sm = Self::new(CommState::Connected);
        
        sm.add_guarded_transition(
            CommState::Connected, 
            CommEvent::QualityDrop, 
            CommState::Degraded,
            |ctx| ctx.packet_loss_rate > 0.1 || ctx.latency_ms > 500.0
        );
        
        sm.add_transition(CommState::Degraded, CommEvent::QualityRestore, CommState::Connected);
        sm.add_transition(CommState::Degraded, CommEvent::Disconnect, CommState::Disconnected);
        sm.add_transition(CommState::Connected, CommEvent::Disconnect, CommState::Disconnected);
        
        sm.add_transition(CommState::Disconnected, CommEvent::ReconnectAttempt, CommState::Reconnecting);
        sm.add_guarded_transition(
            CommState::Reconnecting, 
            CommEvent::ReconnectSuccess, 
            CommState::Connected,
            |ctx| ctx.consecutive_failures == 0
        );
        sm.add_transition(CommState::Reconnecting, CommEvent::ReconnectFailed, CommState::Disconnected);

        sm
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mission_state_machine() {
        let mut sm = MissionStateMachine::new();
        let mut ctx = MissionContext::default();

        assert_eq!(*sm.current_state(), MissionState::Planned);

        let result = sm.transition(MissionEvent::Assign, &mut ctx);
        assert_eq!(result, TransitionResult::Transitioned(MissionState::Assigned));

        let result = sm.transition(MissionEvent::Start, &mut ctx);
        assert_eq!(result, TransitionResult::Transitioned(MissionState::Executing));

        let result = sm.transition(MissionEvent::Pause, &mut ctx);
        assert_eq!(result, TransitionResult::Transitioned(MissionState::Paused));

        let result = sm.transition(MissionEvent::Resume, &mut ctx);
        assert_eq!(result, TransitionResult::Transitioned(MissionState::Executing));

        let result = sm.transition(MissionEvent::Complete, &mut ctx);
        assert_eq!(result, TransitionResult::Transitioned(MissionState::Completed));
    }

    #[test]
    fn test_health_state_machine() {
        let mut sm = VehicleHealthStateMachine::new();
        let mut ctx = HealthContext::default();

        assert_eq!(*sm.current_state(), VehicleHealthState::Nominal);

        ctx.sensor_failures = 1;
        let result = sm.transition(HealthEvent::Degrade, &mut ctx);
        assert_eq!(result, TransitionResult::Transitioned(VehicleHealthState::Degraded));

        ctx.actuator_failures = 1;
        let result = sm.transition(HealthEvent::Critical, &mut ctx);
        assert_eq!(result, TransitionResult::Transitioned(VehicleHealthState::Critical));

        ctx.battery_critical = true;
        let result = sm.transition(HealthEvent::Fail, &mut ctx);
        assert_eq!(result, TransitionResult::Transitioned(VehicleHealthState::Failed));
    }

    #[test]
    fn test_comm_state_machine() {
        let mut sm = CommStateMachine::new();
        let mut ctx = CommContext::default();

        assert_eq!(*sm.current_state(), CommState::Connected);

        ctx.packet_loss_rate = 0.2;
        let result = sm.transition(CommEvent::QualityDrop, &mut ctx);
        assert_eq!(result, TransitionResult::Transitioned(CommState::Degraded));

        let result = sm.transition(CommEvent::Disconnect, &mut ctx);
        assert_eq!(result, TransitionResult::Transitioned(CommState::Disconnected));

        ctx.consecutive_failures = 0;
        let result = sm.transition(CommEvent::ReconnectAttempt, &mut ctx);
        assert_eq!(result, TransitionResult::Transitioned(CommState::Reconnecting));

        let result = sm.transition(CommEvent::ReconnectSuccess, &mut ctx);
        assert_eq!(result, TransitionResult::Transitioned(CommState::Connected));
    }
}
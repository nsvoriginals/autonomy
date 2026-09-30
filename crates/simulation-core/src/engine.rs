//! Main simulation engine with fixed-timestep loop

use autonomy_common::error::Result;
use autonomy_common::time::{SimClock, SimTime};
use autonomy_common::traits::{EventBus, Recorder, SimModule, TimeSource};
use parking_lot::RwLock;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Instant;
use tracing::{debug, info, warn};

/// Simulation engine state
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum SimulationState {
    Uninitialized,
    Initialized,
    Running,
    Paused,
    Stepping,
    Stopped,
    Error,
}

/// Simulation configuration
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct SimulationConfig {
    pub fixed_dt: f64,
    pub max_substeps: u32,
    pub real_time_mode: bool,
    pub time_scale: f64,
    pub deterministic: bool,
    pub seed: u64,
    pub max_steps: Option<u64>,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            fixed_dt: 1.0 / 60.0,
            max_substeps: 5,
            real_time_mode: false,
            time_scale: 1.0,
            deterministic: true,
            seed: 0,
            max_steps: None,
        }
    }
}

/// Simulation statistics
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct SimulationStats {
    pub total_steps: u64,
    pub total_time: f64,
    pub physics_time_us: u64,
    pub sensor_time_us: u64,
    pub estimation_time_us: u64,
    pub planning_time_us: u64,
    pub control_time_us: u64,
    pub vehicle_update_time_us: u64,
    pub telemetry_time_us: u64,
    pub render_time_us: u64,
    pub avg_step_time_us: f64,
    pub max_step_time_us: u64,
}

/// Main simulation engine
pub struct SimulationEngine {
    config: SimulationConfig,
    clock: SimClock,
    state: SimulationState,
    modules: HashMap<String, Box<dyn SimModule>>,
    event_bus: Arc<dyn EventBus>,
    recorder: Option<Box<dyn Recorder>>,
    stats: SimulationStats,
    step_start: Option<Instant>,
    module_order: Vec<String>,
}

impl SimulationEngine {
    pub fn new(config: SimulationConfig, event_bus: Arc<dyn EventBus>) -> Self {
        let clock = SimClock::new(autonomy_common::time::ClockConfig {
            time_scale: config.time_scale,
            max_substeps: config.max_substeps,
            real_time_mode: config.real_time_mode,
        });

        Self {
            config,
            clock,
            state: SimulationState::Uninitialized,
            modules: HashMap::new(),
            event_bus,
            recorder: None,
            stats: SimulationStats::default(),
            step_start: None,
            module_order: Vec::new(),
        }
    }

    pub fn add_module(&mut self, name: impl Into<String>, module: Box<dyn SimModule>) -> Result<()> {
        let name = name.into();
        if self.modules.contains_key(&name) {
            return Err(autonomy_common::error::AutonomyError::Simulation(
                format!("Module '{}' already exists", name),
            ));
        }
        self.module_order.push(name.clone());
        self.modules.insert(name, module);
        Ok(())
    }

    pub fn remove_module(&mut self, name: &str) -> Option<Box<dyn SimModule>> {
        self.module_order.retain(|n| n != name);
        self.modules.remove(name)
    }

    pub fn set_recorder(&mut self, recorder: Box<dyn Recorder>) {
        self.recorder = Some(recorder);
    }

    pub fn initialize(&mut self) -> Result<()> {
        info!("Initializing simulation engine");
        
        if self.config.deterministic {
            autonomy_common::rng::init_sim_rng(self.config.seed);
        }

        for name in &self.module_order {
            if let Some(module) = self.modules.get_mut(name) {
                debug!("Initializing module: {}", name);
                module.initialize(&autonomy_common::traits::ModuleConfig::default())?;
            }
        }

        self.state = SimulationState::Initialized;
        self.publish_event(autonomy_common::telemetry::TelemetryEvent::SimulationEvent(
            autonomy_common::telemetry::SimulationEvent {
                time: self.clock.time(),
                event_type: autonomy_common::telemetry::SimulationEventType::Started,
                details: Default::default(),
            }
        ));

        Ok(())
    }

    pub fn run(&mut self) -> Result<()> {
        if self.state != SimulationState::Initialized && self.state != SimulationState::Paused {
            return Err(autonomy_common::error::AutonomyError::Simulation(
                "Simulation not initialized".into(),
            ));
        }

        self.state = SimulationState::Running;
        info!("Starting simulation loop");

        while self.should_continue() {
            self.step()?;
        }

        self.state = SimulationState::Stopped;
        info!("Simulation stopped after {} steps", self.stats.total_steps);
        Ok(())
    }

    pub fn step(&mut self) -> Result<()> {
        if self.state == SimulationState::Stopped || self.state == SimulationState::Error {
            return Ok(());
        }

        self.step_start = Some(Instant::now());
        let dt = self.config.fixed_dt;

        // Update clock
        let sim_time = self.clock.tick();

        // Execute modules in order
        for name in &self.module_order.clone() {
            let step_start = Instant::now();
            
            if let Some(module) = self.modules.get_mut(name) {
                module.step(sim_time, dt)?;
            }

            let elapsed = step_start.elapsed().as_micros() as u64;
            self.record_module_time(name, elapsed);
        }

        // Record telemetry
        let telemetry_start = Instant::now();
        self.publish_telemetry(sim_time);
        self.stats.telemetry_time_us += telemetry_start.elapsed().as_micros() as u64;

        // Record to recorder if present
        if let Some(recorder) = &mut self.recorder {
            // Events are published to event bus, recorder subscribes
        }

        let total_elapsed = self.step_start.unwrap().elapsed().as_micros() as u64;
        self.stats.total_steps += 1;
        self.stats.total_time += dt;
        self.stats.avg_step_time_us = 
            (self.stats.avg_step_time_us * (self.stats.total_steps - 1) as f64 + total_elapsed as f64) 
            / self.stats.total_steps as f64;
        self.stats.max_step_time_us = self.stats.max_step_time_us.max(total_elapsed);

        Ok(())
    }

    pub fn pause(&mut self) {
        if self.state == SimulationState::Running {
            self.clock.set_paused(true);
            self.state = SimulationState::Paused;
            self.publish_event(autonomy_common::telemetry::TelemetryEvent::SimulationEvent(
                autonomy_common::telemetry::SimulationEvent {
                    time: self.clock.time(),
                    event_type: autonomy_common::telemetry::SimulationEventType::Paused,
                    details: Default::default(),
                }
            ));
        }
    }

    pub fn resume(&mut self) {
        if self.state == SimulationState::Paused {
            self.clock.set_paused(false);
            self.state = SimulationState::Running;
            self.publish_event(autonomy_common::telemetry::TelemetryEvent::SimulationEvent(
                autonomy_common::telemetry::SimulationEvent {
                    time: self.clock.time(),
                    event_type: autonomy_common::telemetry::SimulationEventType::Resumed,
                    details: Default::default(),
                }
            ));
        }
    }

    pub fn single_step(&mut self) -> Result<()> {
        if self.state == SimulationState::Stopped {
            return Ok(());
        }
        self.state = SimulationState::Stepping;
        self.step()?;
        self.state = SimulationState::Paused;
        Ok(())
    }

    pub fn reset(&mut self) -> Result<()> {
        self.clock.reset();
        self.stats = SimulationStats::default();
        
        for module in self.modules.values_mut() {
            module.shutdown()?;
        }
        
        self.initialize()?;
        Ok(())
    }

    pub fn shutdown(&mut self) -> Result<()> {
        info!("Shutting down simulation engine");
        
        for name in self.module_order.iter().rev() {
            if let Some(module) = self.modules.get_mut(name) {
                module.shutdown()?;
            }
        }

        self.state = SimulationState::Stopped;
        self.publish_event(autonomy_common::telemetry::TelemetryEvent::SimulationEvent(
            autonomy_common::telemetry::SimulationEvent {
                time: self.clock.time(),
                event_type: autonomy_common::telemetry::SimulationEventType::ScenarioCompleted,
                details: Default::default(),
            }
        ));

        Ok(())
    }

    fn should_continue(&self) -> bool {
        if self.state != SimulationState::Running {
            return false;
        }
        if let Some(max) = self.config.max_steps {
            if self.stats.total_steps >= max {
                return false;
            }
        }
        true
    }

    fn record_module_time(&mut self, name: &str, elapsed_us: u64) {
        match name {
            "physics" => self.stats.physics_time_us += elapsed_us,
            "sensors" => self.stats.sensor_time_us += elapsed_us,
            "estimation" => self.stats.estimation_time_us += elapsed_us,
            "planning" => self.stats.planning_time_us += elapsed_us,
            "control" => self.stats.control_time_us += elapsed_us,
            "vehicles" => self.stats.vehicle_update_time_us += elapsed_us,
            _ => {}
        }
    }

    fn publish_telemetry(&self, time: SimTime) {
        // Module-specific telemetry is published by modules themselves
        // This is a hook for engine-level telemetry
    }

    fn publish_event(&self, event: autonomy_common::telemetry::TelemetryEvent) {
        self.event_bus.publish(event);
    }

    pub fn state(&self) -> SimulationState {
        self.state
    }

    pub fn clock(&self) -> &SimClock {
        &self.clock
    }

    pub fn stats(&self) -> &SimulationStats {
        &self.stats
    }

    pub fn config(&self) -> &SimulationConfig {
        &self.config
    }

    pub fn get_module<T: SimModule>(&self, name: &str) -> Option<&T> {
        self.modules.get(name)?.downcast_ref()
    }

    pub fn get_module_mut<T: SimModule>(&mut self, name: &str) -> Option<&mut T> {
        self.modules.get_mut(name)?.downcast_mut()
    }
}

impl TimeSource for SimulationEngine {
    fn now(&self) -> SimTime {
        self.clock.time()
    }

    fn real_time(&self) -> Instant {
        Instant::now()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use autonomy_common::telemetry::TelemetryEvent;
    use autonomy_common::traits::{EventBus, EventSubscription};
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::sync::Arc;

    struct TestEventBus {
        count: Arc<AtomicU32>,
    }

    impl EventBus for TestEventBus {
        fn publish(&self, _event: TelemetryEvent) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }

        fn subscribe(&self, _event_type: &str) -> Box<dyn EventSubscription> {
            Box::new(TestSub)
        }
    }

    struct TestSub;
    impl EventSubscription for TestSub {
        fn next(&mut self) -> Option<TelemetryEvent> { None }
        fn try_next(&mut self) -> Option<TelemetryEvent> { None }
    }

    struct TestModule {
        steps: Arc<AtomicU32>,
    }

    impl SimModule for TestModule {
        fn name(&self) -> &'static str { "test" }
        fn step(&mut self, _time: SimTime, _dt: f64) -> Result<()> {
            self.steps.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }
    }

    #[test]
    fn test_simulation_loop() {
        let event_bus = Arc::new(TestEventBus { count: Arc::new(AtomicU32::new(0)) });
        let mut engine = SimulationEngine::new(SimulationConfig {
            max_steps: Some(10),
            ..Default::default()
        }, event_bus.clone());

        engine.add_module("test", Box::new(TestModule { steps: Arc::new(AtomicU32::new(0)) })).unwrap();
        engine.initialize().unwrap();
        engine.run().unwrap();

        assert_eq!(engine.stats().total_steps, 10);
    }

    #[test]
    fn test_pause_resume() {
        let event_bus = Arc::new(TestEventBus { count: Arc::new(AtomicU32::new(0)) });
        let mut engine = SimulationEngine::new(SimulationConfig::default(), event_bus);
        engine.add_module("test", Box::new(TestModule { steps: Arc::new(AtomicU32::new(0)) })).unwrap();
        engine.initialize().unwrap();

        engine.pause();
        assert_eq!(engine.state(), SimulationState::Paused);

        engine.resume();
        assert_eq!(engine.state(), SimulationState::Running);
    }

    #[test]
    fn test_single_step() {
        let event_bus = Arc::new(TestEventBus { count: Arc::new(AtomicU32::new(0)) });
        let mut engine = SimulationEngine::new(SimulationConfig::default(), event_bus);
        engine.add_module("test", Box::new(TestModule { steps: Arc::new(AtomicU32::new(0)) })).unwrap();
        engine.initialize().unwrap();

        engine.single_step().unwrap();
        assert_eq!(engine.stats().total_steps, 1);
        assert_eq!(engine.state(), SimulationState::Paused);
    }
}
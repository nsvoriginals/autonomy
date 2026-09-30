//! Simulation clock with deterministic time management

use autonomy_common::time::{ClockConfig, SimClock, SimTime};
use autonomy_common::traits::TimeSource;
use parking_lot::RwLock;
use std::sync::Arc;
use std::time::Instant;

/// Shared simulation clock
#[derive(Clone)]
pub struct SharedClock {
    inner: Arc<RwLock<SimClock>>,
}

impl SharedClock {
    pub fn new(config: ClockConfig) -> Self {
        Self {
            inner: Arc::new(RwLock::new(SimClock::new(config))),
        }
    }

    pub fn time(&self) -> SimTime {
        self.inner.read().time()
    }

    pub fn is_paused(&self) -> bool {
        self.inner.read().is_paused()
    }

    pub fn set_paused(&self, paused: bool) {
        self.inner.write().set_paused(paused);
    }

    pub fn toggle_pause(&self) {
        self.inner.write().toggle_pause();
    }

    pub fn step(&self) -> SimTime {
        self.inner.write().step()
    }

    pub fn reset(&self) {
        self.inner.write().reset();
    }

    pub fn tick(&self) -> SimTime {
        self.inner.write().tick()
    }

    pub fn set_time_scale(&self, scale: f64) {
        self.inner.write().set_time_scale(scale);
    }

    pub fn time_scale(&self) -> f64 {
        self.inner.read().time_scale()
    }

    pub fn config(&self) -> ClockConfig {
        self.inner.read().config()
    }

    pub fn set_config(&self, config: ClockConfig) {
        self.inner.write().set_config(config);
    }
}

impl TimeSource for SharedClock {
    fn now(&self) -> SimTime {
        self.time()
    }

    fn real_time(&self) -> Instant {
        Instant::now()
    }
}

impl Default for SharedClock {
    fn default() -> Self {
        Self::new(ClockConfig::default())
    }
}

/// Clock controller for external control (UI, CLI, tests)
pub struct ClockController {
    clock: SharedClock,
}

impl ClockController {
    pub fn new(clock: SharedClock) -> Self {
        Self { clock }
    }

    pub fn pause(&self) {
        self.clock.set_paused(true);
    }

    pub fn resume(&self) {
        self.clock.set_paused(false);
    }

    pub fn single_step(&self) -> SimTime {
        self.clock.step()
    }

    pub fn reset(&self) {
        self.clock.reset();
    }

    pub fn set_time_scale(&self, scale: f64) {
        self.clock.set_time_scale(scale);
    }

    pub fn set_real_time_mode(&self, enabled: bool) {
        let mut config = self.clock.config();
        config.real_time_mode = enabled;
        self.clock.set_config(config);
    }

    pub fn current_time(&self) -> SimTime {
        self.clock.time()
    }

    pub fn is_paused(&self) -> bool {
        self.clock.is_paused()
    }
}
//! Deterministic simulation time

use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{Add, AddAssign, Sub, SubAssign};

/// Fixed simulation timestep: 1/60 seconds = ~16.666ms
pub const FIXED_DT: f64 = 1.0 / 60.0;
pub const FIXED_DT_MS: f64 = 1000.0 / 60.0;

/// Simulation time in ticks (deterministic, integer-based)
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct SimTime {
    ticks: u64,
}

impl SimTime {
    pub const ZERO: SimTime = SimTime { ticks: 0 };

    pub fn new(ticks: u64) -> Self {
        Self { ticks }
    }

    pub fn from_seconds(seconds: f64) -> Self {
        Self {
            ticks: (seconds / FIXED_DT).round() as u64,
        }
    }

    pub fn from_millis(millis: f64) -> Self {
        Self {
            ticks: (millis / FIXED_DT_MS).round() as u64,
        }
    }

    pub fn ticks(&self) -> u64 {
        self.ticks
    }

    pub fn as_seconds(&self) -> f64 {
        self.ticks as f64 * FIXED_DT
    }

    pub fn as_millis(&self) -> f64 {
        self.ticks as f64 * FIXED_DT_MS
    }

    pub fn step(&mut self) {
        self.ticks += 1;
    }

    pub fn stepped(self) -> Self {
        Self { ticks: self.ticks + 1 }
    }

    pub fn add_steps(self, steps: u64) -> Self {
        Self { ticks: self.ticks + steps }
    }

    pub fn saturating_sub_steps(self, steps: u64) -> Self {
        Self { ticks: self.ticks.saturating_sub(steps) }
    }
}

impl Add<u64> for SimTime {
    type Output = Self;
    fn add(self, rhs: u64) -> Self {
        Self { ticks: self.ticks + rhs }
    }
}

impl AddAssign<u64> for SimTime {
    fn add_assign(&mut self, rhs: u64) {
        self.ticks += rhs;
    }
}

impl Sub<u64> for SimTime {
    type Output = Self;
    fn sub(self, rhs: u64) -> Self {
        Self { ticks: self.ticks - rhs }
    }
}

impl SubAssign<u64> for SimTime {
    fn sub_assign(&mut self, rhs: u64) {
        self.ticks -= rhs;
    }
}

impl Sub for SimTime {
    type Output = u64;
    fn sub(self, rhs: Self) -> u64 {
        self.ticks.saturating_sub(rhs.ticks)
    }
}

impl fmt::Debug for SimTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SimTime({:.3}s / {} ticks)", self.as_seconds(), self.ticks)
    }
}

impl fmt::Display for SimTime {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:.3}s", self.as_seconds())
    }
}

impl Default for SimTime {
    fn default() -> Self {
        Self::ZERO
    }
}

/// Simulation clock configuration
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ClockConfig {
    pub time_scale: f64,
    pub max_substeps: u32,
    pub real_time_mode: bool,
}

impl Default for ClockConfig {
    fn default() -> Self {
        Self {
            time_scale: 1.0,
            max_substeps: 1,
            real_time_mode: false,
        }
    }
}

/// Simulation clock - controls time progression
#[derive(Clone, Debug)]
pub struct SimClock {
    time: SimTime,
    config: ClockConfig,
    paused: bool,
    last_real_time: Option<std::time::Instant>,
    accumulator: f64,
}

impl SimClock {
    pub fn new(config: ClockConfig) -> Self {
        Self {
            time: SimTime::ZERO,
            config,
            paused: false,
            last_real_time: None,
            accumulator: 0.0,
        }
    }

    pub fn time(&self) -> SimTime {
        self.time
    }

    pub fn is_paused(&self) -> bool {
        self.paused
    }

    pub fn set_paused(&mut self, paused: bool) {
        self.paused = paused;
        if !paused {
            self.last_real_time = Some(std::time::Instant::now());
        }
    }

    pub fn toggle_pause(&mut self) {
        self.set_paused(!self.paused);
    }

    pub fn step(&mut self) -> SimTime {
        self.time.step();
        self.time
    }

    pub fn reset(&mut self) {
        self.time = SimTime::ZERO;
        self.accumulator = 0.0;
        self.last_real_time = None;
    }

    pub fn set_time_scale(&mut self, scale: f64) {
        self.config.time_scale = scale.max(0.0);
    }

    pub fn time_scale(&self) -> f64 {
        self.config.time_scale
    }

    /// Advance simulation time by one fixed step
    pub fn tick(&mut self) -> SimTime {
        if self.paused {
            return self.time;
        }

        if self.config.real_time_mode {
            let now = std::time::Instant::now();
            if let Some(last) = self.last_real_time {
                let dt_real = now.duration_since(last).as_secs_f64();
                self.accumulator += dt_real * self.config.time_scale;

                let max_accum = FIXED_DT * self.config.max_substeps as f64;
                if self.accumulator > max_accum {
                    self.accumulator = max_accum;
                }

                while self.accumulator >= FIXED_DT {
                    self.time.step();
                    self.accumulator -= FIXED_DT;
                }
            }
            self.last_real_time = Some(now);
        } else {
            self.time.step();
        }

        self.time
    }

    pub fn config(&self) -> ClockConfig {
        self.config
    }

    pub fn set_config(&mut self, config: ClockConfig) {
        self.config = config;
    }
}

impl Default for SimClock {
    fn default() -> Self {
        Self::new(ClockConfig::default())
    }
}
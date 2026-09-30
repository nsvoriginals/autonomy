//! Module scheduler for deterministic execution order

use std::collections::{HashMap, HashSet, VecDeque};

/// Module dependency graph for topological scheduling
pub struct ModuleScheduler {
    dependencies: HashMap<String, Vec<String>>,
    modules: Vec<String>,
}

impl ModuleScheduler {
    pub fn new() -> Self {
        Self {
            dependencies: HashMap::new(),
            modules: Vec::new(),
        }
    }

    pub fn add_module(&mut self, name: impl Into<String>, deps: Vec<String>) {
        let name = name.into();
        self.dependencies.insert(name.clone(), deps);
        if !self.modules.contains(&name) {
            self.modules.push(name);
        }
    }

    pub fn compute_order(&self) -> Result<Vec<String>, ScheduleError> {
        let mut visited = HashSet::new();
        let mut temp = HashSet::new();
        let mut order = Vec::new();

        for module in &self.modules {
            self.visit(module, &mut visited, &mut temp, &mut order)?;
        }

        order.reverse();
        Ok(order)
    }

    fn visit(
        &self,
        module: &str,
        visited: &mut HashSet<String>,
        temp: &mut HashSet<String>,
        order: &mut Vec<String>,
    ) -> Result<(), ScheduleError> {
        if temp.contains(module) {
            return Err(ScheduleError::CycleDetected(module.to_string()));
        }
        if visited.contains(module) {
            return Ok(());
        }

        temp.insert(module.to_string());

        if let Some(deps) = self.dependencies.get(module) {
            for dep in deps {
                self.visit(dep, visited, temp, order)?;
            }
        }

        temp.remove(module);
        visited.insert(module.to_string());
        order.push(module.to_string());

        Ok(())
    }
}

/// Default module execution order for simulation
pub fn default_simulation_order() -> Vec<&'static str> {
    vec![
        "environment",    // Update environment (wind, time of day)
        "physics",        // Physics simulation
        "sensors",        // Sensor updates with noise/latency
        "estimation",     // State estimation
        "planning",       // Path/trajectory planning
        "control",        // Control computation
        "vehicles",       // Vehicle state updates
        "fleet",          // Fleet coordination
        "networking",     // Communication simulation
        "telemetry",      // Telemetry emission
    ]
}

#[derive(Debug, thiserror::Error)]
pub enum ScheduleError {
    #[error("Circular dependency detected involving module: {0}")]
    CycleDetected(String),
    #[error("Module not found: {0}")]
    ModuleNotFound(String),
}

/// Dynamic scheduler that can handle module enable/disable
pub struct DynamicScheduler {
    scheduler: ModuleScheduler,
    enabled: HashSet<String>,
    execution_order: Vec<String>,
}

impl DynamicScheduler {
    pub fn new() -> Self {
        Self {
            scheduler: ModuleScheduler::new(),
            enabled: HashSet::new(),
            execution_order: Vec::new(),
        }
    }

    pub fn register(&mut self, name: impl Into<String>, deps: Vec<String>) {
        let name = name.into();
        self.scheduler.add_module(name.clone(), deps);
        self.enabled.insert(name);
        self.recompute();
    }

    pub fn enable(&mut self, name: &str) -> Result<(), ScheduleError> {
        if !self.scheduler.dependencies.contains_key(name) {
            return Err(ScheduleError::ModuleNotFound(name.to_string()));
        }
        self.enabled.insert(name.to_string());
        self.recompute();
        Ok(())
    }

    pub fn disable(&mut self, name: &str) {
        self.enabled.remove(name);
        self.recompute();
    }

    pub fn is_enabled(&self, name: &str) -> bool {
        self.enabled.contains(name)
    }

    pub fn execution_order(&self) -> &[String] {
        &self.execution_order
    }

    fn recompute(&mut self) {
        let all_modules: Vec<String> = self.enabled.iter().cloned().collect();
        let mut temp_scheduler = ModuleScheduler::new();
        
        for module in &all_modules {
            if let Some(deps) = self.scheduler.dependencies.get(module) {
                let enabled_deps: Vec<String> = deps.iter()
                    .filter(|d| self.enabled.contains(*d))
                    .cloned()
                    .collect();
                temp_scheduler.add_module(module.clone(), enabled_deps);
            }
        }

        self.execution_order = temp_scheduler.compute_order().unwrap_or_default();
    }
}

impl Default for DynamicScheduler {
    fn default() -> Self {
        Self::new()
    }
}

/// Phase-based scheduler for simulation steps
pub struct PhaseScheduler {
    phases: Vec<Phase>,
    current_phase: usize,
}

#[derive(Clone, Debug)]
pub struct Phase {
    pub name: String,
    pub modules: Vec<String>,
    pub parallel: bool,
}

impl PhaseScheduler {
    pub fn new() -> Self {
        Self {
            phases: Vec::new(),
            current_phase: 0,
        }
    }

    pub fn add_phase(&mut self, name: impl Into<String>, modules: Vec<String>, parallel: bool) {
        self.phases.push(Phase {
            name: name.into(),
            modules,
            parallel,
        });
    }

    pub fn default_phases() -> Self {
        let mut scheduler = Self::new();
        scheduler.add_phase("pre_physics", vec!["environment".into()], false);
        scheduler.add_phase("physics", vec!["physics".into()], false);
        scheduler.add_phase("sensors", vec!["sensors".into()], false);
        scheduler.add_phase("estimation", vec!["estimation".into()], false);
        scheduler.add_phase("autonomy", vec!["planning".into(), "control".into()], true);
        scheduler.add_phase("vehicle_update", vec!["vehicles".into()], false);
        scheduler.add_phase("fleet", vec!["fleet".into(), "networking".into()], true);
        scheduler.add_phase("telemetry", vec!["telemetry".into()], false);
        scheduler
    }

    pub fn phases(&self) -> &[Phase] {
        &self.phases
    }

    pub fn current_phase(&self) -> Option<&Phase> {
        self.phases.get(self.current_phase)
    }

    pub fn next_phase(&mut self) -> bool {
        if self.current_phase + 1 < self.phases.len() {
            self.current_phase += 1;
            true
        } else {
            false
        }
    }

    pub fn reset(&mut self) {
        self.current_phase = 0;
    }

    pub fn execute_phase<F>(&self, phase_name: &str, mut f: F) 
    where
        F: FnMut(&str),
    {
        if let Some(phase) = self.phases.iter().find(|p| p.name == phase_name) {
            for module in &phase.modules {
                f(module);
            }
        }
    }
}

impl Default for PhaseScheduler {
    fn default() -> Self {
        Self::default_phases()
    }
}
//! Task allocation strategies

use autonomy_common::error::Result;
use autonomy_common::frames::Vec3;
use autonomy_common::ids::{TaskId, VehicleId, VehicleType};
use autonomy_common::state::{BatteryState, HealthStatus};
use crate::vehicle::registry::{VehicleRegistry, VehicleMetadata, VehicleStatus};

/// Task allocation strategy
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum AllocationStrategy {
    Greedy,
    Hungarian,
    Auction,
    DistanceBased,
    CapabilityBased,
}

/// Task to be allocated
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Task {
    pub id: TaskId,
    pub task_type: String,
    pub position: Vec3,
    pub priority: u8,
    pub required_capabilities: Vec<String>,
    pub estimated_duration: f64,
    pub assigned_vehicle: Option<VehicleId>,
    pub status: TaskStatus,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TaskStatus {
    Pending,
    Assigned,
    InProgress,
    Completed,
    Failed,
    Cancelled,
}

/// Task allocator
pub struct TaskAllocator {
    strategy: AllocationStrategy,
    tasks: dashmap::DashMap<TaskId, Task>,
    registry: std::sync::Arc<VehicleRegistry>,
}

impl TaskAllocator {
    pub fn new(strategy: AllocationStrategy) -> Self {
        Self {
            strategy,
            tasks: dashmap::DashMap::new(),
            registry: std::sync::Arc::new(VehicleRegistry::new()),
        }
    }

    pub fn set_registry(&mut self, registry: std::sync::Arc<VehicleRegistry>) {
        self.registry = registry;
    }

    pub fn set_strategy(&mut self, strategy: AllocationStrategy) {
        self.strategy = strategy;
    }

    pub fn add_task(&self, task: Task) {
        self.tasks.insert(task.id, task);
    }

    pub fn get_task(&self, id: TaskId) -> Option<Task> {
        self.tasks.get(&id).map(|v| v.clone())
    }

    pub fn allocate_task(&self, task_id: TaskId, available_vehicles: &[VehicleId]) -> Result<VehicleId> {
        let mut task = self.tasks.get_mut(&task_id).ok_or_else(|| autonomy_common::error::AutonomyError::Fleet("Task not found".into()))?;
        
        let vehicle_id = match self.strategy {
            AllocationStrategy::Greedy => self.greedy_allocate(&task, available_vehicles),
            AllocationStrategy::DistanceBased => self.distance_based_allocate(&task, available_vehicles),
            AllocationStrategy::CapabilityBased => self.capability_based_allocate(&task, available_vehicles),
            AllocationStrategy::Hungarian => self.greedy_allocate(&task, available_vehicles), // Fallback
            AllocationStrategy::Auction => self.greedy_allocate(&task, available_vehicles), // Fallback
        }?;

        task.assigned_vehicle = Some(vehicle_id);
        task.status = TaskStatus::Assigned;
        
        Ok(vehicle_id)
    }

    pub fn reallocate_task(&self, task_id: TaskId, available_vehicles: &[VehicleId]) -> Result<VehicleId> {
        let mut task = self.tasks.get_mut(&task_id).ok_or_else(|| autonomy_common::error::AutonomyError::Fleet("Task not found".into()))?;
        task.assigned_vehicle = None;
        task.status = TaskStatus::Pending;
        self.allocate_task(task_id, available_vehicles)
    }

    fn greedy_allocate(&self, task: &Task, vehicles: &[VehicleId]) -> Result<VehicleId> {
        vehicles.iter()
            .filter_map(|vid| {
                let meta = self.registry.get(*vid)?;
                if meta.status != VehicleStatus::Ready && meta.status != VehicleStatus::Active {
                    return None;
                }
                if meta.health == HealthStatus::Failed {
                    return None;
                }
                if meta.battery.charge_remaining < 0.2 {
                    return None;
                }
                let dist = (meta.position - task.position).magnitude();
                Some((*vid, dist))
            })
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .map(|(vid, _)| vid)
            .ok_or_else(|| autonomy_common::error::AutonomyError::Fleet("No suitable vehicle".into()))
    }

    fn distance_based_allocate(&self, task: &Task, vehicles: &[VehicleId]) -> Result<VehicleId> {
        self.greedy_allocate(task, vehicles)
    }

    fn capability_based_allocate(&self, task: &Task, vehicles: &[VehicleId]) -> Result<VehicleId> {
        vehicles.iter()
            .filter_map(|vid| {
                let meta = self.registry.get(*vid)?;
                if meta.status != VehicleStatus::Ready && meta.status != VehicleStatus::Active {
                    return None;
                }
                // Check capabilities
                let has_caps = task.required_capabilities.iter()
                    .all(|cap| meta.capabilities.sensor_types.iter().any(|st| format!("{:?}", st) == *cap));
                if !has_caps {
                    return None;
                }
                let dist = (meta.position - task.position).magnitude();
                Some((*vid, dist))
            })
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .map(|(vid, _)| vid)
            .ok_or_else(|| autonomy_common::error::AutonomyError::Fleet("No capable vehicle".into()))
    }

    pub fn complete_task(&self, task_id: TaskId) -> Result<()> {
        let mut task = self.tasks.get_mut(&task_id).ok_or_else(|| autonomy_common::error::AutonomyError::Fleet("Task not found".into()))?;
        task.status = TaskStatus::Completed;
        Ok(())
    }

    pub fn fail_task(&self, task_id: TaskId) -> Result<()> {
        let mut task = self.tasks.get_mut(&task_id).ok_or_else(|| autonomy_common::error::AutonomyError::Fleet("Task not found".into()))?;
        task.status = TaskStatus::Failed;
        Ok(())
    }

    pub fn get_pending_tasks(&self) -> Vec<Task> {
        self.tasks.iter()
            .filter(|t| t.status == TaskStatus::Pending)
            .map(|t| t.clone())
            .collect()
    }
}
//! Example formation algorithm for WASM

use autonomy_common::frames::{Pose, Vec3};
use autonomy_common::ids::{VehicleId, AlgorithmId};
use autonomy_common::state::{AlgorithmInput, AlgorithmOutput, EstimatedState, SensorMeasurements, MissionState, MissionExecutionState, AlgorithmConfig, Trajectory, TrajectoryWaypoint, FleetRequest, MissionTransition, EnvironmentSnapshot, NeighborState};
use autonomy_common::time::SimTime;
use autonomy_wasm_api::abi::{AlgorithmInputV1, AlgorithmOutputV1, AlgorithmError, ABI_VERSION};
use postcard;

struct FormationAlgorithm {
    algorithm_id: AlgorithmId,
    formation_type: FormationType,
    spacing: f64,
    position_in_formation: usize,
    leader_id: Option<VehicleId>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum FormationType {
    Line,
    Vee,
    EchelonLeft,
    EchelonRight,
    Diamond,
}

impl FormationAlgorithm {
    fn new() -> Self {
        Self {
            algorithm_id: AlgorithmId::nil(),
            formation_type: FormationType::Vee,
            spacing: 10.0,
            position_in_formation: 0,
            leader_id: None,
        }
    }

    fn init(&mut self, config: &AlgorithmConfig) {
        self.algorithm_id = config.algorithm_id;
        
        if let Some(ft) = config.parameters.get("formation_type") {
            if let Ok(s) = serde_json::from_value::<String>(ft.clone()) {
                self.formation_type = match s.as_str() {
                    "line" => FormationType::Line,
                    "vee" => FormationType::Vee,
                    "echelon_left" => FormationType::EchelonLeft,
                    "echelon_right" => FormationType::EchelonRight,
                    "diamond" => FormationType::Diamond,
                    _ => FormationType::Vee,
                };
            }
        }
        
        if let Some(spacing) = config.parameters.get("spacing") {
            if let Ok(s) = serde_json::from_value::<f64>(spacing.clone()) {
                self.spacing = s;
            }
        }
        
        if let Some(pos) = config.parameters.get("position") {
            if let Ok(p) = serde_json::from_value::<usize>(pos.clone()) {
                self.position_in_formation = p;
            }
        }
    }

    fn tick(&mut self, input: &AlgorithmInputV1) -> AlgorithmOutputV1 {
        let estimated = &input.estimated_state;
        let pos = estimated.pose.position;
        
        // Find leader
        let leader = input.neighbors.iter()
            .find(|n| Some(n.vehicle_id) == self.leader_id)
            .or_else(|| input.neighbors.first());
        
        let mut output = AlgorithmOutputV1 {
            version: ABI_VERSION,
            desired_trajectory: None,
            desired_velocity: None,
            desired_heading: None,
            mission_transition: None,
            fleet_requests: Vec::new(),
            custom_data: Vec::new(),
        };
        
        if let Some(leader) = leader {
            let leader_pos = leader.pose.position;
            let leader_vel = leader.velocity.0;
            let leader_yaw = {
                let (_, _, yaw) = leader.pose.orientation.euler_angles();
                yaw
            };
            
            // Calculate formation offset
            let offset = self.formation_offset(self.position_in_formation, leader_yaw);
            let target_pos = leader_pos + offset;
            
            // Move towards target position
            let to_target = target_pos - pos;
            let dist = to_target.magnitude();
            
            if dist > 1.0 {
                let speed = (dist * 0.5).min(15.0);
                let velocity = to_target.normalize() * speed;
                let heading = to_target.z.atan2(to_target.x);
                
                output.desired_velocity = Some(velocity);
                output.desired_heading = Some(heading);
                
                // Match leader velocity
                output.desired_velocity = Some(leader_vel + velocity);
            }
        }
        
        output
    }
    
    fn formation_offset(&self, position: usize, leader_yaw: f64) -> Vec3 {
        let (x, z) = match self.formation_type {
            FormationType::Line => {
                let idx = position as f64;
                (-idx * self.spacing, 0.0)
            }
            FormationType::Vee => {
                let row = (position as f64 / 2.0).floor();
                let side = if position % 2 == 0 { 1.0 } else { -1.0 };
                (row * self.spacing * 0.7, side * row * self.spacing)
            }
            FormationType::EchelonLeft => {
                let idx = position as f64;
                (idx * self.spacing, -idx * self.spacing)
            }
            FormationType::EchelonRight => {
                let idx = position as f64;
                (idx * self.spacing, idx * self.spacing)
            }
            FormationType::Diamond => {
                match position {
                    0 => (0.0, 0.0), // Leader
                    1 => (-self.spacing, -self.spacing),
                    2 => (-self.spacing, self.spacing),
                    3 => (-2.0 * self.spacing, 0.0),
                    _ => (0.0, 0.0),
                }
            }
        };
        
        // Rotate by leader yaw
        let cos = leader_yaw.cos();
        let sin = leader_yaw.sin();
        Vec3::new(
            cos * x - sin * z,
            0.0,
            sin * x + cos * z,
        )
    }
}

static mut ALGORITHM: Option<FormationAlgorithm> = None;

#[no_mangle]
pub extern "C" fn algorithm_init(config_ptr: *const u8, config_len: usize) -> i32 {
    let config_slice = unsafe { std::slice::from_raw_parts(config_ptr, config_len) };
    let config: autonomy_common::state::AlgorithmConfig = match postcard::from_bytes(config_slice) {
        Ok(c) => c,
        Err(_) => return AlgorithmError::InvalidInput as i32,
    };
    
    unsafe {
        ALGORITHM = Some(FormationAlgorithm::new());
        if let Some(algo) = ALGORITHM.as_mut() {
            algo.init(&config);
        }
    }
    
    AlgorithmError::Success as i32
}

#[no_mangle]
pub extern "C" fn algorithm_tick(input_ptr: *const u8, input_len: usize, output_ptr: *mut u8, output_len: *mut usize) -> i32 {
    let input_slice = unsafe { std::slice::from_raw_parts(input_ptr, input_len) };
    let input: autonomy_wasm_api::abi::AlgorithmInputV1 = match postcard::from_bytes(input_slice) {
        Ok(i) => i,
        Err(_) => return AlgorithmError::InvalidInput as i32,
    };
    
    let output = unsafe {
        if let Some(algo) = ALGORITHM.as_mut() {
            algo.tick(&input)
        } else {
            return AlgorithmError::NotInitialized as i32;
        }
    };
    
    let output_bytes = postcard::to_stdvec(&output).unwrap_or_default();
    
    if output_bytes.len() > 64 * 1024 {
        return AlgorithmError::MemoryError as i32;
    }
    
    unsafe {
        let out_slice = std::slice::from_raw_parts_mut(output_ptr, output_bytes.len());
        out_slice.copy_from_slice(&output_bytes);
        *output_len = output_bytes.len();
    }
    
    AlgorithmError::Success as i32
}

#[no_mangle]
pub extern "C" fn algorithm_cleanup() {
    unsafe {
        ALGORITHM = None;
    }
}
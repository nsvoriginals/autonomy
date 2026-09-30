//! Example obstacle avoidance algorithm for WASM

use autonomy_common::frames::{Pose, Vec3};
use autonomy_common::ids::{VehicleId, AlgorithmId};
use autonomy_common::state::{AlgorithmInput, AlgorithmOutput, EstimatedState, SensorMeasurements, MissionState, MissionExecutionState, AlgorithmConfig, Trajectory, TrajectoryWaypoint, FleetRequest, MissionTransition, EnvironmentSnapshot, NeighborState, Obstacle, ObstacleType};
use autonomy_common::time::SimTime;
use autonomy_wasm_api::abi::{AlgorithmInputV1, AlgorithmOutputV1, AlgorithmError, ABI_VERSION};
use postcard;

struct AvoidanceAlgorithm {
    algorithm_id: AlgorithmId,
    safety_radius: f64,
    prediction_horizon: f64,
    max_speed: f64,
    max_force: f64,
}

impl AvoidanceAlgorithm {
    fn new() -> Self {
        Self {
            algorithm_id: AlgorithmId::nil(),
            safety_radius: 10.0,
            prediction_horizon: 3.0,
            max_speed: 15.0,
            max_force: 5.0,
        }
    }

    fn init(&mut self, config: &AlgorithmConfig) {
        self.algorithm_id = config.algorithm_id;
        
        if let Some(radius) = config.parameters.get("safety_radius") {
            if let Ok(r) = serde_json::from_value::<f64>(radius.clone()) {
                self.safety_radius = r;
            }
        }
        
        if let Some(horizon) = config.parameters.get("prediction_horizon") {
            if let Ok(h) = serde_json::from_value::<f64>(horizon.clone()) {
                self.prediction_horizon = h;
            }
        }
        
        if let Some(speed) = config.parameters.get("max_speed") {
            if let Ok(s) = serde_json::from_value::<f64>(speed.clone()) {
                self.max_speed = s;
            }
        }
    }

    fn tick(&mut self, input: &AlgorithmInputV1) -> AlgorithmOutputV1 {
        let estimated = &input.estimated_state;
        let pos = estimated.pose.position;
        let vel = estimated.linear_velocity.0;
        
        // Compute avoidance force
        let mut avoid_force = Vec3::zeros();
        
        // Avoid obstacles from environment
        for obstacle in &input.environment.obstacles {
            let diff = pos - obstacle.center;
            let dist = diff.magnitude();
            let effective_radius = obstacle.radius + self.safety_radius;
            
            if dist < effective_radius && dist > 0.1 {
                let force_mag = self.max_force * (1.0 - dist / effective_radius);
                avoid_force += diff.normalize() * force_mag;
            }
        }
        
        // Avoid other vehicles
        for neighbor in &input.neighbors {
            let diff = pos - neighbor.pose.position;
            let dist = diff.magnitude();
            let effective_radius = self.safety_radius * 2.0;
            
            if dist < effective_radius && dist > 0.1 {
                let force_mag = self.max_force * (1.0 - dist / effective_radius);
                avoid_force += diff.normalize() * force_mag;
            }
        }
        
        // Avoid restricted zones
        for zone_id in &input.environment.no_fly_zones {
            // Would look up zone position
        }
        
        let mut output = AlgorithmOutputV1 {
            version: ABI_VERSION,
            desired_trajectory: None,
            desired_velocity: None,
            desired_heading: None,
            mission_transition: None,
            fleet_requests: Vec::new(),
            custom_data: Vec::new(),
        };
        
        // Apply avoidance force to velocity
        let desired_vel = vel + avoid_force * 0.1; // dt = 0.1
        let speed = desired_vel.magnitude();
        
        if speed > self.max_speed {
            output.desired_velocity = Some(desired_vel.normalize() * self.max_speed);
        } else {
            output.desired_velocity = Some(desired_vel);
        }
        
        if speed > 0.1 {
            output.desired_heading = Some(desired_vel.z.atan2(desired_vel.x));
        }
        
        output
    }
}

static mut ALGORITHM: Option<AvoidanceAlgorithm> = None;

#[no_mangle]
pub extern "C" fn algorithm_init(config_ptr: *const u8, config_len: usize) -> i32 {
    let config_slice = unsafe { std::slice::from_raw_parts(config_ptr, config_len) };
    let config: autonomy_common::state::AlgorithmConfig = match postcard::from_bytes(config_slice) {
        Ok(c) => c,
        Err(_) => return AlgorithmError::InvalidInput as i32,
    };
    
    unsafe {
        ALGORITHM = Some(AvoidanceAlgorithm::new());
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
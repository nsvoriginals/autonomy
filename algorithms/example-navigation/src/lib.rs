//! Example navigation algorithm for WASM

use autonomy_common::frames::{Pose, Vec3, LinearVelocity, AngularVelocity, LinearAcceleration};
use autonomy_common::ids::{VehicleId, AlgorithmId, MissionId, TaskId};
use autonomy_common::state::{AlgorithmInput, AlgorithmOutput, EstimatedState, SensorMeasurements, MissionState, MissionExecutionState, AlgorithmConfig, Trajectory, TrajectoryWaypoint, FleetRequest, MissionTransition, EnvironmentSnapshot, NeighborState};
use autonomy_common::time::SimTime;
use autonomy_wasm_api::abi::{AlgorithmInputV1, AlgorithmOutputV1, serialize_input, deserialize_output, AlgorithmError, ABI_VERSION};
use postcard;

// Algorithm state
struct NavigationAlgorithm {
    algorithm_id: AlgorithmId,
    current_waypoint: usize,
    waypoints: Vec<Vec3>,
    reached_threshold: f64,
}

impl NavigationAlgorithm {
    fn new() -> Self {
        Self {
            algorithm_id: AlgorithmId::nil(),
            current_waypoint: 0,
            waypoints: Vec::new(),
            reached_threshold: 5.0,
        }
    }

    fn init(&mut self, config: &AlgorithmConfig) {
        self.algorithm_id = config.algorithm_id;
        
        // Parse waypoints from config
        if let Some(waypoints_json) = config.parameters.get("waypoints") {
            if let Ok(wps) = serde_json::from_value::<Vec<[f64; 3]>>(waypoints_json.clone()) {
                self.waypoints = wps.into_iter().map(|w| Vec3::new(w[0], w[1], w[2])).collect();
            }
        }
        
        if let Some(threshold) = config.parameters.get("reached_threshold") {
            if let Ok(t) = serde_json::from_value::<f64>(threshold.clone()) {
                self.reached_threshold = t;
            }
        }
    }

    fn tick(&mut self, input: &AlgorithmInputV1) -> AlgorithmOutputV1 {
        let estimated = &input.estimated_state;
        let pos = estimated.pose.position;
        
        // Check if current waypoint reached
        if self.current_waypoint < self.waypoints.len() {
            let target = self.waypoints[self.current_waypoint];
            let dist = (pos - target).magnitude();
            
            if dist < self.reached_threshold {
                self.current_waypoint += 1;
            }
        }
        
        // Generate output
        let mut output = AlgorithmOutputV1 {
            version: ABI_VERSION,
            desired_trajectory: None,
            desired_velocity: None,
            desired_heading: None,
            mission_transition: None,
            fleet_requests: Vec::new(),
            custom_data: Vec::new(),
        };
        
        if self.current_waypoint < self.waypoints.len() {
            let target = self.waypoints[self.current_waypoint];
            let to_target = target - pos;
            let dist = to_target.magnitude();
            
            // Simple proportional control
            let speed = (dist * 0.5).min(10.0);
            let velocity = if dist > 0.1 { to_target.normalize() * speed } else { Vec3::zeros() };
            let heading = to_target.z.atan2(to_target.x);
            
            output.desired_velocity = Some(velocity);
            output.desired_heading = Some(heading);
            
            // Also provide trajectory
            let traj = Trajectory {
                waypoints: vec![TrajectoryWaypoint {
                    position: target,
                    velocity,
                    acceleration: Vec3::zeros(),
                    yaw: heading,
                    yaw_rate: 0.0,
                    time_from_start: dist / speed.max(0.1),
                }],
                start_time: input.timestamp,
                duration: dist / speed.max(0.1),
            };
            output.desired_trajectory = Some(traj);
        } else {
            // Mission complete
            output.mission_transition = Some(MissionTransition {
                from: MissionExecutionState::Executing,
                to: MissionExecutionState::Completed,
                reason: "All waypoints reached".into(),
            });
        }
        
        output
    }
}

// Static instance for WASM
static mut ALGORITHM: Option<NavigationAlgorithm> = None;

/// WASM entry point: algorithm_init
#[no_mangle]
pub extern "C" fn algorithm_init(config_ptr: *const u8, config_len: usize) -> i32 {
    let config_slice = unsafe { std::slice::from_raw_parts(config_ptr, config_len) };
    let config: AlgorithmConfig = match postcard::from_bytes(config_slice) {
        Ok(c) => c,
        Err(_) => return AlgorithmError::InvalidInput as i32,
    };
    
    unsafe {
        ALGORITHM = Some(NavigationAlgorithm::new());
        if let Some(algo) = ALGORITHM.as_mut() {
            algo.init(&config);
        }
    }
    
    AlgorithmError::Success as i32
}

/// WASM entry point: algorithm_tick
#[no_mangle]
pub extern "C" fn algorithm_tick(input_ptr: *const u8, input_len: usize, output_ptr: *mut u8, output_len: *mut usize) -> i32 {
    let input_slice = unsafe { std::slice::from_raw_parts(input_ptr, input_len) };
    let input: AlgorithmInputV1 = match postcard::from_bytes(input_slice) {
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

/// WASM entry point: algorithm_cleanup
#[no_mangle]
pub extern "C" fn algorithm_cleanup() {
    unsafe {
        ALGORITHM = None;
    }
}
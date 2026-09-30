//! WASM module instance

use autonomy_common::error::Result;
use autonomy_common::frames::Vec3;
use autonomy_common::ids::{VehicleId, AlgorithmId};
use autonomy_common::state::{AlgorithmInput, AlgorithmOutput, EstimatedState, SensorMeasurements, MissionState, Trajectory, FleetRequest, MissionTransition, EnvironmentSnapshot, AlgorithmConfig, NeighborState};
use autonomy_common::time::SimTime;
use autonomy_wasm_api::abi::{ABI_VERSION, AlgorithmInputV1, AlgorithmOutputV1, serialize_input, deserialize_output, AlgorithmError};
use autonomy_wasm_api::host::HostContext;
use wasmtime::{Engine, Module, Store, Instance, Func, TypedFunc, Memory, Val, ValType, Limits};
use parking_lot::RwLock;
use std::sync::Arc;
use std::time::Duration;

/// WASM module instance
pub struct WasmModuleInstance {
    engine: Engine,
    module: Module,
    store: RwLock<Store<Arc<HostContext>>>,
    instance: Instance,
    memory: Memory,
    tick_func: TypedFunc<(i32, i32, i32, i32), i32>,
    init_func: Option<TypedFunc<(i32, i32), i32>>,
    cleanup_func: Option<TypedFunc<(), ()>>,
    host_context: Arc<HostContext>,
    config: InstanceConfig,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct InstanceConfig {
    pub fuel_limit: u64,
    pub memory_limit: usize,
    pub timeout_ms: u64,
    pub allow_wasi: bool,
}

impl Default for InstanceConfig {
    fn default() -> Self {
        Self {
            fuel_limit: 1_000_000,
            memory_limit: 16 * 1024 * 1024, // 16 MB
            timeout_ms: 10,
            allow_wasi: false,
        }
    }
}

impl WasmModuleInstance {
    pub fn new(
        engine: &Engine,
        wasm_bytes: &[u8],
        vehicle_id: VehicleId,
        algorithm_id: AlgorithmId,
        config: InstanceConfig,
    ) -> Result<Self> {
        let module = Module::new(engine, wasm_bytes)?;
        
        let host_context = Arc::new(HostContext::new(vehicle_id, algorithm_id));
        let mut store = Store::new(engine, host_context.clone());
        
        // Configure limits
        store.limiter(|limiter| {
            limiter
                .with_fuel_limit(config.fuel_limit)
                .with_memory_limit(config.memory_limit as u64)
        })?;
        
        let instance = Instance::new(&mut store, &module, &[])?;
        
        let memory = instance.get_memory(&mut store, "memory")
            .ok_or_else(|| autonomy_common::error::AutonomyError::Wasm("Memory export not found".into()))?;
        
        // Get exported functions
        let tick_func = instance.get_typed_func::<(i32, i32, i32, i32), i32>(&mut store, "algorithm_tick")?;
        
        let init_func = instance.get_typed_func::<(i32, i32), i32>(&mut store, "algorithm_init").ok();
        let cleanup_func = instance.get_typed_func::<(), ()>(&mut store, "algorithm_cleanup").ok();
        
        Ok(Self {
            engine: engine.clone(),
            module,
            store: RwLock::new(store),
            instance,
            memory,
            tick_func,
            init_func,
            cleanup_func,
            host_context,
            config,
        })
    }

    pub fn initialize(&mut self, config: &AlgorithmConfig) -> Result<()> {
        if let Some(init_func) = &self.init_func {
            let config_bytes = postcard::to_stdvec(config)?;
            let mut store = self.store.write();
            
            // Write config to memory
            let ptr = self.allocate(&mut store, config_bytes.len())?;
            self.write_memory(&mut store, ptr, &config_bytes)?;
            
            let result = init_func.call(&mut store, (ptr as i32, config_bytes.len() as i32))?;
            
            self.deallocate(&mut store, ptr)?;
            
            if result != AlgorithmError::Success as i32 {
                return Err(autonomy_common::error::AutonomyError::Wasm(
                    format!("Algorithm init failed with code: {}", result)
                ));
            }
        }
        Ok(())
    }

    pub fn tick(&mut self, input: &AlgorithmInput) -> Result<AlgorithmOutput> {
        let mut store = self.store.write();
        
        // Convert input to V1 format
        let input_v1 = AlgorithmInputV1 {
            version: ABI_VERSION,
            timestamp: input.time,
            vehicle_id: input.vehicle_id,
            estimated_state: input.estimated_state.clone(),
            sensor_measurements: input.sensor_measurements.clone(),
            neighbors: input.neighbors.clone(),
            mission_state: input.mission_state.clone(),
            environment: input.environment.clone(),
            config: input.config.clone(),
        };
        
        // Serialize input
        let input_bytes = serialize_input(&input_v1);
        
        // Allocate input buffer
        let input_ptr = self.allocate(&mut store, input_bytes.len())?;
        self.write_memory(&mut store, input_ptr, &input_bytes)?;
        
        // Allocate output buffer
        let output_ptr = self.allocate(&mut store, 64 * 1024)?;
        let output_len_ptr = self.allocate(&mut store, 4)?;
        
        // Call tick function
        let result = self.tick_func.call(&mut store, (
            input_ptr as i32,
            input_bytes.len() as i32,
            output_ptr as i32,
            output_len_ptr as i32,
        ))?;
        
        if result != AlgorithmError::Success as i32 {
            self.deallocate(&mut store, input_ptr)?;
            self.deallocate(&mut store, output_ptr)?;
            self.deallocate(&mut store, output_len_ptr)?;
            return Err(autonomy_common::error::AutonomyError::Wasm(
                format!("Algorithm tick failed with code: {}", result)
            ));
        }
        
        // Read output length
        let mut len_bytes = [0u8; 4];
        self.read_memory(&mut store, output_len_ptr, &mut len_bytes)?;
        let output_len = u32::from_le_bytes(len_bytes) as usize;
        
        // Read output
        let mut output_bytes = vec![0u8; output_len];
        self.read_memory(&mut store, output_ptr, &mut output_bytes)?;
        
        // Cleanup
        self.deallocate(&mut store, input_ptr)?;
        self.deallocate(&mut store, output_ptr)?;
        self.deallocate(&mut store, output_len_ptr)?;
        
        // Deserialize output
        let output_v1 = deserialize_output(&output_bytes)?;
        
        // Convert back to AlgorithmOutput
        Ok(AlgorithmOutput {
            time: input.time,
            vehicle_id: input.vehicle_id,
            desired_trajectory: output_v1.desired_trajectory,
            desired_velocity: output_v1.desired_velocity,
            desired_heading: output_v1.desired_heading,
            mission_transition: output_v1.mission_transition,
            fleet_requests: output_v1.fleet_requests,
            custom_data: output_v1.custom_data,
        })
    }

    pub fn cleanup(&mut self) -> Result<()> {
        if let Some(cleanup_func) = &self.cleanup_func {
            let mut store = self.store.write();
            cleanup_func.call(&mut store, ())?;
        }
        Ok(())
    }

    fn allocate(&self, store: &mut Store<Arc<HostContext>>, size: usize) -> Result<u32> {
        // Simple bump allocator in WASM memory
        // In a real implementation, this would call a host function or use a proper allocator
        // For now, use a fixed offset
        Ok(autonomy_wasm_api::abi::memory::INPUT_BUFFER)
    }

    fn deallocate(&self, _store: &mut Store<Arc<HostContext>>, _ptr: u32) -> Result<()> {
        // No-op for bump allocator
        Ok(())
    }

    fn write_memory(&self, store: &mut Store<Arc<HostContext>>, ptr: u32, data: &[u8]) -> Result<()> {
        self.memory.write(store, ptr as usize, data)
            .map_err(|e| autonomy_common::error::AutonomyError::Wasm(e.to_string()))
    }

    fn read_memory(&self, store: &mut Store<Arc<HostContext>>, ptr: u32, data: &mut [u8]) -> Result<()> {
        self.memory.read(store, ptr as usize, data)
            .map_err(|e| autonomy_common::error::AutonomyError::Wasm(e.to_string()))
    }

    pub fn host_context(&self) -> &Arc<HostContext> {
        &self.host_context
    }

    pub fn algorithm_id(&self) -> AlgorithmId {
        self.host_context.algorithm_id
    }

    pub fn vehicle_id(&self) -> VehicleId {
        self.host_context.vehicle_id
    }
}
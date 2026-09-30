//! WASM module manager

use autonomy_common::error::Result;
use autonomy_common::ids::{VehicleId, AlgorithmId};
use autonomy_common::state::AlgorithmConfig;
use autonomy_wasm_api::host::HostFunctionRegistry;
use crate::instance::{WasmModuleInstance, InstanceConfig};
use wasmtime::Engine;
use parking_lot::RwLock;
use std::collections::HashMap;
use std::sync::Arc;

/// WASM module manager
pub struct WasmModuleManager {
    engine: Engine,
    registry: HostFunctionRegistry,
    instances: RwLock<HashMap<AlgorithmId, WasmModuleInstance>>,
    default_config: InstanceConfig,
}

impl WasmModuleManager {
    pub fn new(default_config: InstanceConfig) -> Result<Self> {
        let mut engine = Engine::default();
        engine.consume_fuel(true);
        
        Ok(Self {
            engine,
            registry: HostFunctionRegistry::new(),
            instances: RwLock::new(HashMap::new()),
            default_config,
        })
    }

    pub fn load_module(
        &mut self,
        algorithm_id: AlgorithmId,
        vehicle_id: VehicleId,
        wasm_bytes: &[u8],
    ) -> Result<()> {
        let instance = WasmModuleInstance::new(
            &self.engine,
            wasm_bytes,
            vehicle_id,
            algorithm_id,
            self.default_config.clone(),
        )?;
        
        self.registry.register(algorithm_id, vehicle_id);
        self.instances.write().insert(algorithm_id, instance);
        Ok(())
    }

    pub fn load_module_with_config(
        &mut self,
        algorithm_id: AlgorithmId,
        vehicle_id: VehicleId,
        wasm_bytes: &[u8],
        config: InstanceConfig,
    ) -> Result<()> {
        let instance = WasmModuleInstance::new(
            &self.engine,
            wasm_bytes,
            vehicle_id,
            algorithm_id,
            config,
        )?;
        
        self.registry.register(algorithm_id, vehicle_id);
        self.instances.write().insert(algorithm_id, instance);
        Ok(())
    }

    pub fn initialize(&self, algorithm_id: AlgorithmId, config: &AlgorithmConfig) -> Result<()> {
        let mut instances = self.instances.write();
        if let Some(instance) = instances.get_mut(&algorithm_id) {
            instance.initialize(config)
        } else {
            Err(autonomy_common::error::AutonomyError::Wasm("Module not loaded".into()))
        }
    }

    pub fn tick(&self, algorithm_id: AlgorithmId, input: &autonomy_common::state::AlgorithmInput) -> Result<autonomy_common::state::AlgorithmOutput> {
        let mut instances = self.instances.write();
        if let Some(instance) = instances.get_mut(&algorithm_id) {
            instance.tick(input)
        } else {
            Err(autonomy_common::error::AutonomyError::Wasm("Module not loaded".into()))
        }
    }

    pub fn cleanup(&self, algorithm_id: AlgorithmId) -> Result<()> {
        let mut instances = self.instances.write();
        if let Some(mut instance) = instances.remove(&algorithm_id) {
            instance.cleanup()?;
            self.registry.remove_context(algorithm_id);
        }
        Ok(())
    }

    pub fn get_instance(&self, algorithm_id: AlgorithmId) -> Option<WasmModuleInstance> {
        self.instances.read().get(&algorithm_id).cloned()
    }

    pub fn is_loaded(&self, algorithm_id: AlgorithmId) -> bool {
        self.instances.read().contains_key(&algorithm_id)
    }

    pub fn loaded_modules(&self) -> Vec<AlgorithmId> {
        self.instances.read().keys().copied().collect()
    }
}
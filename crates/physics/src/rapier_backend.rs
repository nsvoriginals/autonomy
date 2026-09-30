//! Rapier3D backend integration

use autonomy_common::frames::{LinearAcceleration, LinearVelocity, Pose, Vec3};
use autonomy_common::ids::VehicleId;
use autonomy_common::time::SimTime;
use autonomy_common::traits::SimModule;
use rapier3d::prelude::*;
use std::collections::HashMap;
use std::sync::Arc;

use crate::bodies::{BodyConfig, BodyHandle, PhysicsBody, create_rigid_body, pose_to_isometry, vec3_to_rapier};
use crate::collision::{ColliderConfig, CollisionSystem};
use crate::forces::ForceAccumulator;
use crate::integration::{IntegrationMethod, PhysicsState, integrate};

/// Rapier physics backend
pub struct RapierBackend {
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    bodies: HashMap<BodyHandle, PhysicsBody>,
    body_to_vehicle: HashMap<BodyHandle, VehicleId>,
    vehicle_to_body: HashMap<VehicleId, BodyHandle>,
    force_accumulators: HashMap<BodyHandle, ForceAccumulator>,
    collision_system: CollisionSystem,
    integration_method: IntegrationMethod,
    gravity: Vec3,
    step_count: u64,
}

impl RapierBackend {
    pub fn new(gravity: Vec3) -> Self {
        let integration_parameters = IntegrationParameters::default();
        integration_parameters.dt = 1.0 / 60.0;
        
        Self {
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            integration_parameters,
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            bodies: HashMap::new(),
            body_to_vehicle: HashMap::new(),
            vehicle_to_body: HashMap::new(),
            force_accumulators: HashMap::new(),
            collision_system: CollisionSystem::new(),
            integration_method: IntegrationMethod::SemiImplicitEuler,
            gravity,
            step_count: 0,
        }
    }

    pub fn add_body(&mut self, body: PhysicsBody, pose: Pose, collider_config: ColliderConfig) -> BodyHandle {
        let handle = body.handle;
        let rigid_body = create_rigid_body(&body.config, &pose);
        let rb_handle = self.rigid_body_set.insert(rigid_body);
        
        // Override our handle with Rapier's actual handle
        let actual_handle = BodyHandle::from_raw(rb_handle);
        
        // Create collider
        let collider = collider_config.shape.to_rapier()
            .density(collider_config.density)
            .friction(collider_config.friction)
            .restitution(collider_config.restitution)
            .collision_groups(collider_config.collision_groups)
            .solver_groups(collider_config.solver_groups)
            .sensor(collider_config.is_sensor)
            .active_events(collider_config.active_events)
            .active_hooks(collider_config.active_hooks)
            .build();
        
        self.collider_set.insert_with_parent(collider, rb_handle, &mut self.rigid_body_set);
        
        // Update mappings
        self.bodies.insert(actual_handle, body);
        self.force_accumulators.insert(actual_handle, ForceAccumulator::new());
        
        if let Some(vehicle_id) = self.bodies.get(&actual_handle).and_then(|b| b.vehicle_id) {
            self.vehicle_to_body.insert(vehicle_id, actual_handle);
            self.body_to_vehicle.insert(actual_handle, vehicle_id);
        }
        
        actual_handle
    }

    pub fn remove_body(&mut self, handle: BodyHandle) -> Option<PhysicsBody> {
        let rb_handle = handle.raw();
        
        // Remove colliders attached to this body
        let colliders_to_remove: Vec<_> = self.collider_set.iter()
            .filter(|(_, c)| c.parent() == Some(rb_handle))
            .map(|(h, _)| h)
            .collect();
        
        for collider_handle in colliders_to_remove {
            self.collider_set.remove(collider_handle, &mut self.island_manager, &mut self.rigid_body_set, true);
        }
        
        self.rigid_body_set.remove(rb_handle, &mut self.island_manager, &mut self.collider_set, &mut self.impulse_joint_set, &mut self.multibody_joint_set, true);
        
        self.force_accumulators.remove(&handle);
        self.body_to_vehicle.remove(&handle).map(|v| self.vehicle_to_body.remove(&v));
        self.bodies.remove(&handle)
    }

    pub fn get_body(&self, handle: BodyHandle) -> Option<&PhysicsBody> {
        self.bodies.get(&handle)
    }

    pub fn get_body_mut(&mut self, handle: BodyHandle) -> Option<&mut PhysicsBody> {
        self.bodies.get_mut(&handle)
    }

    pub fn get_body_by_vehicle(&self, vehicle_id: VehicleId) -> Option<BodyHandle> {
        self.vehicle_to_body.get(&vehicle_id).copied()
    }

    pub fn apply_force(&mut self, handle: BodyHandle, force: Vec3, name: &str) {
        if let Some(acc) = self.force_accumulators.get_mut(&handle) {
            acc.add_force(force, name);
        }
    }

    pub fn apply_force_at_point(&mut self, handle: BodyHandle, force: Vec3, point: Vec3, name: &str) {
        if let Some(acc) = self.force_accumulators.get_mut(&handle) {
            acc.add_force_at_point(force, point, name);
        }
    }

    pub fn apply_torque(&mut self, handle: BodyHandle, torque: Vec3, name: &str) {
        if let Some(acc) = self.force_accumulators.get_mut(&handle) {
            acc.add_torque(torque, name);
        }
    }

    pub fn step(&mut self, dt: f64) {
        self.integration_parameters.dt = dt;
        
        // Apply accumulated forces
        for (handle, acc) in &mut self.force_accumulators {
            if let Some(rb) = self.rigid_body_set.get_mut(handle.raw()) {
                rb.add_force(vec3_to_rapier(acc.total_force()), true);
                rb.add_torque(vec3_to_rapier(acc.total_torque()), true);
            }
            acc.clear();
        }

        // Step physics
        self.physics_pipeline.step(
            &self.gravity.into(),
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None, // hooks
            None, // event_handler
        );

        // Process collisions
        self.collision_system.process_contacts(
            &self.narrow_phase,
            &self.rigid_body_set,
            &self.collider_set,
        );

        // Update query pipeline
        self.query_pipeline.update(&self.island_manager, &self.rigid_body_set, &self.collider_set);

        self.step_count += 1;
    }

    pub fn get_state(&self, handle: BodyHandle) -> Option<PhysicsState> {
        let rb = self.rigid_body_set.get(handle.raw())?;
        
        Some(PhysicsState {
            pose: crate::bodies::isometry_to_pose(&rb.position()),
            linear_velocity: LinearVelocity(crate::bodies::rapier_to_vec3(rb.linvel())),
            angular_velocity: crate::bodies::rapier_to_vec3(rb.angvel()),
            linear_acceleration: LinearAcceleration(Vec3::zeros()), // Would need to track
            angular_acceleration: Vec3::zeros(),
            mass: rb.mass(),
            inertia_tensor: *rb.mass_properties().local_inertia_tensor,
            inverse_inertia_tensor: rb.mass_properties().local_inertia_tensor.try_inverse().unwrap_or_default(),
        })
    }

    pub fn set_pose(&mut self, handle: BodyHandle, pose: Pose) {
        if let Some(rb) = self.rigid_body_set.get_mut(handle.raw()) {
            rb.set_position(pose_to_isometry(&pose), true);
        }
    }

    pub fn set_velocity(&mut self, handle: BodyHandle, linear: Vec3, angular: Vec3) {
        if let Some(rb) = self.rigid_body_set.get_mut(handle.raw()) {
            rb.set_linvel(vec3_to_rapier(linear), true);
            rb.set_angvel(vec3_to_rapier(angular), true);
        }
    }

    pub fn is_sleeping(&self, handle: BodyHandle) -> bool {
        self.rigid_body_set.get(handle.raw()).map(|rb| rb.is_sleeping()).unwrap_or(true)
    }

    pub fn wake_up(&mut self, handle: BodyHandle) {
        if let Some(rb) = self.rigid_body_set.get_mut(handle.raw()) {
            rb.wake_up(true);
        }
    }

    pub fn ray_cast(&self, origin: Vec3, direction: Vec3, max_distance: f64) -> Option<(BodyHandle, f64)> {
        let ray = Ray::new(origin.into(), direction.into());
        self.query_pipeline.cast_ray(
            &self.rigid_body_set,
            &self.collider_set,
            &ray,
            max_distance,
            true,
            QueryFilter::default(),
        ).map(|(handle, toi)| (BodyHandle::from_raw(handle), toi))
    }

    pub fn collision_events(&self) -> &[crate::collision::CollisionEvent] {
        self.collision_system.events()
    }

    pub fn step_count(&self) -> u64 {
        self.step_count
    }

    pub fn set_integration_method(&mut self, method: IntegrationMethod) {
        self.integration_method = method;
    }

    pub fn gravity(&self) -> Vec3 {
        self.gravity
    }

    pub fn set_gravity(&mut self, gravity: Vec3) {
        self.gravity = gravity;
    }
}

/// Physics module for simulation
pub struct PhysicsModule {
    backend: RapierBackend,
}

impl PhysicsModule {
    pub fn new(gravity: Vec3) -> Self {
        Self {
            backend: RapierBackend::new(gravity),
        }
    }

    pub fn backend(&self) -> &RapierBackend {
        &self.backend
    }

    pub fn backend_mut(&mut self) -> &mut RapierBackend {
        &mut self.backend
    }

    pub fn add_vehicle(&mut self, vehicle_id: VehicleId, config: BodyConfig, pose: Pose, collider: ColliderConfig) -> BodyHandle {
        let body = PhysicsBody::new(config, Some(vehicle_id));
        self.backend.add_body(body, pose, collider)
    }

    pub fn remove_vehicle(&mut self, vehicle_id: VehicleId) -> Option<PhysicsBody> {
        if let Some(handle) = self.backend.get_body_by_vehicle(vehicle_id) {
            self.backend.remove_body(handle)
        } else {
            None
        }
    }
}

impl SimModule for PhysicsModule {
    fn name(&self) -> &'static str {
        "physics"
    }

    fn step(&mut self, _time: SimTime, dt: f64) -> autonomy_common::error::Result<()> {
        self.backend.step(dt);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bodies::{uav_body_config, ugv_body_config};
    use crate::collision::{uav_collider_config, ugv_collider_config};

    #[test]
    fn test_physics_backend() {
        let mut backend = RapierBackend::new(Vec3::new(0.0, -9.81, 0.0));
        
        let config = uav_body_config(1.0, 0.3);
        let collider = uav_collider_config(0.3, 0.1);
        let pose = Pose::from_translation(Vec3::new(0.0, 10.0, 0.0));
        
        let handle = backend.add_body(PhysicsBody::new(config, None), pose, collider);
        
        // Step a few times
        for _ in 0..10 {
            backend.step(1.0 / 60.0);
        }
        
        let state = backend.get_state(handle).unwrap();
        // Should have fallen
        assert!(state.pose.position.y < 10.0);
    }

    #[test]
    fn test_force_application() {
        let mut backend = RapierBackend::new(Vec3::zeros());
        
        let config = uav_body_config(1.0, 0.3);
        let collider = uav_collider_config(0.3, 0.1);
        let pose = Pose::identity();
        
        let handle = backend.add_body(PhysicsBody::new(config, None), pose, collider);
        
        // Apply upward force to counter gravity
        for _ in 0..60 {
            backend.apply_force(handle, Vec3::new(0.0, 9.81, 0.0), "hover");
            backend.step(1.0 / 60.0);
        }
        
        let state = backend.get_state(handle).unwrap();
        // Should stay near origin
        assert!(state.pose.position.y.abs() < 0.5);
    }
}
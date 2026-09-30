//! Physics body definitions

use autonomy_common::frames::{LinearAcceleration, LinearVelocity, Pose, Vec3};
use autonomy_common::ids::VehicleId;
use nalgebra::{Isometry3, Vector3};
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// Physics body type
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum BodyType {
    Dynamic,
    Kinematic,
    Fixed,
    Sensor,
}

/// Physics body configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BodyConfig {
    pub body_type: BodyType,
    pub mass: f64,
    pub center_of_mass: Vec3,
    pub principal_inertia: Vec3,
    pub linear_damping: f64,
    pub angular_damping: f64,
    pub gravity_scale: f64,
    pub ccd_enabled: bool,
}

impl Default for BodyConfig {
    fn default() -> Self {
        Self {
            body_type: BodyType::Dynamic,
            mass: 1.0,
            center_of_mass: Vec3::zeros(),
            principal_inertia: Vec3::new(1.0, 1.0, 1.0),
            linear_damping: 0.0,
            angular_damping: 0.0,
            gravity_scale: 1.0,
            ccd_enabled: false,
        }
    }
}

/// UAV-specific body config
pub fn uav_body_config(mass: f64, arm_length: f64) -> BodyConfig {
    // Approximate inertia for quadcopter: Ixx = Iyy = 2*m*(arm/2)^2, Izz = 4*m*(arm/2)^2
    let arm_half = arm_length * 0.5;
    let ixx = 2.0 * mass * arm_half * arm_half;
    let iyy = ixx;
    let izz = 4.0 * mass * arm_half * arm_half;
    
    BodyConfig {
        body_type: BodyType::Dynamic,
        mass,
        center_of_mass: Vec3::zeros(),
        principal_inertia: Vec3::new(ixx, iyy, izz),
        linear_damping: 0.1,
        angular_damping: 0.5,
        gravity_scale: 1.0,
        ccd_enabled: true,
    }
}

/// UGV-specific body config
pub fn ugv_body_config(mass: f64, wheelbase: f64, track_width: f64) -> BodyConfig {
    // Approximate inertia for rectangular chassis
    let ixx = mass * (track_width * track_width) / 12.0;
    let iyy = mass * (wheelbase * wheelbase) / 12.0;
    let izz = mass * (wheelbase * wheelbase + track_width * track_width) / 12.0;
    
    BodyConfig {
        body_type: BodyType::Dynamic,
        mass,
        center_of_mass: Vec3::new(0.0, 0.0, -0.1), // Slightly below center
        principal_inertia: Vec3::new(ixx, iyy, izz),
        linear_damping: 0.5,
        angular_damping: 1.0,
        gravity_scale: 1.0,
        ccd_enabled: false,
    }
}

/// Physics body handle in the simulation
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct BodyHandle(pub u64);

impl BodyHandle {
    pub fn new() -> Self {
        Self(Uuid::new_v4().as_u64_pair().0)
    }

    pub fn from_raw(handle: RigidBodyHandle) -> Self {
        Self(handle.0 as u64)
    }

    pub fn raw(&self) -> RigidBodyHandle {
        RigidBodyHandle(self.0 as u32)
    }
}

impl Default for BodyHandle {
    fn default() -> Self {
        Self::new()
    }
}

/// Vehicle physics state
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct VehiclePhysicsState {
    pub handle: BodyHandle,
    pub pose: Pose,
    pub linear_velocity: LinearVelocity,
    pub angular_velocity: Vec3,
    pub linear_acceleration: LinearAcceleration,
    pub forces: Vec3,
    pub torques: Vec3,
    pub contacts: Vec<ContactInfo>,
    pub sleeping: bool,
}

/// Contact information
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ContactInfo {
    pub other_body: BodyHandle,
    pub point: Vec3,
    pub normal: Vec3,
    pub penetration: f64,
    pub impulse: Vec3,
}

/// Physics body component for ECS
#[derive(Clone, Debug)]
pub struct PhysicsBody {
    pub handle: BodyHandle,
    pub config: BodyConfig,
    pub vehicle_id: Option<VehicleId>,
    pub user_data: u128,
}

impl PhysicsBody {
    pub fn new(config: BodyConfig, vehicle_id: Option<VehicleId>) -> Self {
        Self {
            handle: BodyHandle::new(),
            config,
            vehicle_id,
            user_data: 0,
        }
    }

    pub fn set_user_data(&mut self, data: u128) {
        self.user_data = data;
    }
}

/// Convert autonomy pose to Rapier isometry
pub fn pose_to_isometry(pose: &Pose) -> Isometry3<f64> {
    Isometry3::from_parts(
        pose.position.into(),
        pose.orientation.into(),
    )
}

/// Convert Rapier isometry to autonomy pose
pub fn isometry_to_pose(iso: &Isometry3<f64>) -> Pose {
    Pose::new(
        iso.translation.vector.into(),
        UnitQuaternion::from_quaternion(iso.rotation.into()),
    )
}

/// Convert autonomy Vec3 to Rapier Vector3
pub fn vec3_to_rapier(v: Vec3) -> Vector3<f64> {
    Vector3::new(v.x, v.y, v.z)
}

/// Convert Rapier Vector3 to autonomy Vec3
pub fn rapier_to_vec3(v: Vector3<f64>) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

/// Create Rapier rigid body from config
pub fn create_rigid_body(config: &BodyConfig, pose: &Pose) -> RigidBody {
    let mut body = match config.body_type {
        BodyType::Dynamic => RigidBodyBuilder::dynamic(),
        BodyType::Kinematic => RigidBodyBuilder::kinematic_position_based(),
        BodyType::Fixed => RigidBodyBuilder::fixed(),
        BodyType::Sensor => RigidBodyBuilder::dynamic().can_sleep(false),
    }
    .position(pose_to_isometry(pose))
    .mass(config.mass)
    .linear_damping(config.linear_damping)
    .angular_damping(config.angular_damping)
    .gravity_scale(config.gravity_scale)
    .ccd_enabled(config.ccd_enabled);

    if config.center_of_mass != Vec3::zeros() {
        body = body.center_of_mass(vec3_to_rapier(config.center_of_mass).into());
    }

    if config.principal_inertia != Vec3::new(1.0, 1.0, 1.0) {
        body = body.additional_mass_properties(
            rapier3d::prelude::MassProperties::new(
                config.mass,
                vec3_to_rapier(config.center_of_mass),
                nalgebra::Matrix3::from_diagonal(&vec3_to_rapier(config.principal_inertia)),
            )
        );
    }

    body.build()
}

use nalgebra::UnitQuaternion;
//! Collision detection and response

use autonomy_common::frames::Vec3;
use autonomy_common::ids::VehicleId;
use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Collision group filters
pub mod groups {
    use rapier3d::prelude::*;
    
    pub const VEHICLE: Group = Group::GROUP_1;
    pub const ENVIRONMENT: Group = Group::GROUP_2;
    pub const SENSOR: Group = Group::GROUP_3;
    pub const TRIGGER: Group = Group::GROUP_4;
    pub const PROJECTILE: Group = Group::GROUP_5;
}

/// Collision event
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CollisionEvent {
    pub body_a: super::BodyHandle,
    pub body_b: super::BodyHandle,
    pub contacts: Vec<ContactPoint>,
    pub started: bool,
}

/// Contact point details
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ContactPoint {
    pub point: Vec3,
    pub normal: Vec3,
    pub penetration: f64,
    pub impulse: Vec3,
}

/// Collision shape types
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CollisionShape {
    Ball { radius: f64 },
    Cuboid { half_extents: Vec3 },
    Capsule { radius: f64, half_height: f64 },
    Cylinder { radius: f64, half_height: f64 },
    Cone { radius: f64, half_height: f64 },
    ConvexHull { points: Vec<Vec3> },
    TriMesh { vertices: Vec<Vec3>, indices: Vec<[u32; 3]> },
    Heightfield { heights: Vec<f32>, scale: Vec3 },
}

impl CollisionShape {
    pub fn to_rapier(&self) -> ColliderBuilder {
        use rapier3d::prelude::*;
        match self {
            Self::Ball { radius } => ColliderBuilder::ball(*radius),
            Self::Cuboid { half_extents } => ColliderBuilder::cuboid(
                half_extents.x, half_extents.y, half_extents.z
            ),
            Self::Capsule { radius, half_height } => ColliderBuilder::capsule(
                Vec3::new(0.0, *half_height, 0.0),
                Vec3::new(0.0, -*half_height, 0.0),
                *radius
            ),
            Self::Cylinder { radius, half_height } => ColliderBuilder::cylinder(*half_height, *radius),
            Self::Cone { radius, half_height } => ColliderBuilder::cone(*half_height, *radius),
            Self::ConvexHull { points } => {
                let rapier_points: Vec<Point<Real>> = points.iter()
                    .map(|p| Point::new(p.x, p.y, p.z))
                    .collect();
                ColliderBuilder::convex_hull(&rapier_points).unwrap()
            }
            Self::TriMesh { vertices, indices } => {
                let rapier_vertices: Vec<Point<Real>> = vertices.iter()
                    .map(|p| Point::new(p.x, p.y, p.z))
                    .collect();
                let rapier_indices: Vec<[u32; 3]> = indices.clone();
                ColliderBuilder::trimesh(rapier_vertices, rapier_indices)
            }
            Self::Heightfield { heights, scale } => {
                let nrows = (heights.len() as f64).sqrt() as usize;
                let ncols = nrows;
                let mut hf = vec![vec![0.0; ncols]; nrows];
                for (i, h) in heights.iter().enumerate() {
                    hf[i / ncols][i % ncols] = *h as Real;
                }
                ColliderBuilder::heightfield(hf, Vec3::new(scale.x, scale.y, scale.z).into())
            }
        }
    }
}

/// Collider configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ColliderConfig {
    pub shape: CollisionShape,
    pub density: f64,
    pub friction: f64,
    pub restitution: f64,
    pub collision_groups: InteractionGroups,
    pub solver_groups: InteractionGroups,
    pub is_sensor: bool,
    pub active_events: ActiveEvents,
    pub active_hooks: ActiveHooks,
}

impl Default for ColliderConfig {
    fn default() -> Self {
        Self {
            shape: CollisionShape::Ball { radius: 0.5 },
            density: 1.0,
            friction: 0.5,
            restitution: 0.1,
            collision_groups: InteractionGroups::all(),
            solver_groups: InteractionGroups::all(),
            is_sensor: false,
            active_events: ActiveEvents::CONTACT_EVENTS,
            active_hooks: ActiveHooks::empty(),
        }
    }
}

/// Vehicle collider configs
pub fn uav_collider_config(arm_length: f64, body_radius: f64) -> ColliderConfig {
    ColliderConfig {
        shape: CollisionShape::Capsule {
            radius: body_radius,
            half_height: arm_length * 0.6,
        },
        density: 100.0,
        friction: 0.3,
        restitution: 0.1,
        collision_groups: InteractionGroups::new(
            groups::VEHICLE,
            groups::VEHICLE | groups::ENVIRONMENT,
        ),
        solver_groups: InteractionGroups::all(),
        is_sensor: false,
        active_events: ActiveEvents::CONTACT_EVENTS,
        active_hooks: ActiveHooks::empty(),
    }
}

pub fn ugv_collider_config(length: f64, width: f64, height: f64) -> ColliderConfig {
    ColliderConfig {
        shape: CollisionShape::Cuboid {
            half_extents: Vec3::new(length * 0.5, height * 0.5, width * 0.5),
        },
        density: 200.0,
        friction: 0.7,
        restitution: 0.05,
        collision_groups: InteractionGroups::new(
            groups::VEHICLE,
            groups::VEHICLE | groups::ENVIRONMENT,
        ),
        solver_groups: InteractionGroups::all(),
        is_sensor: false,
        active_events: ActiveEvents::CONTACT_EVENTS,
        active_hooks: ActiveHooks::empty(),
    }
}

/// Collision system for simulation
pub struct CollisionSystem {
    events: Vec<CollisionEvent>,
    contact_cache: HashMap<(super::BodyHandle, super::BodyHandle), Vec<ContactPoint>>,
}

impl CollisionSystem {
    pub fn new() -> Self {
        Self {
            events: Vec::new(),
            contact_cache: HashMap::new(),
        }
    }

    pub fn process_contacts(
        &mut self,
        narrow_phase: &NarrowPhase,
        bodies: &RigidBodySet,
        colliders: &ColliderSet,
    ) {
        self.events.clear();
        
        for contact_pair in narrow_phase.contact_pairs() {
            let collider_a = colliders.get(contact_pair.collider1()).unwrap();
            let collider_b = colliders.get(contact_pair.collider2()).unwrap();
            
            let body_a = bodies.get(collider_a.parent().unwrap()).unwrap();
            let body_b = bodies.get(collider_b.parent().unwrap()).unwrap();
            
            let handle_a = super::BodyHandle::from_raw(body_a.handle());
            let handle_b = super::BodyHandle::from_raw(body_b.handle());
            
            let manifold = contact_pair.deepest_contact().unwrap();
            let contact = ContactPoint {
                point: super::rapier_to_vec3(manifold.contact.point1),
                normal: super::rapier_to_vec3(manifold.contact.normal1),
                penetration: manifold.contact.dist,
                impulse: Vec3::zeros(), // Would need impulse info
            };
            
            self.contact_cache.insert((handle_a, handle_b), vec![contact]);
            
            self.events.push(CollisionEvent {
                body_a: handle_a,
                body_b: handle_b,
                contacts: vec![contact],
                started: false, // Would track state
            });
        }
    }

    pub fn events(&self) -> &[CollisionEvent] {
        &self.events
    }

    pub fn contacts_between(&self, a: super::BodyHandle, b: super::BodyHandle) -> Option<&Vec<ContactPoint>> {
        self.contact_cache.get(&(a, b)).or_else(|| self.contact_cache.get(&(b, a)))
    }
}

impl Default for CollisionSystem {
    fn default() -> Self {
        Self::new()
    }
}

/// Safety radius for collision avoidance
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct SafetyRadius {
    pub radius: f64,
    pub height: f64,
    pub vehicle_type: VehicleType,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum VehicleType {
    UavSmall,
    UavMedium,
    UavLarge,
    UgvSmall,
    UgvMedium,
    UgvLarge,
}

impl SafetyRadius {
    pub fn for_vehicle_type(vtype: VehicleType) -> Self {
        match vtype {
            VehicleType::UavSmall => Self { radius: 0.5, height: 0.3, vehicle_type: vtype },
            VehicleType::UavMedium => Self { radius: 1.0, height: 0.5, vehicle_type: vtype },
            VehicleType::UavLarge => Self { radius: 2.0, height: 1.0, vehicle_type: vtype },
            VehicleType::UgvSmall => Self { radius: 0.5, height: 0.5, vehicle_type: vtype },
            VehicleType::UgvMedium => Self { radius: 1.0, height: 1.0, vehicle_type: vtype },
            VehicleType::UgvLarge => Self { radius: 2.0, height: 2.0, vehicle_type: vtype },
        }
    }

    pub fn check_collision(&self, pos_a: Vec3, pos_b: Vec3) -> bool {
        let dx = pos_a.x - pos_b.x;
        let dz = pos_a.z - pos_b.z;
        let horizontal_dist = (dx * dx + dz * dz).sqrt();
        let vertical_overlap = (pos_a.y - pos_b.y).abs() < (self.height + self.height) * 0.5;
        horizontal_dist < (self.radius + self.radius) && vertical_overlap
    }
}
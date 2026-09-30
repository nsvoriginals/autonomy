//! Strongly-typed identifiers for all entities

use serde::{Deserialize, Serialize};
use std::fmt;
use std::hash::Hash;
use uuid::Uuid;

macro_rules! define_id {
    ($name:ident) => {
        #[derive(Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
        #[serde(transparent)]
        pub struct $name(Uuid);

        impl $name {
            pub fn new() -> Self {
                Self(Uuid::new_v4())
            }

            pub fn from_uuid(uuid: Uuid) -> Self {
                Self(uuid)
            }

            pub fn as_uuid(&self) -> Uuid {
                self.0
            }

            pub fn nil() -> Self {
                Self(Uuid::nil())
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self::new()
            }
        }

        impl fmt::Debug for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{}({})", stringify!($name), self.0)
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{}", self.0)
            }
        }

        impl From<Uuid> for $name {
            fn from(uuid: Uuid) -> Self {
                Self(uuid)
            }
        }

        impl From<$name> for Uuid {
            fn from(id: $name) -> Self {
                id.0
            }
        }
    };
}

define_id!(VehicleId);
define_id!(MissionId);
define_id!(TaskId);
define_id!(SensorId);
define_id!(ZoneId);
define_id!(ScenarioId);
define_id!(RecordingId);
define_id!(AlgorithmId);
define_id!(NetworkNodeId);

impl VehicleId {
    pub fn uav(n: u32) -> Self {
        Self(Uuid::from_u128((1u128 << 96) | (n as u128)))
    }

    pub fn ugv(n: u32) -> Self {
        Self(Uuid::from_u128((2u128 << 96) | (n as u128)))
    }

    pub fn base(n: u32) -> Self {
        Self(Uuid::from_u128((3u128 << 96) | (n as u128)))
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Debug)]
pub enum VehicleType {
    Uav,
    Ugv,
    BaseStation,
    Unknown,
}

impl VehicleType {
    pub fn from_id(id: VehicleId) -> Self {
        let uuid = id.as_uuid();
        let bytes = uuid.as_bytes();
        match (bytes[0] >> 4) & 0xF {
            1 => VehicleType::Uav,
            2 => VehicleType::Ugv,
            3 => VehicleType::BaseStation,
            _ => VehicleType::Unknown,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize, Debug)]
pub enum SensorType {
    Imu,
    Gps,
    Camera,
    Depth,
    Lidar,
    Barometer,
    Magnetometer,
    Custom(u16),
}
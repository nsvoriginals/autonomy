//! UAV configuration presets

use crate::dynamics::UavConfig;

/// Common UAV configurations
pub struct UavPresets;

impl UavPresets {
    /// Small quadcopter (DJI Phantom class)
    pub fn phantom() -> UavConfig {
        UavConfig {
            mass: 1.3,
            arm_length: 0.17,
            rotor_radius: 0.12,
            rotor_count: 4,
            max_thrust_per_rotor: 4.0,
            max_torque_per_rotor: 0.08,
            motor_time_constant: 0.05,
            max_rpm: 8000.0,
            battery_capacity_wh: 81.0,
            drag_coefficient: 0.1,
            body_drag: autonomy_common::frames::Vec3::new(0.1, 0.1, 0.1),
            ground_effect: true,
        }
    }

    /// Medium quadcopter (DJI Matrice class)
    pub fn matrice() -> UavConfig {
        UavConfig {
            mass: 6.5,
            arm_length: 0.4,
            rotor_radius: 0.21,
            rotor_count: 4,
            max_thrust_per_rotor: 12.0,
            max_torque_per_rotor: 0.3,
            motor_time_constant: 0.08,
            max_rpm: 6000.0,
            battery_capacity_wh: 200.0,
            drag_coefficient: 0.15,
            body_drag: autonomy_common::frames::Vec3::new(0.15, 0.15, 0.15),
            ground_effect: true,
        }
    }

    /// Heavy lift hexacopter
    pub fn heavy_lift_hex() -> UavConfig {
        UavConfig {
            mass: 20.0,
            arm_length: 0.6,
            rotor_radius: 0.3,
            rotor_count: 6,
            max_thrust_per_rotor: 25.0,
            max_torque_per_rotor: 0.8,
            motor_time_constant: 0.1,
            max_rpm: 5000.0,
            battery_capacity_wh: 500.0,
            drag_coefficient: 0.2,
            body_drag: autonomy_common::frames::Vec3::new(0.2, 0.2, 0.2),
            ground_effect: true,
        }
    }

    /// Racing quadcopter
    pub fn racing_quad() -> UavConfig {
        UavConfig {
            mass: 0.5,
            arm_length: 0.12,
            rotor_radius: 0.08,
            rotor_count: 4,
            max_thrust_per_rotor: 3.0,
            max_torque_per_rotor: 0.05,
            motor_time_constant: 0.02,
            max_rpm: 25000.0,
            battery_capacity_wh: 30.0,
            drag_coefficient: 0.05,
            body_drag: autonomy_common::frames::Vec3::new(0.05, 0.05, 0.05),
            ground_effect: true,
        }
    }

    /// Fixed-wing VTOL (simplified as quad for now)
    pub fn vtol() -> UavConfig {
        UavConfig {
            mass: 5.0,
            arm_length: 0.5,
            rotor_radius: 0.15,
            rotor_count: 4,
            max_thrust_per_rotor: 8.0,
            max_torque_per_rotor: 0.2,
            motor_time_constant: 0.06,
            max_rpm: 7000.0,
            battery_capacity_wh: 300.0,
            drag_coefficient: 0.12,
            body_drag: autonomy_common::frames::Vec3::new(0.1, 0.3, 0.1),
            ground_effect: true,
        }
    }

    /// Nano quadcopter (Crazyflie class)
    pub fn nano() -> UavConfig {
        UavConfig {
            mass: 0.03,
            arm_length: 0.045,
            rotor_radius: 0.035,
            rotor_count: 4,
            max_thrust_per_rotor: 0.15,
            max_torque_per_rotor: 0.002,
            motor_time_constant: 0.01,
            max_rpm: 40000.0,
            battery_capacity_wh: 2.0,
            drag_coefficient: 0.02,
            body_drag: autonomy_common::frames::Vec3::new(0.02, 0.02, 0.02),
            ground_effect: true,
        }
    }
}

/// UAV type enumeration for config selection
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum UavType {
    Nano,
    Racing,
    Phantom,
    Matrice,
    HeavyLiftHex,
    VTOL,
}

impl UavType {
    pub fn config(self) -> UavConfig {
        match self {
            Self::Nano => UavPresets::nano(),
            Self::Racing => UavPresets::racing_quad(),
            Self::Phantom => UavPresets::phantom(),
            Self::Matrice => UavPresets::matrice(),
            Self::HeavyLiftHex => UavPresets::heavy_lift_hex(),
            Self::VTOL => UavPresets::vtol(),
        }
    }
}
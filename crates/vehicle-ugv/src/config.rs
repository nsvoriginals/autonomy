//! UGV configuration presets

use crate::dynamics::{UgvConfig, DrivetrainType};

/// Common UGV configurations
pub struct UgvPresets;

impl UgvPresets {
    /// Small wheeled robot (Jackal class)
    pub fn jackal() -> UgvConfig {
        UgvConfig {
            mass: 17.0,
            length: 0.51,
            width: 0.43,
            height: 0.25,
            wheelbase: 0.33,
            track_width: 0.34,
            wheel_radius: 0.08,
            wheel_count: 4,
            max_motor_torque: 2.5,
            max_brake_torque: 5.0,
            max_steering_angle: 0.0, // Skid steer
            motor_time_constant: 0.05,
            friction_coefficient: 0.8,
            rolling_resistance: 0.02,
            drag_coefficient: 0.5,
            frontal_area: 0.15,
            battery_capacity_wh: 100.0,
            drivetrain: DrivetrainType::SkidSteer,
        }
    }

    /// Medium wheeled robot (Husky class)
    pub fn husky() -> UgvConfig {
        UgvConfig {
            mass: 55.0,
            length: 1.0,
            width: 0.67,
            height: 0.45,
            wheelbase: 0.55,
            track_width: 0.56,
            wheel_radius: 0.15,
            wheel_count: 4,
            max_motor_torque: 10.0,
            max_brake_torque: 20.0,
            max_steering_angle: 0.0, // Skid steer
            motor_time_constant: 0.1,
            friction_coefficient: 0.8,
            rolling_resistance: 0.02,
            drag_coefficient: 0.6,
            frontal_area: 0.3,
            battery_capacity_wh: 500.0,
            drivetrain: DrivetrainType::SkidSteer,
        }
    }

    /// Four-wheel steering robot
    pub fn four_wheel_steer() -> UgvConfig {
        UgvConfig {
            mass: 30.0,
            length: 0.8,
            width: 0.5,
            height: 0.4,
            wheelbase: 0.5,
            track_width: 0.4,
            wheel_radius: 0.12,
            wheel_count: 4,
            max_motor_torque: 5.0,
            max_brake_torque: 10.0,
            max_steering_angle: 0.7, // ~40 degrees
            motor_time_constant: 0.08,
            friction_coefficient: 0.8,
            rolling_resistance: 0.015,
            drag_coefficient: 0.4,
            frontal_area: 0.2,
            battery_capacity_wh: 200.0,
            drivetrain: DrivetrainType::FourWheelDrive,
        }
    }

    /// Tracked robot
    pub fn tracked() -> UgvConfig {
        UgvConfig {
            mass: 100.0,
            length: 1.2,
            width: 0.7,
            height: 0.5,
            wheelbase: 0.8,
            track_width: 0.5,
            wheel_radius: 0.1, // Sprocket radius
            wheel_count: 2, // Two tracks
            max_motor_torque: 50.0,
            max_brake_torque: 100.0,
            max_steering_angle: 0.0, // Skid steer
            motor_time_constant: 0.2,
            friction_coefficient: 0.9,
            rolling_resistance: 0.05,
            drag_coefficient: 0.7,
            frontal_area: 0.4,
            battery_capacity_wh: 1000.0,
            drivetrain: DrivetrainType::Tracked,
        }
    }

    /// Small delivery robot
    pub fn delivery_bot() -> UgvConfig {
        UgvConfig {
            mass: 25.0,
            length: 0.6,
            width: 0.4,
            height: 0.35,
            wheelbase: 0.4,
            track_width: 0.35,
            wheel_radius: 0.1,
            wheel_count: 4,
            max_motor_torque: 3.0,
            max_brake_torque: 6.0,
            max_steering_angle: 0.6,
            motor_time_constant: 0.05,
            friction_coefficient: 0.7,
            rolling_resistance: 0.02,
            drag_coefficient: 0.3,
            frontal_area: 0.12,
            battery_capacity_wh: 150.0,
            drivetrain: DrivetrainType::FourWheelDrive,
        }
    }

    /// Agricultural robot
    pub fn ag_robot() -> UgvConfig {
        UgvConfig {
            mass: 500.0,
            length: 2.5,
            width: 1.5,
            height: 1.2,
            wheelbase: 1.5,
            track_width: 1.2,
            wheel_radius: 0.3,
            wheel_count: 4,
            max_motor_torque: 200.0,
            max_brake_torque: 400.0,
            max_steering_angle: 0.5,
            motor_time_constant: 0.3,
            friction_coefficient: 0.6,
            rolling_resistance: 0.08,
            drag_coefficient: 0.8,
            frontal_area: 1.8,
            battery_capacity_wh: 5000.0,
            drivetrain: DrivetrainType::FourWheelDrive,
        }
    }
}

/// UGV type enumeration for config selection
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum UgvType {
    Jackal,
    Husky,
    FourWheelSteer,
    Tracked,
    DeliveryBot,
    AgRobot,
}

impl UgvType {
    pub fn config(self) -> UgvConfig {
        match self {
            Self::Jackal => UgvPresets::jackal(),
            Self::Husky => UgvPresets::husky(),
            Self::FourWheelSteer => UgvPresets::four_wheel_steer(),
            Self::Tracked => UgvPresets::tracked(),
            Self::DeliveryBot => UgvPresets::delivery_bot(),
            Self::AgRobot => UgvPresets::ag_robot(),
        }
    }
}
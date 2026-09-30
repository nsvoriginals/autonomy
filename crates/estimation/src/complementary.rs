//! Complementary filter estimator

use autonomy_common::error::Result;
use autonomy_common::frames::{LinearAcceleration, LinearVelocity, Pose, Vec3, AngularVelocity, UnitQuaternion};
use autonomy_common::ids::VehicleId;
use autonomy_common::state::{EstimatedState, SensorMeasurements, StateCovariance, EstimatorType};
use autonomy_common::time::SimTime;
use autonomy_common::traits::StateEstimator;
use autonomy_common::math::{low_pass_filter, wrap_angle};
use parking_lot::RwLock;

/// Complementary filter state estimator
pub struct ComplementaryFilterEstimator {
    vehicle_id: VehicleId,
    state: RwLock<EstimatedState>,
    last_time: RwLock<SimTime>,
    alpha_accel: f64,
    alpha_mag: f64,
    gravity: f64,
}

impl ComplementaryFilterEstimator {
    pub fn new(vehicle_id: VehicleId, initial: EstimatedState, config: &crate::traits::EstimatorConfig) -> Self {
        Self {
            vehicle_id,
            state: RwLock::new(initial),
            last_time: RwLock::new(SimTime::ZERO),
            alpha_accel: 0.98, // High-pass for gyro, low-pass for accel
            alpha_mag: 0.95,
            gravity: config.gravity,
        }
    }
}

impl StateEstimator for ComplementaryFilterEstimator {
    fn estimate(&mut self, time: SimTime, measurements: &SensorMeasurements) -> Result<EstimatedState> {
        let mut state = self.state.write();
        let mut last_time = self.last_time.write();
        
        let dt = (time - *last_time).as_seconds();
        *last_time = time;
        
        if dt <= 0.0 {
            return Ok(state.clone());
        }

        // Get IMU data
        let (accel, gyro) = if let Some(imu) = &measurements.imu {
            if let autonomy_common::state::SensorData::Imu { accel, gyro } = &imu.data {
                (accel.clone(), gyro.clone())
            } else {
                (Vec3::zeros(), Vec3::zeros())
            }
        } else {
            (Vec3::zeros(), Vec3::zeros())
        };

        // High-frequency: integrate gyro
        let delta_angle = gyro * dt;
        let delta_rot = UnitQuaternion::from_scaled_axis(delta_angle);
        let mut predicted_orientation = delta_rot * state.pose.orientation;
        predicted_orientation = predicted_orientation.normalize();

        // Low-frequency: accelerometer tilt correction
        let gravity_body = predicted_orientation * Vec3::new(0.0, -self.gravity, 0.0);
        let accel_corrected = accel - gravity_body;
        
        // Estimate tilt from accelerometer
        let accel_norm = accel.magnitude();
        let mut accel_orientation = UnitQuaternion::identity();
        if accel_norm > 0.1 {
            let tilt_axis = Vec3::new(-accel.y, accel.x, 0.0).normalize();
            let tilt_angle = (accel.z / accel_norm).clamp(-1.0, 1.0).acos();
            accel_orientation = UnitQuaternion::from_axis_angle(&tilt_axis, tilt_angle);
        }

        // Complementary filter for orientation
        let fused_orientation = UnitQuaternion::slerp(&predicted_orientation, &accel_orientation, 1.0 - self.alpha_accel);
        state.pose.orientation = fused_orientation.normalize();

        // Magnetometer heading correction (if available)
        if let Some(mag) = &measurements.magnetometer {
            if let autonomy_common::state::SensorData::Magnetometer { field } = &mag.data {
                // Project magnetic field to horizontal plane
                let mag_body = state.pose.orientation.inverse() * *field;
                let mag_horizontal = Vec3::new(mag_body.x, 0.0, mag_body.z);
                if mag_horizontal.magnitude() > 0.1 {
                    let mag_heading = mag_horizontal.z.atan2(mag_horizontal.x);
                    
                    // Current heading
                    let (_, _, current_yaw) = state.pose.orientation.euler_angles();
                    
                    // Complementary filter for yaw
                    let fused_yaw = current_yaw * self.alpha_mag + mag_heading * (1.0 - self.alpha_mag);
                    let fused_yaw = wrap_angle(fused_yaw);
                    
                    // Reconstruct orientation with fused yaw
                    let (roll, pitch, _) = state.pose.orientation.euler_angles();
                    state.pose.orientation = UnitQuaternion::from_euler_angles(roll, pitch, fused_yaw);
                }
            }
        }

        // Velocity and position from GPS (if available)
        if let Some(gps) = &measurements.gps {
            if let autonomy_common::state::SensorData::Gps { position, velocity, .. } = &gps.data {
                // Fuse GPS position (low frequency)
                state.pose.position = state.pose.position * self.alpha_accel + position * (1.0 - self.alpha_accel);
                state.linear_velocity = LinearVelocity(state.linear_velocity.0 * self.alpha_accel + velocity * (1.0 - self.alpha_accel));
            }
        } else {
            // Dead reckon velocity and position
            let gravity_body = state.pose.orientation * Vec3::new(0.0, -self.gravity, 0.0);
            let linear_accel = accel - gravity_body;
            
            state.linear_velocity = LinearVelocity(state.linear_velocity.0 + linear_accel * dt);
            state.pose.position += state.linear_velocity.0 * dt;
        }

        state.angular_velocity = AngularVelocity(gyro);
        state.linear_acceleration = LinearAcceleration(accel - state.pose.orientation * Vec3::new(0.0, -self.gravity, 0.0));

        // Update covariance
        let mut cov = state.covariance;
        for i in 0..3 {
            cov.pos[i][i] += 0.005 * dt;
            cov.vel[i][i] += 0.02 * dt;
        }
        state.covariance = cov;

        state.time = time;
        state.vehicle_id = self.vehicle_id;
        state.estimator_type = EstimatorType::ComplementaryFilter;

        Ok(state.clone())
    }

    fn get_estimate(&self) -> &EstimatedState {
        unimplemented!()
    }

    fn reset(&mut self) {
        let mut state = self.state.write();
        state.pose = Pose::identity();
        state.linear_velocity = LinearVelocity(Vec3::zeros());
        state.angular_velocity = AngularVelocity(Vec3::zeros());
        state.linear_acceleration = LinearAcceleration(Vec3::zeros());
        state.covariance = StateCovariance::new();
    }

    fn estimator_type(&self) -> EstimatorType {
        EstimatorType::ComplementaryFilter
    }
}
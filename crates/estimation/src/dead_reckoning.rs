//! Dead reckoning estimator

use autonomy_common::error::Result;
use autonomy_common::frames::{LinearAcceleration, LinearVelocity, Pose, Vec3, AngularVelocity};
use autonomy_common::ids::VehicleId;
use autonomy_common::state::{EstimatedState, SensorMeasurements, StateCovariance, EstimatorType};
use autonomy_common::time::SimTime;
use autonomy_common::traits::StateEstimator;
use parking_lot::RwLock;
use std::sync::Arc;

/// Dead reckoning state estimator
pub struct DeadReckoningEstimator {
    vehicle_id: VehicleId,
    state: RwLock<EstimatedState>,
    last_time: RwLock<SimTime>,
}

impl DeadReckoningEstimator {
    pub fn new(vehicle_id: VehicleId, initial: EstimatedState) -> Self {
        Self {
            vehicle_id,
            state: RwLock::new(initial),
            last_time: RwLock::new(SimTime::ZERO),
        }
    }
}

impl StateEstimator for DeadReckoningEstimator {
    fn estimate(&mut self, time: SimTime, measurements: &SensorMeasurements) -> Result<EstimatedState> {
        let mut state = self.state.write();
        let mut last_time = self.last_time.write();
        
        let dt = (time - *last_time).as_seconds();
        *last_time = time;
        
        if dt <= 0.0 {
            return Ok(state.clone());
        }

        // Get IMU measurement if available
        let (accel, gyro) = if let Some(imu) = &measurements.imu {
            if let autonomy_common::state::SensorData::Imu { accel, gyro } = &imu.data {
                (accel.clone(), gyro.clone())
            } else {
                (Vec3::zeros(), Vec3::zeros())
            }
        } else {
            (Vec3::zeros(), Vec3::zeros())
        };

        // Remove gravity from accelerometer
        let gravity = Vec3::new(0.0, -9.80665, 0.0);
        let gravity_body = state.pose.orientation * gravity;
        let linear_accel = accel - gravity_body;

        // Integrate velocity
        state.linear_velocity = LinearVelocity(state.linear_velocity.0 + linear_accel * dt);
        
        // Integrate position
        state.pose.position += state.linear_velocity.0 * dt;
        
        // Integrate orientation
        let delta_angle = gyro * dt;
        let delta_rot = autonomy_common::frames::UnitQuaternion::from_scaled_axis(delta_angle);
        state.pose.orientation = delta_rot * state.pose.orientation;
        state.pose.orientation = state.pose.orientation.normalize();

        // Update angular velocity
        state.angular_velocity = AngularVelocity(gyro);
        state.linear_acceleration = LinearAcceleration(linear_accel);

        // Increase covariance (uncertainty grows)
        let mut cov = state.covariance;
        for i in 0..3 {
            cov.pos[i][i] += 0.01 * dt;
            cov.vel[i][i] += 0.05 * dt;
        }
        state.covariance = cov;

        state.time = time;
        state.vehicle_id = self.vehicle_id;
        state.estimator_type = EstimatorType::DeadReckoning;

        Ok(state.clone())
    }

    fn get_estimate(&self) -> &EstimatedState {
        // Can't return reference to RwLock guard
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
        EstimatorType::DeadReckoning
    }
}
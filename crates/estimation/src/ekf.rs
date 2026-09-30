//! Extended Kalman Filter estimator

use autonomy_common::error::Result;
use autonomy_common::frames::{LinearAcceleration, LinearVelocity, Pose, Vec3, AngularVelocity, UnitQuaternion, Mat3};
use autonomy_common::ids::VehicleId;
use autonomy_common::state::{EstimatedState, SensorMeasurements, StateCovariance, EstimatorType, GpsFixType};
use autonomy_common::time::SimTime;
use autonomy_common::traits::StateEstimator;
use nalgebra::{Matrix3, Matrix6, Matrix9, Vector3, Vector6, UnitQuaternion as NalgebraUnitQuaternion};
use parking_lot::RwLock;

/// EKF state estimator with 9-state (position, velocity, orientation error)
pub struct EkfEstimator {
    vehicle_id: VehicleId,
    state: RwLock<EkfState>,
    last_time: RwLock<SimTime>,
    config: EkfConfig,
}

/// EKF configuration
#[derive(Clone, Debug)]
pub struct EkfConfig {
    pub process_noise_pos: f64,
    pub process_noise_vel: f64,
    pub process_noise_att: f64,
    pub accel_noise: f64,
    pub gyro_noise: f64,
    pub gps_pos_noise: f64,
    pub gps_vel_noise: f64,
    pub mag_noise: f64,
    pub baro_noise: f64,
    pub gravity: f64,
}

impl Default for EkfConfig {
    fn default() -> Self {
        Self {
            process_noise_pos: 0.01,
            process_noise_vel: 0.1,
            process_noise_att: 0.001,
            accel_noise: 0.1,
            gyro_noise: 0.01,
            gps_pos_noise: 2.0,
            gps_vel_noise: 0.5,
            mag_noise: 0.1,
            baro_noise: 1.0,
            gravity: 9.80665,
        }
    }
}

/// EKF internal state
#[derive(Clone, Debug)]
struct EkfState {
    pose: Pose,
    velocity: LinearVelocity,
    angular_velocity: AngularVelocity,
    accel_bias: Vec3,
    gyro_bias: Vec3,
    covariance: Matrix9<f64>,
    time: SimTime,
    initialized: bool,
}

impl EkfEstimator {
    pub fn new(vehicle_id: VehicleId, initial: EstimatedState, config: &crate::traits::EstimatorConfig) -> Self {
        let ekf_config = EkfConfig {
            process_noise_pos: config.process_noise,
            process_noise_vel: config.process_noise,
            process_noise_att: config.process_noise,
            ..Default::default()
        };

        let mut covariance = Matrix9::zeros();
        covariance.fill_diagonal(&Vector3::new(config.initial_covariance, config.initial_covariance, config.initial_covariance).repeat(3));

        let state = EkfState {
            pose: initial.pose,
            velocity: initial.linear_velocity,
            angular_velocity: initial.angular_velocity,
            accel_bias: Vec3::zeros(),
            gyro_bias: Vec3::zeros(),
            covariance,
            time: initial.time,
            initialized: true,
        };

        Self {
            vehicle_id,
            state: RwLock::new(state),
            last_time: RwLock::new(SimTime::ZERO),
            config: ekf_config,
        }
    }
}

impl StateEstimator for EkfEstimator {
    fn estimate(&mut self, time: SimTime, measurements: &SensorMeasurements) -> Result<EstimatedState> {
        let mut state = self.state.write();
        let mut last_time = self.last_time.write();
        
        let dt = (time - *last_time).as_seconds();
        *last_time = time;
        
        if dt <= 0.0 || dt > 1.0 {
            return Ok(self.state_to_estimated(&state));
        }

        // Prediction step
        self.predict(&mut state, dt, measurements);
        
        // Update step
        self.update(&mut state, measurements);

        // Convert to estimated state
        Ok(self.state_to_estimated(&state))
    }

    fn get_estimate(&self) -> &EstimatedState {
        unimplemented!()
    }

    fn reset(&mut self) {
        let mut state = self.state.write();
        state.pose = Pose::identity();
        state.velocity = LinearVelocity(Vec3::zeros());
        state.angular_velocity = AngularVelocity(Vec3::zeros());
        state.accel_bias = Vec3::zeros();
        state.gyro_bias = Vec3::zeros();
        state.covariance = Matrix9::zeros();
        state.initialized = false;
    }

    fn estimator_type(&self) -> EstimatorType {
        EstimatorType::Ekf
    }
}

impl EkfEstimator {
    fn predict(&self, state: &mut EkfState, dt: f64, measurements: &SensorMeasurements) {
        // Get IMU measurements
        let (accel_meas, gyro_meas) = if let Some(imu) = &measurements.imu {
            if let autonomy_common::state::SensorData::Imu { accel, gyro } = &imu.data {
                (accel.clone(), gyro.clone())
            } else {
                (Vec3::zeros(), Vec3::zeros())
            }
        } else {
            (Vec3::zeros(), Vec3::zeros())
        };

        // Correct for biases
        let accel_corrected = accel_meas - state.accel_bias;
        let gyro_corrected = gyro_meas - state.gyro_bias;

        // Remove gravity from accelerometer
        let gravity_world = Vec3::new(0.0, -self.config.gravity, 0.0);
        let gravity_body = state.pose.orientation * gravity_world;
        let linear_accel = accel_corrected - gravity_body;

        // Transform to world frame
        let linear_accel_world = state.pose.orientation * linear_accel;

        // Predict velocity
        state.velocity = LinearVelocity(state.velocity.0 + linear_accel_world * dt);

        // Predict position
        state.pose.position += state.velocity.0 * dt;

        // Predict orientation
        let delta_angle = gyro_corrected * dt;
        let delta_rot = UnitQuaternion::from_scaled_axis(delta_angle);
        state.pose.orientation = delta_rot * state.pose.orientation;
        state.pose.orientation = state.pose.orientation.normalize();

        // Update angular velocity
        state.angular_velocity = AngularVelocity(gyro_corrected);

        // State transition matrix F (9x9)
        // State vector: [pos(3), vel(3), attitude_error(3)]
        let mut F = Matrix9::identity();
        
        // Position depends on velocity
        for i in 0..3 {
            F[(i, i + 3)] = dt;
        }
        
        // Velocity depends on attitude error (through gravity)
        let R = state.pose.orientation.to_rotation_matrix();
        let g = Vec3::new(0.0, -self.config.gravity, 0.0);
        let accel_skew = self.skew_symmetric(R * g);
        
        for i in 0..3 {
            for j in 0..3 {
                F[(i + 3, i + 6)] = -accel_skew[(i, j)] * dt;
            }
        }

        // Process noise Q
        let mut Q = Matrix9::zeros();
        let q_pos = self.config.process_noise_pos * dt;
        let q_vel = self.config.process_noise_vel * dt;
        let q_att = self.config.process_noise_att * dt;
        let q_accel_bias = self.config.accel_noise * dt;
        let q_gyro_bias = self.config.gyro_noise * dt;
        
        for i in 0..3 {
            Q[(i, i)] = q_pos;
            Q[(i + 3, i + 3)] = q_vel;
            Q[(i + 6, i + 6)] = q_att;
        }

        // Propagate covariance: P = F * P * F^T + Q
        state.covariance = F * state.covariance * F.transpose() + Q;

        state.time = time;
    }

    fn update(&self, state: &mut EkfState, measurements: &SensorMeasurements) {
        // GPS position update
        if let Some(gps) = &measurements.gps {
            if let autonomy_common::state::SensorData::Gps { position, velocity, fix_type, .. } = &gps.data {
                if *fix_type == GpsFixType::Fix3D || *fix_type == GpsFixType::RtkFixed || *fix_type == GpsFixType::RtkFloat {
                    self.update_gps_position(state, position, velocity);
                }
            }
        }

        // Magnetometer update
        if let Some(mag) = &measurements.magnetometer {
            if let autonomy_common::state::SensorData::Magnetometer { field } = &mag.data {
                self.update_magnetometer(state, field);
            }
        }

        // Barometer update
        if let Some(baro) = &measurements.barometer {
            if let autonomy_common::state::SensorData::Barometer { altitude, .. } = &baro.data {
                self.update_barometer(state, *altitude);
            }
        }
    }

    fn update_gps_position(&self, state: &mut EkfState, position: &Vec3, velocity: &Vec3) {
        // Measurement model: z = [pos; vel] = H * x + v
        // H = [I 0 0; 0 I 0] (6x9)
        let mut H = Matrix6::zeros();
        H[(0, 0)] = 1.0; H[(1, 1)] = 1.0; H[(2, 2)] = 1.0;
        H[(3, 3)] = 1.0; H[(4, 4)] = 1.0; H[(5, 5)] = 1.0;

        // Measurement noise
        let mut R = Matrix6::zeros();
        R[(0, 0)] = self.config.gps_pos_noise * self.config.gps_pos_noise;
        R[(1, 1)] = self.config.gps_pos_noise * self.config.gps_pos_noise;
        R[(2, 2)] = self.config.gps_pos_noise * self.config.gps_pos_noise * 4.0; // Worse vertical
        R[(3, 3)] = self.config.gps_vel_noise * self.config.gps_vel_noise;
        R[(4, 4)] = self.config.gps_vel_noise * self.config.gps_vel_noise;
        R[(5, 5)] = self.config.gps_vel_noise * self.config.gps_vel_noise * 4.0;

        // Innovation
        let mut z = Vector6::zeros();
        z[0] = position.x - state.pose.position.x;
        z[1] = position.y - state.pose.position.y;
        z[2] = position.z - state.pose.position.z;
        z[3] = velocity.x - state.velocity.0.x;
        z[4] = velocity.y - state.velocity.0.y;
        z[5] = velocity.z - state.velocity.0.z;

        // Kalman gain: K = P * H^T * (H * P * H^T + R)^-1
        let P = state.covariance;
        let S = H * P * H.transpose() + R;
        
        if let Some(S_inv) = S.try_inverse() {
            let K = P * H.transpose() * S_inv;
            
            // State update: x = x + K * z
            let dx = K * z;
            
            // Position correction
            state.pose.position.x += dx[0];
            state.pose.position.y += dx[1];
            state.pose.position.z += dx[2];
            
            // Velocity correction
            state.velocity.0.x += dx[3];
            state.velocity.0.y += dx[4];
            state.velocity.0.z += dx[5];
            
            // Attitude correction (from velocity innovation)
            // Convert attitude error to quaternion correction
            let att_err = Vec3::new(dx[6], dx[7], dx[8]);
            if att_err.magnitude() > 1e-6 {
                let delta_q = UnitQuaternion::from_scaled_axis(att_err);
                state.pose.orientation = delta_q * state.pose.orientation;
                state.pose.orientation = state.pose.orientation.normalize();
            }
            
            // Covariance update: P = (I - K * H) * P
            let I = Matrix9::identity();
            state.covariance = (I - K * H) * state.covariance;
        }
    }

    fn update_magnetometer(&self, state: &mut EkfState, field: &Vec3) {
        // Simplified: use magnetometer for heading correction
        let mag_body = state.pose.orientation.inverse() * *field;
        let mag_heading = mag_body.z.atan2(mag_body.x);
        
        let (_, _, current_yaw) = state.pose.orientation.euler_angles();
        let heading_error = autonomy_common::math::wrap_angle(mag_heading - current_yaw);
        
        if heading_error.abs() > 0.1 {
            // Create measurement
            let H = Matrix3::zeros(); // Would need proper Jacobian
            // For now, simple correction
            let correction = heading_error * 0.1;
            let delta_q = UnitQuaternion::from_axis_angle(&Vec3::y_axis(), correction);
            state.pose.orientation = delta_q * state.pose.orientation;
        }
    }

    fn update_barometer(&self, state: &mut EkfState, altitude: f64) {
        let height_error = altitude - state.pose.position.y;
        let correction = height_error * 0.1;
        state.pose.position.y += correction;
    }

    fn skew_symmetric(&self, v: Vec3) -> Matrix3<f64> {
        Matrix3::new(
            0.0, -v.z, v.y,
            v.z, 0.0, -v.x,
            -v.y, v.x, 0.0,
        )
    }

    fn state_to_estimated(&self, state: &EkfState) -> EstimatedState {
        // Extract position/velocity covariance from full 9x9
        let mut pos_cov = [[0.0; 3]; 3];
        let mut vel_cov = [[0.0; 3]; 3];
        let mut pos_vel_cov = [[0.0; 3]; 3];
        
        for i in 0..3 {
            for j in 0..3 {
                pos_cov[i][j] = state.covariance[(i, j)];
                vel_cov[i][j] = state.covariance[(i + 3, j + 3)];
                pos_vel_cov[i][j] = state.covariance[(i, j + 3)];
            }
        }

        EstimatedState {
            time: state.time,
            vehicle_id: self.vehicle_id,
            pose: state.pose,
            linear_velocity: state.velocity,
            angular_velocity: state.angular_velocity,
            linear_acceleration: LinearAcceleration(Vec3::zeros()), // Would need to compute
            covariance: StateCovariance {
                pos: pos_cov,
                vel: vel_cov,
                pos_vel: pos_vel_cov,
            },
            estimator_type: EstimatorType::Ekf,
            gps_fixed: true, // Would track actual GPS status
            innovation: None,
        }
    }
}
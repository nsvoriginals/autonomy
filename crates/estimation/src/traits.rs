//! Estimation traits

use autonomy_common::error::Result;
use autonomy_common::frames::{LinearAcceleration, LinearVelocity, Pose, Vec3};
use autonomy_common::ids::VehicleId;
use autonomy_common::state::{EstimatedState, SensorMeasurements, StateCovariance};
use autonomy_common::time::SimTime;
use autonomy_common::traits::StateEstimator;

/// State estimator configuration
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct EstimatorConfig {
    pub estimator_type: autonomy_common::state::EstimatorType,
    pub process_noise: f64,
    pub measurement_noise: f64,
    pub initial_covariance: f64,
    pub gravity: f64,
}

impl Default for EstimatorConfig {
    fn default() -> Self {
        Self {
            estimator_type: autonomy_common::state::EstimatorType::Ekf,
            process_noise: 0.1,
            measurement_noise: 1.0,
            initial_covariance: 1.0,
            gravity: 9.80665,
        }
    }
}

/// Estimator factory
pub fn create_estimator(
    config: &EstimatorConfig,
    vehicle_id: VehicleId,
    initial_state: EstimatedState,
) -> Box<dyn StateEstimator> {
    match config.estimator_type {
        autonomy_common::state::EstimatorType::DeadReckoning => {
            Box::new(DeadReckoningEstimator::new(vehicle_id, initial_state))
        }
        autonomy_common::state::EstimatorType::ComplementaryFilter => {
            Box::new(ComplementaryFilterEstimator::new(vehicle_id, initial_state, config))
        }
        autonomy_common::state::EstimatorType::Ekf => {
            Box::new(EkfEstimator::new(vehicle_id, initial_state, config))
        }
        _ => {
            Box::new(DeadReckoningEstimator::new(vehicle_id, initial_state))
        }
    }
}
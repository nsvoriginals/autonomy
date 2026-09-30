//! Noise models for sensors

use autonomy_common::frames::Vec3;
use autonomy_common::rng::SimRng;
use serde::{Deserialize, Serialize};

/// Noise model types
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum NoiseModel {
    Gaussian,
    Uniform,
    Rayleigh,
    Rice,
    Custom,
}

/// Gaussian noise parameters
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GaussianNoise {
    pub mean: f64,
    pub std_dev: f64,
}

impl GaussianNoise {
    pub fn new(std_dev: f64) -> Self {
        Self { mean: 0.0, std_dev }
    }

    pub fn sample(&self, rng: &mut SimRng) -> f64 {
        use autonomy_common::rng::sample_normal;
        sample_normal(rng, self.mean, self.std_dev)
    }

    pub fn sample_vec3(&self, rng: &mut SimRng) -> Vec3 {
        Vec3::new(
            self.sample(rng),
            self.sample(rng),
            self.sample(rng),
        )
    }
}

/// Bias model (slowly varying offset)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BiasModel {
    pub initial_bias: f64,
    pub random_walk_std: f64, // per sqrt(second)
    pub correlation_time: f64, // seconds
    pub current_bias: f64,
}

impl BiasModel {
    pub fn new(random_walk_std: f64, correlation_time: f64) -> Self {
        Self {
            initial_bias: 0.0,
            random_walk_std,
            correlation_time,
            current_bias: 0.0,
        }
    }

    pub fn step(&mut self, dt: f64, rng: &mut SimRng) -> f64 {
        use autonomy_common::rng::sample_normal;
        
        if self.correlation_time > 0.0 {
            // Gauss-Markov process
            let alpha = (-dt / self.correlation_time).exp();
            let sigma = self.random_walk_std * (1.0 - alpha * alpha).sqrt();
            self.current_bias = self.current_bias * alpha + sample_normal(rng, 0.0, sigma);
        } else {
            // Random walk
            self.current_bias += sample_normal(rng, 0.0, self.random_walk_std * dt.sqrt());
        }
        
        self.current_bias
    }

    pub fn reset(&mut self) {
        self.current_bias = self.initial_bias;
    }
}

/// Scale factor error (multiplicative)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ScaleFactorError {
    pub nominal: f64,
    pub error_std: f64,
    pub current: f64,
}

impl ScaleFactorError {
    pub fn new(nominal: f64, error_std: f64) -> Self {
        Self {
            nominal,
            error_std,
            current: nominal,
        }
    }

    pub fn sample(&mut self, rng: &mut SimRng) -> f64 {
        use autonomy_common::rng::sample_normal;
        self.current = self.nominal + sample_normal(rng, 0.0, self.error_std);
        self.current
    }
}

/// Misalignment matrix (non-orthogonal axes)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MisalignmentMatrix {
    pub matrix: [[f64; 3]; 3],
}

impl MisalignmentMatrix {
    pub fn identity() -> Self {
        Self {
            matrix: [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
        }
    }

    pub fn from_angles(roll: f64, pitch: f64, yaw: f64) -> Self {
        let cr = roll.cos();
        let sr = roll.sin();
        let cp = pitch.cos();
        let sp = pitch.sin();
        let cy = yaw.cos();
        let sy = yaw.sin();

        Self {
            matrix: [
                [cp * cy, cp * sy, -sp],
                [sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, sr * cp],
                [cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp],
            ],
        }
    }

    pub fn apply(&self, vec: Vec3) -> Vec3 {
        Vec3::new(
            self.matrix[0][0] * vec.x + self.matrix[0][1] * vec.y + self.matrix[0][2] * vec.z,
            self.matrix[1][0] * vec.x + self.matrix[1][1] * vec.y + self.matrix[1][2] * vec.z,
            self.matrix[2][0] * vec.x + self.matrix[2][1] * vec.y + self.matrix[2][2] * vec.z,
        )
    }
}

/// Quantization noise
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct QuantizationNoise {
    pub resolution: f64,
}

impl QuantizationNoise {
    pub fn new(resolution: f64) -> Self {
        Self { resolution }
    }

    pub fn apply(&self, value: f64) -> f64 {
        (value / self.resolution).round() * self.resolution
    }
}

/// Combined sensor noise model
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SensorNoiseModel {
    pub gaussian: GaussianNoise,
    pub bias: BiasModel,
    pub scale_factor: ScaleFactorError,
    pub misalignment: MisalignmentMatrix,
    pub quantization: Option<QuantizationNoise>,
}

impl SensorNoiseModel {
    pub fn new(gaussian_std: f64, bias_rw: f64, bias_corr: f64, scale_error: f64) -> Self {
        Self {
            gaussian: GaussianNoise::new(gaussian_std),
            bias: BiasModel::new(bias_rw, bias_corr),
            scale_factor: ScaleFactorError::new(1.0, scale_error),
            misalignment: MisalignmentMatrix::identity(),
            quantization: None,
        }
    }

    pub fn apply(&mut self, true_value: f64, dt: f64, rng: &mut SimRng) -> f64 {
        let scale = self.scale_factor.sample(rng);
        let bias = self.bias.step(dt, rng);
        let noise = self.gaussian.sample(rng);
        
        let mut result = true_value * scale + bias + noise;
        
        if let Some(q) = &self.quantization {
            result = q.apply(result);
        }
        
        result
    }

    pub fn apply_vec3(&mut self, true_value: Vec3, dt: f64, rng: &mut SimRng) -> Vec3 {
        let aligned = self.misalignment.apply(true_value);
        
        Vec3::new(
            self.apply(aligned.x, dt, rng),
            self.apply(aligned.y, dt, rng),
            self.apply(aligned.z, dt, rng),
        )
    }
}
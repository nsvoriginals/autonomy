//! Weather and atmospheric models

use autonomy_common::frames::Vec3;
use autonomy_common::rng::SimRng;
use serde::{Deserialize, Serialize};

/// Weather preset types
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum WeatherPreset {
    Clear,
    Cloudy,
    Foggy,
    Rainy,
    Stormy,
    Windy,
    Custom,
}

/// Detailed weather state
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WeatherState {
    pub preset: WeatherPreset,
    pub wind_velocity: Vec3,
    pub wind_turbulence: f64,
    pub temperature: f64,
    pub pressure: f64,
    pub humidity: f64,
    pub visibility: f64,
    pub precipitation_rate: f64,
    pub cloud_coverage: f64,
    pub cloud_base_altitude: f64,
    pub icing_conditions: bool,
}

impl Default for WeatherState {
    fn default() -> Self {
        Self::clear()
    }
}

impl WeatherState {
    pub fn clear() -> Self {
        Self {
            preset: WeatherPreset::Clear,
            wind_velocity: Vec3::zeros(),
            wind_turbulence: 0.05,
            temperature: 20.0,
            pressure: 101325.0,
            humidity: 0.4,
            visibility: 10000.0,
            precipitation_rate: 0.0,
            cloud_coverage: 0.1,
            cloud_base_altitude: 2000.0,
            icing_conditions: false,
        }
    }

    pub fn foggy() -> Self {
        Self {
            preset: WeatherPreset::Foggy,
            wind_velocity: Vec3::zeros(),
            wind_turbulence: 0.02,
            temperature: 10.0,
            pressure: 101500.0,
            humidity: 0.95,
            visibility: 200.0,
            precipitation_rate: 0.0,
            cloud_coverage: 1.0,
            cloud_base_altitude: 50.0,
            icing_conditions: false,
        }
    }

    pub fn windy() -> Self {
        Self {
            preset: WeatherPreset::Windy,
            wind_velocity: Vec3::new(15.0, 0.0, 5.0),
            wind_turbulence: 0.3,
            temperature: 15.0,
            pressure: 101000.0,
            humidity: 0.6,
            visibility: 8000.0,
            precipitation_rate: 0.0,
            cloud_coverage: 0.5,
            cloud_base_altitude: 1500.0,
            icing_conditions: false,
        }
    }

    pub fn stormy() -> Self {
        Self {
            preset: WeatherPreset::Stormy,
            wind_velocity: Vec3::new(25.0, 0.0, 10.0),
            wind_turbulence: 0.5,
            temperature: 12.0,
            pressure: 99500.0,
            humidity: 0.9,
            visibility: 1000.0,
            precipitation_rate: 10.0,
            cloud_coverage: 1.0,
            cloud_base_altitude: 300.0,
            icing_conditions: true,
        }
    }

    pub fn with_wind(mut self, wind: Vec3) -> Self {
        self.wind_velocity = wind;
        self
    }

    pub fn with_turbulence(mut self, intensity: f64) -> Self {
        self.wind_turbulence = intensity.clamp(0.0, 1.0);
        self
    }
}

/// Atmospheric model for sensor simulation
#[derive(Clone, Debug)]
pub struct Atmosphere {
    pub temperature: f64,
    pub pressure: f64,
    pub humidity: f64,
}

impl Atmosphere {
    pub fn from_weather(weather: &WeatherState) -> Self {
        Self {
            temperature: weather.temperature,
            pressure: weather.pressure,
            humidity: weather.humidity,
        }
    }

    /// Speed of sound at current conditions (m/s)
    pub fn speed_of_sound(&self) -> f64 {
        331.3 + 0.606 * self.temperature
    }

    /// Air density (kg/m^3)
    pub fn density(&self, altitude: f64) -> f64 {
        let temp_k = self.temperature + 273.15;
        let pressure_at_alt = self.pressure * (-altitude * 0.00012).exp();
        pressure_at_alt / (287.058 * temp_k) * (1.0 - 0.378 * self.humidity)
    }

    /// Dynamic viscosity (Pa·s)
    pub fn viscosity(&self) -> f64 {
        let temp_k = self.temperature + 273.15;
        1.716e-5 * (temp_k / 273.15).powf(1.5) * (273.15 + 110.4) / (temp_k + 110.4)
    }

    /// Kinematic viscosity (m^2/s)
    pub fn kinematic_viscosity(&self, altitude: f64) -> f64 {
        self.viscosity() / self.density(altitude)
    }
}

/// Wind field with spatial variation
#[derive(Clone, Debug)]
pub struct WindField {
    base_velocity: Vec3,
    gradient: Vec3, // Velocity change per meter
    turbulence: f64,
    shear_exponent: f64, // Power law exponent for altitude variation
}

impl WindField {
    pub fn new(base_velocity: Vec3) -> Self {
        Self {
            base_velocity,
            gradient: Vec3::zeros(),
            turbulence: 0.1,
            shear_exponent: 0.14, // Typical for open terrain
        }
    }

    pub fn with_gradient(mut self, gradient: Vec3) -> Self {
        self.gradient = gradient;
        self
    }

    pub fn with_turbulence(mut self, intensity: f64) -> Self {
        self.turbulence = intensity.clamp(0.0, 1.0);
        self
    }

    pub fn with_shear(mut self, exponent: f64) -> Self {
        self.shear_exponent = exponent;
        self
    }

    pub fn velocity_at(&self, position: Vec3, altitude_ref: f64, rng: &mut SimRng) -> Vec3 {
        use autonomy_common::rng::sample_normal;
        
        let altitude_factor = if position.y > 0.0 {
            (position.y / altitude_ref).max(0.01).powf(self.shear_exponent)
        } else {
            0.0
        };

        let mut vel = self.base_velocity * altitude_factor;
        vel += self.gradient * position;

        if self.turbulence > 0.0 {
            vel += Vec3::new(
                sample_normal(rng, 0.0, self.turbulence * vel.magnitude()),
                sample_normal(rng, 0.0, self.turbulence * vel.magnitude() * 0.1),
                sample_normal(rng, 0.0, self.turbulence * vel.magnitude()),
            );
        }

        vel
    }
}

/// Visibility model for optical sensors
#[derive(Clone, Debug)]
pub struct VisibilityModel {
    base_visibility: f64,
    extinction_coefficient: f64, // km^-1
}

impl VisibilityModel {
    pub fn new(visibility_km: f64) -> Self {
        // Koschmieder's formula: V = 3.912 / beta
        let extinction = 3.912 / visibility_km;
        Self {
            base_visibility: visibility_km * 1000.0,
            extinction_coefficient: extinction,
        }
    }

    pub fn from_weather(weather: &WeatherState) -> Self {
        Self::new(weather.visibility / 1000.0)
    }

    /// Transmittance over distance (Beer-Lambert law)
    pub fn transmittance(&self, distance: f64) -> f64 {
        (-self.extinction_coefficient * distance / 1000.0).exp()
    }

    /// Maximum range for given contrast threshold
    pub fn max_range(&self, contrast_threshold: f64) -> f64 {
        if contrast_threshold <= 0.0 || contrast_threshold >= 1.0 {
            return self.base_visibility;
        }
        (-contrast_threshold.ln() / self.extinction_coefficient) * 1000.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_atmosphere() {
        let weather = WeatherState::clear();
        let atm = Atmosphere::from_weather(&weather);
        
        let sos = atm.speed_of_sound();
        assert!((sos - 343.0).abs() < 5.0);

        let density = atm.density(0.0);
        assert!((density - 1.2).abs() < 0.1);
    }

    #[test]
    fn test_visibility() {
        let vis = VisibilityModel::new(10.0); // 10 km
        let t = vis.transmittance(5000.0); // 5 km
        assert!(t > 0.5 && t < 1.0);
    }

    #[test]
    fn test_wind_field() {
        let field = WindField::new(Vec3::new(10.0, 0.0, 0.0))
            .with_shear(0.2);
        let mut rng = autonomy_common::rng::seeded_rng(42);
        
        let v1 = field.velocity_at(Vec3::new(0.0, 10.0, 0.0), 10.0, &mut rng);
        let v2 = field.velocity_at(Vec3::new(0.0, 100.0, 0.0), 10.0, &mut rng);
        
        // Higher altitude should have higher wind speed
        assert!(v2.x > v1.x);
    }
}
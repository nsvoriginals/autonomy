//! World model combining terrain, zones, and environment state

use autonomy_common::frames::Vec3;
use autonomy_common::ids::{ZoneId, VehicleId};
use autonomy_common::rng::SimRng;
use autonomy_common::time::SimTime;
use autonomy_common::traits::SimModule;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;

use crate::terrain::{TerrainProvider, create_terrain, TerrainType};
use crate::zones::{Zone, ZoneManager, ZoneType, RestrictedZone, CommunicationNode};

/// World configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WorldConfig {
    pub terrain: TerrainType,
    pub gravity: f64,
    pub magnetic_declination: f64,
    pub magnetic_inclination: f64,
    pub atmospheric_density: f64,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            terrain: TerrainType::Flat { height: 0.0 },
            gravity: 9.80665,
            magnetic_declination: 0.0,
            magnetic_inclination: -66.0,
            atmospheric_density: 1.225,
        }
    }
}

/// Wind model
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WindModel {
    pub base_wind: Vec3,
    pub turbulence_intensity: f64,
    pub turbulence_scale: f64,
    pub gust_probability: f64,
    pub gust_magnitude: f64,
    pub seed: u64,
}

impl Default for WindModel {
    fn default() -> Self {
        Self {
            base_wind: Vec3::zeros(),
            turbulence_intensity: 0.1,
            turbulence_scale: 100.0,
            gust_probability: 0.01,
            gust_magnitude: 5.0,
            seed: 0,
        }
    }
}

impl WindModel {
    pub fn new(base_wind: Vec3) -> Self {
        Self {
            base_wind,
            ..Default::default()
        }
    }

    pub fn wind_at(&self, position: Vec3, time: f64, rng: &mut SimRng) -> Vec3 {
        use autonomy_common::rng::{sample_normal, sample_uniform, sample_bool};
        
        let mut wind = self.base_wind;

        // Add turbulence
        if self.turbulence_intensity > 0.0 {
            let turb_x = sample_normal(rng, 0.0, self.turbulence_intensity);
            let turb_y = sample_normal(rng, 0.0, self.turbulence_intensity * 0.1);
            let turb_z = sample_normal(rng, 0.0, self.turbulence_intensity);
            wind += Vec3::new(turb_x, turb_y, turb_z);
        }

        // Add gusts
        if sample_bool(rng, self.gust_probability) {
            let gust_dir = Vec3::new(
                sample_uniform(rng, -1.0, 1.0),
                sample_uniform(rng, -0.2, 0.2),
                sample_uniform(rng, -1.0, 1.0),
            ).normalize();
            wind += gust_dir * self.gust_magnitude;
        }

        wind
    }
}

/// Weather conditions
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Weather {
    pub wind: WindModel,
    pub temperature: f64,
    pub pressure: f64,
    pub humidity: f64,
    pub visibility: f64,
    pub precipitation: f64,
    pub cloud_base: f64,
}

impl Default for Weather {
    fn default() -> Self {
        Self {
            wind: WindModel::default(),
            temperature: 20.0,
            pressure: 101325.0,
            humidity: 0.5,
            visibility: 10000.0,
            precipitation: 0.0,
            cloud_base: 1000.0,
        }
    }
}

/// Main world model
pub struct World {
    config: WorldConfig,
    terrain: Arc<dyn TerrainProvider>,
    zones: ZoneManager,
    weather: Weather,
    rng: SimRng,
    time_of_day: f64, // 0.0 to 24.0
}

impl World {
    pub fn new(config: WorldConfig, rng: SimRng) -> Self {
        let terrain = create_terrain(&config.terrain);
        Self {
            config,
            terrain,
            zones: ZoneManager::new(),
            weather: Weather::default(),
            rng,
            time_of_day: 12.0,
        }
    }

    pub fn config(&self) -> &WorldConfig {
        &self.config
    }

    pub fn terrain(&self) -> &Arc<dyn TerrainProvider> {
        &self.terrain
    }

    pub fn zones(&self) -> &ZoneManager {
        &self.zones
    }

    pub fn zones_mut(&mut self) -> &mut ZoneManager {
        &mut self.zones
    }

    pub fn weather(&self) -> &Weather {
        &self.weather
    }

    pub fn weather_mut(&mut self) -> &mut Weather {
        &mut self.weather
    }

    pub fn gravity(&self) -> f64 {
        self.config.gravity
    }

    pub fn magnetic_field(&self) -> Vec3 {
        let dec = self.config.magnetic_declination.to_radians();
        let inc = self.config.magnetic_inclination.to_radians();
        let h = 50.0; // microtesla horizontal component
        Vec3::new(
            h * inc.cos() * dec.cos(),
            h * inc.cos() * dec.sin(),
            h * inc.sin(),
        )
    }

    pub fn atmospheric_density(&self, altitude: f64) -> f64 {
        // Standard atmosphere model
        self.config.atmospheric_density * (-altitude / 8500.0).exp()
    }

    pub fn wind_at(&mut self, position: Vec3) -> Vec3 {
        let time = self.time_of_day * 3600.0;
        self.weather.wind.wind_at(position, time, &mut self.rng)
    }

    pub fn time_of_day(&self) -> f64 {
        self.time_of_day
    }

    pub fn set_time_of_day(&mut self, time: f64) {
        self.time_of_day = time % 24.0;
    }

    pub fn advance_time(&mut self, dt: f64) {
        self.time_of_day = (self.time_of_day + dt / 3600.0) % 24.0;
    }

    pub fn height_at(&self, position: Vec3) -> f64 {
        self.terrain.height_at(position)
    }

    pub fn normal_at(&self, position: Vec3) -> Vec3 {
        self.terrain.normal_at(position)
    }

    pub fn is_restricted(&self, position: Vec3, vehicle_type: autonomy_common::ids::VehicleType) -> bool {
        self.zones.is_restricted(position, vehicle_type, self.time_of_day * 3600.0)
    }

    pub fn nearest_landing_zone(&self, position: Vec3, vehicle_size: f64, vehicle_weight: f64) -> Option<&crate::zones::LandingZone> {
        self.zones.nearest_landing_zone(position, vehicle_size, vehicle_weight)
    }

    pub fn nearest_charging_zone(&self, position: Vec3) -> Option<&crate::zones::ChargingZone> {
        self.zones.nearest_charging_zone(position)
    }
}

/// World module for simulation
pub struct WorldModule {
    world: World,
}

impl WorldModule {
    pub fn new(config: WorldConfig, seed: u64) -> Self {
        let rng = autonomy_common::rng::seeded_rng(seed);
        Self {
            world: World::new(config, rng),
        }
    }

    pub fn world(&self) -> &World {
        &self.world
    }

    pub fn world_mut(&mut self) -> &mut World {
        &mut self.world
    }
}

impl SimModule for WorldModule {
    fn name(&self) -> &'static str {
        "world"
    }

    fn step(&mut self, time: SimTime, dt: f64) -> autonomy_common::error::Result<()> {
        self.world.advance_time(dt);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_world_creation() {
        let config = WorldConfig::default();
        let world = World::new(config, autonomy_common::rng::seeded_rng(42));
        assert_eq!(world.gravity(), 9.80665);
    }

    #[test]
    fn test_wind_model() {
        let wind = WindModel::new(Vec3::new(5.0, 0.0, 0.0));
        let mut rng = autonomy_common::rng::seeded_rng(123);
        let w = wind.wind_at(Vec3::zeros(), 0.0, &mut rng);
        // Should have base wind plus some turbulence
        assert!((w.x - 5.0).abs() < 2.0);
    }

    #[test]
    fn test_atmospheric_density() {
        let config = WorldConfig::default();
        let world = World::new(config, autonomy_common::rng::seeded_rng(42));
        let sea_level = world.atmospheric_density(0.0);
        let high_alt = world.atmospheric_density(1000.0);
        assert!(high_alt < sea_level);
    }
}
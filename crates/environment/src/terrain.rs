//! Terrain model for simulation

use autonomy_common::frames::Vec3;
use autonomy_common::ids::ZoneId;
use autonomy_common::rng::SimRng;
use autonomy_common::traits::SimModule;
use nalgebra::Vector2;
use noise::{NoiseFn, Perlin};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;

/// Terrain configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TerrainConfig {
    pub size: f64,
    pub resolution: f64,
    pub max_height: f64,
    pub noise_scale: f64,
    pub noise_octaves: usize,
    pub noise_persistence: f64,
    pub noise_lacunarity: f64,
    pub seed: u32,
}

impl Default for TerrainConfig {
    fn default() -> Self {
        Self {
            size: 2000.0,
            resolution: 5.0,
            max_height: 100.0,
            noise_scale: 0.01,
            noise_octaves: 4,
            noise_persistence: 0.5,
            noise_lacunarity: 2.0,
            seed: 42,
        }
    }
}

/// Heightmap-based terrain
#[derive(Clone, Debug)]
pub struct Terrain {
    config: TerrainConfig,
    heightmap: Vec<f32>,
    width: usize,
    height: usize,
    noise: Perlin,
}

impl Terrain {
    pub fn new(config: TerrainConfig) -> Self {
        let width = (config.size / config.resolution) as usize + 1;
        let height = width;
        let noise = Perlin::new(config.seed);
        
        let mut terrain = Self {
            config: config.clone(),
            heightmap: vec![0.0; width * height],
            width,
            height,
            noise,
        };
        
        terrain.generate_heightmap();
        terrain
    }

    fn generate_heightmap(&mut self) {
        let half_size = self.config.size * 0.5;
        
        for y in 0..self.height {
            for x in 0..self.width {
                let wx = (x as f64 * self.config.resolution) - half_size;
                let wy = (y as f64 * self.config.resolution) - half_size;
                
                let h = self.sample_noise(wx, wy);
                self.heightmap[y * self.width + x] = (h * self.config.max_height) as f32;
            }
        }
    }

    fn sample_noise(&self, x: f64, z: f64) -> f64 {
        let mut value = 0.0;
        let mut amplitude = 1.0;
        let mut frequency = self.config.noise_scale;
        let mut max_amplitude = 0.0;

        for _ in 0..self.config.noise_octaves {
            value += self.noise.get([
                x * frequency,
                z * frequency,
            ]) * amplitude;
            
            max_amplitude += amplitude;
            amplitude *= self.config.noise_persistence;
            frequency *= self.config.noise_lacunarity;
        }

        value / max_amplitude
    }

    pub fn height_at(&self, position: Vec3) -> f64 {
        let half_size = self.config.size * 0.5;
        let x = (position.x + half_size) / self.config.resolution;
        let z = (position.z + half_size) / self.config.resolution;

        if x < 0.0 || z < 0.0 || x >= (self.width - 1) as f64 || z >= (self.height - 1) as f64 {
            return 0.0;
        }

        let x0 = x.floor() as usize;
        let z0 = z.floor() as usize;
        let x1 = x0 + 1;
        let z1 = z0 + 1;

        let tx = x - x0 as f64;
        let tz = z - z0 as f64;

        let h00 = self.heightmap[z0 * self.width + x0] as f64;
        let h10 = self.heightmap[z0 * self.width + x1] as f64;
        let h01 = self.heightmap[z1 * self.width + x0] as f64;
        let h11 = self.heightmap[z1 * self.width + x1] as f64;

        // Bilinear interpolation
        let h0 = h00 * (1.0 - tx) + h10 * tx;
        let h1 = h01 * (1.0 - tx) + h11 * tx;
        h0 * (1.0 - tz) + h1 * tz
    }

    pub fn normal_at(&self, position: Vec3) -> Vec3 {
        let eps = self.config.resolution * 0.1;
        let h = self.height_at(position);
        
        let hx = self.height_at(position + Vec3::new(eps, 0.0, 0.0));
        let hz = self.height_at(position + Vec3::new(0.0, 0.0, eps));
        
        let nx = h - hx;
        let nz = h - hz;
        let ny = eps * 2.0;

        Vec3::new(nx, ny, nz).normalize()
    }

    pub fn config(&self) -> &TerrainConfig {
        &self.config
    }

    pub fn width(&self) -> usize {
        self.width
    }

    pub fn height(&self) -> usize {
        self.height
    }

    pub fn heightmap(&self) -> &[f32] {
        &self.heightmap
    }

    /// Get height and normal for rendering
    pub fn sample(&self, x: f64, z: f64) -> (f64, Vec3) {
        let pos = Vec3::new(x, 0.0, z);
        let h = self.height_at(pos);
        let n = self.normal_at(pos);
        (h, n)
    }
}

/// Flat terrain for simple scenarios
#[derive(Clone, Debug, Default)]
pub struct FlatTerrain {
    pub height: f64,
}

impl FlatTerrain {
    pub fn new(height: f64) -> Self {
        Self { height }
    }

    pub fn height_at(&self, _position: Vec3) -> f64 {
        self.height
    }

    pub fn normal_at(&self, _position: Vec3) -> Vec3 {
        Vec3::y_axis()
    }
}

/// Terrain type enum for configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum TerrainType {
    Flat { height: f64 },
    Procedural(TerrainConfig),
    Heightmap { path: String, scale: f64 },
}

/// Terrain factory
pub fn create_terrain(terrain_type: &TerrainType) -> Box<dyn TerrainProvider> {
    match terrain_type {
        TerrainType::Flat { height } => Box::new(FlatTerrain::new(*height)),
        TerrainType::Procedural(config) => Box::new(Terrain::new(config.clone())),
        TerrainType::Heightmap { path, scale } => {
            // TODO: Load from file
            Box::new(FlatTerrain::new(0.0))
        }
    }
}

/// Trait for terrain providers
pub trait TerrainProvider: Send + Sync {
    fn height_at(&self, position: Vec3) -> f64;
    fn normal_at(&self, position: Vec3) -> Vec3;
}

impl TerrainProvider for Terrain {
    fn height_at(&self, position: Vec3) -> f64 {
        self.height_at(position)
    }

    fn normal_at(&self, position: Vec3) -> Vec3 {
        self.normal_at(position)
    }
}

impl TerrainProvider for FlatTerrain {
    fn height_at(&self, position: Vec3) -> f64 {
        self.height_at(position)
    }

    fn normal_at(&self, position: Vec3) -> Vec3 {
        self.normal_at(position)
    }
}

/// Terrain module for simulation
pub struct TerrainModule {
    terrain: Arc<dyn TerrainProvider>,
}

impl TerrainModule {
    pub fn new(terrain: Arc<dyn TerrainProvider>) -> Self {
        Self { terrain }
    }

    pub fn terrain(&self) -> &Arc<dyn TerrainProvider> {
        &self.terrain
    }
}

impl SimModule for TerrainModule {
    fn name(&self) -> &'static str {
        "terrain"
    }

    fn step(&mut self, _time: autonomy_common::time::SimTime, _dt: f64) -> autonomy_common::error::Result<()> {
        // Terrain is static in current implementation
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_flat_terrain() {
        let terrain = FlatTerrain::new(10.0);
        assert_eq!(terrain.height_at(Vec3::new(100.0, 0.0, 200.0)), 10.0);
        assert_eq!(terrain.normal_at(Vec3::zeros()), Vec3::y_axis());
    }

    #[test]
    fn test_procedural_terrain() {
        let config = TerrainConfig {
            size: 100.0,
            resolution: 1.0,
            max_height: 50.0,
            seed: 123,
            ..Default::default()
        };
        let terrain = Terrain::new(config);
        
        let h = terrain.height_at(Vec3::zeros());
        assert!(h >= 0.0 && h <= 50.0);
        
        let n = terrain.normal_at(Vec3::zeros());
        assert!((n.magnitude() - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_terrain_deterministic() {
        let config = TerrainConfig {
            size: 100.0,
            resolution: 1.0,
            max_height: 50.0,
            seed: 456,
            ..Default::default()
        };
        let terrain1 = Terrain::new(config.clone());
        let terrain2 = Terrain::new(config);

        let h1 = terrain1.height_at(Vec3::new(10.0, 0.0, 20.0));
        let h2 = terrain2.height_at(Vec3::new(10.0, 0.0, 20.0));
        assert_eq!(h1, h2);
    }
}
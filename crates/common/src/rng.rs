//! Deterministic random number generation for simulation

use rand::{Rng, SeedableRng};
use rand_distr::{Distribution, Normal, Uniform};
use std::cell::RefCell;
use std::sync::atomic::{AtomicU64, Ordering};

/// Simulation RNG using StdRng (deterministic, seedable)
pub type SimRng = rand::rngs::StdRng;

/// Thread-local RNG for simulation use
thread_local! {
    static THREAD_RNG: RefCell<Option<SimRng>> = const { RefCell::new(None) };
}

/// Global RNG seed counter for deriving child RNGs
static RNG_SEED_COUNTER: AtomicU64 = AtomicU64::new(0);

/// Initialize the global simulation RNG with a seed
pub fn init_sim_rng(seed: u64) {
    let rng = SimRng::seed_from_u64(seed);
    THREAD_RNG.with(|r| *r.borrow_mut() = Some(rng));
    RNG_SEED_COUNTER.store(0, Ordering::Relaxed);
}

/// Get the thread-local RNG, initializing with a derived seed if needed
pub fn sim_rng() -> SimRng {
    THREAD_RNG.with(|r| {
        let mut cell = r.borrow_mut();
        if cell.is_none() {
            let base_seed = RNG_SEED_COUNTER.fetch_add(1, Ordering::Relaxed);
            *cell = Some(SimRng::seed_from_u64(base_seed));
        }
        cell.as_ref().unwrap().clone()
    })
}

/// Create a new independent RNG with a derived seed
pub fn derive_rng() -> SimRng {
    let seed = RNG_SEED_COUNTER.fetch_add(1, Ordering::Relaxed);
    SimRng::seed_from_u64(seed)
}

/// Create an RNG from a specific seed (for reproducible sub-systems)
pub fn seeded_rng(seed: u64) -> SimRng {
    SimRng::seed_from_u64(seed)
}

/// Sample from a uniform distribution [low, high)
pub fn sample_uniform(rng: &mut SimRng, low: f64, high: f64) -> f64 {
    Uniform::new(low, high).sample(rng)
}

/// Sample from a normal distribution
pub fn sample_normal(rng: &mut SimRng, mean: f64, std_dev: f64) -> f64 {
    Normal::new(mean, std_dev).expect("invalid normal distribution").sample(rng)
}

/// Sample a boolean with given probability
pub fn sample_bool(rng: &mut SimRng, p: f64) -> bool {
    rng.gen::<f64>() < p
}

/// Deterministic shuffle using Fisher-Yates
pub fn shuffle<T>(slice: &mut [T], rng: &mut SimRng) {
    for i in (1..slice.len()).rev() {
        let j = rng.gen_range(0..=i);
        slice.swap(i, j);
    }
}

/// Chooses a random element from a slice
pub fn choose<'a, T>(slice: &'a [T], rng: &mut SimRng) -> Option<&'a T> {
    if slice.is_empty() {
        None
    } else {
        let idx = rng.gen_range(0..slice.len());
        Some(&slice[idx])
    }
}

/// Chooses a random element mutably from a slice
pub fn choose_mut<'a, T>(slice: &'a mut [T], rng: &mut SimRng) -> Option<&'a mut T> {
    if slice.is_empty() {
        None
    } else {
        let idx = rng.gen_range(0..slice.len());
        Some(&mut slice[idx])
    }
}

/// RNG context for components that need their own deterministic RNG
#[derive(Clone, Copy, Debug)]
pub struct RngContext {
    seed: u64,
}

impl RngContext {
    pub fn new(seed: u64) -> Self {
        Self { seed }
    }

    pub fn derive(&self, salt: u64) -> Self {
        Self {
            seed: self.seed.wrapping_add(salt).wrapping_mul(0x9E3779B97F4A7C15),
        }
    }

    pub fn make_rng(&self) -> SimRng {
        SimRng::seed_from_u64(self.seed)
    }
}

impl Default for RngContext {
    fn default() -> Self {
        let seed = RNG_SEED_COUNTER.fetch_add(1, Ordering::Relaxed);
        Self { seed }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_deterministic_rng() {
        init_sim_rng(42);
        let mut rng1 = sim_rng();
        let mut rng2 = sim_rng();

        let a: Vec<f64> = (0..100).map(|_| rng1.gen()).collect();
        let b: Vec<f64> = (0..100).map(|_| rng2.gen()).collect();

        assert_eq!(a, b);
    }

    #[test]
    fn test_derive_rng() {
        init_sim_rng(123);
        let rng1 = derive_rng();
        let rng2 = derive_rng();

        let a: Vec<f64> = (0..10).map(|_| rng1.clone().gen()).collect();
        let b: Vec<f64> = (0..10).map(|_| rng2.clone().gen()).collect();

        assert_ne!(a, b);
    }

    #[test]
    fn test_seeded_rng() {
        let rng1 = seeded_rng(999);
        let rng2 = seeded_rng(999);

        let a: Vec<f64> = (0..10).map(|_| rng1.clone().gen()).collect();
        let b: Vec<f64> = (0..10).map(|_| rng2.clone().gen()).collect();

        assert_eq!(a, b);
    }

    #[test]
    fn test_rng_context() {
        let ctx = RngContext::new(42);
        let mut rng1 = ctx.make_rng();
        let mut rng2 = ctx.make_rng();

        let a: Vec<f64> = (0..10).map(|_| rng1.gen()).collect();
        let b: Vec<f64> = (0..10).map(|_| rng2.gen()).collect();

        assert_eq!(a, b);
    }
}
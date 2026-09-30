//! Error types for Autonomy Lab

use thiserror::Error;

#[derive(Error, Debug)]
pub enum AutonomyError {
    #[error("Simulation error: {0}")]
    Simulation(String),

    #[error("Physics error: {0}")]
    Physics(String),

    #[error("Vehicle error: {0}")]
    Vehicle(String),

    #[error("Sensor error: {0}")]
    Sensor(String),

    #[error("Estimation error: {0}")]
    Estimation(String),

    #[error("Planning error: {0}")]
    Planning(String),

    #[error("Control error: {0}")]
    Control(String),

    #[error("Fleet error: {0}")]
    Fleet(String),

    #[error("Network error: {0}")]
    Network(String),

    #[error("WASM error: {0}")]
    Wasm(String),

    #[error("Scenario error: {0}")]
    Scenario(String),

    #[error("Replay error: {0}")]
    Replay(String),

    #[error("Configuration error: {0}")]
    Config(String),

    #[error("Serialization error: {0}")]
    Serialization(#[from] postcard::Error),

    #[error("Bincode error: {0}")]
    Bincode(#[from] bincode::Error),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Invalid state: {0}")]
    InvalidState(String),

    #[error("Not found: {0}")]
    NotFound(String),

    #[error("Timeout: {0}")]
    Timeout(String),
}

pub type Result<T> = std::result::Result<T, AutonomyError>;
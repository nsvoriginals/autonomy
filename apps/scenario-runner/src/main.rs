//! Scenario runner application

use autonomy_common::error::Result;
use autonomy_scenarios::loader::{load_scenario, validate_scenario};
use clap::Parser;
use std::path::PathBuf;
use tracing::{info, error};

#[derive(Parser, Debug)]
#[command(name = "autonomy-scenario-runner", version, about = "Autonomy Lab Scenario Runner")]
struct Args {
    /// Scenario file to run
    #[arg(short, long)]
    scenario: PathBuf,
    
    /// Validate only, don't run
    #[arg(long)]
    validate_only: bool,
    
    /// Generate scenario from template
    #[arg(long)]
    generate: Option<String>,
    
    /// Output path for generated scenario
    #[arg(long)]
    output: Option<PathBuf>,
    
    /// Random seed
    #[arg(short, long)]
    seed: Option<u64>,
    
    /// Log level
    #[arg(short, long, default_value = "info")]
    log_level: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    
    init_logging(&args.log_level);
    
    info!("Starting Autonomy Lab Scenario Runner");
    
    if let Some(template_name) = args.generate {
        info!("Generating scenario from template: {}", template_name);
        // Would generate scenario
        return Ok(());
    }
    
    info!("Loading scenario: {}", args.scenario.display());
    let scenario = load_scenario(&args.scenario)?;
    
    info!("Validating scenario...");
    validate_scenario(&scenario)?;
    
    info!("Scenario valid!");
    info!("Name: {}", scenario.metadata.name);
    info!("Vehicles: {}", scenario.vehicles.len());
    info!("Missions: {}", scenario.missions.len());
    info!("Zones: {}", scenario.zones.len());
    info!("Failures: {}", scenario.failures.len());
    
    if args.validate_only {
        return Ok(());
    }
    
    // Would run scenario
    info!("Running scenario... (not yet implemented)");
    
    Ok(())
}

fn init_logging(level: &str) {
    use tracing_subscriber::{fmt, EnvFilter};
    
    let filter = EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| EnvFilter::new(level));
    
    fmt()
        .with_env_filter(filter)
        .init();
}
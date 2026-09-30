//! Fleet console application

use autonomy_common::error::Result;
use autonomy_common::ids::{VehicleId, VehicleType};
use clap::Parser;
use std::path::PathBuf;
use tracing::{info, warn};

#[derive(Parser, Debug)]
#[command(name = "autonomy-fleet-console", version, about = "Autonomy Lab Fleet Console")]
struct Args {
    /// Connect to simulator host
    #[arg(short, long, default_value = "127.0.0.1:5000")]
    connect: String,
    
    /// Scenario file to load
    #[arg(short, long)]
    scenario: Option<PathBuf>,
    
    /// Log level
    #[arg(short, long, default_value = "info")]
    log_level: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    
    init_logging(&args.log_level);
    
    info!("Starting Autonomy Lab Fleet Console");
    info!("Connecting to: {}", args.connect);
    
    // Would connect to simulator and run UI
    // For now, just print help
    println!("Fleet Console - Connect to simulator at {}", args.connect);
    println!("Press Ctrl+C to exit");
    
    // Run UI event loop
    tokio::signal::ctrl_c().await?;
    
    info!("Shutting down");
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
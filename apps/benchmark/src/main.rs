//! Benchmark runner application

use autonomy_common::error::Result;
use autonomy_common::ids::AlgorithmId;
use autonomy_metrics::benchmark::{BenchmarkConfig, BenchmarkRunner};
use clap::Parser;
use std::path::PathBuf;
use tracing::{info, error};

#[derive(Parser, Debug)]
#[command(name = "autonomy-benchmark", version, about = "Autonomy Lab Benchmark Runner")]
struct Args {
    /// Scenario file to benchmark
    #[arg(short, long)]
    scenario: PathBuf,
    
    /// Algorithm IDs to benchmark (comma-separated)
    #[arg(short, long)]
    algorithms: String,
    
    /// Number of runs per algorithm
    #[arg(short, long, default_value = "100")]
    runs: usize,
    
    /// Base random seed
    #[arg(long, default_value = "42")]
    seed: u64,
    
    /// Output file for results (JSON)
    #[arg(short, long)]
    output: Option<PathBuf>,
    
    /// Run in parallel
    #[arg(long, default_value = "true")]
    parallel: bool,
    
    /// Maximum steps per run
    #[arg(long)]
    max_steps: Option<u64>,
    
    /// Log level
    #[arg(short, long, default_value = "info")]
    log_level: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    let args = Args::parse();
    
    init_logging(&args.log_level);
    
    info!("Starting Autonomy Lab Benchmark");
    
    // Parse algorithm IDs
    let algorithm_ids: Result<Vec<AlgorithmId>, _> = args.algorithms
        .split(',')
        .map(|s| s.trim().parse())
        .collect();
    
    let algorithm_ids = algorithm_ids?;
    
    if algorithm_ids.is_empty() {
        error!("No algorithms specified");
        return Err(autonomy_common::error::AutonomyError::Config("No algorithms specified".into()));
    }
    
    info!("Benchmarking {} algorithms", algorithm_ids.len());
    info!("Runs per algorithm: {}", args.runs);
    
    let config = BenchmarkConfig {
        scenario_path: args.scenario.to_string_lossy().to_string(),
        algorithm_ids,
        runs: args.runs,
        max_steps: args.max_steps,
        seed_base: args.seed,
        parallel: args.parallel,
    };
    
    let runner = BenchmarkRunner::new(config);
    let report = runner.run().await?;
    
    info!("Benchmark completed!");
    info!("Success rate: {:.2}%", report.success_rate * 100.0);
    info!("Mission completion rate: {:.2}%", report.mission_completion_rate * 100.0);
    info!("Mean mission time: {:.2}s", report.mission_time.mean);
    info!("Mean position error: {:.2}m", report.position_error_mean.mean);
    info!("Mean energy: {:.2}Wh", report.energy_consumed.mean);
    info!("Avg CPU time: {:.2}ms", report.avg_cpu_time_ms.mean);
    
    // Save results if output specified
    if let Some(output_path) = args.output {
        let json = serde_json::to_string_pretty(&report)?;
        std::fs::write(output_path, json)?;
        info!("Results saved to {}", output_path.display());
    }
    
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
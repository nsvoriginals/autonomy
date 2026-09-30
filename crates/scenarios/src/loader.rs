//! Scenario loader

use autonomy_common::error::Result;
use crate::definition::Scenario;
use std::path::Path;

/// Load scenario from file
pub fn load_scenario(path: &Path) -> Result<Scenario> {
    let ext = path.extension().and_then(|s| s.to_str()).unwrap_or("");
    
    let content = std::fs::read_to_string(path)?;
    
    match ext {
        "yaml" | "yml" => serde_yaml::from_str(&content).map_err(|e| autonomy_common::error::AutonomyError::Scenario(e.to_string())),
        "json" => serde_json::from_str(&content).map_err(|e| autonomy_common::error::AutonomyError::Scenario(e.to_string())),
        _ => Err(autonomy_common::error::AutonomyError::Scenario("Unsupported format".into())),
    }
}

/// Save scenario to file
pub fn save_scenario(scenario: &Scenario, path: &Path) -> Result<()> {
    let ext = path.extension().and_then(|s| s.to_str()).unwrap_or("");
    
    let content = match ext {
        "yaml" | "yml" => serde_yaml::to_string(scenario).map_err(|e| autonomy_common::error::AutonomyError::Scenario(e.to_string()))?,
        "json" => serde_json::to_string_pretty(scenario).map_err(|e| autonomy_common::error::AutonomyError::Scenario(e.to_string()))?,
        _ => return Err(autonomy_common::error::AutonomyError::Scenario("Unsupported format".into())),
    };
    
    std::fs::write(path, content)?;
    Ok(())
}

/// Validate scenario
pub fn validate_scenario(scenario: &Scenario) -> Result<()> {
    // Check required fields
    if scenario.metadata.name.is_empty() {
        return Err(autonomy_common::error::AutonomyError::Scenario("Scenario name is empty".into()));
    }
    
    if scenario.vehicles.is_empty() {
        return Err(autonomy_common::error::AutonomyError::Scenario("No vehicles defined".into()));
    }
    
    // Check for duplicate IDs
    let mut vehicle_ids = std::collections::HashSet::new();
    for v in &scenario.vehicles {
        if !vehicle_ids.insert(v.id) {
            return Err(autonomy_common::error::AutonomyError::Scenario(format!("Duplicate vehicle ID: {:?}", v.id)));
        }
    }
    
    let mut mission_ids = std::collections::HashSet::new();
    for m in &scenario.missions {
        if !mission_ids.insert(m.id) {
            return Err(autonomy_common::error::AutonomyError::Scenario(format!("Duplicate mission ID: {:?}", m.id)));
        }
    }
    
    // Validate missions reference existing vehicles
    let vehicle_ids: std::collections::HashSet<_> = scenario.vehicles.iter().map(|v| v.id).collect();
    for mission in &scenario.missions {
        for vid in &mission.assigned_vehicles {
            if !vehicle_ids.contains(vid) {
                return Err(autonomy_common::error::AutonomyError::Scenario(format!("Mission references unknown vehicle: {:?}", vid)));
            }
        }
    }
    
    Ok(())
}
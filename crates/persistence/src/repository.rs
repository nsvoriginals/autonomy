//! Repository patterns for data access

use autonomy_common::error::Result;
use autonomy_common::ids::{AlgorithmId, MissionId, RecordingId, ScenarioId, VehicleId};
use autonomy_scenarios::definition::Scenario;
use sqlx::SqlitePool;
use std::sync::Arc;

/// Scenario repository
pub struct ScenarioRepository {
    pool: Arc<SqlitePool>,
}

impl ScenarioRepository {
    pub fn new(pool: Arc<SqlitePool>) -> Self {
        Self { pool }
    }

    pub async fn save(&self, scenario: &Scenario) -> Result<()> {
        let data = serde_yaml::to_vec(scenario)?;
        let now = chrono::Utc::now().timestamp_millis();
        
        sqlx::query(
            r#"
            INSERT OR REPLACE INTO scenarios (id, name, description, version, seed, duration, data, created_at, updated_at)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
            "#
        )
        .bind(scenario.metadata.name.clone()) // This would be the ID
        .bind(&scenario.metadata.name)
        .bind(&scenario.metadata.description)
        .bind(&scenario.metadata.version)
        .bind(scenario.metadata.seed as i64)
        .bind(scenario.metadata.duration)
        .bind(&data)
        .bind(now)
        .bind(now)
        .execute(self.pool.as_ref())
        .await?;
        
        Ok(())
    }

    pub async fn get(&self, id: ScenarioId) -> Result<Option<Scenario>> {
        let row = sqlx::query(
            "SELECT data FROM scenarios WHERE id = ?"
        )
        .bind(id.to_string())
        .fetch_optional(self.pool.as_ref())
        .await?;
        
        if let Some(row) = row {
            let data: Vec<u8> = row.get("data");
            let scenario: Scenario = serde_yaml::from_slice(&data)?;
            Ok(Some(scenario))
        } else {
            Ok(None)
        }
    }

    pub async fn list(&self) -> Result<Vec<Scenario>> {
        let rows = sqlx::query(
            "SELECT data FROM scenarios ORDER BY updated_at DESC"
        )
        .fetch_all(self.pool.as_ref())
        .await?;
        
        let mut scenarios = Vec::new();
        for row in rows {
            let data: Vec<u8> = row.get("data");
            if let Ok(scenario) = serde_yaml::from_slice(&data) {
                scenarios.push(scenario);
            }
        }
        
        Ok(scenarios)
    }

    pub async fn delete(&self, id: ScenarioId) -> Result<()> {
        sqlx::query("DELETE FROM scenarios WHERE id = ?")
            .bind(id.to_string())
            .execute(self.pool.as_ref())
            .await?;
        Ok(())
    }
}

/// Recording repository
pub struct RecordingRepository {
    pool: Arc<SqlitePool>,
}

impl RecordingRepository {
    pub fn new(pool: Arc<SqlitePool>) -> Self {
        Self { pool }
    }

    pub async fn save(&self, recording: &crate::replay::format::ReplayHeader, file_path: &str) -> Result<()> {
        let metadata = serde_json::to_string(&recording.metadata)?;
        let now = chrono::Utc::now().timestamp_millis();
        
        sqlx::query(
            r#"
            INSERT INTO recordings (id, scenario_id, name, seed, start_time, end_time, frame_count, file_path, metadata, created_at)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            "#
        )
        .bind(recording.scenario_name.clone()) // Using scenario name as ID for now
        .bind("") // scenario_id
        .bind(&recording.scenario_name)
        .bind(recording.seed as i64)
        .bind(recording.start_time as i64)
        .bind(recording.end_time as i64)
        .bind(recording.frame_count as i64)
        .bind(file_path)
        .bind(&metadata)
        .bind(now)
        .execute(self.pool.as_ref())
        .await?;
        
        Ok(())
    }

    pub async fn get(&self, id: RecordingId) -> Result<Option<crate::replay::format::ReplayHeader>> {
        let row = sqlx::query(
            "SELECT * FROM recordings WHERE id = ?"
        )
        .bind(id.to_string())
        .fetch_optional(self.pool.as_ref())
        .await?;
        
        if let Some(row) = row {
            // Would reconstruct header
            Ok(None)
        } else {
            Ok(None)
        }
    }
}

/// Benchmark repository
pub struct BenchmarkRepository {
    pool: Arc<SqlitePool>,
}

impl BenchmarkRepository {
    pub fn new(pool: Arc<SqlitePool>) -> Self {
        Self { pool }
    }

    pub async fn save_result(&self, result: &autonomy_metrics::report::BenchmarkResult) -> Result<()> {
        let custom = serde_json::to_string(&result.custom_metrics)?;
        let now = chrono::Utc::now().timestamp_millis();
        
        sqlx::query(
            r#"
            INSERT INTO benchmark_results (id, algorithm_id, scenario_id, seed, run_id, success, mission_completed, mission_time, position_error_mean, position_error_max, energy_consumed, avg_cpu_time_ms, max_cpu_time_ms, communication_loss, custom_metrics, created_at)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            "#
        )
        .bind(uuid::Uuid::new_v4().to_string())
        .bind(result.algorithm_id.to_string())
        .bind("") // scenario_id
        .bind(result.seed as i64)
        .bind(result.run_id as i64)
        .bind(result.success as i64)
        .bind(result.mission_completed as i64)
        .bind(result.mission_time)
        .bind(result.position_error_mean)
        .bind(result.position_error_max)
        .bind(result.energy_consumed)
        .bind(result.avg_cpu_time_ms)
        .bind(result.max_cpu_time_ms)
        .bind(result.communication_loss)
        .bind(&custom)
        .bind(now)
        .execute(self.pool.as_ref())
        .await?;
        
        Ok(())
    }

    pub async fn get_results(&self, algorithm_id: AlgorithmId) -> Result<Vec<autonomy_metrics::report::BenchmarkResult>> {
        let rows = sqlx::query(
            "SELECT * FROM benchmark_results WHERE algorithm_id = ? ORDER BY created_at"
        )
        .bind(algorithm_id.to_string())
        .fetch_all(self.pool.as_ref())
        .await?;
        
        let mut results = Vec::new();
        for row in rows {
            // Would deserialize
        }
        
        Ok(results)
    }
}
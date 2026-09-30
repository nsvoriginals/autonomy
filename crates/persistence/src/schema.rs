//! Database schema

/// SQL schema for persistence
pub const SCHEMA: &str = r#"
-- Scenarios table
CREATE TABLE IF NOT EXISTS scenarios (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    description TEXT,
    version TEXT,
    seed INTEGER NOT NULL,
    duration REAL NOT NULL,
    data BLOB NOT NULL, -- Serialized scenario (YAML/JSON)
    created_at INTEGER NOT NULL,
    updated_at INTEGER NOT NULL
);

-- Recordings table
CREATE TABLE IF NOT EXISTS recordings (
    id TEXT PRIMARY KEY,
    scenario_id TEXT NOT NULL,
    name TEXT NOT NULL,
    seed INTEGER NOT NULL,
    start_time INTEGER NOT NULL,
    end_time INTEGER NOT NULL,
    frame_count INTEGER NOT NULL,
    file_path TEXT NOT NULL,
    metadata TEXT, -- JSON metadata
    created_at INTEGER NOT NULL,
    FOREIGN KEY (scenario_id) REFERENCES scenarios(id)
);

-- Vehicles table
CREATE TABLE IF NOT EXISTS vehicles (
    id TEXT PRIMARY KEY,
    vehicle_type INTEGER NOT NULL,
    callsign TEXT NOT NULL,
    config BLOB NOT NULL, -- Serialized vehicle config
    created_at INTEGER NOT NULL
);

-- Missions table
CREATE TABLE IF NOT EXISTS missions (
    id TEXT PRIMARY KEY,
    scenario_id TEXT NOT NULL,
    mission_type TEXT NOT NULL,
    state INTEGER NOT NULL,
    assigned_vehicles TEXT NOT NULL, -- JSON array of vehicle IDs
    start_time INTEGER,
    end_time INTEGER,
    parameters TEXT, -- JSON
    created_at INTEGER NOT NULL,
    FOREIGN KEY (scenario_id) REFERENCES scenarios(id)
);

-- Tasks table
CREATE TABLE IF NOT EXISTS tasks (
    id TEXT PRIMARY KEY,
    mission_id TEXT NOT NULL,
    task_type TEXT NOT NULL,
    status INTEGER NOT NULL,
    assigned_vehicle TEXT,
    position_x REAL,
    position_y REAL,
    position_z REAL,
    parameters TEXT, -- JSON
    priority INTEGER NOT NULL,
    created_at INTEGER NOT NULL,
    completed_at INTEGER,
    FOREIGN KEY (mission_id) REFERENCES missions(id)
);

-- Telemetry events table (for querying)
CREATE TABLE IF NOT EXISTS telemetry_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    recording_id TEXT NOT NULL,
    time INTEGER NOT NULL,
    event_type TEXT NOT NULL,
    vehicle_id TEXT,
    data BLOB NOT NULL, -- Serialized event
    FOREIGN KEY (recording_id) REFERENCES recordings(id)
);

CREATE INDEX IF NOT EXISTS idx_telemetry_recording_time ON telemetry_events(recording_id, time);
CREATE INDEX IF NOT EXISTS idx_telemetry_vehicle ON telemetry_events(vehicle_id);

-- Benchmark results table
CREATE TABLE IF NOT EXISTS benchmark_results (
    id TEXT PRIMARY KEY,
    algorithm_id TEXT NOT NULL,
    scenario_id TEXT NOT NULL,
    seed INTEGER NOT NULL,
    run_id INTEGER NOT NULL,
    success INTEGER NOT NULL,
    mission_completed INTEGER NOT NULL,
    mission_time REAL NOT NULL,
    position_error_mean REAL NOT NULL,
    position_error_max REAL NOT NULL,
    energy_consumed REAL NOT NULL,
    avg_cpu_time_ms REAL NOT NULL,
    max_cpu_time_ms REAL NOT NULL,
    communication_loss REAL NOT NULL,
    custom_metrics TEXT, -- JSON
    created_at INTEGER NOT NULL,
    FOREIGN KEY (scenario_id) REFERENCES scenarios(id)
);

CREATE INDEX IF NOT EXISTS idx_benchmark_algorithm ON benchmark_results(algorithm_id);
CREATE INDEX IF NOT EXISTS idx_benchmark_scenario ON benchmark_results(scenario_id);

-- Algorithm modules table
CREATE TABLE IF NOT EXISTS algorithm_modules (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    version TEXT NOT NULL,
    wasm_binary BLOB NOT NULL,
    abi_version INTEGER NOT NULL,
    description TEXT,
    created_at INTEGER NOT NULL
);

-- Fleet configurations
CREATE TABLE IF NOT EXISTS fleet_configs (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL,
    vehicle_ids TEXT NOT NULL, -- JSON array
    formation_type TEXT,
    spacing REAL,
    created_at INTEGER NOT NULL
);
"#;

/// Migration SQL
pub const MIGRATIONS: &str = r#"
-- Migration 001: Initial schema
-- Run the SCHEMA above

-- Migration 002: Add index for telemetry queries
CREATE INDEX IF NOT EXISTS idx_telemetry_type ON telemetry_events(event_type);

-- Migration 003: Add vehicle health tracking
ALTER TABLE vehicles ADD COLUMN last_health INTEGER;
ALTER TABLE vehicles ADD COLUMN last_position_x REAL;
ALTER TABLE vehicles ADD COLUMN last_position_y REAL;
ALTER TABLE vehicles ADD COLUMN last_position_z REAL;
"#;
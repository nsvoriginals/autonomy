//! Replay format definitions

use autonomy_common::time::SimTime;
use serde::{Deserialize, Serialize};

/// Replay file header
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ReplayHeader {
    pub magic: [u8; 8],
    pub version: u32,
    pub scenario_name: String,
    pub seed: u64,
    pub start_time: u64,
    pub end_time: u64,
    pub frame_count: u64,
    pub metadata: ReplayMetadata,
}

/// Replay metadata
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ReplayMetadata {
    pub vehicle_count: u32,
    pub mission_count: u32,
    pub duration_seconds: f64,
    pub custom: std::collections::HashMap<String, String>,
}

/// Replay frame
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ReplayFrame {
    pub time: SimTime,
    pub event_type: String,
    pub data: Vec<u8>,
}

/// Compressed replay frame (for storage)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CompressedReplayFrame {
    pub time: SimTime,
    pub event_type: u16, // Index into event type table
    pub data: Vec<u8>,
}

/// Event type table for compression
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct EventTypeTable {
    pub types: Vec<String>,
    pub indices: std::collections::HashMap<String, u16>,
}

impl EventTypeTable {
    pub fn get_or_insert(&mut self, event_type: &str) -> u16 {
        if let Some(&idx) = self.indices.get(event_type) {
            return idx;
        }
        let idx = self.types.len() as u16;
        self.types.push(event_type.to_string());
        self.indices.insert(event_type.to_string(), idx);
        idx
    }
}
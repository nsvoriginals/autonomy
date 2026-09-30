//! Replay recorder

use autonomy_common::error::Result;
use autonomy_common::telemetry::TelemetryEvent;
use autonomy_common::time::SimTime;
use autonomy_common::traits::Recorder;
use crate::format::{ReplayHeader, ReplayFrame, ReplayMetadata};
use bincode;
use flate2::write::ZlibEncoder;
use flate2::Compression;
use parking_lot::RwLock;
use std::collections::VecDeque;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::sync::Arc;

/// Replay recorder
pub struct ReplayRecorder {
    file: RwLock<Option<BufWriter<ZlibEncoder<File>>>>,
    header: ReplayHeader,
    frame_buffer: RwLock<VecDeque<ReplayFrame>>,
    buffer_size: usize,
    flush_interval: usize,
    frames_written: RwLock<usize>,
    bytes_written: RwLock<u64>,
    start_time: SimTime,
}

impl ReplayRecorder {
    pub fn new(path: &Path, scenario_name: &str, seed: u64, buffer_size: usize) -> Result<Self> {
        let file = File::create(path)?;
        let encoder = ZlibEncoder::new(file, Compression::default());
        let writer = BufWriter::new(encoder);
        
        let header = ReplayHeader {
            magic: *b"AUTONOMY",
            version: 1,
            scenario_name: scenario_name.to_string(),
            seed,
            start_time: 0,
            end_time: 0,
            frame_count: 0,
            metadata: ReplayMetadata::default(),
        };
        
        Ok(Self {
            file: RwLock::new(Some(writer)),
            header,
            frame_buffer: RwLock::new(VecDeque::new()),
            buffer_size,
            flush_interval: buffer_size / 10,
            frames_written: RwLock::new(0),
            bytes_written: RwLock::new(0),
            start_time: SimTime::ZERO,
        })
    }

    pub fn set_start_time(&mut self, time: SimTime) {
        self.start_time = time;
        self.header.start_time = time.ticks();
    }

    pub fn record_event(&self, event: &TelemetryEvent) -> Result<()> {
        let frame = ReplayFrame {
            time: autonomy_common::time::SimTime::ZERO, // Would use actual time
            event_type: format!("{:?}", event),
            data: bincode::serialize(event)?,
        };
        
        let mut buffer = self.frame_buffer.write();
        buffer.push_back(frame);
        
        if buffer.len() >= self.flush_interval {
            self.flush_buffer()?;
        }
        
        Ok(())
    }

    fn flush_buffer(&self) -> Result<()> {
        let mut buffer = self.frame_buffer.write();
        let mut file = self.file.write();
        
        if let Some(writer) = file.as_mut() {
            for frame in buffer.drain(..) {
                let data = bincode::serialize(&frame)?;
                let len = data.len() as u32;
                writer.write_all(&len.to_le_bytes())?;
                writer.write_all(&data)?;
                *self.frames_written.write() += 1;
                *self.bytes_written.write() += (len + 4) as u64;
            }
            writer.flush()?;
        }
        
        Ok(())
    }

    pub fn finalize(&mut self, end_time: SimTime) -> Result<()> {
        self.header.end_time = end_time.ticks();
        self.header.frame_count = *self.frames_written.read();
        
        // Flush remaining
        self.flush_buffer()?;
        
        // Write header at the beginning
        let mut file = self.file.write();
        if let Some(writer) = file.as_mut() {
            // Seek to beginning and write header
            // Note: This is simplified - in practice would need proper file positioning
            let header_data = bincode::serialize(&self.header)?;
            writer.write_all(&header_data)?;
            writer.flush()?;
        }
        
        Ok(())
    }

    pub fn frames_written(&self) -> usize {
        *self.frames_written.read()
    }

    pub fn bytes_written(&self) -> u64 {
        *self.bytes_written.read()
    }
}

impl Recorder for ReplayRecorder {
    fn record(&mut self, event: &TelemetryEvent) -> Result<()> {
        self.record_event(event)
    }

    fn flush(&mut self) -> Result<()> {
        self.flush_buffer()
    }
}
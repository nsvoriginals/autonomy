//! Replay replayer

use autonomy_common::error::Result;
use autonomy_common::telemetry::TelemetryEvent;
use autonomy_common::time::SimTime;
use autonomy_common::traits::Replayer;
use crate::format::{ReplayHeader, ReplayFrame};
use bincode;
use flate2::read::ZlibDecoder;
use parking_lot::RwLock;
use std::fs::File;
use std::io::{BufReader, Read};
use std::path::Path;
use std::sync::Arc;

/// Replay replayer
pub struct ReplayReplayer {
    reader: RwLock<BufReader<ZlibDecoder<File>>>,
    header: ReplayHeader,
    current_frame: RwLock<Option<ReplayFrame>>,
    current_time: RwLock<SimTime>,
    finished: RwLock<bool>,
}

impl ReplayReplayer {
    pub fn new(path: &Path) -> Result<Self> {
        let file = File::open(path)?;
        let decoder = ZlibDecoder::new(file);
        let mut reader = BufReader::new(decoder);
        
        // Read header
        let header: ReplayHeader = bincode::deserialize_from(&mut reader)?;
        
        Ok(Self {
            reader: RwLock::new(reader),
            header,
            current_frame: RwLock::new(None),
            current_time: RwLock::new(SimTime::new(header.start_time)),
            finished: RwLock::new(false),
        })
    }

    pub fn header(&self) -> &ReplayHeader {
        &self.header
    }

    pub fn next_event(&mut self) -> Result<Option<TelemetryEvent>> {
        let mut reader = self.reader.write();
        
        loop {
            // Read frame length
            let mut len_bytes = [0u8; 4];
            if reader.read_exact(&mut len_bytes).is_err() {
                *self.finished.write() = true;
                return Ok(None);
            }
            
            let len = u32::from_le_bytes(len_bytes) as usize;
            if len == 0 {
                continue;
            }
            
            // Read frame data
            let mut frame_data = vec![0u8; len];
            reader.read_exact(&mut frame_data)?;
            
            let frame: ReplayFrame = bincode::deserialize(&frame_data)?;
            
            *self.current_time.write() = frame.time;
            *self.current_frame.write() = Some(frame.clone());
            
            // Deserialize event
            let event: TelemetryEvent = bincode::deserialize(&frame.data)?;
            return Ok(Some(event));
        }
    }

    pub fn seek(&mut self, time: SimTime) -> Result<()> {
        // Simplified - in practice would need index
        *self.current_time.write() = time;
        Ok(())
    }

    pub fn current_time(&self) -> SimTime {
        *self.current_time.read()
    }

    pub fn is_finished(&self) -> bool {
        *self.finished.read()
    }

    pub fn progress(&self) -> f64 {
        if self.header.frame_count == 0 {
            return 0.0;
        }
        // Would track actual frame count
        0.0
    }
}

impl Replayer for ReplayReplayer {
    fn next_event(&mut self) -> Result<Option<TelemetryEvent>> {
        self.next_event()
    }

    fn seek(&mut self, time: SimTime) -> Result<()> {
        self.seek(time)
    }

    fn current_time(&self) -> SimTime {
        self.current_time()
    }
}
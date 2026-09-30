//! Network link simulation

use autonomy_common::frames::Vec3;
use autonomy_common::ids::{VehicleId, NetworkNodeId};
use autonomy_common::rng::SimRng;
use autonomy_common::time::SimTime;
use parking_lot::RwLock;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::sync::Arc;

/// Network link configuration
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LinkConfig {
    pub bandwidth_bps: f64,
    pub latency_ms: f64,
    pub jitter_ms: f64,
    pub packet_loss_rate: f64,
    pub max_packet_size: usize,
    pub queue_size: usize,
}

impl Default for LinkConfig {
    fn default() -> Self {
        Self {
            bandwidth_bps: 10_000_000.0, // 10 Mbps
            latency_ms: 50.0,
            jitter_ms: 10.0,
            packet_loss_rate: 0.01,
            max_packet_size: 1500,
            queue_size: 1000,
        }
    }
}

/// Simulated network link
pub struct NetworkLink {
    config: LinkConfig,
    source: VehicleId,
    destination: VehicleId,
    queue: RwLock<VecDeque<QueuedPacket>>,
    rng: RwLock<SimRng>,
    stats: RwLock<LinkStats>,
}

#[derive(Clone, Debug)]
struct QueuedPacket {
    data: Vec<u8>,
    send_time: SimTime,
    size: usize,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct LinkStats {
    pub packets_sent: u64,
    pub packets_received: u64,
    pub packets_lost: u64,
    pub packets_corrupted: u64,
    pub total_latency_ms: f64,
    pub total_bytes: u64,
    pub avg_latency_ms: f64,
    pub max_latency_ms: f64,
}

impl NetworkLink {
    pub fn new(source: VehicleId, destination: VehicleId, config: LinkConfig, rng: SimRng) -> Self {
        Self {
            config,
            source,
            destination,
            queue: RwLock::new(VecDeque::new()),
            rng: RwLock::new(rng),
            stats: RwLock::new(LinkStats::default()),
        }
    }

    pub fn send(&self, time: SimTime, data: Vec<u8>) -> Result<(), NetworkError> {
        if data.len() > self.config.max_packet_size {
            return Err(NetworkError::PacketTooLarge);
        }

        let mut queue = self.queue.write();
        if queue.len() >= self.config.queue_size {
            return Err(NetworkError::QueueFull);
        }

        queue.push_back(QueuedPacket {
            data,
            send_time: time,
            size: data.len(),
        });

        let mut stats = self.stats.write();
        stats.packets_sent += 1;
        stats.total_bytes += data.len() as u64;

        Ok(())
    }

    pub fn receive(&self, current_time: SimTime) -> Vec<Vec<u8>> {
        let mut queue = self.queue.write();
        let mut received = Vec::new();
        let mut rng = self.rng.write();
        let mut stats = self.stats.write();

        while let Some(packet) = queue.pop_front() {
            // Check packet loss
            if autonomy_common::rng::sample_bool(&mut rng, self.config.packet_loss_rate) {
                stats.packets_lost += 1;
                continue;
            }

            // Calculate latency
            let base_latency = self.config.latency_ms / 1000.0;
            let jitter = autonomy_common::rng::sample_normal(&mut rng, 0.0, self.config.jitter_ms / 1000.0);
            let total_latency = (base_latency + jitter).max(0.0);

            let arrival_time = packet.send_time + autonomy_common::time::SimTime::from_seconds(total_latency);
            
            if arrival_time <= current_time {
                // Packet arrived
                stats.packets_received += 1;
                stats.total_latency_ms += total_latency * 1000.0;
                stats.max_latency_ms = stats.max_latency_ms.max(total_latency * 1000.0);
                stats.avg_latency_ms = stats.total_latency_ms / stats.packets_received as f64;
                received.push(packet.data);
            } else {
                // Not yet arrived, put back
                queue.push_front(packet);
                break; // Queue is ordered by send time
            }
        }

        received
    }

    pub fn get_stats(&self) -> LinkStats {
        self.stats.read().clone()
    }

    pub fn update_config(&mut self, config: LinkConfig) {
        self.config = config;
    }

    pub fn destination(&self) -> VehicleId {
        self.destination
    }
}

#[derive(Debug, thiserror::Error)]
pub enum NetworkError {
    #[error("Packet too large")]
    PacketTooLarge,
    #[error("Queue full")]
    QueueFull,
    #[error("Link down")]
    LinkDown,
}

/// Network link quality estimator
pub fn estimate_link_quality(source_pos: Vec3, dest_pos: Vec3, max_range: f64) -> f64 {
    let dist = (source_pos - dest_pos).magnitude();
    if dist > max_range {
        return 0.0;
    }
    
    // Quality decreases with distance
    1.0 - (dist / max_range).min(1.0)
}
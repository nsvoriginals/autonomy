//! Event bus for decoupled communication between subsystems

use crate::event_bus::EventBus;
use autonomy_common::telemetry::TelemetryEvent;
use autonomy_common::traits::{EventBus as EventBusTrait, EventSubscription};
use crossbeam::channel::{unbounded, Receiver, Sender};
use dashmap::DashMap;
use std::any::TypeId;
use std::collections::HashMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

/// Internal event wrapper with type information
struct TypedEvent {
    event: TelemetryEvent,
    type_name: &'static str,
}

/// High-performance event bus using crossbeam channels
pub struct SimulationEventBus {
    sender: Sender<TypedEvent>,
    subscribers: Arc<DashMap<String, Vec<Sender<TypedEvent>>>>,
    shutdown: Arc<AtomicBool>,
}

impl SimulationEventBus {
    pub fn new() -> Self {
        let (tx, _rx) = unbounded();
        Self {
            sender: tx,
            subscribers: Arc::new(DashMap::new()),
            shutdown: Arc::new(AtomicBool::new(false)),
        }
    }

    pub fn create_subscription(&self, event_types: &[&str]) -> Box<dyn EventSubscription> {
        let (tx, rx) = unbounded();
        
        for event_type in event_types {
            self.subscribers
                .entry(event_type.to_string())
                .or_default()
                .push(tx.clone());
        }

        Box::new(Subscription {
            receiver: rx,
            event_bus: self.clone(),
            event_types: event_types.iter().map(|s| s.to_string()).collect(),
        })
    }

    fn deliver(&self, event: TypedEvent) {
        if self.shutdown.load(Ordering::Relaxed) {
            return;
        }

        // Deliver to specific subscribers
        if let Some(subs) = self.subscribers.get(&event.type_name) {
            for tx in subs.iter() {
                let _ = tx.try_send(event.clone());
            }
        }

        // Deliver to wildcard subscribers
        if let Some(subs) = self.subscribers.get("*") {
            for tx in subs.iter() {
                let _ = tx.try_send(event.clone());
            }
        }
    }
}

impl Clone for SimulationEventBus {
    fn clone(&self) -> Self {
        Self {
            sender: self.sender.clone(),
            subscribers: self.subscribers.clone(),
            shutdown: self.shutdown.clone(),
        }
    }
}

impl EventBusTrait for SimulationEventBus {
    fn publish(&self, event: TelemetryEvent) {
        let type_name = event_type_name(&event);
        self.deliver(TypedEvent { event, type_name });
    }

    fn subscribe(&self, event_type: &str) -> Box<dyn EventSubscription> {
        self.create_subscription(&[event_type])
    }
}

struct Subscription {
    receiver: Receiver<TypedEvent>,
    event_bus: SimulationEventBus,
    event_types: Vec<String>,
}

impl EventSubscription for Subscription {
    fn next(&mut self) -> Option<TelemetryEvent> {
        self.receiver.recv().ok().map(|e| e.event)
    }

    fn try_next(&mut self) -> Option<TelemetryEvent> {
        self.receiver.try_recv().ok().map(|e| e.event)
    }
}

impl Drop for Subscription {
    fn drop(&mut self) {
        for event_type in &self.event_types {
            if let Some(mut subs) = self.event_bus.subscribers.get_mut(event_type) {
                subs.retain(|tx| !tx.same_channel(&self.receiver));
            }
        }
    }
}

fn event_type_name(event: &TelemetryEvent) -> &'static str {
    match event {
        TelemetryEvent::VehicleState(_) => "VehicleState",
        TelemetryEvent::SensorMeasurement(_) => "SensorMeasurement",
        TelemetryEvent::MissionEvent(_) => "MissionEvent",
        TelemetryEvent::HealthEvent(_) => "HealthEvent",
        TelemetryEvent::NetworkEvent(_) => "NetworkEvent",
        TelemetryEvent::AlgorithmEvent(_) => "AlgorithmEvent",
        TelemetryEvent::FleetEvent(_) => "FleetEvent",
        TelemetryEvent::SimulationEvent(_) => "SimulationEvent",
        TelemetryEvent::Custom(_) => "CustomEvent",
    }
}

impl Default for SimulationEventBus {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use autonomy_common::telemetry::{MissionEvent, MissionEventType, SimulationEvent, SimulationEventType};
    use autonomy_common::time::SimTime;
    use autonomy_common::ids::MissionId;

    #[test]
    fn test_event_bus_publish_subscribe() {
        let bus = SimulationEventBus::new();
        let mut sub = bus.subscribe("SimulationEvent");

        bus.publish(TelemetryEvent::SimulationEvent(SimulationEvent {
            time: SimTime::ZERO,
            event_type: SimulationEventType::Started,
            details: Default::default(),
        }));

        let event = sub.next();
        assert!(matches!(event, Some(TelemetryEvent::SimulationEvent(_))));
    }

    #[test]
    fn test_multiple_subscribers() {
        let bus = SimulationEventBus::new();
        let mut sub1 = bus.subscribe("MissionEvent");
        let mut sub2 = bus.subscribe("MissionEvent");

        bus.publish(TelemetryEvent::MissionEvent(MissionEvent {
            time: SimTime::ZERO,
            mission_id: MissionId::nil(),
            vehicle_id: Default::default(),
            task_id: None,
            event_type: MissionEventType::MissionStarted,
            details: Default::default(),
        }));

        assert!(sub1.next().is_some());
        assert!(sub2.next().is_some());
    }
}
# Autonomy Lab Architecture

## Overview

Autonomy Lab is a deterministic, Rust-first simulation and fleet-control platform for testing autonomous aerial and ground vehicles inside a 3D digital twin.

## Core Principles

1. **Determinism First** - Given identical seed, scenario, algorithm, and configuration, the simulation produces identical results
2. **Rust Throughout** - Entire stack in Rust; WASM for algorithm isolation
3. **Separation of Concerns** - Rendering, simulation, physics, autonomy, fleet, networking, telemetry are distinct crates
4. **Modularity** - Each subsystem is a replaceable crate with well-defined interfaces
5. **Observability** - Structured telemetry, tracing, metrics built-in
6. **Safety Boundaries** - No weapon/engagement functionality; focus on navigation, coordination, inspection, logistics

## Crate Organization

```
crates/
├── common/              # Shared types, traits, utilities, error handling
├── simulation-core/     # Simulation clock, deterministic timestep, event bus, state machine
├── simulation-runtime/  # High-level simulation orchestration, tick loop, pause/step/resume
├── physics/             # Physics integration, Rapier integration, collision detection
├── environment/         # World model, terrain, buildings, zones, coordinate frames
├── vehicle/             # Vehicle trait, base vehicle state, actuator abstraction
├── vehicle-uav/         # UAV dynamics, multirotor model, UAV-specific sensors
├── vehicle-ugv/         # UGV dynamics, wheeled/tracked model, UGV-specific sensors
├── sensors/             # Sensor abstraction, noise models, failure injection
├── estimation/          # State estimation: dead reckoning, complementary filter, EKF
├── planning/            # Planning abstractions: global, local, waypoint, graph-based
├── control/             # Control abstractions: PID, position, velocity, attitude
├── fleet/               # Fleet manager, registry, mission manager, task allocator
├── networking/          # Communication simulation: latency, loss, partitions, topology
├── telemetry/           # Structured telemetry, event types, serialization
├── scenarios/           # Scenario definition, parsing, procedural generation
├── wasm-runtime/        # WASM module loading, validation, execution, sandboxing
├── wasm-api/            # WASM ABI definitions, host functions, versioned messages
├── replay/              # Deterministic recording and replay
├── metrics/             # Performance metrics, benchmarking infrastructure
└── persistence/         # SQLite/Postgres persistence for scenarios, recordings
```

## Simulation Loop

```
simulation_time (fixed dt = 1/60s)
      |
      v
physics update (Rapier)
      |
      v
sensor update (with noise, latency, failure models)
      |
      v
state estimation (EKF, complementary filter, etc.)
      |
      v
planning (global -> local -> trajectory)
      |
      v
control (PID, position/velocity/attitude controllers)
      |
      v
vehicle state update (actuators -> dynamics)
      |
      v
telemetry emission
      |
      v
rendering (consumes simulation state, independent)
```

## Coordinate Frames

| Frame | Description | Convention |
|-------|-------------|------------|
| World (W) | Global simulation frame | ENU (East-North-Up), origin at scenario reference |
| Vehicle Body (B) | Vehicle center of mass | FRD (Forward-Right-Down) |
| Sensor (S) | Sensor-specific | Defined per sensor, documented |
| Camera (C) | Optical frame | Right-Down-Forward (OpenCV) |
| Local (L) | Local tangent plane | ENU, origin at vehicle spawn |

**Critical**: Never mix coordinate frames without explicit transformation. All transformations go through `common::frames`.

## Vehicle Abstraction

```rust
trait Vehicle: Send + Sync {
    fn id(&self) -> VehicleId;
    fn vehicle_type(&self) -> VehicleType;
    fn state(&self) -> VehicleState;
    fn sensors(&self) -> &SensorSuite;
    fn actuators(&self) -> &ActuatorSet;
    fn autonomy(&self) -> &dyn AutonomyModule;
    fn health(&self) -> HealthStatus;
    fn step(&mut self, dt: f64, estimation: &EstimatedState);
}
```

## Sensor Model

```
Ground Truth
      |
  Sensor Model (physics-based)
      |
  Noise Model (configurable: Gaussian, bias, dropout)
      |
  Latency Model (fixed + jitter)
      |
  Failure Injection (scripted or triggered)
      |
  Measurement Output
```

## WASM Autonomy System

### ABI Design

- **Versioned messages**: `AlgorithmInput v1`, `AlgorithmOutput v1`
- **Stable serialization**: Postcard (compact, deterministic)
- **Host functions**: Time, RNG (deterministic), logging, metrics
- **Sandboxing**: Fuel limits, memory limits, no filesystem/network/syscalls

### Algorithm Interface

```rust
// Input (host -> WASM)
struct AlgorithmInput {
    version: u32,
    timestamp: SimTime,
    vehicle_id: VehicleId,
    estimated_state: EstimatedState,
    sensor_measurements: SensorMeasurements,
    neighbors: Vec<NeighborState>,
    mission_state: MissionState,
    environment: EnvironmentSnapshot,
    config: AlgorithmConfig,
}

// Output (WASM -> host)
struct AlgorithmOutput {
    version: u32,
    desired_trajectory: Option<Trajectory>,
    desired_velocity: Option<Vector3>,
    desired_heading: Option<f64>,
    mission_transition: Option<MissionTransition>,
    fleet_requests: Vec<FleetRequest>,
}
```

## Fleet Architecture

```
FleetCoordinator
    |
    +-- VehicleRegistry (id, type, capabilities, status, health, connectivity)
    |
    +-- MissionManager (mission state machines, task decomposition)
    |
    +-- TaskAllocator (strategy: heuristic, Hungarian, auction)
    |
    +-- CommunicationManager (simulated network, topology)
    |
    +-- TelemetryManager (aggregation, alerting, persistence)
    |
    +-- HealthMonitor (battery, sensors, actuators, autonomy, estimator)
```

## Network Simulation

Each vehicle has a `CommInterface` with:
- Bandwidth limit
- Latency (base + jitter)
- Packet loss rate (configurable per link)
- Disconnect/reconnect events
- Network partitions (graph-based)

Topology updates as vehicles move (distance-based, LOS-based, infrastructure-based).

## Determinism Strategy

1. **Simulation RNG**: `StdRng` seeded from `--seed`, used for all stochastic processes
2. **Fixed Timestep**: `dt = 1/60` exactly, no variable timestep
3. **Deterministic Physics**: Rapier with `deterministic` feature
4. **Ordered Execution**: Vehicle updates in deterministic ID order
5. **WASM Fuel**: Deterministic instruction counting
6. **Event Ordering**: All events timestamped with simulation time, processed in order

## Replay System

Records:
- Scenario definition + seed
- Per-tick: vehicle states, sensor events, fleet events, network events, algorithm outputs
- Compressed binary format (bincode + zstd)

Replay reproduces exact simulation by feeding recorded events back into simulation loop.

## Scenario System

Data-driven scenarios in YAML:
```yaml
scenario:
  name: gps_denied_test
  seed: 12345
  duration: 300
environment:
  terrain: mountain
  weather: fog
vehicles:
  - id: UAV-001
    type: uav
    position: [0, 0, 50]
    autonomy: wasm:formation-v2
failures:
  - time: 30
    target: UAV-001
    type: gps_loss
missions:
  - id: survey-001
    type: area_coverage
    params: { area: [0,0,1000,1000] }
```

## Benchmarking

```
benchmark --scenario X --algorithm A --runs 1000
```

Outputs statistical comparison:
- Success rate
- Position error (mean, max, percentile)
- Energy consumption
- Mission completion time
- Communication degradation
- CPU/memory per algorithm

## Development Phases

| Phase | Focus | Milestone |
|-------|-------|-----------|
| 1 | Foundation | UAV moves in 3D world, deterministic clock, basic rendering |
| 2 | Vehicle Stack | Believable dynamics, imperfect sensors (IMU, GPS), battery |
| 3 | Autonomy | Estimator + planner + controller, waypoint mission |
| 4 | WASM | Rust algorithm -> WASM -> controls vehicle |
| 5 | Fleet | 10 vehicles, centralized orchestration |
| 6 | Distributed Fleet | Degraded comms, topology, reconnection |
| 7 | Digital Twin | Scenarios, failure injection, replay |
| 8 | Benchmarking | 1000-run comparison, statistical reports |

## Dependencies Rationale

| Dependency | Purpose | Why |
|------------|---------|-----|
| `bevy` | 3D rendering, ECS, input, windowing | Mature Rust-native, WebGPU ready |
| `rapier3d` | Physics simulation | Rust-native, deterministic feature, parallel |
| `nalgebra` | Linear algebra for estimation/control | Mathematical rigor, dimensional analysis |
| `glam` | Graphics math, transforms | Bevy integration, SIMD optimized |
| `wasmtime` | WASM runtime | Industry standard, component model, async |
| `tokio` | Async runtime | Fleet comms, telemetry, benchmarking |
| `tracing` | Structured logging | Observability, JSON output, spans |
| `postcard` | WASM ABI serialization | Deterministic, no-std, compact |
| `bincode` | Replay serialization | Fast, deterministic |
| `clap` | CLI | Derive-based, feature complete |
| `config` | Configuration layering | File + env + CLI precedence |

## Safety Boundaries (Enforced)

- No weapon targeting, firing solutions, autonomous engagement
- No target selection for attack, weapon guidance, lethal optimization
- Autonomy algorithms receive only navigation/observation/logistics tasks
- Fleet coordination limited to formation, coverage, rendezvous, convoy, avoidance
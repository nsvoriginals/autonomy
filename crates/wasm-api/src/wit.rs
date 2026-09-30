//! WIT (WASM Interface Types) definitions

/// WIT world for autonomy algorithms
pub const AUTONOMY_WIT: &str = r#"
package autonomy:algorithm

interface algorithm {
    /// Initialize the algorithm with configuration
    algorithm-init: func(config: list<u8>) -> expected<(), error>

    /// Main tick function - called every simulation step
    algorithm-tick: func(input: list<u8>) -> expected<list<u8>, error>

    /// Cleanup function
    algorithm-cleanup: func()
}

interface host {
    /// Get current simulation time (ticks)
    get-time: func() -> u64

    /// Log a message
    log: func(level: u32, message: string)

    /// Record a metric
    record-metric: func(name: string, value: f64)

    /// Get deterministic random number
    random: func() -> u64

    /// Request fleet action
    request-fleet-action: func(request-type: string, data: list<u8>) -> expected<u32, error>
}

world autonomy-algorithm {
    export algorithm
    import host
}
"#;

/// Algorithm input WIT type
pub const ALGORITHM_INPUT_WIT: &str = r#"
type algorithm-input = record {
    version: u32,
    timestamp: u64,
    vehicle-id: u128,
    estimated-state: estimated-state,
    sensor-measurements: sensor-measurements,
    neighbors: list<neighbor-state>,
    mission-state: mission-state,
    environment: environment-snapshot,
    config: algorithm-config,
}

type estimated-state = record {
    time: u64,
    vehicle-id: u128,
    pose: pose,
    linear-velocity: vec3,
    angular-velocity: vec3,
    linear-acceleration: vec3,
    covariance: covariance,
    estimator-type: u32,
    gps-fixed: bool,
    innovation: option<vec3>,
}

type pose = record {
    position: vec3,
    orientation: quaternion,
}

type vec3 = record { x: f64, y: f64, z: f64 }

type quaternion = record { w: f64, x: f64, y: f64, z: f64 }

type covariance = record {
    pos: list<list<f64>>,  // 3x3
    vel: list<list<f64>>,  // 3x3
    pos-vel: list<list<f64>>,  // 3x3
}

type sensor-measurements = record {
    imu: option<sensor-measurement>,
    gps: option<sensor-measurement>,
    camera: list<sensor-measurement>,
    depth: list<sensor-measurement>,
    lidar: list<sensor-measurement>,
    barometer: option<sensor-measurement>,
    magnetometer: option<sensor-measurement>,
    custom: list<sensor-measurement>,
}

type sensor-measurement = record {
    time: u64,
    sensor-id: u128,
    sensor-type: u16,
    vehicle-id: u128,
    data: sensor-data,
    latency: f64,
    noise-applied: bool,
}

type sensor-data = variant {
    imu: record { accel: vec3, gyro: vec3 },
    gps: record { position: vec3, velocity: vec3, hdop: f64, vdop: f64, satellites: u8, fix-type: u8 },
    camera: record { image-id: u64, width: u32, height: u32, exposure: f64 },
    depth: record { points: list<vec3>, intensity: list<f32> },
    lidar: record { points: list<vec3>, intensities: list<f32> },
    barometer: record { pressure: f64, altitude: f64, temperature: f64 },
    magnetometer: record { field: vec3 },
    custom: record { type-name: string, data: list<u8> },
}

type neighbor-state = record {
    vehicle-id: u128,
    vehicle-type: u8,
    pose: pose,
    velocity: vec3,
    estimated-state: option<estimated-state>,
    distance: f64,
    relative-velocity: vec3,
    communication-quality: f64,
    last-update: u64,
}

type mission-state = record {
    mission-id: u128,
    current-task: option<u128>,
    task-progress: f64,
    waypoint-index: u32,
    state: u8,
    parameters: map<string, json>,
}

type environment-snapshot = record {
    time: u64,
    wind: vec3,
    gravity: f64,
    magnetic-field: vec3,
    no-fly-zones: list<u128>,
    obstacles: list<obstacle>,
    terrain-height: option<func(vec3) -> f64>,
}

type obstacle = record {
    id: u128,
    center: vec3,
    radius: f64,
    height: f64,
    obstacle-type: u8,
}

type algorithm-config = record {
    algorithm-id: u128,
    parameters: map<string, json>,
}

type json = variant {
    null: (),
    bool: bool,
    number: f64,
    string: string,
    array: list<json>,
    object: map<string, json>,
}
"#;

/// Algorithm output WIT type
pub const ALGORITHM_OUTPUT_WIT: &str = r#"
type algorithm-output = record {
    version: u32,
    desired-trajectory: option<trajectory>,
    desired-velocity: option<vec3>,
    desired-heading: option<f64>,
    mission-transition: option<mission-transition>,
    fleet-requests: list<fleet-request>,
    custom-data: list<u8>,
}

type trajectory = record {
    waypoints: list<trajectory-waypoint>,
    start-time: u64,
    duration: f64,
}

type trajectory-waypoint = record {
    position: vec3,
    velocity: vec3,
    acceleration: vec3,
    yaw: f64,
    yaw-rate: f64,
    time-from-start: f64,
}

type mission-transition = record {
    from: u8,
    to: u8,
    reason: string,
}

type fleet-request = variant {
    task-request: record { task-type: string, priority: u8, location: option<vec3> },
    coordination-request: record { target-vehicle: u128, request-type: string, data: list<u8> },
    status-report: record { status: vehicle-status-report },
}

type vehicle-status-report = record {
    vehicle-id: u128,
    health: u8,
    battery: battery-state,
    mission-state: mission-state,
    estimator-quality: f64,
}

type battery-state = record {
    voltage: f64,
    current: f64,
    charge-remaining: f64,
    capacity-wh: f64,
    temperature: f64,
    health: f64,
}
"#;
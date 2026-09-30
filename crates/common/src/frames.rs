//! Coordinate frame definitions and transformations

use nalgebra::{Matrix3, Matrix4, Point3, Quaternion, UnitQuaternion, Vector3};
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::fmt;

pub fn serialize_vec3<S>(vec: &Vector3<f64>, serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    let arr = [vec.x, vec.y, vec.z];
    arr.serialize(serializer)
}

pub fn deserialize_vec3<'de, D>(deserializer: D) -> Result<Vector3<f64>, D::Error>
where
    D: Deserializer<'de>,
{
    let arr: [f64; 3] = Deserialize::deserialize(deserializer)?;
    Ok(Vector3::new(arr[0], arr[1], arr[2]))
}

pub fn serialize_vec3_vec<S>(vecs: &Vec<Vector3<f64>>, serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    let arr: Vec<[f64; 3]> = vecs.iter().map(|v| [v.x, v.y, v.z]).collect();
    arr.serialize(serializer)
}

pub fn deserialize_vec3_vec<'de, D>(deserializer: D) -> Result<Vec<Vector3<f64>>, D::Error>
where
    D: Deserializer<'de>,
{
    let arr: Vec<[f64; 3]> = Deserialize::deserialize(deserializer)?;
    Ok(arr.into_iter().map(|a| Vector3::new(a[0], a[1], a[2])).collect())
}

pub fn serialize_quat<S>(quat: &UnitQuaternion<f64>, serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    let arr = [quat.w, quat.i, quat.j, quat.k];
    arr.serialize(serializer)
}

pub fn deserialize_quat<'de, D>(deserializer: D) -> Result<UnitQuaternion<f64>, D::Error>
where
    D: Deserializer<'de>,
{
    let arr: [f64; 4] = Deserialize::deserialize(deserializer)?;
    Ok(UnitQuaternion::from_quaternion(Quaternion::new(arr[0], arr[1], arr[2], arr[3])))
}

pub fn serialize_opt_vec3<S>(opt: &Option<Vector3<f64>>, serializer: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    match opt {
        Some(vec) => {
            let arr = [vec.x, vec.y, vec.z];
            serializer.serialize_some(&arr)
        }
        None => serializer.serialize_none(),
    }
}

pub fn deserialize_opt_vec3<'de, D>(deserializer: D) -> Result<Option<Vector3<f64>>, D::Error>
where
    D: Deserializer<'de>,
{
    let opt: Option<[f64; 3]> = Deserialize::deserialize(deserializer)?;
    Ok(opt.map(|arr| Vector3::new(arr[0], arr[1], arr[2])))
}

/// World frame: ENU (East-North-Up), origin at scenario reference point
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct WorldFrame;

/// Vehicle body frame: FRD (Forward-Right-Down), origin at center of mass
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct BodyFrame;

/// Sensor frame: Defined per sensor, typically FRD or FLU
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct SensorFrame(pub SensorId);

/// Camera frame: Right-Down-Forward (OpenCV convention)
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct CameraFrame;

/// Local tangent plane frame: ENU, origin at vehicle spawn or local reference
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct LocalFrame;

/// Geographic frame: WGS84 Lat/Lon/Alt
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct GeographicFrame;

use crate::ids::SensorId;

/// 3D pose: position + orientation
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
    pub position: Vector3<f64>,
    #[serde(serialize_with = "serialize_quat", deserialize_with = "deserialize_quat")]
    pub orientation: UnitQuaternion<f64>,
}

impl Pose {
    pub fn new(position: Vector3<f64>, orientation: UnitQuaternion<f64>) -> Self {
        Self { position, orientation }
    }

    pub fn identity() -> Self {
        Self {
            position: Vector3::zeros(),
            orientation: UnitQuaternion::identity(),
        }
    }

    pub fn from_translation(translation: Vector3<f64>) -> Self {
        Self {
            position: translation,
            orientation: UnitQuaternion::identity(),
        }
    }

    pub fn from_rotation(rotation: UnitQuaternion<f64>) -> Self {
        Self {
            position: Vector3::zeros(),
            orientation: rotation,
        }
    }

    pub fn transform_point(&self, point: Vector3<f64>) -> Vector3<f64> {
        self.orientation * point + self.position
    }

    pub fn transform_vector(&self, vector: Vector3<f64>) -> Vector3<f64> {
        self.orientation * vector
    }

    pub fn inverse(&self) -> Self {
        let inv_rot = self.orientation.inverse();
        Self {
            position: -(inv_rot * self.position),
            orientation: inv_rot,
        }
    }

    pub fn compose(&self, other: &Pose) -> Self {
        Self {
            position: self.position + self.orientation * other.position,
            orientation: self.orientation * other.orientation,
        }
    }
}

impl Default for Pose {
    fn default() -> Self {
        Self::identity()
    }
}

impl fmt::Display for Pose {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Pose(pos=[{:.2}, {:.2}, {:.2}], rot=[{:.3}, {:.3}, {:.3}, {:.3}])",
            self.position.x,
            self.position.y,
            self.position.z,
            self.orientation.w,
            self.orientation.i,
            self.orientation.j,
            self.orientation.k
        )
    }
}

/// Transform between coordinate frames
pub trait Transform<From, To> {
    fn transform(&self, point: Vector3<f64>) -> Vector3<f64>;
    fn transform_pose(&self, pose: Pose) -> Pose;
}

/// World to Body transform (vehicle pose in world frame)
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct WorldToBody {
    pub pose: Pose,
}

impl WorldToBody {
    pub fn new(pose: Pose) -> Self {
        Self { pose }
    }

    pub fn from_position_yaw(pos: Vector3<f64>, yaw: f64) -> Self {
        let rot = UnitQuaternion::from_euler_angles(0.0, 0.0, yaw);
        Self::new(Pose::new(pos, rot))
    }
}

impl Transform<WorldFrame, BodyFrame> for WorldToBody {
    fn transform(&self, point: Vector3<f64>) -> Vector3<f64> {
        self.pose.inverse().transform_point(point)
    }

    fn transform_pose(&self, pose: Pose) -> Pose {
        self.pose.inverse().compose(&pose)
    }
}

impl Transform<BodyFrame, WorldFrame> for WorldToBody {
    fn transform(&self, point: Vector3<f64>) -> Vector3<f64> {
        self.pose.transform_point(point)
    }

    fn transform_pose(&self, pose: Pose) -> Pose {
        self.pose.compose(&pose)
    }
}

/// Body to Sensor transform (extrinsic calibration)
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct BodyToSensor {
    #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
    pub offset: Vector3<f64>,
    #[serde(serialize_with = "serialize_quat", deserialize_with = "deserialize_quat")]
    pub rotation: UnitQuaternion<f64>,
}

impl BodyToSensor {
    pub fn new(offset: Vector3<f64>, rotation: UnitQuaternion<f64>) -> Self {
        Self { offset, rotation }
    }

    pub fn identity() -> Self {
        Self {
            offset: Vector3::zeros(),
            rotation: UnitQuaternion::identity(),
        }
    }
}

impl Transform<BodyFrame, SensorFrame> for BodyToSensor {
    fn transform(&self, point: Vector3<f64>) -> Vector3<f64> {
        self.rotation * point + self.offset
    }

    fn transform_pose(&self, pose: Pose) -> Pose {
        Pose::new(self.offset, self.rotation).compose(&pose)
    }
}

impl Transform<SensorFrame, BodyFrame> for BodyToSensor {
    fn transform(&self, point: Vector3<f64>) -> Vector3<f64> {
        self.rotation.inverse() * (point - self.offset)
    }

    fn transform_pose(&self, pose: Pose) -> Pose {
        Pose::new(-(self.rotation.inverse() * self.offset), self.rotation.inverse()).compose(&pose)
    }
}

/// Geographic to World transform (WGS84 to local ENU)
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct GeographicToWorld {
    pub origin_lat: f64,
    pub origin_lon: f64,
    pub origin_alt: f64,
}

impl GeographicToWorld {
    pub fn new(origin_lat: f64, origin_lon: f64, origin_alt: f64) -> Self {
        Self {
            origin_lat,
            origin_lon,
            origin_alt,
        }
    }

    /// Convert geodetic (lat, lon, alt) to local ENU
    /// Uses WGS84 ellipsoid approximation
    pub fn geodetic_to_enu(&self, lat: f64, lon: f64, alt: f64) -> Vector3<f64> {
        const WGS84_A: f64 = 6378137.0;
        const WGS84_E2: f64 = 6.69437999014e-3;

        let lat_rad = lat.to_radians();
        let lon_rad = lon.to_radians();
        let origin_lat_rad = self.origin_lat.to_radians();
        let origin_lon_rad = self.origin_lon.to_radians();

        let d_lat = lat_rad - origin_lat_rad;
        let d_lon = lon_rad - origin_lon_rad;

        let sin_lat = origin_lat_rad.sin();
        let cos_lat = origin_lat_rad.cos();

        let N = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();

        let east = (N + alt) * d_lon * cos_lat;
        let north = (N * (1.0 - WGS84_E2) + alt) * d_lat;
        let up = alt - self.origin_alt;

        Vector3::new(east, north, up)
    }

    /// Convert local ENU to geodetic (approximate inverse)
    pub fn enu_to_geodetic(&self, enu: Vector3<f64>) -> (f64, f64, f64) {
        const WGS84_A: f64 = 6378137.0;
        const WGS84_E2: f64 = 6.69437999014e-3;

        let sin_lat = self.origin_lat.to_radians().sin();
        let cos_lat = self.origin_lat.to_radians().cos();

        let N = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();

        let d_lat = enu.y / (N * (1.0 - WGS84_E2) + enu.z);
        let d_lon = enu.x / ((N + enu.z) * cos_lat);

        let lat = self.origin_lat + d_lat.to_degrees();
        let lon = self.origin_lon + d_lon.to_degrees();
        let alt = self.origin_alt + enu.z;

        (lat, lon, alt)
    }
}

/// Convenience type aliases for common vector types
pub type Vec3 = Vector3<f64>;
pub type Mat3 = Matrix3<f64>;
pub type Mat4 = Matrix4<f64>;
pub type Quat = UnitQuaternion<f64>;
pub type Point3d = Point3<f64>;

/// Angular velocity in body frame (rad/s)
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct AngularVelocity(
    #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
    pub Vector3<f64>
);

/// Linear velocity in world frame (m/s)
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinearVelocity(
    #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
    pub Vector3<f64>
);

/// Linear acceleration in world frame (m/s^2)
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LinearAcceleration(
    #[serde(serialize_with = "serialize_vec3", deserialize_with = "deserialize_vec3")]
    pub Vector3<f64>
);

/// Euler angles (roll, pitch, yaw) in radians
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct EulerAngles {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

impl EulerAngles {
    pub fn new(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self { roll, pitch, yaw }
    }

    pub fn to_quaternion(&self) -> UnitQuaternion<f64> {
        UnitQuaternion::from_euler_angles(self.roll, self.pitch, self.yaw)
    }

    pub fn from_quaternion(q: UnitQuaternion<f64>) -> Self {
        let (roll, pitch, yaw) = q.euler_angles();
        Self { roll, pitch, yaw }
    }
}

impl Default for EulerAngles {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }
}
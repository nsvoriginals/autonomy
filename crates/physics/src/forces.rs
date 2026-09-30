//! Force and torque computation for vehicles

use autonomy_common::frames::{LinearAcceleration, Vec3};
use autonomy_common::ids::VehicleId;
use nalgebra::Vector3;

/// Force accumulator for a physics body
#[derive(Clone, Debug, Default)]
pub struct ForceAccumulator {
    pub forces: Vec3,
    pub torques: Vec3,
    pub force_contributions: Vec<ForceContribution>,
}

#[derive(Clone, Debug)]
pub struct ForceContribution {
    pub name: String,
    pub force: Vec3,
    pub torque: Vec3,
    pub application_point: Vec3, // Relative to center of mass
}

impl ForceAccumulator {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_force(&mut self, force: Vec3, name: impl Into<String>) {
        self.forces += force;
        self.force_contributions.push(ForceContribution {
            name: name.into(),
            force,
            torque: Vec3::zeros(),
            application_point: Vec3::zeros(),
        });
    }

    pub fn add_force_at_point(&mut self, force: Vec3, point: Vec3, name: impl Into<String>) {
        self.forces += force;
        let torque = point.cross(&force);
        self.torques += torque;
        self.force_contributions.push(ForceContribution {
            name: name.into(),
            force,
            torque,
            application_point: point,
        });
    }

    pub fn add_torque(&mut self, torque: Vec3, name: impl Into<String>) {
        self.torques += torque;
        self.force_contributions.push(ForceContribution {
            name: name.into(),
            force: Vec3::zeros(),
            torque,
            application_point: Vec3::zeros(),
        });
    }

    pub fn clear(&mut self) {
        self.forces = Vec3::zeros();
        self.torques = Vec3::zeros();
        self.force_contributions.clear();
    }

    pub fn total_force(&self) -> Vec3 {
        self.forces
    }

    pub fn total_torque(&self) -> Vec3 {
        self.torques
    }
}

/// Aerodynamic forces for UAV
pub struct AerodynamicForces {
    pub air_density: f64,
    pub drag_coefficient: f64,
    pub lift_coefficient: f64,
    pub reference_area: f64,
    pub body_drag_coefficients: Vec3, // Per-axis drag
}

impl AerodynamicForces {
    pub fn new(air_density: f64, drag_coeff: f64, reference_area: f64) -> Self {
        Self {
            air_density,
            drag_coefficient: drag_coeff,
            lift_coefficient: 0.0,
            reference_area,
            body_drag_coefficients: Vec3::new(drag_coeff, drag_coeff, drag_coeff),
        }
    }

    pub fn compute_drag(&self, velocity: Vec3, body_orientation: Vec3) -> Vec3 {
        let speed = velocity.magnitude();
        if speed < 0.01 {
            return Vec3::zeros();
        }

        // Transform velocity to body frame (simplified)
        let v_body = velocity; // Would need proper rotation
        
        // Quadratic drag model: F = 0.5 * rho * v^2 * Cd * A
        let dynamic_pressure = 0.5 * self.air_density * speed * speed;
        let drag_magnitude = dynamic_pressure * self.drag_coefficient * self.reference_area;
        
        // Oppose velocity
        -velocity.normalize() * drag_magnitude
    }

    pub fn compute_body_drag(&self, linear_vel: Vec3, angular_vel: Vec3) -> (Vec3, Vec3) {
        let speed = linear_vel.magnitude();
        if speed < 0.01 {
            return (Vec3::zeros(), Vec3::zeros());
        }

        let dynamic_pressure = 0.5 * self.air_density * speed * speed;
        
        // Body-frame drag
        let drag_force = Vec3::new(
            dynamic_pressure * self.body_drag_coefficients.x * self.reference_area,
            dynamic_pressure * self.body_drag_coefficients.y * self.reference_area,
            dynamic_pressure * self.body_drag_coefficients.z * self.reference_area,
        ) * -linear_vel.normalize();

        // Angular drag (simplified)
        let angular_speed = angular_vel.magnitude();
        let angular_drag = -angular_vel * self.air_density * angular_speed * 0.01;

        (drag_force, angular_drag)
    }
}

/// Ground effect model for UAV near ground
pub struct GroundEffect {
    pub rotor_radius: f64,
    pub max_effect_factor: f64,
}

impl GroundEffect {
    pub fn new(rotor_radius: f64) -> Self {
        Self {
            rotor_radius,
            max_effect_factor: 1.5,
        }
    }

    /// Compute thrust multiplier due to ground effect
    /// Based on Cheeseman-Bennett model
    pub fn thrust_multiplier(&self, height_above_ground: f64) -> f64 {
        if height_above_ground <= 0.0 {
            return self.max_effect_factor;
        }
        
        let ratio = self.rotor_radius / (4.0 * height_above_ground);
        if ratio >= 1.0 {
            self.max_effect_factor
        } else {
            1.0 + ratio * (self.max_effect_factor - 1.0)
        }
    }
}

/// Magnetic forces (for magnetometer simulation)
pub fn magnetic_torque(magnetic_moment: Vec3, magnetic_field: Vec3) -> Vec3 {
    magnetic_moment.cross(&magnetic_field)
}

/// Gravity force
pub fn gravity_force(mass: f64, gravity: f64) -> Vec3 {
    Vec3::new(0.0, -mass * gravity, 0.0)
}

/// Buoyancy force (for air/fluid displacement)
pub fn buoyancy_force(displaced_volume: f64, fluid_density: f64, gravity: f64) -> Vec3 {
    Vec3::new(0.0, displaced_volume * fluid_density * gravity, 0.0)
}

/// Spring-damper force for contact/landing gear
pub fn spring_damper_force(
    position: Vec3,
    velocity: Vec3,
    rest_position: Vec3,
    spring_constant: f64,
    damping_coefficient: f64,
) -> Vec3 {
    let displacement = position - rest_position;
    let spring_force = -displacement * spring_constant;
    let damper_force = -velocity * damping_coefficient;
    spring_force + damper_force
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_force_accumulator() {
        let mut acc = ForceAccumulator::new();
        acc.add_force(Vec3::new(10.0, 0.0, 0.0), "thrust");
        acc.add_force(Vec3::new(0.0, -5.0, 0.0), "gravity");
        
        assert_eq!(acc.total_force(), Vec3::new(10.0, -5.0, 0.0));
    }

    #[test]
    fn test_force_at_point() {
        let mut acc = ForceAccumulator::new();
        // Force at (1, 0, 0) in +Z direction creates torque around -Y
        acc.add_force_at_point(Vec3::new(0.0, 0.0, 10.0), Vec3::new(1.0, 0.0, 0.0), "motor");
        
        assert_eq!(acc.total_force(), Vec3::new(0.0, 0.0, 10.0));
        assert_eq!(acc.total_torque(), Vec3::new(0.0, -10.0, 0.0));
    }

    #[test]
    fn test_ground_effect() {
        let ge = GroundEffect::new(0.5); // 0.5m rotor radius
        
        // At ground level
        assert!((ge.thrust_multiplier(0.0) - 1.5).abs() < 0.01);
        
        // At rotor radius height
        assert!((ge.thrust_multiplier(0.5) - 1.25).abs() < 0.01);
        
        // Far above ground
        assert!((ge.thrust_multiplier(10.0) - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_aerodynamic_drag() {
        let aero = AerodynamicForces::new(1.225, 0.1, 0.1);
        let drag = aero.compute_drag(Vec3::new(10.0, 0.0, 0.0), Vec3::zeros());
        
        // Should oppose velocity
        assert!(drag.x < 0.0);
        assert_eq!(drag.y, 0.0);
        assert_eq!(drag.z, 0.0);
    }
}
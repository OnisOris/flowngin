use rapier3d::prelude::{Real, Vector};
pub mod boid_controller;
pub mod motion_controller;
pub mod pid_controller;
use crate::constants::Accuracy;

trait Controller {
    fn update(&mut self, setpoint: Vector, measurement: Vector, dt: Accuracy) -> Vector;
    fn reset(&mut self);
}

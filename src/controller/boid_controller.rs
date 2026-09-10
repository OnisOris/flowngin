use crate::controller::pid_controller::PidController;
use crate::environment::Environment;
use rapier3d::math::Vector;
use rapier3d::prelude::Real;
pub struct BoidsController {
    perception_radius: Real,
    separation_coeff: Real,
    cohision_weight: Real,
    pub max_speed: Real,
    pid_controller: PidController,
}

impl Default for BoidsController {
    fn default() -> Self {
        Self {
            perception_radius: 1.,
            separation_coeff: 1.,
            cohision_weight: 1.,
            max_speed: 2.,
            pid_controller: PidController::default(),
        }
    }
}

impl BoidsController {
    fn update(set_point: Vector) -> Vector {}
}

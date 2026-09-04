use crate::constants::Accuracy;
use crate::controller::{Controller, pid_controller::PidController};
use rapier3d::math::Vector;
pub struct MotionController {
    speed_controller: PidController,
    position_controller: PidController,
}

impl MotionController {}

impl Controller for MotionController {
    fn update(&mut self, desired_state: Vector, actual_state: Vector, dt: Accuracy) -> Vector {
        self.pid_controller
            .update(desired_velocity, actual_velocity, dt)
    }

    fn reset(&mut self) {
        self.pid_controller.reset();
    }
}

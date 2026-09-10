use crate::constants::Accuracy;
use crate::{
    agent::State,
    controller::{Controller, pid_controller::PidController},
};
use rapier3d::math::Vector;
pub struct MotionController {
    speed_controller: PidController,
    position_controller: PidController,
}

impl MotionController {}

impl Default for MotionController {
    fn default() -> Self {
        Self {
            speed_controller: PidController::default(),
            position_controller: PidController::default(),
        }
    }
}

impl MotionController {
    pub fn update(
        &mut self,
        desired_position: Vector,
        actual_state: &State,
        dt: Accuracy,
    ) -> Vector {
        let desired_velocity =
            self.position_controller
                .update(desired_position, actual_state.position, dt);
        self.speed_controller
            .update(desired_velocity, actual_state.velocity, dt)
    }

    pub fn reset(&mut self) {
        self.position_controller.reset();
        self.speed_controller.reset();
    }
}

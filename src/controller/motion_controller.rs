use crate::constants::Accuracy;
use crate::controller::{Controller, pid_controller::PidController};
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
    fn update(&mut self, desired_state: State, &mut actual_state: State, dt: Accuracy) -> Vector {
        let speed_vector = self
            .speed_controller
            .update(desired_state, actual_state, dt); 
        let position_vector = self
            .position_controller.update(desired_position, actual_position, dt);
        let error: ErrorModel 
        
    }

    fn reset(&mut self) {
        self.speed_controller.reset();
    }
}

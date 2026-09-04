use rapier3d::prelude::Real;
pub struct BoidsController {
    perception_radius: Real,
    separation_coeff: Real,
    cohision_weight: Real, 
    pub max_speed: Real,
}

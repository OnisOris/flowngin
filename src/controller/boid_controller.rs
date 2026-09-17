use crate::agent::State;
use crate::controller::pid_controller::PidController;
use crate::environment::Environment;
use rapier3d::math::Vector;
use rapier3d::prelude::Real;

// pub struct BoidsController {
//     pub perception_radius: Real,
//     pub separation_coeff: Real,
//     pub cohesion_coeff: Real,
//     pub alignment_coeff: Real,
//     pub separation_weight: Real,
//     pub cohesion_weight: Real,
//     pub alignment_weight: Real,
//     pub max_speed: Real,
//
//     separation_pid: PidController,
//     cohesion_pid: PidController,
//     alignment_pid: PidController,
// }

pub struct BoidsController {
    pub interaction_radius: Real,
    pub desired_distance: Real,

    pub formation_gain: Real,
    pub alignment_gain: Real,

    pub max_force: Real,
}

impl Default for BoidsController {
    fn default() -> Self {
        Self {
            interaction_radius: 6.0,
            desired_distance: 3.,
            formation_gain: 1.,
            alignment_gain: 0.5,
            max_force: 5.,
        }
    }
}

impl BoidsController {
    pub fn new(interaction_radius: Real) -> Self {
        Self {
            interaction_radius,
            ..Default::default()
        }
    }

    pub fn update(&mut self, environment: &Environment, agent_id: usize, _dt: Real) -> Vector {
        let own_state = match environment.state(agent_id) {
            Some(s) => s,
            None => State::ZERO,
        };

        let neighbors =
            environment.neighbors(own_state.position, self.interaction_radius, agent_id);
        if neighbors.is_empty() {
            return Vector::ZERO;
        }
        let formation_force = self.formation_force(&own_state, &neighbors);
        let alighment_force = self.alignment_force(&own_state, &neighbors);
        let mut total = formation_force + alighment_force;
        let magnitude = total.length();
        if magnitude > self.max_force {
            total *= self.max_force / magnitude;
        }
        total
    }

    fn formation_force(&self, own: &State, neighbors: &[(usize, &State)]) -> Vector {
        let mut force = Vector::ZERO;

        for (_, neighbor) in neighbors {
            let delta = neighbor.position - own.position;
            let distance = delta.length();

            if distance < 1e-6 || distance >= self.interaction_radius {
                continue;
            }

            let direction = delta / distance;

            let magnitude = self.formation_gain * (distance - self.desired_distance);

            force += direction * magnitude;
        }

        force
    }

    fn alignment_force(&self, own: &State, neighbors: &[(usize, &State)]) -> Vector {
        let mut force = Vector::ZERO;

        for (_, neighbor) in neighbors {
            force += neighbor.velocity - own.velocity;
        }

        force * self.alignment_gain
    }
    //     /// Отталкивание от соседей.
    //     ///
    //     /// В сильной зоне (`separation_zone`) работает жёсткое отталкивание `1/d²`.
    //     /// Вне её, вплоть до края восприятия, действует мягкое линейно затухающее
    //     /// сопротивление — иначе cohesion стягивал бы агентов без всякого противовеса.
    //     fn separation(&self, own: &State, neighbors: &[(usize, &State)]) -> Vector {
    //         let mut force = Vector::ZERO;
    //         for (_, neighbor) in neighbors {
    //             let diff = own.position - neighbor.position;
    //             let dist = diff.length();
    //             if dist < 1e-6 {
    //                 continue;
    //             }
    //             let unit = diff / dist;
    //             if dist < self.separation_coeff {
    //                 // Сильная зона: резкое 1/d² вблизи.
    //                 force += unit / dist;
    //             } else {
    //                 // Слабая зона: линейное затухание до нуля на краю восприятия.
    //                 let falloff = 1.0 - (dist / self.perception_radius);
    //                 if falloff > 0.0 {
    //                     force += unit * falloff * 0.5;
    //                 }
    //             }
    //         }
    //         force
    //     }
    //
    //     fn cohesion(&self, own: &State, neighbors: &[(usize, &State)]) -> Vector {
    //         let mut center = Vector::ZERO;
    //         for (_, neighbor) in neighbors {
    //             center += neighbor.position;
    //         }
    //         center /= neighbors.len() as Real;
    //         center - own.position
    //     }
    //
    //     fn alignment(&self, own: &State, neighbors: &[(usize, &State)]) -> Vector {
    //         let mut avg_velocity = Vector::ZERO;
    //
    //         for (_, neighbor) in neighbors {
    //             avg_velocity += neighbor.velocity;
    //         }
    //
    //         avg_velocity /= neighbors.len() as Real;
    //
    //         avg_velocity - own.velocity
    //     }
    //
    //     pub fn reset(&mut self) {
    //         self.separation_pid.reset();
    //         self.cohesion_pid.reset();
    //         self.alignment_pid.reset();
    //     }
}

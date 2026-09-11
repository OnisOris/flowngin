use crate::agent::State;
use crate::controller::pid_controller::PidController;
use crate::environment::Environment;
use rapier3d::math::Vector;
use rapier3d::prelude::Real;

pub struct BoidsController {
    pub perception_radius: Real,
    pub separation_coeff: Real,
    pub cohesion_coeff: Real,
    pub alignment_coeff: Real,
    pub separation_weight: Real,
    pub cohesion_weight: Real,
    pub alignment_weight: Real,
    pub max_speed: Real,

    separation_pid: PidController,
    cohesion_pid: PidController,
    alignment_pid: PidController,
}

impl Default for BoidsController {
    fn default() -> Self {
        Self {
            perception_radius: 6.0,
            separation_coeff: 3.0,
            cohesion_coeff: 1.0,
            alignment_coeff: 1.0,
            separation_weight: 4.0,
            cohesion_weight: 0.5,
            alignment_weight: 0.5,
            max_speed: 5.0,

            separation_pid: PidController::new(
                Vector::splat(6.0),
                Vector::splat(0.0),
                Vector::splat(0.2),
            ),
            cohesion_pid: PidController::new(
                Vector::splat(0.8),
                Vector::splat(0.0),
                Vector::splat(0.05),
            ),
            alignment_pid: PidController::new(
                Vector::splat(1.2),
                Vector::splat(0.0),
                Vector::splat(0.08),
            ),
        }
    }
}

impl BoidsController {
    pub fn new(perception_radius: Real) -> Self {
        Self {
            perception_radius,
            ..Default::default()
        }
    }

    pub fn update(&mut self, environment: &Environment, agent_id: usize, dt: Real) -> Vector {
        let own_state = match environment.state(agent_id) {
            Some(s) => s,
            None => return Vector::ZERO,
        };

        let neighbors = environment.neighbors(own_state.position, self.perception_radius, agent_id);

        if neighbors.is_empty() {
            return Vector::ZERO;
        }

        let sep = self.separation(own_state, &neighbors);
        let coh = self.cohesion(own_state, &neighbors);
        let ali = self.alignment(own_state, &neighbors);

        let sep_force = self.separation_pid.update(sep, Vector::ZERO, dt);
        let coh_force = self.cohesion_pid.update(coh, Vector::ZERO, dt);
        let ali_force = self.alignment_pid.update(ali, Vector::ZERO, dt);

        let mut total = sep_force * self.separation_weight
            + coh_force * self.cohesion_weight
            + ali_force * self.alignment_weight;

        let speed = total.length();
        if speed > self.max_speed {
            total = total * (self.max_speed / speed);
        }

        total
    }

    /// Отталкивание от соседей.
    ///
    /// В сильной зоне (`separation_zone`) работает жёсткое отталкивание `1/d²`.
    /// Вне её, вплоть до края восприятия, действует мягкое линейно затухающее
    /// сопротивление — иначе cohesion стягивал бы агентов без всякого противовеса.
    fn separation(&self, own: &State, neighbors: &[(usize, &State)]) -> Vector {
        let mut force = Vector::ZERO;
        for (_, neighbor) in neighbors {
            let diff = own.position - neighbor.position;
            let dist = diff.length();
            if dist < 1e-6 {
                continue;
            }
            let unit = diff / dist;
            if dist < self.separation_coeff {
                // Сильная зона: резкое 1/d² вблизи.
                force += unit / dist;
            } else {
                // Слабая зона: линейное затухание до нуля на краю восприятия.
                let falloff = 1.0 - (dist / self.perception_radius);
                if falloff > 0.0 {
                    force += unit * falloff * 0.5;
                }
            }
        }
        force
    }

    fn cohesion(&self, own: &State, neighbors: &[(usize, &State)]) -> Vector {
        let mut center = Vector::ZERO;
        for (_, neighbor) in neighbors {
            center += neighbor.position;
        }
        center /= neighbors.len() as Real;
        center - own.position
    }

    fn alignment(&self, _own: &State, neighbors: &[(usize, &State)]) -> Vector {
        let mut avg_velocity = Vector::ZERO;
        for (_, neighbor) in neighbors {
            avg_velocity += neighbor.velocity;
        }
        avg_velocity /= neighbors.len() as Real;
        avg_velocity
    }

    pub fn reset(&mut self) {
        self.separation_pid.reset();
        self.cohesion_pid.reset();
        self.alignment_pid.reset();
    }
}

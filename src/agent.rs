use crate::controller::{boid_controller::BoidsController, motion_controller::MotionController};
use crate::environment::Environment;
use rapier3d::{
    math::Vector,
    prelude::{ColliderBuilder, Real},
};
use std::fmt;

#[derive(Clone, Copy, Debug)]
pub struct State {
    pub position: Vector,
    pub velocity: Vector,
}

impl Default for State {
    fn default() -> Self {
        Self {
            position: Vector::ZERO,
            velocity: Vector::ZERO,
        }
    }
}

impl State {
    pub fn new(position: Vector, velocity: Vector) -> Self {
        Self {
            position,
            velocity,
        }
    }
}

pub struct Agent {
    pub id: usize,
    pub state: State,
    pub model: AgentModel,
    pub motion_controller: MotionController,
    pub boid_controller: BoidsController,
}

#[derive(Clone, Copy, Debug)]
pub enum AgentShape {
    Ball { radius: Real },
    Cuboid { half_extents: Vector },
}

pub struct AgentModel {
    pub shape: AgentShape,
    pub mass: Real,
    pub friction: Real,
    pub restitution: Real,
}

impl AgentModel {
    pub fn collider(&self) -> ColliderBuilder {
        let builder = match self.shape {
            AgentShape::Ball { radius } => ColliderBuilder::ball(radius),
            AgentShape::Cuboid { half_extents } => {
                ColliderBuilder::cuboid(half_extents.x, half_extents.y, half_extents.z)
            }
        };
        builder
            .mass(self.mass)
            .friction(self.friction)
            .restitution(self.restitution)
    }
    fn half_heigh(&self) -> Real {
        match self.shape {
            AgentShape::Ball { radius } => radius,
            AgentShape::Cuboid { half_extents } => half_extents.y,
        }
    }
}

impl Agent {
    pub fn new(id: usize) -> Self {
        Self {
            id,
            state: State::default(),
            motion_controller: MotionController::default(),
            boid_controller: BoidsController::default(),
            model: AgentModel {
                shape: AgentShape::Ball { radius: 0.2 },
                mass: 0.2,
                friction: 0.0,
                restitution: 0.1,
            },
        }
    }

    pub fn get_status(&self) -> &'static str {
        "Hello"
    }

    pub fn update(
        &mut self,
        desired_position: Vector,
        actual_state: State,
        dt: Real,
        environment: &Environment,
    ) -> Vector {
        println!("Agent[{}] Desired Position: {:?}", self.id, desired_position);
        println!("Time Step: {:?}", dt);
        self.state = actual_state;

        let flock_force = self.boid_controller.update(environment, self.id, dt);

        let motion_force = self
            .motion_controller
            .update(desired_position, &self.state, dt);

        motion_force + flock_force
    }

    pub fn reset(&mut self) {
        self.motion_controller.reset();
        self.boid_controller.reset();
    }
}

impl Default for Agent {
    fn default() -> Self {
        Self::new(0)
    }
}

impl fmt::Display for Agent {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "Agent[{}] (x={}, y={}, z={})",
            self.id, self.state.velocity.x, self.state.velocity.y, self.state.velocity.z
        )
    }
}

use crate::constants::Accuracy;
use crate::controller::boid_controller::BoidsController;
use crate::controller::{motion_controller::MotionController, pid_controller::PidController};
use rapier3d::geometry::Ball;
use rapier3d::{
    math::{Vector, Vector3},
    na::Vector6,
    prelude::{ColliderBuilder, Real},
};
use std::fmt;

pub struct State {
    pub position: Vector,
    pub velocity: Vector,
}

impl Default for State {
    fn default() -> Self {
        Self {
            x: 0.0,
            velocity: Vector::new(0., 0., 0.),
        }
    }
}

impl State {
    // мне кажется это весьма затратно каждый раз создавать вектор
    fn get_vector_speed(&self) -> Vector {
        Vector::new(self.x, self.y, self.z)
    }
}
pub struct Agent {
    //state of agent: x, y, z, vx, vy, vz
    pub state: State,
    pub model: AgentModel,
    pub motion_controller: MotionController,
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

impl Default for Agent {
    fn default() -> Self {
        Self {
            state: State::default(),
            motion_controller: MotionController::default(),
            model: AgentModel {
                shape: AgentShape::Ball { radius: 10.0 },
                mass: 0.2,
                friction: 0.0,
                restitution: 0.1,
            },
        }
    }
}

impl fmt::Display for Agent {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "(x={}, y={}, z={})",
            self.state.x, self.state.y, self.state.z
        )
    }
}

impl Agent {
    pub fn get_status(&self) -> &'static str {
        "Hello"
    }

    pub fn update(&mut self, setpoint: Vector, measurement: Vector, dt: Real) -> Vector {
        self.motion_controller.update(setpoint, measurement, dt)
    }
}

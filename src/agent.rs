use crate::constants::Accuracy;
use crate::controller::boid_controller::BoidsController;
use crate::controller::pid_controller::PidController;
use rapier3d::geometry::Ball;
use rapier3d::{
    math::{Vector, Vector3},
    na::Vector6,
    prelude::{ColliderBuilder, Real},
};
use std::fmt;

pub struct Agent {
    //state of agent: x, y, z, vx, vy, vz
    x: Accuracy,
    y: Accuracy,
    z: Accuracy,
    vx: Accuracy,
    vy: Accuracy,
    vz: Accuracy,
    ax: Accuracy,
    ay: Accuracy,
    az: Accuracy,
    pub controller: PidController,
    pub model: AgentModel,
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
            x: 0.,
            y: 0.,
            z: 0.,
            vx: 0.,
            vy: 0.,
            vz: 0.,
            ax: 0.,
            ay: 0.,
            az: 0.,
            controller: PidController::from_scalar(1., 1., 1.),
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
        write!(f, "(x={}, y={}, z={})", self.x, self.y, self.z)
    }
}

impl Agent {
    pub fn get_status(&self) -> &'static str {
        "Hello"
    }

    pub fn update(&mut self, setpoint: Vector, measurement: Vector, dt: Real) -> Vector {
        self.controller.update(setpoint, measurement, dt)
    }
}

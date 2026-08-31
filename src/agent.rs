use crate::controller::PidController;
use rapier3d::{math::Vector3, na::Vector6};
type Accuracy = f32;

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
        }
    }
}

impl Agent {}

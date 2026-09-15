use rapier3d::prelude::{Real, Vector};

use crate::agent::State;

pub struct Environment {
    states: Vec<State>,
}

impl Default for Environment {
    fn default() -> Self {
        Self { states: Vec::new() }
    }
}

impl Environment {
    pub fn new(agent_count: usize) -> Self {
        Self {
            states: vec![State::default(); agent_count],
        }
    }

    pub fn update_from_states(&mut self, states: &[State]) {
        self.states.clear();
        self.states.extend_from_slice(states);
    }

    pub fn set_states(&mut self, states: Vec<State>) {
        self.states = states;
    }

    pub fn neighbors(
        &self,
        position: Vector,
        radius: Real,
        self_id: usize,
    ) -> Vec<(usize, &State)> {
        let radius_sq = radius * radius;
        self.states
            .iter()
            .enumerate()
            .filter(|(i, s)| *i != self_id && (s.position - position).length_squared() <= radius_sq)
            .collect()
    }

    pub fn agent_count(&self) -> usize {
        self.states.len()
    }

    pub fn state(&self, id: usize) -> Option<&State> {
        self.states.get(id)
    }

    pub fn states(&self) -> &[State] {
        &self.states
    }
}

use crate::agent::{Agent, State};
use crate::environment::Environment;

pub struct Swarm {
    pub agents: Vec<Agent>,
    pub environment: Environment,
}

impl Swarm {
    pub fn new(count: usize) -> Self {
        let agents: Vec<Agent> = (0..count).map(Agent::new).collect();
        let environment = Environment::new(count);
        Self {
            agents,
            environment,
        }
    }

    pub fn update_environment(&mut self) {
        let states: Vec<State> = self.agents.iter().map(|a| a.state.clone()).collect();
        self.environment.set_states(states);
    }

    pub fn agent_count(&self) -> usize {
        self.agents.len()
    }

    pub fn reset(&mut self) {
        for agent in &mut self.agents {
            agent.reset();
        }
    }
}

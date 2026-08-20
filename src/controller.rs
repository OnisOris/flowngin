pub struct PidController1 {
    pub kp: f32,
}

impl PidController1 {
    pub fn new(kp: f32) -> Self {
        Self { kp }
    }

    pub fn compute(&self, error: f32) -> f32 {
        self.kp * error
    }
}

pub struct PidController3 {
    pub kp: Vector,
}

impl PidController1 {
    pub fn new(kp: f32) -> Self {
        Self { kp }
    }

    pub fn compute(&self, error: f32) -> f32 {
        self.kp * error
    }
}

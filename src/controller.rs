use rapier3d::prelude::{Real, Vector};

#[derive(Debug, Clone)]
pub struct PidController {
    pub kp: Vector,
    pub ki: Vector,
    pub kd: Vector,

    integral: Vector,
    prev_error: Option<Vector>,
    prev_measurement: Option<Vector>,
    prev_derivative: Vector,

    out_min: Option<Vector>,
    out_max: Option<Vector>,
    out_max_norm: Option<Real>,

    derivative_on_measurement: bool,
    derivative_filter: Vector,
}

impl Default for PidController {
    fn default() -> Self {
        Self {
            kp: Vector::ZERO,
            ki: Vector::ZERO,
            kd: Vector::ZERO,
            integral: Vector::ZERO,
            prev_error: None,
            prev_measurement: None,
            prev_derivative: Vector::ZERO,
            out_min: None,
            out_max: None,
            out_max_norm: None,
            derivative_on_measurement: true,
            derivative_filter: Vector::ONE,
        }
    }
}

impl PidController {
    pub fn from_scalar(kp: Real, ki: Real, kd: Real) -> Self {
        Self {
            kp: Vector::splat(kp),
            ki: Vector::splat(ki),
            kd: Vector::splat(kd),
            ..Default::default()
        }
    }

    pub fn new(kp: Vector, ki: Vector, kd: Vector) -> Self {
        Self {
            kp,
            ki,
            kd,
            ..Default::default()
        }
    }

    pub fn set_tunings(&mut self, kp: Vector, ki: Vector, kd: Vector) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    pub fn set_outputs_limits(&mut self, min: Vector, max: Vector) {
        assert!(
            min.x <= max.x && min.y <= max.y && min.z <= max.z,
            "min must be <= max in every axis"
        );
        self.out_min = Some(min);
        self.out_max = Some(max);
    }

    pub fn set_outputs_limits_norm(&mut self, max_norm: Real) {
        assert!(max_norm >= 0.0, "norm must be >= 0");
        self.out_max_norm = Some(max_norm);
    }

    pub fn set_derivative_on_mesurement(&mut self, enabled: bool) {
        self.derivative_on_measurement = enabled;
    }

    pub fn set_derivative_filter(&mut self, alpha: Vector) {
        assert!(
            (0.0..=1.0).contains(&alpha.x)
                && (0.0..=1.0).contains(&alpha.y)
                && (0.0..=1.0).contains(&alpha.z),
            "Alpha должен быть в [0, 1] по каждой оси"
        );
        self.derivative_filter = alpha;
    }

    pub fn reset(&mut self) {
        self.integral = Vector::ZERO;
        self.prev_error = None;
        self.prev_measurement = None;
        self.prev_derivative = Vector::ZERO;
    }
    pub fn update(&mut self, setpoint: Vector, measurement: Vector, dt: Real) -> Vector {
        assert!(dt > 0.0, "dt должен быть положительным");

        let error = setpoint - measurement;

        // --- Пропорциональная составляющая ---
        let proportional = self.kp * error;

        // --- Интегральная составляющая ---
        self.integral += self.ki * error * dt;

        // --- Дифференциальная составляющая ---
        let derivative = if self.derivative_on_measurement {
            match self.prev_measurement {
                Some(prev) => {
                    let raw = -self.kd * (measurement - prev) / dt;
                    let filtered = self.prev_derivative
                        + self.derivative_filter * (raw - self.prev_derivative);
                    self.prev_derivative = filtered;
                    filtered
                }
                None => Vector::ZERO,
            }
        } else {
            match self.prev_error {
                Some(prev) => {
                    let raw = self.kd * (error - prev) / dt;
                    let filtered = self.prev_derivative
                        + self.derivative_filter * (raw - self.prev_derivative);
                    self.prev_derivative = filtered;
                    filtered
                }
                None => Vector::ZERO,
            }
        };

        // Обновляем историю
        self.prev_error = Some(error);
        self.prev_measurement = Some(measurement);

        // Суммируем
        let mut output = proportional + self.integral + derivative;

        // --- Anti-windup + ограничение по компонентам ---
        if let (Some(min), Some(max)) = (&self.out_min, &self.out_max) {
            for i in 0..3 {
                let clamped = output[i].clamp(min[i], max[i]);
                if output[i] != clamped {
                    let saturation = output[i] - clamped;
                    // Если интеграл толкает в сторону насыщения — откатываем
                    if saturation * self.integral[i] > 0.0 {
                        self.integral[i] -= self.ki[i] * error[i] * dt;
                    }
                }
                output[i] = clamped;
            }
        }

        // --- Ограничение по норме (например, круговой конус тяги) ---
        if let Some(max_norm) = self.out_max_norm {
            let norm = output.length();
            if norm > max_norm && norm > 0.0 {
                let scale = max_norm / norm;
                output *= scale;

                // Back-calculation anti-windup для нормы:
                // уменьшаем интеграл пропорционально перенасыщению
                let excess = (norm - max_norm) / norm;
                self.integral -= self.integral * excess * 0.1;
            }
        }

        output
    }

    /// Текущая интегральная сумма.
    pub fn integral(&self) -> Vector {
        self.integral
    }

    /// Текущие коэффициенты.
    pub fn tunings(&self) -> (Vector, Vector, Vector) {
        (self.kp, self.ki, self.kd)
    }
}

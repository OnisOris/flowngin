use nalgebra::{Vector3, clamp};

#[derive(Debug, Clone)]
pub struct PidController {
    pub kp: Vector3<f64>,
    pub ki: Vector3<f64>,
    pub kd: Vector3<f64>,

    integral: Vector3<f64>,
    prev_error: Option<Vector3<f64>>,
    prev_measurment: Option<Vector3<f64>>,
    prev_derivative: Vector3<f64>,

    out_min: Option<Vector3<f64>>,
    out_max: Option<Vector3<f64>>,
    out_max_norm: Option<f64>,

    derivative_on_mesurement: bool,
    derivative_filter: Vector3<f64>,
}

impl Default for PidController {
    fn default() -> Self {
        Self {
            kp: Vector3::zeros(),
            ki: Vector3::zeros(),
            kd: Vector3::zeros(),
            integral: Vector3::zeros(),
            prev_error: None,
            prev_measurment: None,
            prev_derivative: Vector3::zeros(),
            out_min: None,
            out_max: None,
            out_max_norm: None,
            derivative_on_mesurement: true,
            derivative_filter: Vector3::new(1., 1., 1.),
        }
    }
}

impl PidController {
    pub fn from_scalar(kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp: Vector3::new(kp, kp, kp),
            ki: Vector3::new(ki, ki, ki),
            kd: Vector3::new(kd, kd, kd),
            ..Default::default()
        }
    }

    pub fn new(kp: Vector3<f64>, ki: Vector3<f64>, kd: Vector3<f64>) -> Self {
        Self {
            kp,
            ki,
            kd,
            ..Default::default()
        }
    }

    pub fn set_tunings(&mut self, kp: Vector3<f64>, ki: Vector3<f64>, kd: Vector3<f64>) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }

    pub fn set_outputs_limits(&mut self, min: Vector3<f64>, max: Vector3<f64>) {
        assert!(
            min.iter().zip(max.iter()).all(|(a, b)| a <= b),
            "min must be <= max in every axis"
        );
    }

    pub fn set_outputs_limits_norm(&mut self, max_norm: f64) {
        assert!(max_norm >= 0.0, "norm must be >= 0");
        self.out_max_norm = Some(max_norm);
    }

    pub fn set_derivative_on_mesurement(&mut self, enabled: bool) {
        self.derivative_on_mesurement = enabled;
    }

    pub fn set_derivative_filter(&mut self, alpha: Vector3<f64>) {
        assert!(
            alpha.iter().all(|&a| (0.0..=1.0).contains(&a)),
            "Alpha должен быть в [0, 1] по каждой оси"
        );
    }

    pub fn reset(&mut self) {
        self.integral = Vector3::zeros();
        self.prev_error = None;
        self.prev_measurment = None;
        self.prev_derivative = Vector3::zeros();
    }
    pub fn update(
        &mut self,
        setpoint: Vector3<f64>,
        measurement: Vector3<f64>,
        dt: f64,
    ) -> Vector3<f64> {
        assert!(dt > 0.0, "dt должен быть положительным");

        let error = setpoint - measurement;

        // --- Пропорциональная составляющая ---
        let proportional = self.kp.component_mul(&error);

        // --- Интегральная составляющая ---
        self.integral += self.ki.component_mul(&error) * dt;

        // --- Дифференциальная составляющая ---
        let derivative = if self.derivative_on_measurement {
            match self.prev_measurement {
                Some(prev) => {
                    let raw = -self.kd.component_mul(&(measurement - prev)) / dt;
                    let filtered = self.prev_derivative
                        + self
                            .derivative_filter
                            .component_mul(&(raw - self.prev_derivative));
                    self.prev_derivative = filtered;
                    filtered
                }
                None => Vector3::zeros(),
            }
        } else {
            match self.prev_error {
                Some(prev) => {
                    let raw = self.kd.component_mul(&(error - prev)) / dt;
                    let filtered = self.prev_derivative
                        + self
                            .derivative_filter
                            .component_mul(&(raw - self.prev_derivative));
                    self.prev_derivative = filtered;
                    filtered
                }
                None => Vector3::zeros(),
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
                let clamped = clamp(output[i], min[i], max[i]);
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
            let norm = output.norm();
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
    pub fn integral(&self) -> Vector3<f64> {
        self.integral
    }

    /// Текущие коэффициенты.
    pub fn tunings(&self) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>) {
        (self.kp, self.ki, self.kd)
    }
}

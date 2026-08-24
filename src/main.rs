// Импортируем цвет, чтобы визуально отличить шар и целевую точку.
use kiss3d::color::Color;
// Импортируем основные типы Rapier 3D: физический мир, тела, коллайдеры и векторы.
use rapier3d::prelude::*;
// Импортируем описание демо и нативное окно визуализации Rapier Testbed.
use rapier_testbed3d::{ExampleEntry, TestbedViewer};

mod controller;

use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

// Половина высоты земли: её верхняя поверхность находится на Y = 0.1.
const GROUND_HALF_HEIGHT: f32 = 0.1;
// Радиус единственного динамического шара.
const BALL_RADIUS: f32 = 0.5;
// Коэффициент P-регулятора: чем он больше, тем сильнее шар тянется к цели.
const P_GAIN: f32 = 1.5;
// Ограничиваем силу, чтобы шар преимущественно катился, а не скользил по земле.
const MAX_FORCE: f32 = 4.0;

// Храним физический мир и данные, необходимые контроллеру и визуализатору.
struct Simulation {
    // Полный физический мир Rapier.
    world: PhysicsWorld,
    // Идентификатор динамического шара внутри `world.bodies`.
    ball_handle: RigidBodyHandle,
    // Идентификатор неподвижной земли для настройки её цвета.
    ground_handle: RigidBodyHandle,
    // Идентификатор сенсорной метки цели для настройки её цвета.
    target_collider_handle: ColliderHandle,
    // Точка, к которой должен катиться центр шара.
    target: Vector,
}

// Этот атрибут подготавливает асинхронный цикл нативного окна Kiss3d.
#[kiss3d::main]
// Главная асинхронная функция — с неё начинается выполнение программы.
pub async fn main() {
    let mut controller = controller::PidController::from_scalar(1., 1., 1.);
    // Этот флаг сообщает графическому циклу, что пользователь нажал Ctrl+C.
    let shutdown_requested = Arc::new(AtomicBool::new(false));
    // Передаём обработчику отдельную ссылку на тот же атомарный флаг.
    let signal_flag = Arc::clone(&shutdown_requested);
    // Регистрируем обработчик SIGINT один раз перед запуском графического окна.
    ctrlc::set_handler(move || {
        // Обработчик только выставляет флаг; закрытием окна занимается основной поток.
        signal_flag.store(true, Ordering::Release);
    })
    .expect("Не удалось установить обработчик Ctrl+C");

    // Создаём окно и регистрируем одно демо в интерфейсе Testbed.
    let mut viewer = TestbedViewer::new(vec![ExampleEntry::new("Demo", "P-регулятор шара")]).await;

    // Внешний цикл заново создаёт сцену после нажатия кнопки Restart.
    loop {
        // Очищаем графику и состояние предыдущего запуска симуляции.
        viewer.clear_scene();
        // Создаём новый мир с землёй, одним шаром и целевой меткой.
        let mut simulation = create_simulation();
        // Restart должен также очищать накопленное состояние PID-регулятора.
        controller.reset();

        // Регистрируем все тела и коллайдеры мира в визуализаторе.
        viewer.set_world(&mut simulation.world);
        // Красим землю в спокойный серый цвет.
        viewer.set_initial_body_color(simulation.ground_handle, Color::new(0.45, 0.48, 0.52, 1.0));
        // Красим управляемый шар в синий цвет.
        viewer.set_initial_body_color(simulation.ball_handle, Color::new(0.15, 0.4, 0.95, 1.0));
        // Красим сенсорную метку целевой точки в зелёный цвет.
        viewer.set_initial_collider_color(
            simulation.target_collider_handle,
            Color::new(0.1, 0.85, 0.25, 1.0),
        );

        // Ставим камеру так, чтобы одновременно были видны старт и цель.
        viewer.look_at(
            // Позиция камеры в трёхмерном мире.
            Vector::new(12.0, 9.0, 15.0),
            // Точка между стартовой позицией шара и его целью.
            Vector::new(4.0, 0.0, -1.5),
        );

        // Отрисовываем кадры, пока пользователь не закроет окно или не нажмёт Restart.
        while viewer.render_frame(&mut simulation.world).await {
            // Проверяем флаг после каждого отрисованного кадра.
            if shutdown_requested.load(Ordering::Acquire) {
                // Просим Kiss3d штатно закрыть нативное окно.
                viewer.window_mut().close();
                // Выходим из внутреннего цикла отрисовки.
                break;
            }

            // Учитываем кнопки Play, Pause и Step в интерфейсе Testbed.
            if viewer.simulating() {
                // Пересчитываем управляющую силу по текущему положению шара.
                apply_p_controller(&mut simulation, &mut controller);
                // Продвигаем физический мир на один фиксированный временной шаг.
                simulation.world.step();
            }
        }

        // Ctrl+C или закрытие окна завершает приложение; Restart продолжает внешний цикл.
        if shutdown_requested.load(Ordering::Acquire) || viewer.quitting() {
            // Выходим из внешнего цикла и завершаем `main`.
            break;
        }
    }

    // После выхода из `main` Rust штатно уничтожит viewer и PhysicsWorld.
    println!("Симуляция завершена");
}

// Создаём исходное состояние всей демонстрационной сцены.
fn create_simulation() -> Simulation {
    // PhysicsWorld уже содержит гравитацию, pipeline, тела, коллайдеры и решатели Rapier.
    let mut world = PhysicsWorld::new();

    // Создаём неподвижную землю размером 40 × 0.2 × 40 единиц.
    let (ground_handle, _) = world.insert(
        // Фиксированное тело не двигается под действием сил и гравитации.
        RigidBodyBuilder::fixed(),
        // Аргументы cuboid являются полуразмерами геометрической формы.
        ColliderBuilder::cuboid(20.0, GROUND_HALF_HEIGHT, 20.0).friction(1.0),
    );

    // Ставим центр шара непосредственно над верхней поверхностью земли.
    let ball_start = Vector::new(0.0, GROUND_HALF_HEIGHT + BALL_RADIUS, 0.0);
    // Цель находится в стороне от шара, но остаётся на той же высоте.
    let target = Vector::new(8.0, GROUND_HALF_HEIGHT + BALL_RADIUS, -3.0);

    // Создаём единственный динамический шар и сохраняем его идентификатор.
    let (ball_handle, _) = world.insert(
        // Небольшое линейное и угловое затухание помогает P-регулятору успокоить колебания.
        RigidBodyBuilder::dynamic()
            .translation(ball_start)
            .linear_damping(0.8)
            .angular_damping(0.3),
        // Высокое трение заставляет шар катиться; малая упругость убирает лишние прыжки.
        ColliderBuilder::ball(BALL_RADIUS)
            .friction(1.0)
            .restitution(0.1),
    );

    // Размещаем плоскую метку немного выше поверхности земли.
    let target_marker_position = Vector::new(target.x, GROUND_HALF_HEIGHT + 0.02, target.z);
    // Создаём зелёный цилиндр-маркер, который не участвует в столкновениях.
    let (_, target_collider_handle) = world.insert(
        // Метка неподвижна и только показывает положение целевой точки.
        RigidBodyBuilder::fixed().translation(target_marker_position),
        // `sensor(true)` позволяет шару свободно проезжать через метку.
        ColliderBuilder::cylinder(0.02, 0.4).sensor(true),
    );

    // Возвращаем мир вместе с handle’ами и координатами цели.
    Simulation {
        world,
        ball_handle,
        ground_handle,
        target_collider_handle,
        target,
    }
}

// Вычисляем и прикладываем горизонтальную силу P-регулятора.
fn apply_p_controller(simulation: &mut Simulation, controller: &mut controller::PidController) {
    // Копируем значения до изменяемого заимствования физического тела.
    let ball_handle = simulation.ball_handle;
    // Копируем целевую точку, чтобы использовать её в расчёте ошибки.
    let mut setpoint = simulation.target;
    // PID вызывается один раз на физический шаг, поэтому берём dt из Rapier.
    let dt = simulation.world.integration_parameters.dt;
    // Получаем изменяемую ссылку на единственный динамический шар.
    let ball = &mut simulation.world.bodies[ball_handle];
    let mut measurement = ball.translation();
    // Высотой управляют гравитация и контакт с землёй.
    measurement.y = 0.0;
    setpoint.y = 0.0;
    // PID работает с тем же Vector<f32>, что и Rapier.
    let requested_force = controller.update(setpoint, measurement, dt);
    // Ограничиваем модуль силы для предсказуемого движения и сохранения сцепления с землёй.
    let controller_force = clamp_magnitude(requested_force, MAX_FORCE);

    // Удаляем силу, рассчитанную на предыдущем шаге; гравитацию это не отключает.
    ball.reset_forces(false);
    // Прикладываем новое воздействие и будим шар, если Rapier успел его усыпить.
    ball.add_force(controller_force, true);
}

// Ограничиваем длину вектора заданным максимальным значением.
fn clamp_magnitude(vector: Vector, maximum: f32) -> Vector {
    // Находим текущую длину вектора.
    let length = vector.length();
    // Масштабируем только слишком длинные векторы.
    if length > maximum {
        // Деление на length безопасно, потому что здесь length строго больше положительного maximum.
        vector * (maximum / length)
    } else {
        // Короткий вектор возвращаем без изменений.
        vector
    }
}

// Эти проверки запускаются только командой `cargo test` и не попадают в обычную программу.
#[cfg(test)]
mod tests {
    // Импортируем функции и типы из основного модуля.
    use super::*;

    // Проверяем, что в демонстрации остаётся ровно один динамический объект.
    #[test]
    fn scene_contains_one_dynamic_ball() {
        // Создаём сцену тем же способом, что и основная программа.
        let simulation = create_simulation();
        // Считаем только динамические тела; земля и метка цели являются фиксированными.
        let dynamic_body_count = simulation
            .world
            .bodies
            .iter()
            .filter(|(_, body)| body.is_dynamic())
            .count();

        // Ошибка теста покажет, если в сцене случайно снова появятся лишние шары.
        assert_eq!(dynamic_body_count, 1);
    }

    // Проверяем, что P-регулятор действительно перемещает шар в сторону цели.
    #[test]
    fn controller_moves_ball_towards_target() {
        // Создаём отдельную физическую сцену без открытия графического окна.
        let mut simulation = create_simulation();
        let mut controller = controller::PidController::from_scalar(1.0, 1.0, 1.0);
        // Запоминаем начальное расстояние до целевой точки.
        let initial_distance = distance_to_target(&simulation);

        // Выполняем десять секунд физического времени при стандартных 60 шагах в секунду.
        for _ in 0..600 {
            // Перед каждым шагом пересчитываем управляющую силу.
            apply_p_controller(&mut simulation, &mut controller);
            // Продвигаем физический мир на один шаг.
            simulation.world.step();
        }

        // Измеряем расстояние после работы регулятора.
        let final_distance = distance_to_target(&simulation);
        // Требуем, чтобы шар оказался существенно ближе к цели, чем в начале.
        assert!(final_distance < initial_distance * 0.25);
    }

    // Вычисляем горизонтальное расстояние от шара до его целевой точки.
    fn distance_to_target(simulation: &Simulation) -> f32 {
        // Получаем текущую позицию шара.
        let ball_position = simulation.world.bodies[simulation.ball_handle].translation();
        // Вычисляем ошибку положения.
        let mut difference = simulation.target - ball_position;
        // Не учитываем вертикальную координату, поскольку контроллер работает в плоскости XZ.
        difference.y = 0.0;
        // Возвращаем длину горизонтального вектора.
        difference.length()
    }
}

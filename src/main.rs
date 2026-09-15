// Импортируем цвет, чтобы визуально отличить шар и целевую точку.
use kiss3d::color::Color;
// Импортируем события окна: кнопки мыши и их состояние.
use kiss3d::event::{Action, MouseButton, WindowEvent};
// Импортируем основные типы Rapier 3D: физический мир, тела, коллайдеры и векторы.
use rapier3d::prelude::*;
// Импортируем описание демо и нативное окно визуализации Rapier Testbed.
use rapier_testbed3d::{ExampleEntry, TestbedViewer};
use std::path::Path;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
mod agent;
mod constants;
mod controller;
mod environment;
mod swarm;
mod terrain;
// use crate::agent;
// use crate::controller::PidController;
//
// Радиус динамических шаров как стартовая высота.
const BALL_RADIUS: f32 = 0.5;
// Количество агентов в рое.
const AGENT_COUNT: usize = 5;
// Сколько агентов летят к первой цели (остальные — ко второй).
const GROUP_A_COUNT: usize = 3;

// Храним физический мир и данные, необходимые контроллеру и визуализатору.
struct Simulation {
    // Полный физический мир Rapier.
    world: PhysicsWorld,
    // Идентификаторы динамических шаров внутри `world.bodies`.
    ball_handles: Vec<RigidBodyHandle>,
    // Идентификатор неподвижной земли для настройки её цвета.
    ground_handle: RigidBodyHandle,
    // Идентификаторы сенсорных меток целей для настройки их цвета.
    target_collider_handles: Vec<ColliderHandle>,
    // Тела-носители маркеров целей (перемещаются перетаскиванием мышью).
    target_body_handles: Vec<RigidBodyHandle>,
    // Точки, к которым должны катиться центры шаров.
    targets: Vec<Vector>,
    // Генерируемый STL-рельеф, служащий полом.
    terrain: terrain::Terrain,
}

// Этот атрибут подготавливает асинхронный цикл нативного окна Kiss3d.
#[kiss3d::main]
// Главная асинхронная функция — с неё начинается выполнение программы.
pub async fn main() {
    let mut swarm = swarm::Swarm::new(AGENT_COUNT);
    let info = swarm.agents[0].get_status();
    println!("{}", info);
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
        // Восстанавливаем панель настроек агентов (например, после смены примера).
        register_agent_settings(&mut viewer);
        // Создаём новый мир с землёй, роем из пяти шаров и двумя целями.
        let mut simulation = create_simulation(&swarm.agents);
        // Restart должен также очищать накопленное состояние PID-регулятора.
        swarm.reset();

        // Регистрируем все тела и коллайдеры мира в визуализаторе.
        viewer.set_world(&mut simulation.world);
        // Красим землю в спокойный серый цвет.
        viewer.set_initial_body_color(simulation.ground_handle, Color::new(0.45, 0.48, 0.52, 1.0));
        // Первая группа — синие, вторая — оранжевые.
        for (i, handle) in simulation.ball_handles.iter().enumerate() {
            let color = if i < GROUP_A_COUNT {
                Color::new(0.15, 0.4, 0.95, 1.0)
            } else {
                Color::new(0.9, 0.6, 0.1, 1.0)
            };
            viewer.set_initial_body_color(*handle, color);
        }
        // Красим сенсорные метки целей в зелёный цвет.
        for handle in &simulation.target_collider_handles {
            viewer.set_initial_collider_color(*handle, Color::new(0.1, 0.85, 0.25, 1.0));
        }

        // Ставим камеру так, чтобы были видны старт, обе цели и рельеф.
        viewer.look_at(
            // Позиция камеры в трёхмерном мире.
            Vector::new(30.0, 26.0, 42.0),
            // Точка между стартовыми позициями шаров и их целями.
            Vector::new(0.0, 0.0, 0.0),
        );

        // Отрисовываем кадры, пока пользователь не закроет окно или не нажмёт Restart.
        // Правая кнопка мыши перетаскивает маркер цели по земле.
        let mut prev_right_down = false;
        let mut dragged_target: Option<usize> = None;
        while viewer.render_frame(&mut simulation.world).await {
            // Проверяем флаг после каждого отрисованного кадра.
            if shutdown_requested.load(Ordering::Acquire) {
                // Просим Kiss3d штатно закрыть нативное окно.
                viewer.window_mut().close();
                // Выходим из внутреннего цикла отрисовки.
                break;
            }

            // Перетаскивание цели работает и на паузе.
            handle_target_dragging(
                &mut simulation,
                &viewer,
                &mut prev_right_down,
                &mut dragged_target,
            );

            // Учитываем кнопки Play, Pause и Step в интерфейсе Testbed.
            if viewer.simulating() {
                // Пересчитываем управляющую силу по текущему положению шара.
                apply_swarm_controller(&mut simulation, &mut swarm, &mut viewer);
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

// Регистрируем живые настройки роя в панели "Example Settings" тестбеда.
// `set_restart_on_change(false)` не даёт изменению перезапускать симуляцию:
// значения читаются в `apply_swarm_controller` каждый физический шаг.
fn register_agent_settings(viewer: &mut TestbedViewer) {
    let settings = viewer.example_settings_mut();
    settings.set_restart_on_change("perception_radius", false);
    settings.get_or_set_f32("perception_radius", 6.0, 0.0..=15.0);
    settings.set_restart_on_change("separation_zone", false);
    settings.get_or_set_f32("separation_zone", 3.0, 0.0..=12.0);
    settings.set_restart_on_change("separation_weight", false);
    settings.get_or_set_f32("separation_weight", 4.0, 0.0..=20.0);
    settings.set_restart_on_change("cohesion_weight", false);
    settings.get_or_set_f32("cohesion_weight", 0.5, 0.0..=10.0);
    settings.set_restart_on_change("alignment_weight", false);
    settings.get_or_set_f32("alignment_weight", 0.5, 0.0..=10.0);
    settings.set_restart_on_change("max_speed", false);
    settings.get_or_set_f32("max_speed", 5.0, 0.0..=15.0);
    settings.set_restart_on_change("draw_arrows", false);
    settings.get_or_set_bool("draw_arrows", true);
    settings.set_restart_on_change("arrow_scale", false);
    settings.get_or_set_f32("arrow_scale", 0.4, 0.0..=3.0);
    settings.set_restart_on_change("drag_targets", false);
    settings.get_or_set_bool("drag_targets", true);
}

// Перетаскивание маркера цели правой кнопкой мыши по плоскости земли.
fn handle_target_dragging(
    simulation: &mut Simulation,
    viewer: &TestbedViewer,
    prev_right_down: &mut bool,
    dragged_target: &mut Option<usize>,
) {
    // Пользователь мог отключить перетаскивание в панели настроек.
    let enabled = viewer
        .example_settings()
        .get_bool("drag_targets")
        .unwrap_or(true);
    if !enabled {
        *prev_right_down = false;
        *dragged_target = None;
        return;
    }

    let right_down = viewer.window().get_mouse_button(MouseButton::Button3) == Action::Press;

    // Точка под курсором на рельефе (raycast по миру).
    let hit = viewer
        .mouse()
        .ray
        .and_then(|(origin, dir)| ground_hit(&simulation.world, origin, dir));

    if right_down && !*prev_right_down {
        // Начали тащить: привязываемся к ближайшей к курсору цели.
        *dragged_target = hit.map(|position| nearest_target_index(&position, &simulation.targets));
    }

    if right_down {
        if let (Some(hit), Some(index)) = (hit, *dragged_target) {
            move_target(simulation, index, hit);
        }
    } else {
        *dragged_target = None;
    }

    *prev_right_down = right_down;
}

// Первое пересечение луча из камеры с любым фиксированным коллайдером (полом).
fn ground_hit(
    world: &PhysicsWorld,
    ray_origin: glamx::Vec3,
    ray_dir: glamx::Vec3,
) -> Option<Vector> {
    let ray = Ray::new(ray_origin, ray_dir);
    // Ищем только в неподвижном теле-поле и игнорируем сенсорные метки целей.
    let filter = QueryFilter::only_fixed().exclude_sensors();
    let (_, intersection) = world.cast_ray_and_get_normal(&ray, Real::MAX, true, filter)?;
    Some(ray_origin + ray_dir * intersection.time_of_impact)
}

// Индекс цели, ближайшей к заданной точке (в плоскости XZ).
fn nearest_target_index(position: &Vector, targets: &[Vector]) -> usize {
    let mut best = 0;
    let mut best_distance_sq = f32::MAX;
    for (i, target) in targets.iter().enumerate() {
        let dx = target.x - position.x;
        let dz = target.z - position.z;
        let distance_sq = dx * dx + dz * dz;
        if distance_sq < best_distance_sq {
            best_distance_sq = distance_sq;
            best = i;
        }
    }
    best
}

// Перемещаем цель и её зелёный маркер в новую точку.
fn move_target(simulation: &mut Simulation, index: usize, position: Vector) {
    let mut target = simulation.targets[index];
    target.x = position.x;
    target.z = position.z;
    // Цель должна лежать на высоте рельефа в этом месте.
    target.y = simulation.terrain.height_at(position.x, position.z) + BALL_RADIUS;
    simulation.targets[index] = target;

    let marker_position = Vector::new(
        position.x,
        simulation.terrain.height_at(position.x, position.z) + 0.02,
        position.z,
    );
    simulation.world.bodies[simulation.target_body_handles[index]]
        .set_translation(marker_position, false);
}

// Создаём исходное состояние всей демонстрационной сцены.
fn create_simulation(agents: &[agent::Agent]) -> Simulation {
    // PhysicsWorld уже содержит гравитацию, pipeline, тела, коллайдеры и решатели Rapier.
    let mut world = PhysicsWorld::new();

    // Рельеф 80×80: загружаем из terrain.stl, а если файла ещё нет — генерируем и пишем.
    let terrain = match terrain::Terrain::load_stl(Path::new("terrain.stl")) {
        Ok(t) => t,
        Err(err) => {
            eprintln!("загрузка terrain.stl не удалась ({err}); генерирую заново");
            let t = terrain::Terrain::generate();
            t.write_stl(Path::new("terrain_2.stl"))
                .expect("не удалось записать terrain.stl");
            t
        }
    };

    // Пол — STL-меш, прикреплённый к неподвижному телу.
    let (ground_handle, _) = world.insert(
        // Фиксированное тело не двигается под действием сил и гравитации.
        RigidBodyBuilder::fixed(),
        // Коллайдер построен из вершин и треугольников STL-рельефа.
        terrain.collider(),
    );

    // Стартовые позиции пяти шаров: тесный кластер, чтобы роевые силы были заметны.
    let starts = [
        Vector::new(0.0, 0.0, 0.0),
        Vector::new(0.6, 0.0, 0.0),
        Vector::new(-0.6, 0.0, 0.3),
        Vector::new(0.3, 0.0, -0.6),
        Vector::new(-0.3, 0.0, -0.4),
        // Vector::new(-1.0, 0.0, -1.4),
    ];

    // Создаём динамические шары и сохраняем их идентификаторы.
    let mut ball_handles = Vec::new();
    for (i, start) in starts.iter().enumerate() {
        // Ставим шар на высоту рельефа в его точке старта.
        let position = Vector::new(
            start.x,
            terrain.height_at(start.x, start.z) + BALL_RADIUS,
            start.z,
        );
        let (ball_handle, _) = world.insert(
            // Небольшое линейное и угловое затухание помогает PID успокоить колебания.
            RigidBodyBuilder::dynamic()
                .translation(position)
                .linear_damping(0.8)
                .angular_damping(0.3),
            agents[i].model.collider(),
        );
        ball_handles.push(ball_handle);
    }

    // Две целевые точки на высоте рельефа в местах целей.
    let targets = [
        Vector::new(8.0, terrain.height_at(8.0, -3.0) + BALL_RADIUS, -3.0),
        Vector::new(-7.0, terrain.height_at(-7.0, 3.0) + BALL_RADIUS, 3.0),
    ];

    // Размещаем плоские метки немного выше поверхности земли.
    let mut target_collider_handles = Vec::new();
    let mut target_body_handles = Vec::new();
    for target in &targets {
        let target_marker_position = Vector::new(
            target.x,
            terrain.height_at(target.x, target.z) + 0.02,
            target.z,
        );
        // Создаём зелёный цилиндр-маркер, который не участвует в столкновениях.
        let (target_body_handle, target_collider_handle) = world.insert(
            // Метка неподвижна и только показывает положение целевой точки.
            RigidBodyBuilder::fixed().translation(target_marker_position),
            // `sensor(true)` позволяет шарам свободно проезжать через метку.
            ColliderBuilder::cylinder(0.02, 0.4).sensor(true),
        );
        target_collider_handles.push(target_collider_handle);
        target_body_handles.push(target_body_handle);
    }

    // Возвращаем мир вместе с handle'ами и координатами целей.
    Simulation {
        world,
        ball_handles,
        ground_handle,
        target_collider_handles,
        target_body_handles,
        targets: targets.to_vec(),
        terrain,
    }
}

// Вычисляем и прикладываем горизонтальную силу для каждого агента роя.
fn apply_swarm_controller(
    simulation: &mut Simulation,
    swarm: &mut swarm::Swarm,
    viewer: &mut TestbedViewer,
) {
    // Читаем настройки из UI-панели один раз за физический шаг.
    let settings = viewer.example_settings();
    let perception_radius = settings.get_f32("perception_radius").unwrap_or(6.0);
    let separation_zone = settings.get_f32("separation_zone").unwrap_or(3.0);
    let separation_weight = settings.get_f32("separation_weight").unwrap_or(4.0);
    let cohesion_weight = settings.get_f32("cohesion_weight").unwrap_or(0.5);
    let alignment_weight = settings.get_f32("alignment_weight").unwrap_or(0.5);
    let max_speed = settings.get_f32("max_speed").unwrap_or(5.0);
    let draw_arrows = settings.get_bool("draw_arrows").unwrap_or(true);
    let arrow_scale = settings.get_f32("arrow_scale").unwrap_or(0.4);

    // Переносим настройки в каждого агента.
    for agent in &mut swarm.agents {
        agent.boid_controller.perception_radius = perception_radius;
        agent.boid_controller.separation_coeff = separation_zone;
        agent.boid_controller.separation_weight = separation_weight;
        agent.boid_controller.cohesion_weight = cohesion_weight;
        agent.boid_controller.alignment_weight = alignment_weight;
        agent.boid_controller.max_speed = max_speed;
    }

    // PID вызывается один раз на физический шаг, поэтому берём dt из Rapier.
    let dt = simulation.world.integration_parameters.dt;

    // Первый проход: собираем актуальные состояния всех шаров в среду,
    // чтобы каждый агент видел соседей по их текущим позициям.
    let states: Vec<agent::State> = simulation
        .ball_handles
        .iter()
        .map(|handle| {
            let body = &simulation.world.bodies[*handle];
            agent::State::new(body.translation(), body.linvel())
        })
        .collect();
    swarm.environment.set_states(states);
    let environment = &swarm.environment;

    // Второй проход: считаем силу каждому агенту и прикладываем к его шару.
    for i in 0..swarm.agent_count() {
        // Первая группа летит к первой цели, остальные — ко второй.
        let target = if i < GROUP_A_COUNT {
            simulation.targets[0]
        } else {
            simulation.targets[1]
        };
        let setpoint = target;

        let ball_handle = simulation.ball_handles[i];
        let ball = &mut simulation.world.bodies[ball_handle];
        let state = agent::State::new(ball.translation(), ball.linvel());

        let requested_force = swarm.agents[i].update(setpoint, state, dt, environment);
        println!("Requested Force: {:?}", requested_force);

        ball.reset_forces(false);
        ball.add_force(requested_force, true);

        // Визуализируем итоговый вектор управления в цвете группы агента.
        if draw_arrows {
            let color = if i < GROUP_A_COUNT {
                Color::new(0.15, 0.4, 0.95, 1.0)
            } else {
                Color::new(0.9, 0.6, 0.1, 1.0)
            };
            draw_force_arrow(
                viewer.window_mut(),
                ball.translation(),
                requested_force,
                color,
                arrow_scale,
            );
        }
    }
}

// Рисуем стрелку силы из точки `start` вдоль вектора `force`.
// Длина стрелки пропорциональна модулю силы: `length = |force| * scale`.
fn draw_force_arrow(
    window: &mut kiss3d::window::Window,
    start: Vector,
    force: Vector,
    color: Color,
    scale: f32,
) {
    let magnitude = force.length();
    if magnitude < 1e-6 {
        // Нулевая сила — никакой стрелки.
        return;
    }
    // Верхний предел защищает от гигантских всплесков силы (например, на старте).
    let length = (magnitude * scale).min(6.0);
    let unit = force / magnitude;
    let end = start + unit * length;

    // Основная линия стрелки.
    window.draw_line(
        glamx::Vec3::new(start.x, start.y, start.z),
        glamx::Vec3::new(end.x, end.y, end.z),
        color,
        2.0,
        false,
    );

    // Наконечник «Рогаткой»: размеры привязаны к длине стрелки, чтобы
    // наконечник не превышал древка у коротких стрелок.
    let perpendicular = Vector::new(-unit.z, 0.0, unit.x);
    let head = length.clamp(0.08, 0.6) * 0.5;
    let flair = head * 0.4;
    let feather_a = end + (-unit * head + perpendicular * flair);
    let feather_b = end + (-unit * head - perpendicular * flair);
    window.draw_line(
        glamx::Vec3::new(end.x, end.y, end.z),
        glamx::Vec3::new(feather_a.x, feather_a.y, feather_a.z),
        color,
        2.0,
        false,
    );
    window.draw_line(
        glamx::Vec3::new(end.x, end.y, end.z),
        glamx::Vec3::new(feather_b.x, feather_b.y, feather_b.z),
        color,
        2.0,
        false,
    );
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

// // Эти проверки запускаются только командой `cargo test` и не попадают в обычную программу.
// #[cfg(test)]
// mod tests {
//     // Импортируем функции и типы из основного модуля.
//     use super::*;
//
//     // Проверяем, что в демонстрации остаётся ровно один динамический объект.
//     #[test]
//     fn scene_contains_one_dynamic_ball() {
//         // Создаём сцену тем же способом, что и основная программа.
//         let simulation = create_simulation();
//         // Считаем только динамические тела; земля и метка цели являются фиксированными.
//         let dynamic_body_count = simulation
//             .world
//             .bodies
//             .iter()
//             .filter(|(_, body)| body.is_dynamic())
//             .count();
//
//         // Ошибка теста покажет, если в сцене случайно снова появятся лишние шары.
//         assert_eq!(dynamic_body_count, 1);
//     }

//     // Проверяем, что P-регулятор действительно перемещает шар в сторону цели.
//     #[test]
//     fn controller_moves_ball_towards_target() {
//         // Создаём отдельную физическую сцену без открытия графического окна.
//         let mut simulation = create_simulation();
//         let mut controller = controller::PidController::from_scalar(5000.0, 4000000.0, 4.0);
//         let limit = 9999999999.;
//         let limit_vector: [f32; 3] = [limit, limit, limit];
//         let limit_vector = Vec3::from_array(limit_vector);
//         // let limit_vector = Vec3()
//         controller.set_outputs_limits_norm(limit);
//         controller.set_outputs_limits(limit_vector, limit_vector);
//         // Запоминаем начальное расстояние до целевой точки.
//         let initial_distance = distance_to_target(&simulation);
//
//         // Выполняем десять секунд физического времени при стандартных 60 шагах в секунду.
//         for _ in 0..600 {
//             // Перед каждым шагом пересчитываем управляющую силу.
//             apply_p_controller(&mut simulation, &mut controller);
//             // Продвигаем физический мир на один шаг.
//             simulation.world.step();
//         }
//
//         // Измеряем расстояние после работы регулятора.
//         let final_distance = distance_to_target(&simulation);
//         // Требуем, чтобы шар оказался существенно ближе к цели, чем в начале.
//         assert!(final_distance < initial_distance * 0.25);
//     }
//
//     // Вычисляем горизонтальное расстояние от шара до его целевой точки.
//     fn distance_to_target(simulation: &Simulation) -> f32 {
//         // Получаем текущую позицию шара.
//         let ball_position = simulation.world.bodies[simulation.ball_handle].translation();
//         // Вычисляем ошибку положения.
//         let mut difference = simulation.target - ball_position;
//         // Не учитываем вертикальную координату, поскольку контроллер работает в плоскости XZ.
//         difference.y = 0.0;
//         // Возвращаем длину горизонтального вектора.
//         difference.length()
//     }
// }

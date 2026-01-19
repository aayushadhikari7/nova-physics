//! 3D Interactive Physics Playground
//!
//! Controls:
//! - Left click: Spawn box where you're looking
//! - Right click: Spawn ball where you're looking
//! - Space: Reset
//! - R: Toggle rain mode
//! - WASD: Move camera
//! - Mouse: Look around
//! - Q/E: Camera up/down
//! - 1-5: Spawn preset structures

use macroquad::prelude::*;
use nova::prelude::{
    BoxShape, CollisionShape, PhysicsWorld, RigidBodyHandle, RigidBodyType, SphereShape,
};

// Nova uses a different version of glam, so we need conversion helpers
type NovaVec3 = nova::prelude::Vec3;
type NovaQuat = nova::prelude::Quat;

fn from_nova_vec3(v: NovaVec3) -> Vec3 {
    vec3(v.x, v.y, v.z)
}

fn from_nova_quat(q: NovaQuat) -> Quat {
    Quat::from_xyzw(q.x, q.y, q.z, q.w)
}

struct PhysicsObject {
    body_handle: RigidBodyHandle,
    is_sphere: bool,
    size: Vec3,
    color: Color,
}

fn random_color() -> Color {
    Color::new(
        rand::gen_range(0.3, 1.0),
        rand::gen_range(0.3, 1.0),
        rand::gen_range(0.3, 1.0),
        1.0,
    )
}

fn bright_color() -> Color {
    let hue = rand::gen_range(0.0, 360.0);
    let s = 0.8;
    let v = 0.9;

    let c = v * s;
    let x = c * (1.0 - ((hue / 60.0) % 2.0 - 1.0).abs());
    let m = v - c;

    let (r, g, b) = match (hue / 60.0) as i32 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };

    Color::new(r + m, g + m, b + m, 1.0)
}

fn spawn_box_at(world: &mut PhysicsWorld, objects: &mut Vec<PhysicsObject>, pos: NovaVec3, size: Vec3) {
    let half = NovaVec3::new(size.x / 2.0, size.y / 2.0, size.z / 2.0);
    let mass = size.x * size.y * size.z;
    let body = world.create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(pos)
        .mass(mass)
        .build();
    let handle = world.insert_body(body);
    world.insert_collider(
        world.create_collider(handle, CollisionShape::Box(BoxShape::new(half)))
            .friction(0.6)
            .restitution(0.2)
            .build()
    );
    objects.push(PhysicsObject {
        body_handle: handle,
        is_sphere: false,
        size,
        color: bright_color(),
    });
}

fn spawn_ball_at(world: &mut PhysicsWorld, objects: &mut Vec<PhysicsObject>, pos: NovaVec3, radius: f32) {
    let mass = 4.0 / 3.0 * 3.14159 * radius * radius * radius;
    let body = world.create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(pos)
        .mass(mass)
        .build();
    let handle = world.insert_body(body);
    world.insert_collider(
        world.create_collider(handle, CollisionShape::Sphere(SphereShape::new(radius)))
            .friction(0.4)
            .restitution(0.6)
            .build()
    );
    objects.push(PhysicsObject {
        body_handle: handle,
        is_sphere: true,
        size: vec3(radius * 2.0, radius * 2.0, radius * 2.0),
        color: bright_color(),
    });
}

fn setup_world() -> (PhysicsWorld, Vec<PhysicsObject>) {
    let mut world = PhysicsWorld::new();
    world.set_gravity(NovaVec3::new(0.0, -20.0, 0.0));

    // Ground - thick slab
    let ground = world.create_body()
        .body_type(RigidBodyType::Static)
        .position(NovaVec3::new(0.0, -1.0, 0.0))
        .build();
    let gh = world.insert_body(ground);
    world.insert_collider(
        world.create_collider(gh, CollisionShape::Box(BoxShape::new(NovaVec3::new(20.0, 1.0, 20.0))))
            .friction(0.8)
            .build()
    );

    // Walls - taller and thicker
    let wall_height = 8.0;
    let wall_positions = [
        (NovaVec3::new(-20.0, wall_height / 2.0, 0.0), NovaVec3::new(0.5, wall_height / 2.0, 20.0)),
        (NovaVec3::new(20.0, wall_height / 2.0, 0.0), NovaVec3::new(0.5, wall_height / 2.0, 20.0)),
        (NovaVec3::new(0.0, wall_height / 2.0, -20.0), NovaVec3::new(20.0, wall_height / 2.0, 0.5)),
        (NovaVec3::new(0.0, wall_height / 2.0, 20.0), NovaVec3::new(20.0, wall_height / 2.0, 0.5)),
    ];

    for (pos, half_extents) in wall_positions {
        let wall = world.create_body()
            .body_type(RigidBodyType::Static)
            .position(pos)
            .build();
        let wh = world.insert_body(wall);
        world.insert_collider(
            world.create_collider(wh, CollisionShape::Box(BoxShape::new(half_extents)))
                .friction(0.5)
                .build()
        );
    }

    (world, Vec::new())
}

fn spawn_tower(world: &mut PhysicsWorld, objects: &mut Vec<PhysicsObject>, base: Vec3, height: i32) {
    for i in 0..height {
        let pos = NovaVec3::new(base.x, base.y + 1.0 + i as f32 * 1.05, base.z);
        spawn_box_at(world, objects, pos, vec3(1.0, 1.0, 1.0));
    }
}

fn spawn_pyramid(world: &mut PhysicsWorld, objects: &mut Vec<PhysicsObject>, base: Vec3, layers: i32) {
    for layer in 0..layers {
        let count = layers - layer;
        let offset = -(count as f32 - 1.0) / 2.0;
        for i in 0..count {
            for j in 0..count {
                let pos = NovaVec3::new(
                    base.x + offset + i as f32,
                    base.y + 0.5 + layer as f32 * 1.0,
                    base.z + offset + j as f32,
                );
                spawn_box_at(world, objects, pos, vec3(0.95, 0.95, 0.95));
            }
        }
    }
}

fn spawn_wall(world: &mut PhysicsWorld, objects: &mut Vec<PhysicsObject>, base: Vec3, width: i32, height: i32) {
    for y in 0..height {
        for x in 0..width {
            let offset_x = if y % 2 == 0 { 0.0 } else { 0.5 };
            let pos = NovaVec3::new(
                base.x + x as f32 * 1.0 - width as f32 / 2.0 + offset_x,
                base.y + 0.5 + y as f32 * 1.0,
                base.z,
            );
            spawn_box_at(world, objects, pos, vec3(1.0, 1.0, 0.5));
        }
    }
}

fn spawn_ball_pile(world: &mut PhysicsWorld, objects: &mut Vec<PhysicsObject>, center: Vec3, count: i32) {
    for _ in 0..count {
        let pos = NovaVec3::new(
            center.x + rand::gen_range(-2.0, 2.0),
            center.y + rand::gen_range(5.0, 15.0),
            center.z + rand::gen_range(-2.0, 2.0),
        );
        spawn_ball_at(world, objects, pos, rand::gen_range(0.3, 0.8));
    }
}

fn draw_box_rotated(pos: Vec3, size: Vec3, rot: Quat, color: Color) {
    // Get the 8 corners of the box
    let hx = size.x / 2.0;
    let hy = size.y / 2.0;
    let hz = size.z / 2.0;

    let corners = [
        vec3(-hx, -hy, -hz), vec3(hx, -hy, -hz),
        vec3(hx, -hy, hz), vec3(-hx, -hy, hz),
        vec3(-hx, hy, -hz), vec3(hx, hy, -hz),
        vec3(hx, hy, hz), vec3(-hx, hy, hz),
    ];

    let transformed: Vec<Vec3> = corners.iter()
        .map(|c| pos + rot.mul_vec3(*c))
        .collect();

    // Draw faces as triangles
    let faces = [
        // Bottom
        (0, 1, 2), (0, 2, 3),
        // Top
        (4, 6, 5), (4, 7, 6),
        // Front
        (0, 4, 5), (0, 5, 1),
        // Back
        (2, 6, 7), (2, 7, 3),
        // Left
        (0, 3, 7), (0, 7, 4),
        // Right
        (1, 5, 6), (1, 6, 2),
    ];

    for (i0, i1, i2) in faces {
        draw_triangle3d(transformed[i0], transformed[i1], transformed[i2], color);
    }

    // Draw edges
    let edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ];

    let edge_color = Color::new(0.0, 0.0, 0.0, 0.5);
    for (a, b) in edges {
        draw_line_3d(transformed[a], transformed[b], edge_color);
    }
}

fn draw_sphere_better(pos: Vec3, radius: f32, color: Color) {
    // Draw with more segments for smoother look
    let rings = 12;
    let sectors = 16;

    for i in 0..rings {
        let phi1 = std::f32::consts::PI * (i as f32 / rings as f32 - 0.5);
        let phi2 = std::f32::consts::PI * ((i + 1) as f32 / rings as f32 - 0.5);

        for j in 0..sectors {
            let theta1 = 2.0 * std::f32::consts::PI * j as f32 / sectors as f32;
            let theta2 = 2.0 * std::f32::consts::PI * (j + 1) as f32 / sectors as f32;

            let p1 = pos + vec3(
                radius * phi1.cos() * theta1.cos(),
                radius * phi1.sin(),
                radius * phi1.cos() * theta1.sin(),
            );
            let p2 = pos + vec3(
                radius * phi1.cos() * theta2.cos(),
                radius * phi1.sin(),
                radius * phi1.cos() * theta2.sin(),
            );
            let p3 = pos + vec3(
                radius * phi2.cos() * theta2.cos(),
                radius * phi2.sin(),
                radius * phi2.cos() * theta2.sin(),
            );
            let p4 = pos + vec3(
                radius * phi2.cos() * theta1.cos(),
                radius * phi2.sin(),
                radius * phi2.cos() * theta1.sin(),
            );

            // Shade based on height for simple lighting effect
            let shade = 0.7 + 0.3 * ((p1.y + p3.y) / 2.0 - pos.y + radius) / (2.0 * radius);
            let shaded = Color::new(color.r * shade, color.g * shade, color.b * shade, 1.0);

            draw_triangle3d(p1, p2, p3, shaded);
            draw_triangle3d(p1, p3, p4, shaded);
        }
    }
}

#[macroquad::main("Nova Physics 3D Playground")]
async fn main() {
    let (mut world, mut objects) = setup_world();
    let mut rain_mode = false;
    let mut rain_timer = 0.0;
    let mut paused = false;
    let mut mouse_captured = true;

    // Camera
    let mut cam_pos = vec3(0.0, 12.0, 30.0);
    let mut cam_yaw: f32 = 0.0;
    let mut cam_pitch: f32 = -0.3;

    set_cursor_grab(true);
    show_mouse(false);

    loop {
        clear_background(Color::new(0.02, 0.02, 0.05, 1.0));

        // Input handling
        if mouse_captured {
            let mouse_delta = mouse_delta_position();
            cam_yaw -= mouse_delta.x * 0.2;
            cam_pitch -= mouse_delta.y * 0.2;
            cam_pitch = cam_pitch.clamp(-1.4, 1.4);
        }

        // Camera movement
        let forward = vec3(cam_yaw.sin(), 0.0, -cam_yaw.cos());
        let right = vec3(cam_yaw.cos(), 0.0, cam_yaw.sin());
        let speed = if is_key_down(KeyCode::LeftShift) { 30.0 } else { 15.0 };
        let move_speed = speed * get_frame_time();

        if is_key_down(KeyCode::W) { cam_pos += forward * move_speed; }
        if is_key_down(KeyCode::S) { cam_pos -= forward * move_speed; }
        if is_key_down(KeyCode::A) { cam_pos -= right * move_speed; }
        if is_key_down(KeyCode::D) { cam_pos += right * move_speed; }
        if is_key_down(KeyCode::Q) { cam_pos.y -= move_speed; }
        if is_key_down(KeyCode::E) { cam_pos.y += move_speed; }

        // Get look direction for spawning
        let look_dir = vec3(
            cam_yaw.sin() * cam_pitch.cos(),
            cam_pitch.sin(),
            -cam_yaw.cos() * cam_pitch.cos(),
        );
        let spawn_pos = cam_pos + look_dir * 8.0;
        let spawn_nova = NovaVec3::new(spawn_pos.x, spawn_pos.y, spawn_pos.z);

        // Reset
        if is_key_pressed(KeyCode::Space) {
            let (new_world, new_objects) = setup_world();
            world = new_world;
            objects = new_objects;
        }

        if is_key_pressed(KeyCode::R) { rain_mode = !rain_mode; }
        if is_key_pressed(KeyCode::P) { paused = !paused; }

        // Mouse capture toggle
        if is_key_pressed(KeyCode::Escape) {
            mouse_captured = false;
            set_cursor_grab(false);
            show_mouse(true);
        }
        if is_mouse_button_pressed(MouseButton::Middle) {
            mouse_captured = true;
            set_cursor_grab(true);
            show_mouse(false);
        }

        // Spawn objects where looking
        if mouse_captured {
            if is_mouse_button_pressed(MouseButton::Left) {
                let size = rand::gen_range(0.8, 1.5);
                spawn_box_at(&mut world, &mut objects, spawn_nova, vec3(size, size, size));
            }
            if is_mouse_button_pressed(MouseButton::Right) {
                spawn_ball_at(&mut world, &mut objects, spawn_nova, rand::gen_range(0.4, 0.9));
            }
        }

        // Preset structures
        if is_key_pressed(KeyCode::Key1) {
            spawn_tower(&mut world, &mut objects, vec3(0.0, 0.0, 0.0), 10);
        }
        if is_key_pressed(KeyCode::Key2) {
            spawn_pyramid(&mut world, &mut objects, vec3(0.0, 0.0, 0.0), 5);
        }
        if is_key_pressed(KeyCode::Key3) {
            spawn_wall(&mut world, &mut objects, vec3(0.0, 0.0, 5.0), 8, 6);
        }
        if is_key_pressed(KeyCode::Key4) {
            spawn_ball_pile(&mut world, &mut objects, vec3(0.0, 0.0, 0.0), 30);
        }
        if is_key_pressed(KeyCode::Key5) {
            // Chaos mode - spawn everything
            spawn_tower(&mut world, &mut objects, vec3(-8.0, 0.0, -8.0), 8);
            spawn_tower(&mut world, &mut objects, vec3(8.0, 0.0, -8.0), 8);
            spawn_tower(&mut world, &mut objects, vec3(-8.0, 0.0, 8.0), 8);
            spawn_tower(&mut world, &mut objects, vec3(8.0, 0.0, 8.0), 8);
            spawn_pyramid(&mut world, &mut objects, vec3(0.0, 0.0, 0.0), 4);
        }

        // Rain mode
        if rain_mode && objects.len() < 500 {
            rain_timer += get_frame_time();
            if rain_timer > 0.03 {
                rain_timer = 0.0;
                let x = rand::gen_range(-15.0, 15.0);
                let z = rand::gen_range(-15.0, 15.0);
                let pos = NovaVec3::new(x, 20.0, z);
                if rand::gen_range(0.0, 1.0) > 0.5 {
                    spawn_box_at(&mut world, &mut objects, pos, vec3(0.8, 0.8, 0.8));
                } else {
                    spawn_ball_at(&mut world, &mut objects, pos, 0.5);
                }
            }
        }

        // Physics step
        if !paused {
            world.step(1.0 / 60.0);
        }

        // Remove objects that fell too far
        let mut to_remove = Vec::new();
        for (i, obj) in objects.iter().enumerate() {
            if let Some(body) = world.get_body(obj.body_handle) {
                if body.position.y < -20.0 {
                    to_remove.push(i);
                }
            }
        }
        for i in to_remove.into_iter().rev() {
            objects.remove(i);
        }

        // Setup 3D camera
        let look_at = cam_pos + look_dir;
        set_camera(&Camera3D {
            position: cam_pos,
            target: look_at,
            up: vec3(0.0, 1.0, 0.0),
            fovy: 60.0,
            projection: Projection::Perspective,
            ..Default::default()
        });

        // Draw ground with grid
        let ground_color = Color::new(0.15, 0.2, 0.15, 1.0);
        draw_cube(vec3(0.0, -1.0, 0.0), vec3(40.0, 2.0, 40.0), None, ground_color);

        // Grid lines
        for i in -20..=20 {
            let alpha = if i % 5 == 0 { 0.4 } else { 0.15 };
            let grid_color = Color::new(0.4, 0.5, 0.4, alpha);
            draw_line_3d(vec3(i as f32, 0.01, -20.0), vec3(i as f32, 0.01, 20.0), grid_color);
            draw_line_3d(vec3(-20.0, 0.01, i as f32), vec3(20.0, 0.01, i as f32), grid_color);
        }

        // Draw walls (subtle)
        let wall_color = Color::new(0.2, 0.2, 0.25, 0.3);
        draw_cube(vec3(-20.0, 4.0, 0.0), vec3(1.0, 8.0, 40.0), None, wall_color);
        draw_cube(vec3(20.0, 4.0, 0.0), vec3(1.0, 8.0, 40.0), None, wall_color);
        draw_cube(vec3(0.0, 4.0, -20.0), vec3(40.0, 8.0, 1.0), None, wall_color);
        draw_cube(vec3(0.0, 4.0, 20.0), vec3(40.0, 8.0, 1.0), None, wall_color);

        // Draw physics objects with rotation
        for obj in &objects {
            if let Some(body) = world.get_body(obj.body_handle) {
                let pos = from_nova_vec3(body.position);
                let rot = from_nova_quat(body.rotation);

                if obj.is_sphere {
                    draw_sphere_better(pos, obj.size.x / 2.0, obj.color);
                } else {
                    draw_box_rotated(pos, obj.size, rot, obj.color);
                }
            }
        }

        // Draw spawn preview
        if mouse_captured {
            draw_sphere(spawn_pos, 0.1, None, Color::new(1.0, 1.0, 1.0, 0.5));
        }

        // 2D UI
        set_default_camera();

        let ui_color = WHITE;
        draw_text(&format!("Objects: {}", objects.len()), 10.0, 30.0, 30.0, ui_color);
        draw_text(&format!("FPS: {}", get_fps()), 10.0, 60.0, 30.0, ui_color);

        if rain_mode {
            draw_text("RAIN MODE [R]", 10.0, 90.0, 30.0, GREEN);
        }
        if paused {
            draw_text("PAUSED [P]", 10.0, 120.0, 30.0, YELLOW);
        }

        // Controls help
        let help_y = screen_height() - 80.0;
        draw_text("WASD: Move | Mouse: Look | Shift: Fast", 10.0, help_y, 20.0, GRAY);
        draw_text("Left/Right Click: Spawn Box/Ball | 1-5: Presets", 10.0, help_y + 25.0, 20.0, GRAY);
        draw_text("R: Rain | P: Pause | Space: Reset | ESC: Release mouse", 10.0, help_y + 50.0, 20.0, GRAY);

        // Crosshair
        if mouse_captured {
            let cx = screen_width() / 2.0;
            let cy = screen_height() / 2.0;
            draw_circle(cx, cy, 3.0, WHITE);
            draw_ring(cx, cy, 8.0, 10.0, 0.0, 360.0, WHITE);
        }

        next_frame().await
    }
}

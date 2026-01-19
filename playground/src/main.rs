//! Nova Physics Playground - MEGA EDITION!
//!
//! A Minecraft-themed 3D physics sandbox built with Bevy, integrating the Nova physics engine.
//! NOW WITH 50+ FEATURES, TOOLS, AND PRESETS!

mod arena;
mod components;
mod convert;
mod plugins;
mod presets;
mod resources;

use bevy::prelude::*;
use bevy::window::{PresentMode, WindowMode};

use plugins::{
    CameraPlugin, ConsolePlugin, DebugPlugin, EffectsPlugin, InputPlugin, InventoryPlugin,
    PhysicsPlugin, SettingsPlugin, SpawningPlugin, TimeControlPlugin, ToolsPlugin, VehiclePlugin,
};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Nova Physics Playground - MEGA EDITION".to_string(),
                // Start in borderless fullscreen for native resolution
                mode: WindowMode::BorderlessFullscreen(bevy::window::MonitorSelection::Primary),
                present_mode: PresentMode::AutoVsync,
                ..default()
            }),
            ..default()
        }))
        // Core plugins
        .add_plugins((
            PhysicsPlugin,
            CameraPlugin,
            InputPlugin,
            InventoryPlugin,
            SpawningPlugin,
            ToolsPlugin,
            TimeControlPlugin,
            DebugPlugin,
            SettingsPlugin,
            EffectsPlugin,
            ConsolePlugin,
            VehiclePlugin,
        ))
        // Setup systems
        .add_systems(Startup, arena::setup_arena)
        .add_systems(
            Update,
            (preset_spawning, scene_reset, quick_spawn_keys, zone_spawning),
        )
        .run();
}

/// Handle preset spawning via keyboard shortcuts
fn preset_spawning(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut nova: ResMut<resources::NovaWorld>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut handle_to_entity: ResMut<resources::HandleToEntity>,
    camera_query: Query<&GlobalTransform, With<plugins::camera::PlayerCamera>>,
) {
    let Ok(camera_transform) = camera_query.get_single() else {
        return;
    };

    let cam_pos = camera_transform.translation();
    let cam_forward = camera_transform.forward().as_vec3();
    let spawn_pos = cam_pos + cam_forward * 5.0;

    // T - Tower
    if keyboard.just_pressed(KeyCode::KeyT) {
        presets::spawn_tower(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            10,
        );
    }

    // Y - Pyramid
    if keyboard.just_pressed(KeyCode::KeyY) {
        presets::spawn_pyramid(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            5,
        );
    }

    // U - Newton's Cradle
    if keyboard.just_pressed(KeyCode::KeyU) {
        presets::spawn_newtons_cradle(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 3.0, 0.0),
            5,
        );
    }

    // I - Wrecking Ball
    if keyboard.just_pressed(KeyCode::KeyI) {
        presets::spawn_wrecking_ball(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 6.0, 0.0),
            8,
        );
    }

    // O - Dominos
    if keyboard.just_pressed(KeyCode::KeyO) {
        presets::spawn_dominos(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            20,
            Vec3::new(cam_forward.x, 0.0, cam_forward.z).normalize(),
        );
    }

    // H - Ragdoll
    if keyboard.just_pressed(KeyCode::KeyH) {
        presets::spawn_ragdoll(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 2.0, 0.0),
        );
    }

    // J - Bridge
    if keyboard.just_pressed(KeyCode::KeyJ) {
        presets::spawn_bridge(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            10,
        );
    }

    // K - Catapult
    if keyboard.just_pressed(KeyCode::KeyK) {
        presets::spawn_catapult(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
        );
    }

    // L - Pendulum Wall
    if keyboard.just_pressed(KeyCode::KeyL) {
        presets::spawn_pendulum_wall(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 5.0, 0.0),
            5,
            3,
        );
    }

    // N - Controllable Car (press F to enter/exit)
    if keyboard.just_pressed(KeyCode::KeyN) {
        plugins::vehicle::spawn_controllable_car(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 1.0, 0.0),
        );
    }

    // M - Ferris Wheel
    if keyboard.just_pressed(KeyCode::KeyM) {
        presets::spawn_ferris_wheel(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 5.0, 0.0),
            6,
        );
    }

    // Comma - Windmill
    if keyboard.just_pressed(KeyCode::Comma) {
        presets::spawn_windmill(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 3.0, 0.0),
        );
    }

    // Period - Spiral Staircase
    if keyboard.just_pressed(KeyCode::Period) {
        presets::spawn_spiral_staircase(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            20,
        );
    }

    // Slash - Box Rain
    if keyboard.just_pressed(KeyCode::Slash) {
        presets::spawn_box_rain(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 10.0, 0.0),
            30,
        );
    }

    // Backquote (`) - Bowling Alley
    if keyboard.just_pressed(KeyCode::Backquote) {
        presets::spawn_bowling(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            Vec3::new(cam_forward.x, 0.0, cam_forward.z).normalize(),
        );
    }

    // Digit0 - Seesaw
    if keyboard.just_pressed(KeyCode::Digit0) {
        presets::spawn_seesaw(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
        );
    }

    // Minus - Swing Set
    if keyboard.just_pressed(KeyCode::Minus) {
        presets::spawn_swing(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
        );
    }

    // Equal - Brick Wall
    if keyboard.just_pressed(KeyCode::Equal) {
        presets::spawn_wall(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            8,  // width
            6,  // height
        );
    }

    // Backslash - Ball Avalanche
    if keyboard.just_pressed(KeyCode::Backslash) {
        presets::spawn_avalanche(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 8.0, 0.0),
            40,
        );
    }

    // Q - Jenga Tower
    if keyboard.just_pressed(KeyCode::KeyQ) {
        presets::spawn_jenga(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            15, // layers
        );
    }

    // X - Pool/Billiards
    if keyboard.just_pressed(KeyCode::KeyX) {
        presets::spawn_pool_table(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
        );
    }

    // Z - Trebuchet
    if keyboard.just_pressed(KeyCode::KeyZ) {
        presets::spawn_trebuchet(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
        );
    }

    // C - Ramp
    if keyboard.just_pressed(KeyCode::KeyC) {
        presets::spawn_ramp(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            Vec3::new(cam_forward.x, 0.0, cam_forward.z).normalize(),
            6.0,  // length
            25.0, // angle in degrees
        );
    }

    // Quote - Spinning Platform
    if keyboard.just_pressed(KeyCode::Quote) {
        presets::spawn_spinner_platform(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 0.5, 0.0),
            4, // arm count
        );
    }

    // BracketRight - Ball Pit
    if keyboard.just_pressed(KeyCode::BracketRight) {
        presets::spawn_ball_pit(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            50, // ball count
        );
    }

    // ============ BONKERS MODE KEYS ============

    // Numpad1 - DOMINO SPIRAL (100 dominos!)
    if keyboard.just_pressed(KeyCode::Numpad1) {
        presets::spawn_domino_spiral(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            100,
        );
    }

    // Numpad2 - GIANT PENDULUM OF DOOM
    if keyboard.just_pressed(KeyCode::Numpad2) {
        presets::spawn_giant_pendulum(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
        );
    }

    // Numpad3 - WRECKING BALL VS TOWER!
    if keyboard.just_pressed(KeyCode::Numpad3) {
        presets::spawn_wrecking_ball_vs_tower(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
        );
    }

    // Numpad4 - VOLCANO ERUPTION!
    if keyboard.just_pressed(KeyCode::Numpad4) {
        presets::spawn_volcano(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            40,
        );
    }

    // Numpad5 - STAIRCASE OF DOOM
    if keyboard.just_pressed(KeyCode::Numpad5) {
        presets::spawn_staircase_of_doom(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 5.0, 0.0),
            15,
        );
    }

    // Numpad6 - PACHINKO BOARD
    if keyboard.just_pressed(KeyCode::Numpad6) {
        presets::spawn_pachinko(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
        );
    }

    // Numpad7 - BALL TSUNAMI!!!
    if keyboard.just_pressed(KeyCode::Numpad7) {
        presets::spawn_ball_tsunami(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos - Vec3::new(8.0, 0.0, 0.0),
            100,
        );
    }

    // Numpad8 - CHAIN REACTION
    if keyboard.just_pressed(KeyCode::Numpad8) {
        presets::spawn_chain_reaction(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
        );
    }

    // Numpad9 - GYROSCOPE
    if keyboard.just_pressed(KeyCode::Numpad9) {
        presets::spawn_gyroscope(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 3.0, 0.0),
        );
    }

    // Numpad0 - HAMSTER WHEEL
    if keyboard.just_pressed(KeyCode::Numpad0) {
        presets::spawn_hamster_wheel(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 4.0, 0.0),
        );
    }

    // NumpadAdd - CHAOS CUBE EXPLOSION!
    if keyboard.just_pressed(KeyCode::NumpadAdd) {
        presets::spawn_chaos_cube(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            5, // 5x5x5 = 125 balls!
        );
    }

    // NumpadSubtract - DOUBLE PENDULUM (chaotic!)
    if keyboard.just_pressed(KeyCode::NumpadSubtract) {
        presets::spawn_double_pendulum(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 5.0, 0.0),
        );
    }

    // NumpadMultiply - METEOR SHOWER!!!
    if keyboard.just_pressed(KeyCode::NumpadMultiply) {
        presets::spawn_meteor_shower(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            30,
        );
    }

    // NumpadDivide - BALL FOUNTAIN!
    if keyboard.just_pressed(KeyCode::NumpadDivide) {
        presets::spawn_ball_fountain(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            50,
        );
    }

    // NumpadDecimal - FIREWORKS!!!
    if keyboard.just_pressed(KeyCode::NumpadDecimal) {
        presets::spawn_fireworks(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            5, // 5 bursts!
        );
    }

    // NumpadEnter - BOUNCY CASTLE!
    if keyboard.just_pressed(KeyCode::NumpadEnter) {
        presets::spawn_bouncy_castle(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
        );
    }

    // Insert - MARBLE RUN!
    if keyboard.just_pressed(KeyCode::Insert) {
        presets::spawn_marble_run(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 8.0, 0.0),
        );
    }

    // Delete - BALL CANNON (fires forward!)
    if keyboard.just_pressed(KeyCode::Delete) {
        presets::spawn_ball_cannon(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            Vec3::new(cam_forward.x, 0.0, cam_forward.z).normalize(),
            20,
        );
    }

    // Home - ULTIMATE DESTRUCTION!!!
    if keyboard.just_pressed(KeyCode::Home) {
        presets::spawn_ultimate_destruction(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
        );
    }

    // End - PENDULUM WAVE (mesmerizing!)
    if keyboard.just_pressed(KeyCode::End) {
        presets::spawn_pendulum_wave(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos + Vec3::new(0.0, 5.0, 0.0),
            20,
        );
    }

    // PageUp - BALL TORNADO!
    if keyboard.just_pressed(KeyCode::PageUp) {
        presets::spawn_ball_tornado(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            80,
        );
    }
}

/// Quick spawn keys for shapes
fn quick_spawn_keys(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut nova: ResMut<resources::NovaWorld>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut handle_to_entity: ResMut<resources::HandleToEntity>,
    camera_query: Query<&GlobalTransform, With<plugins::camera::PlayerCamera>>,
) {
    // F1-F4 are for debug, skip those

    // F5 - Quick spawn box
    if keyboard.just_pressed(KeyCode::F5) {
        let Ok(camera_transform) = camera_query.get_single() else {
            return;
        };
        let spawn_pos =
            camera_transform.translation() + camera_transform.forward().as_vec3() * 3.0;

        plugins::spawning::spawn_box(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            Vec3::splat(0.5),
            Color::srgb(0.8, 0.3, 0.3),
            components::PhysicsMaterialType::Normal,
        );
    }

    // F6 - Quick spawn sphere
    if keyboard.just_pressed(KeyCode::F6) {
        let Ok(camera_transform) = camera_query.get_single() else {
            return;
        };
        let spawn_pos =
            camera_transform.translation() + camera_transform.forward().as_vec3() * 3.0;

        plugins::spawning::spawn_sphere(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            0.5,
            Color::srgb(0.3, 0.8, 0.3),
            components::PhysicsMaterialType::Bouncy,
        );
    }

    // F7 - Quick spawn heavy ball
    if keyboard.just_pressed(KeyCode::F7) {
        let Ok(camera_transform) = camera_query.get_single() else {
            return;
        };
        let spawn_pos =
            camera_transform.translation() + camera_transform.forward().as_vec3() * 3.0;

        plugins::spawning::spawn_sphere(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            0.8,
            Color::srgb(0.3, 0.3, 0.3),
            components::PhysicsMaterialType::Heavy,
        );
    }

    // F8 - Quick spawn chain
    if keyboard.just_pressed(KeyCode::F8) {
        let Ok(camera_transform) = camera_query.get_single() else {
            return;
        };
        let spawn_pos =
            camera_transform.translation() + camera_transform.forward().as_vec3() * 3.0;

        plugins::spawning::spawn_chain(
            &mut nova,
            &mut commands,
            &mut meshes,
            &mut materials,
            &mut handle_to_entity,
            spawn_pos,
            10,
        );
    }
}

/// Reset the scene (R key)
fn scene_reset(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut commands: Commands,
    mut nova: ResMut<resources::NovaWorld>,
    mut handle_to_entity: ResMut<resources::HandleToEntity>,
    spawned_objects: Query<Entity, With<components::SpawnedObject>>,
    mut stats: ResMut<resources::PlaygroundStats>,
) {
    if keyboard.just_pressed(KeyCode::KeyR) {
        // Remove all spawned entities (includes dynamic bodies, zones, visuals, etc.)
        for entity in spawned_objects.iter() {
            commands.entity(entity).despawn_recursive();
        }

        // Clear all joints first
        let joint_handles: Vec<_> = nova.world.joints().map(|(h, _)| h).collect();
        for handle in joint_handles {
            nova.world.remove_joint(handle);
        }

        // Clear Nova world bodies (except arena static bodies)
        // Arena bodies are: floor, walls, ceiling - they have no entity mapping
        let bodies_to_remove: Vec<_> = nova
            .world
            .bodies()
            .filter_map(|(handle, body)| {
                // Only keep bodies that are static AND not in our handle_to_entity map
                // (arena bodies are not tracked in handle_to_entity)
                if handle_to_entity.bodies.contains_key(&handle) {
                    Some(handle)
                } else if body.body_type != nova::prelude::RigidBodyType::Static {
                    Some(handle)
                } else {
                    None
                }
            })
            .collect();

        for handle in bodies_to_remove {
            nova.world.remove_body(handle);
        }

        // Clear all mappings
        handle_to_entity.bodies.clear();
        handle_to_entity.colliders.clear();

        // Reset time scale
        nova.time_scale = 1.0;
        nova.paused = false;

        // Update stats
        stats.total_deleted = 0;
        stats.total_spawned = 0;
        stats.joints_created = 0;
    }
}

/// Spawn gravity zones and force fields with keyboard shortcuts
fn zone_spawning(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    camera_query: Query<&GlobalTransform, With<plugins::camera::PlayerCamera>>,
) {
    let Ok(camera_transform) = camera_query.get_single() else {
        return;
    };

    let spawn_pos = camera_transform.translation() + camera_transform.forward().as_vec3() * 8.0;

    // G - Spawn Gravity Zone (reverse gravity)
    if keyboard.just_pressed(KeyCode::KeyG) {
        plugins::effects::spawn_gravity_zone(
            &mut commands,
            &mut meshes,
            &mut materials,
            spawn_pos,
            8.0,                        // radius
            Vec3::new(0.0, 15.0, 0.0),  // upward gravity
            true,                       // falloff
        );
    }

    // V - Spawn Vortex Force Field
    if keyboard.just_pressed(KeyCode::KeyV) {
        plugins::effects::spawn_force_field(
            &mut commands,
            &mut meshes,
            &mut materials,
            spawn_pos,
            10.0, // radius
            plugins::effects::ForceFieldMode::Vortex,
            200.0, // strength
        );
    }

    // B - Spawn Radial Push Force Field
    if keyboard.just_pressed(KeyCode::KeyB) {
        plugins::effects::spawn_force_field(
            &mut commands,
            &mut meshes,
            &mut materials,
            spawn_pos,
            8.0, // radius
            plugins::effects::ForceFieldMode::Radial,
            300.0, // strength (positive = push away)
        );
    }

    // F9 - Spawn Turbulence Field
    if keyboard.just_pressed(KeyCode::F9) {
        plugins::effects::spawn_force_field(
            &mut commands,
            &mut meshes,
            &mut materials,
            spawn_pos,
            12.0, // radius
            plugins::effects::ForceFieldMode::Turbulence,
            150.0, // strength
        );
    }

    // F10 - Spawn Upward Wind
    if keyboard.just_pressed(KeyCode::F10) {
        plugins::effects::spawn_force_field(
            &mut commands,
            &mut meshes,
            &mut materials,
            spawn_pos,
            6.0, // radius
            plugins::effects::ForceFieldMode::Directional(Vec3::new(0.0, 1.0, 0.0)),
            500.0, // strength
        );
    }

    // F11 - BLACK HOLE! (dangerous - deletes objects that get too close!)
    if keyboard.just_pressed(KeyCode::F11) {
        plugins::effects::spawn_black_hole(
            &mut commands,
            &mut meshes,
            &mut materials,
            spawn_pos,
            15.0,    // radius of attraction
            5000.0,  // strength - VERY STRONG
        );
    }

    // F12 - Slow Motion Zone
    if keyboard.just_pressed(KeyCode::F12) {
        plugins::effects::spawn_slow_motion_zone(
            &mut commands,
            &mut meshes,
            &mut materials,
            spawn_pos,
            8.0,  // radius
            0.85, // factor (15% speed reduction per frame)
        );
    }

    // Semicolon - Bounce Pad
    if keyboard.just_pressed(KeyCode::Semicolon) {
        plugins::effects::spawn_bounce_pad(
            &mut commands,
            &mut meshes,
            &mut materials,
            spawn_pos - Vec3::Y * 2.0, // Spawn on ground level
            Vec3::new(4.0, 0.3, 4.0),  // size
            25.0, // launch strength
        );
    }
}

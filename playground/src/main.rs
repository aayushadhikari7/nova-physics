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
    PhysicsPlugin, SettingsPlugin, SpawningPlugin, TimeControlPlugin, ToolsPlugin,
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

    // N - Car
    if keyboard.just_pressed(KeyCode::KeyN) {
        presets::spawn_car(
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
}

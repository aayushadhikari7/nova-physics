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
    VisualEffectsPlugin,
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
            VisualEffectsPlugin,
        ))
        // Setup systems
        .add_systems(Startup, arena::setup_arena)
        .add_systems(Update, scene_reset)
        .run();
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


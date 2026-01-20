//! Nova-Bevy physics synchronization plugin

use bevy::prelude::*;

use crate::components::{PhysicsBody, SpawnedObject};
use crate::convert::{to_bevy_quat, to_bevy_vec3};
use crate::plugins::camera::{ROOM_HALF_LENGTH, ROOM_HALF_WIDTH, ROOM_HEIGHT};
use crate::resources::{HandleToEntity, NovaWorld};

pub struct PhysicsPlugin;

/// How far outside the room before objects get deleted
const OUT_OF_BOUNDS_MARGIN: f32 = 5.0;

/// Ground plane Y position (top of floor)
const GROUND_Y: f32 = 0.0;

/// Minimum object radius for ground enforcement
const MIN_OBJECT_RADIUS: f32 = 0.1;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<NovaWorld>()
            // Use fixed timestep for consistent physics - 240Hz for better collision
            .insert_resource(Time::<Fixed>::from_hz(240.0))
            .add_systems(FixedUpdate, (step_physics, enforce_ground_plane).chain())
            .add_systems(
                PostUpdate,
                (sync_transforms, delete_out_of_bounds),
            );
    }
}

/// Step the Nova physics simulation with aggressive substeps to prevent tunneling
fn step_physics(mut nova: ResMut<NovaWorld>, time: Res<Time<Fixed>>) {
    if nova.paused {
        return;
    }

    let dt = time.delta_secs() * nova.time_scale;
    if dt <= 0.0 {
        return;
    }

    // Use many small substeps to prevent tunneling through floors/walls
    // Max step size of 1/960s (~1ms) for very aggressive collision detection
    let max_substep = 1.0 / 960.0;
    let num_substeps = ((dt / max_substep).ceil() as u32).max(1).min(32);
    let substep_dt = dt / num_substeps as f32;

    for _ in 0..num_substeps {
        nova.world.step(substep_dt);
    }
}

/// Delete objects that go too far outside the room bounds
fn delete_out_of_bounds(
    mut commands: Commands,
    mut nova: ResMut<NovaWorld>,
    mut handle_to_entity: ResMut<HandleToEntity>,
    query: Query<(Entity, &PhysicsBody, &Transform), With<SpawnedObject>>,
) {
    let bounds_x = ROOM_HALF_WIDTH + OUT_OF_BOUNDS_MARGIN;
    let bounds_z = ROOM_HALF_LENGTH + OUT_OF_BOUNDS_MARGIN;
    let bounds_y_min = -OUT_OF_BOUNDS_MARGIN;
    let bounds_y_max = ROOM_HEIGHT + OUT_OF_BOUNDS_MARGIN;

    for (entity, body, transform) in query.iter() {
        let pos = transform.translation;

        // Check if out of bounds
        if pos.x.abs() > bounds_x
            || pos.z.abs() > bounds_z
            || pos.y < bounds_y_min
            || pos.y > bounds_y_max
        {
            // Remove from Nova physics world
            nova.world.remove_body(body.handle);
            handle_to_entity.bodies.remove(&body.handle);

            // Despawn Bevy entity
            commands.entity(entity).despawn_recursive();
        }
    }
}

/// Enforce ground plane - safety net to prevent tunneling through floor
/// Runs after physics step to catch any objects that slipped through
fn enforce_ground_plane(mut nova: ResMut<NovaWorld>) {
    // Get all dynamic bodies and check if they're below ground
    let body_handles: Vec<_> = nova.world.bodies().map(|(h, _)| h).collect();

    for handle in body_handles {
        if let Some(body) = nova.world.get_body_mut(handle) {
            // Skip static and kinematic bodies
            if !body.body_type.is_dynamic() {
                continue;
            }

            // Estimate object size from colliders (use a minimum radius)
            // For simplicity, use MIN_OBJECT_RADIUS as the minimum clearance
            let min_y = GROUND_Y + MIN_OBJECT_RADIUS;

            // If object center is below the minimum allowed Y
            if body.position.y < min_y {
                // Teleport back above ground
                body.position.y = min_y;

                // Kill downward velocity to prevent repeated penetration
                if body.linear_velocity.y < 0.0 {
                    // Apply restitution-like bounce (damped)
                    body.linear_velocity.y = -body.linear_velocity.y * 0.3;
                }
            }
        }
    }
}

/// Sync transforms from Nova to Bevy
fn sync_transforms(nova: Res<NovaWorld>, mut query: Query<(&PhysicsBody, &mut Transform)>) {
    for (body, mut transform) in query.iter_mut() {
        if let Some(nova_body) = nova.world.get_body(body.handle) {
            transform.translation = to_bevy_vec3(nova_body.position);
            transform.rotation = to_bevy_quat(nova_body.rotation);
        }
    }
}

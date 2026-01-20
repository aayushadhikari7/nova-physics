//! Nova-Bevy physics synchronization plugin

use bevy::prelude::*;

use crate::components::PhysicsBody;
use crate::convert::{to_bevy_quat, to_bevy_vec3};
use crate::resources::NovaWorld;

pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<NovaWorld>()
            // Use fixed timestep for consistent physics
            .insert_resource(Time::<Fixed>::from_hz(120.0)) // 120Hz physics updates
            .add_systems(FixedUpdate, step_physics)
            .add_systems(
                PostUpdate,
                sync_transforms,
            );
    }
}

/// Step the Nova physics simulation with substeps to prevent tunneling
fn step_physics(mut nova: ResMut<NovaWorld>, time: Res<Time<Fixed>>) {
    if nova.paused {
        return;
    }

    let dt = time.delta_secs() * nova.time_scale;
    if dt <= 0.0 {
        return;
    }

    // Use multiple small substeps to prevent tunneling through floors/walls
    // Max step size of 1/240s (4ms) for better collision detection
    let max_substep = 1.0 / 240.0;
    let num_substeps = ((dt / max_substep).ceil() as u32).max(1).min(8);
    let substep_dt = dt / num_substeps as f32;

    for _ in 0..num_substeps {
        nova.world.step(substep_dt);
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

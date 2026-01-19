//! Nova-Bevy physics synchronization plugin

use bevy::prelude::*;

use crate::components::PhysicsBody;
use crate::convert::{to_bevy_quat, to_bevy_vec3};
use crate::resources::NovaWorld;

pub struct PhysicsPlugin;

impl Plugin for PhysicsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<NovaWorld>()
            .add_systems(FixedUpdate, step_physics)
            .add_systems(
                PostUpdate,
                sync_transforms.after(step_physics),
            );
    }
}

/// Step the Nova physics simulation
fn step_physics(mut nova: ResMut<NovaWorld>, time: Res<Time>) {
    if !nova.paused {
        let dt = time.delta_secs() * nova.time_scale;
        // Cap dt to prevent tunneling - smaller = more stable
        let capped_dt = dt.min(1.0 / 60.0);
        if capped_dt > 0.0 {
            nova.world.step(capped_dt);
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

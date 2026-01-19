//! Debug visualization: contacts, joints, velocities, AABBs

use bevy::prelude::*;

use crate::components::PhysicsBody;
use crate::convert::to_bevy_vec3;
use crate::resources::{DebugVisuals, NovaWorld};

pub struct DebugPlugin;

impl Plugin for DebugPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DebugVisuals>()
            .add_systems(Update, (toggle_debug_visuals, draw_debug_gizmos));
    }
}

fn toggle_debug_visuals(keyboard: Res<ButtonInput<KeyCode>>, mut debug: ResMut<DebugVisuals>) {
    if keyboard.just_pressed(KeyCode::F1) {
        debug.show_contacts = !debug.show_contacts;
    }
    if keyboard.just_pressed(KeyCode::F2) {
        debug.show_joints = !debug.show_joints;
    }
    if keyboard.just_pressed(KeyCode::F3) {
        debug.show_velocities = !debug.show_velocities;
    }
    if keyboard.just_pressed(KeyCode::F4) {
        debug.show_aabbs = !debug.show_aabbs;
    }
}

fn draw_debug_gizmos(
    mut gizmos: Gizmos,
    nova: Res<NovaWorld>,
    debug: Res<DebugVisuals>,
    bodies: Query<&PhysicsBody>,
) {
    // Draw velocity arrows
    if debug.show_velocities {
        for (_, body) in nova.world.bodies() {
            let pos = to_bevy_vec3(body.position);
            let vel = to_bevy_vec3(body.linear_velocity);

            if vel.length() > 0.1 {
                let end = pos + vel * 0.2;
                gizmos.arrow(pos, end, Color::srgb(0.0, 1.0, 0.0));
            }

            // Angular velocity as a different colored arrow
            let ang_vel = to_bevy_vec3(body.angular_velocity);
            if ang_vel.length() > 0.1 {
                let end = pos + ang_vel * 0.3;
                gizmos.arrow(pos, end, Color::srgb(1.0, 0.5, 0.0));
            }
        }
    }

    // Draw joint connections
    if debug.show_joints {
        for (_, joint) in nova.world.joints() {
            let body_a = nova.world.get_body(joint.body_a);
            let body_b = nova.world.get_body(joint.body_b);

            if let (Some(a), Some(b)) = (body_a, body_b) {
                let pos_a = to_bevy_vec3(a.position);
                let pos_b = to_bevy_vec3(b.position);

                // Draw line between joint anchors
                let world_anchor_a = pos_a + to_bevy_vec3(joint.local_anchor_a);
                let world_anchor_b = pos_b + to_bevy_vec3(joint.local_anchor_b);

                gizmos.line(world_anchor_a, world_anchor_b, Color::srgb(0.3, 0.7, 1.0));

                // Draw anchor points
                gizmos.sphere(
                    Isometry3d::from_translation(world_anchor_a),
                    0.05,
                    Color::srgb(1.0, 1.0, 0.0),
                );
                gizmos.sphere(
                    Isometry3d::from_translation(world_anchor_b),
                    0.05,
                    Color::srgb(1.0, 1.0, 0.0),
                );
            }
        }
    }

    // Draw AABBs
    if debug.show_aabbs {
        for (_, collider) in nova.world.colliders() {
            if let Some(body) = nova.world.get_body(collider.parent) {
                let body_transform = nova::prelude::Isometry::new(body.position, body.rotation);
                let aabb = collider.compute_world_aabb(&body_transform);

                let min = to_bevy_vec3(aabb.min);
                let max = to_bevy_vec3(aabb.max);
                let center = (min + max) * 0.5;
                let size = max - min;

                gizmos.cuboid(
                    Transform::from_translation(center).with_scale(size),
                    Color::srgba(0.5, 0.5, 1.0, 0.5),
                );
            }
        }
    }

    // Draw contact points from collision events
    if debug.show_contacts {
        // Note: drain_collision_events() consumes the events
        for event in nova.world.drain_collision_events() {
            for contact in &event.contacts {
                let point = to_bevy_vec3(contact.point_a);
                let normal = to_bevy_vec3(contact.normal);

                gizmos.sphere(
                    Isometry3d::from_translation(point),
                    0.03,
                    Color::srgb(1.0, 0.0, 0.0),
                );
                gizmos.arrow(point, point + normal * 0.2, Color::srgb(1.0, 0.0, 0.0));
            }
        }
    }
}

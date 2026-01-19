//! Debug visualization: contacts, joints, velocities, AABBs

use bevy::prelude::*;

use crate::components::{Highlighted, PhysicsBody, Selected};
use crate::convert::to_bevy_vec3;
use crate::resources::{DebugVisuals, NovaWorld};

pub struct DebugPlugin;

impl Plugin for DebugPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DebugVisuals>()
            .add_systems(Update, (toggle_debug_visuals, draw_debug_gizmos, draw_selection_gizmos));
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

/// Draw visual feedback for highlighted and selected objects
fn draw_selection_gizmos(
    mut gizmos: Gizmos,
    highlighted_query: Query<&Transform, (With<Highlighted>, Without<Selected>)>,
    selected_query: Query<&Transform, With<Selected>>,
    both_query: Query<&Transform, (With<Highlighted>, With<Selected>)>,
    time: Res<Time>,
) {
    let t = time.elapsed_secs();

    // Draw highlight effect (white outline) for hovered objects
    for transform in highlighted_query.iter() {
        let pos = transform.translation;
        let scale = transform.scale.max_element() * 0.6;

        // Pulsing white circle
        let pulse = (t * 4.0).sin() * 0.1 + 0.9;
        gizmos.circle(
            Isometry3d::new(pos + Vec3::Y * 0.01, Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
            scale * pulse,
            Color::srgba(1.0, 1.0, 1.0, 0.6),
        );
    }

    // Draw selection effect (cyan outline) for selected objects
    for transform in selected_query.iter() {
        let pos = transform.translation;
        let scale = transform.scale.max_element() * 0.65;

        // Rotating cyan ring
        let rotation = Quat::from_rotation_y(t * 2.0) * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
        gizmos.circle(
            Isometry3d::new(pos, rotation),
            scale,
            Color::srgba(0.0, 1.0, 1.0, 0.8),
        );

        // Small dots at corners
        let corner_dist = scale * 0.7;
        for i in 0..4 {
            let angle = (i as f32 * std::f32::consts::FRAC_PI_2) + t * 1.5;
            let offset = Vec3::new(angle.cos() * corner_dist, 0.0, angle.sin() * corner_dist);
            gizmos.sphere(
                Isometry3d::from_translation(pos + offset),
                0.05,
                Color::srgb(0.0, 1.0, 1.0),
            );
        }
    }

    // Objects that are both highlighted and selected get a golden effect
    for transform in both_query.iter() {
        let pos = transform.translation;
        let scale = transform.scale.max_element() * 0.7;

        let pulse = (t * 5.0).sin() * 0.15 + 1.0;
        gizmos.sphere(
            Isometry3d::from_translation(pos),
            scale * pulse,
            Color::srgba(1.0, 0.8, 0.0, 0.3),
        );
    }
}

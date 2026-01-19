//! Visual effects and special physics zones - MEGA EDITION!

use bevy::prelude::*;
use nova::prelude::RigidBodyType;

use crate::components::{DynamicBody, PhysicsBody, SpawnedObject};
use crate::convert::{to_bevy_vec3, to_nova_vec3};
use crate::resources::NovaWorld;

pub struct EffectsPlugin;

impl Plugin for EffectsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ActiveGravityZones>()
            .init_resource::<ActiveForceFields>()
            .add_systems(
                Update,
                (
                    apply_gravity_zones,
                    apply_force_fields,
                    update_zone_visuals,
                    spawn_explosion_effects,
                    update_trails,
                ),
            );
    }
}

// ============ GRAVITY ZONES ============

/// Component for gravity zone entities
#[derive(Component)]
pub struct GravityZoneEntity {
    pub gravity: Vec3,
    pub radius: f32,
    pub falloff: bool,
    pub priority: i32,
}

/// Resource tracking all active gravity zones
#[derive(Resource, Default)]
pub struct ActiveGravityZones {
    pub zones: Vec<Entity>,
}

/// Apply custom gravity from gravity zones to bodies inside them
fn apply_gravity_zones(
    mut nova: ResMut<NovaWorld>,
    zones: Query<(&Transform, &GravityZoneEntity)>,
    bodies: Query<&PhysicsBody, With<DynamicBody>>,
) {
    for body in bodies.iter() {
        if let Some(nova_body) = nova.world.get_body(body.handle) {
            let body_pos = to_bevy_vec3(nova_body.position);

            // Check each gravity zone
            for (zone_transform, zone) in zones.iter() {
                let zone_pos = zone_transform.translation;
                let distance = body_pos.distance(zone_pos);

                if distance < zone.radius {
                    // Calculate gravity force
                    let mut gravity_force = zone.gravity;

                    if zone.falloff {
                        // Linear falloff based on distance
                        let factor = 1.0 - (distance / zone.radius);
                        gravity_force *= factor;
                    }

                    // Apply as force (not replacing world gravity, adding to it)
                    if let Some(body_mut) = nova.world.get_body_mut(body.handle) {
                        let mass = body_mut.mass;
                        let force = to_nova_vec3(gravity_force * mass);
                        body_mut.apply_force(force);
                    }
                }
            }
        }
    }
}

// ============ FORCE FIELDS ============

/// Component for force field entities
#[derive(Component)]
pub struct ForceFieldEntity {
    pub force_type: ForceFieldMode,
    pub strength: f32,
    pub radius: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ForceFieldMode {
    Directional(Vec3), // Push in a specific direction
    Radial,            // Push away from center (positive) or toward (negative)
    Vortex,            // Spin around Y axis
    Turbulence,        // Random forces
}

/// Resource tracking all active force fields
#[derive(Resource, Default)]
pub struct ActiveForceFields {
    pub fields: Vec<Entity>,
}

/// Apply forces from force fields
fn apply_force_fields(
    mut nova: ResMut<NovaWorld>,
    time: Res<Time>,
    fields: Query<(&Transform, &ForceFieldEntity)>,
    bodies: Query<&PhysicsBody, With<DynamicBody>>,
) {
    let t = time.elapsed_secs();

    for body in bodies.iter() {
        if let Some(nova_body) = nova.world.get_body(body.handle) {
            let body_pos = to_bevy_vec3(nova_body.position);

            for (field_transform, field) in fields.iter() {
                let field_pos = field_transform.translation;
                let distance = body_pos.distance(field_pos);

                if distance < field.radius && distance > 0.1 {
                    let to_body = (body_pos - field_pos).normalize_or_zero();
                    let falloff = 1.0 - (distance / field.radius);

                    let force = match field.force_type {
                        ForceFieldMode::Directional(dir) => dir * field.strength * falloff,
                        ForceFieldMode::Radial => to_body * field.strength * falloff,
                        ForceFieldMode::Vortex => {
                            // Perpendicular force for spinning
                            let tangent = Vec3::new(-to_body.z, 0.0, to_body.x);
                            tangent * field.strength * falloff
                        }
                        ForceFieldMode::Turbulence => {
                            // Pseudo-random based on position and time
                            let noise_x = (body_pos.x * 0.5 + t * 2.0).sin();
                            let noise_y = (body_pos.y * 0.7 + t * 1.5).cos();
                            let noise_z = (body_pos.z * 0.3 + t * 2.5).sin();
                            Vec3::new(noise_x, noise_y, noise_z) * field.strength * falloff
                        }
                    };

                    if let Some(body_mut) = nova.world.get_body_mut(body.handle) {
                        body_mut.apply_force(to_nova_vec3(force));
                    }
                }
            }
        }
    }
}

// ============ ZONE VISUALS ============

#[derive(Component)]
pub struct ZoneVisual;

fn update_zone_visuals(
    mut gizmos: Gizmos,
    gravity_zones: Query<(&Transform, &GravityZoneEntity)>,
    force_fields: Query<(&Transform, &ForceFieldEntity)>,
) {
    // Draw gravity zones
    for (transform, zone) in gravity_zones.iter() {
        let pos = transform.translation;

        // Draw sphere outline
        gizmos.sphere(
            Isometry3d::from_translation(pos),
            zone.radius,
            Color::srgba(0.2, 0.8, 0.2, 0.3),
        );

        // Draw gravity direction arrow
        let arrow_end = pos + zone.gravity.normalize_or_zero() * 3.0;
        gizmos.arrow(pos, arrow_end, Color::srgb(0.3, 1.0, 0.3));
    }

    // Draw force fields
    for (transform, field) in force_fields.iter() {
        let pos = transform.translation;

        let color = match field.force_type {
            ForceFieldMode::Directional(_) => Color::srgba(0.8, 0.8, 0.2, 0.3),
            ForceFieldMode::Radial => Color::srgba(0.8, 0.2, 0.2, 0.3),
            ForceFieldMode::Vortex => Color::srgba(0.2, 0.2, 0.8, 0.3),
            ForceFieldMode::Turbulence => Color::srgba(0.8, 0.2, 0.8, 0.3),
        };

        gizmos.sphere(Isometry3d::from_translation(pos), field.radius, color);

        // Draw direction indicator
        match field.force_type {
            ForceFieldMode::Directional(dir) => {
                gizmos.arrow(pos, pos + dir.normalize() * 3.0, Color::srgb(1.0, 1.0, 0.3));
            }
            ForceFieldMode::Vortex => {
                // Draw rotation indicator
                for i in 0..8 {
                    let angle = i as f32 * std::f32::consts::TAU / 8.0;
                    let start = pos + Vec3::new(angle.cos(), 0.0, angle.sin()) * field.radius * 0.5;
                    let end_angle = angle + 0.5;
                    let end =
                        pos + Vec3::new(end_angle.cos(), 0.0, end_angle.sin()) * field.radius * 0.5;
                    gizmos.line(start, end, Color::srgb(0.3, 0.3, 1.0));
                }
            }
            _ => {}
        }
    }
}

// ============ EXPLOSION EFFECTS ============

/// Component for explosion visual effect
#[derive(Component)]
pub struct ExplosionEffect {
    pub timer: f32,
    pub max_radius: f32,
    pub start_radius: f32,
}

/// Event to trigger explosion visual
#[derive(Event)]
pub struct ExplosionEvent {
    pub position: Vec3,
    pub radius: f32,
    pub force: f32,
}

fn spawn_explosion_effects(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut explosions: Query<(Entity, &mut ExplosionEffect, &mut Transform)>,
    time: Res<Time>,
) {
    let dt = time.delta_secs();

    for (entity, mut effect, mut transform) in explosions.iter_mut() {
        effect.timer -= dt;

        if effect.timer <= 0.0 {
            commands.entity(entity).despawn();
        } else {
            // Expand the explosion sphere
            let progress = 1.0 - (effect.timer / 0.5);
            let current_radius =
                effect.start_radius + (effect.max_radius - effect.start_radius) * progress;
            transform.scale = Vec3::splat(current_radius);
        }
    }
}

/// Spawn an explosion visual at a position
pub fn spawn_explosion_visual(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
    position: Vec3,
    radius: f32,
) {
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(1.0, 0.6, 0.1, 0.5),
            emissive: LinearRgba::new(10.0, 5.0, 0.5, 1.0),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_translation(position).with_scale(Vec3::splat(0.5)),
        ExplosionEffect {
            timer: 0.5,
            max_radius: radius,
            start_radius: 0.5,
        },
    ));
}

// ============ TRAILS ============

#[derive(Component)]
pub struct TrailEmitter {
    pub points: Vec<Vec3>,
    pub max_points: usize,
    pub min_velocity: f32,
    pub last_pos: Vec3,
}

fn update_trails(
    mut gizmos: Gizmos,
    mut emitters: Query<(&mut TrailEmitter, &Transform, &PhysicsBody)>,
    nova: Res<NovaWorld>,
) {
    for (mut trail, transform, body) in emitters.iter_mut() {
        let pos = transform.translation;

        // Check velocity
        if let Some(nova_body) = nova.world.get_body(body.handle) {
            let vel = to_bevy_vec3(nova_body.linear_velocity);
            let speed = vel.length();

            if speed > trail.min_velocity {
                // Add point if moved enough
                if trail.last_pos.distance(pos) > 0.2 {
                    trail.points.push(pos);
                    trail.last_pos = pos;

                    // Limit points
                    while trail.points.len() > trail.max_points {
                        trail.points.remove(0);
                    }
                }
            } else {
                // Clear trail when slow
                trail.points.clear();
            }
        }

        // Draw trail
        if trail.points.len() > 1 {
            for i in 0..trail.points.len() - 1 {
                let alpha = i as f32 / trail.points.len() as f32;
                let color = Color::srgba(1.0, 0.5, 0.1, alpha * 0.5);
                gizmos.line(trail.points[i], trail.points[i + 1], color);
            }
        }
    }
}

// ============ HELPER FUNCTIONS ============

/// Spawn a gravity zone at a position
pub fn spawn_gravity_zone(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    position: Vec3,
    radius: f32,
    gravity: Vec3,
    falloff: bool,
) -> Entity {
    commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(radius))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(0.2, 0.8, 0.2, 0.15),
                alpha_mode: AlphaMode::Blend,
                cull_mode: None,
                ..default()
            })),
            Transform::from_translation(position),
            GravityZoneEntity {
                gravity,
                radius,
                falloff,
                priority: 0,
            },
            ZoneVisual,
            SpawnedObject,
        ))
        .id()
}

/// Spawn a force field at a position
pub fn spawn_force_field(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    position: Vec3,
    radius: f32,
    force_type: ForceFieldMode,
    strength: f32,
) -> Entity {
    let color = match force_type {
        ForceFieldMode::Directional(_) => Color::srgba(0.8, 0.8, 0.2, 0.15),
        ForceFieldMode::Radial => Color::srgba(0.8, 0.2, 0.2, 0.15),
        ForceFieldMode::Vortex => Color::srgba(0.2, 0.2, 0.8, 0.15),
        ForceFieldMode::Turbulence => Color::srgba(0.8, 0.2, 0.8, 0.15),
    };

    commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(radius))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                alpha_mode: AlphaMode::Blend,
                cull_mode: None,
                ..default()
            })),
            Transform::from_translation(position),
            ForceFieldEntity {
                force_type,
                strength,
                radius,
            },
            ZoneVisual,
            SpawnedObject,
        ))
        .id()
}

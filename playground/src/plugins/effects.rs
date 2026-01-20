//! Visual effects and special physics zones - MEGA EDITION!

use bevy::prelude::*;
use nova::prelude::RigidBodyType;
use rand::Rng;

use crate::components::{
    AutoDespawn, Breakable, DynamicBody, Explosive, Glowing, MagnetObject, PhysicsBody,
    Spinner, SpawnedObject,
};
use crate::convert::{to_bevy_vec3, to_nova_vec3};
use crate::plugins::settings::GameSettings;
use crate::plugins::visual_effects::ScreenShakeEvent;
use crate::resources::{HandleToEntity, NovaWorld, PlaygroundStats};

pub struct EffectsPlugin;

impl Plugin for EffectsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ActiveGravityZones>()
            .init_resource::<ActiveForceFields>()
            .add_systems(
                Update,
                (
                    // Physics zones
                    apply_gravity_zones,
                    apply_force_fields,
                    apply_conveyor_belts,
                    apply_portal_teleportation,
                    apply_black_holes,
                    apply_slow_motion_zones,
                    apply_bounce_pads,
                    // Object behaviors
                    apply_spinner_rotation,
                    apply_magnet_objects,
                    check_breakable_objects,
                    check_explosive_objects,
                    update_auto_despawn,
                    update_glowing_objects,
                    // Visuals
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

// ============ CONVEYOR BELTS ============

use crate::plugins::spawning::ConveyorBelt;

/// Apply conveyor belt forces to objects on top of them
fn apply_conveyor_belts(
    mut nova: ResMut<NovaWorld>,
    conveyors: Query<(&Transform, &ConveyorBelt)>,
    bodies: Query<&PhysicsBody, With<DynamicBody>>,
) {
    for (conveyor_transform, conveyor) in conveyors.iter() {
        let conveyor_pos = conveyor_transform.translation;
        let conveyor_size = Vec3::new(2.0, 0.3, 0.8); // Approximate size

        for body in bodies.iter() {
            if let Some(nova_body) = nova.world.get_body(body.handle) {
                let body_pos = to_bevy_vec3(nova_body.position);

                // Check if body is approximately on top of conveyor
                let dx = (body_pos.x - conveyor_pos.x).abs();
                let dz = (body_pos.z - conveyor_pos.z).abs();
                let dy = body_pos.y - conveyor_pos.y;

                // Body is on conveyor if within bounds and slightly above
                if dx < conveyor_size.x && dz < conveyor_size.z && dy > 0.0 && dy < 1.5 {
                    if let Some(body_mut) = nova.world.get_body_mut(body.handle) {
                        // Apply conveyor force in the belt direction
                        let force = to_nova_vec3(conveyor.direction * conveyor.speed * 10.0);
                        body_mut.apply_force(force);
                    }
                }
            }
        }
    }
}

// ============ PORTAL TELEPORTATION ============

use crate::plugins::spawning::PortalEntity;

/// Teleport bodies that enter portals
fn apply_portal_teleportation(
    mut nova: ResMut<NovaWorld>,
    mut portals: Query<(&Transform, &mut PortalEntity)>,
    bodies: Query<&PhysicsBody, With<DynamicBody>>,
    time: Res<Time>,
) {
    let dt = time.delta_secs();

    for (portal_transform, mut portal) in portals.iter_mut() {
        // Update cooldown
        portal.cooldown = (portal.cooldown - dt).max(0.0);

        if portal.cooldown > 0.0 {
            continue;
        }

        let portal_pos = portal_transform.translation;

        for body in bodies.iter() {
            if let Some(nova_body) = nova.world.get_body(body.handle) {
                let body_pos = to_bevy_vec3(nova_body.position);
                let distance = body_pos.distance(portal_pos);

                // Check if body is within portal radius
                if distance < portal.radius {
                    // Teleport the body
                    if let Some(body_mut) = nova.world.get_body_mut(body.handle) {
                        // Calculate offset from portal center
                        let offset = body_pos - portal_pos;

                        // Move to destination
                        body_mut.position = to_nova_vec3(portal.destination + offset);

                        // Optionally clear velocity
                        if !portal.preserve_velocity {
                            body_mut.linear_velocity = nova::prelude::Vec3::ZERO;
                            body_mut.angular_velocity = nova::prelude::Vec3::ZERO;
                        }

                        body_mut.wake_up();

                        // Set cooldown to prevent immediate re-teleportation
                        portal.cooldown = 0.5;
                        break; // Only teleport one body per frame per portal
                    }
                }
            }
        }
    }
}

// ============ SPINNER ROTATION ============

/// Rotate objects with Spinner component
fn apply_spinner_rotation(
    mut nova: ResMut<NovaWorld>,
    spinners: Query<(&PhysicsBody, &Spinner)>,
    time: Res<Time>,
) {
    let dt = time.delta_secs();

    for (physics_body, spinner) in spinners.iter() {
        if let Some(body) = nova.world.get_body_mut(physics_body.handle) {
            // Apply angular velocity based on spinner settings
            let angular_vel = to_nova_vec3(spinner.axis.normalize() * spinner.speed);
            body.angular_velocity = angular_vel;
            body.wake_up();
        }
    }
}

// ============ MAGNET OBJECTS ============

/// Objects with MagnetObject component attract/repel nearby dynamic bodies
fn apply_magnet_objects(
    mut nova: ResMut<NovaWorld>,
    magnets: Query<(&Transform, &MagnetObject)>,
    bodies: Query<&PhysicsBody, With<DynamicBody>>,
) {
    for (magnet_transform, magnet) in magnets.iter() {
        let magnet_pos = magnet_transform.translation;

        for body in bodies.iter() {
            if let Some(nova_body) = nova.world.get_body(body.handle) {
                let body_pos = to_bevy_vec3(nova_body.position);
                let distance = body_pos.distance(magnet_pos);

                if distance < magnet.radius && distance > 0.1 {
                    let direction = if magnet.attract {
                        (magnet_pos - body_pos).normalize()
                    } else {
                        (body_pos - magnet_pos).normalize()
                    };

                    let falloff = 1.0 - (distance / magnet.radius);
                    let force = direction * magnet.strength * falloff;

                    if let Some(body_mut) = nova.world.get_body_mut(body.handle) {
                        body_mut.apply_force(to_nova_vec3(force));
                    }
                }
            }
        }
    }
}

// ============ BREAKABLE OBJECTS ============

/// Check for high-impulse collisions and break objects
fn check_breakable_objects(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut nova: ResMut<NovaWorld>,
    mut handle_to_entity: ResMut<HandleToEntity>,
    breakables: Query<(Entity, &PhysicsBody, &Breakable, &Transform)>,
    mut stats: ResMut<PlaygroundStats>,
) {
    let mut to_break = Vec::new();

    // Check velocities for high-impact collisions
    for (entity, physics_body, breakable, transform) in breakables.iter() {
        if let Some(body) = nova.world.get_body(physics_body.handle) {
            let speed = to_bevy_vec3(body.linear_velocity).length();
            // Break if moving fast (simulating impact)
            if speed > breakable.impulse_threshold {
                to_break.push((entity, physics_body.handle, transform.translation, breakable.pieces));
            }
        }
    }

    // Break objects and spawn pieces
    for (entity, body_handle, position, pieces) in to_break {
        // Remove original
        commands.entity(entity).despawn_recursive();
        nova.world.remove_body(body_handle);
        handle_to_entity.bodies.remove(&body_handle);

        // Spawn smaller pieces
        let mut rng = rand::thread_rng();
        for _ in 0..pieces {
            let offset = Vec3::new(
                rng.gen_range(-0.5..0.5),
                rng.gen_range(-0.5..0.5),
                rng.gen_range(-0.5..0.5),
            );
            let piece_pos = position + offset;
            let piece_size = 0.2;

            // Create small debris sphere
            let body = nova
                .world
                .create_body()
                .body_type(RigidBodyType::Dynamic)
                .position(to_nova_vec3(piece_pos))
                .linear_velocity(to_nova_vec3(offset.normalize() * 5.0))
                .build();
            let body_handle = nova.world.insert_body(body);

            let collider = nova
                .world
                .create_collider(
                    body_handle,
                    nova::prelude::CollisionShape::Sphere(nova::prelude::SphereShape::new(piece_size)),
                )
                .friction(0.5)
                .restitution(0.3)
                .build();
            let collider_handle = nova.world.insert_collider(collider);

            let piece_entity = commands
                .spawn((
                    Mesh3d(meshes.add(Sphere::new(piece_size))),
                    MeshMaterial3d(materials.add(StandardMaterial {
                        base_color: Color::srgb(0.5, 0.4, 0.3),
                        ..default()
                    })),
                    Transform::from_translation(piece_pos),
                    PhysicsBody { handle: body_handle },
                    crate::components::PhysicsCollider { handle: collider_handle },
                    DynamicBody,
                    SpawnedObject,
                    AutoDespawn { timer: 5.0 }, // Debris disappears after 5 seconds
                ))
                .id();

            handle_to_entity.bodies.insert(body_handle, piece_entity);
            stats.total_spawned += 1;
        }

        stats.total_deleted += 1;
    }
}

// ============ EXPLOSIVE OBJECTS ============

/// Check for collisions and trigger explosions
fn check_explosive_objects(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut nova: ResMut<NovaWorld>,
    mut handle_to_entity: ResMut<HandleToEntity>,
    mut explosives: Query<(Entity, &PhysicsBody, &mut Explosive, &Transform)>,
    bodies: Query<&PhysicsBody, With<DynamicBody>>,
    mut stats: ResMut<PlaygroundStats>,
    mut screen_shake: EventWriter<ScreenShakeEvent>,
) {
    let mut to_explode = Vec::new();

    // Check for contact (simplified: check if moving and near another body)
    for (entity, physics_body, mut explosive, transform) in explosives.iter_mut() {
        if explosive.triggered {
            continue;
        }

        if let Some(body) = nova.world.get_body(physics_body.handle) {
            let speed = to_bevy_vec3(body.linear_velocity).length();
            let pos = to_bevy_vec3(body.position);

            // Trigger if moving fast enough (impact detection)
            if speed > 3.0 {
                // Check proximity to other bodies
                for other_body in bodies.iter() {
                    if other_body.handle == physics_body.handle {
                        continue;
                    }
                    if let Some(other) = nova.world.get_body(other_body.handle) {
                        let other_pos = to_bevy_vec3(other.position);
                        if pos.distance(other_pos) < 1.5 {
                            to_explode.push((entity, physics_body.handle, transform.translation, explosive.radius, explosive.force));
                            break;
                        }
                    }
                }
            }
        }
    }

    // Trigger explosions
    for (entity, body_handle, position, radius, force) in to_explode {
        // Remove explosive
        commands.entity(entity).despawn_recursive();
        nova.world.remove_body(body_handle);
        handle_to_entity.bodies.remove(&body_handle);

        // Apply explosion force to nearby bodies
        let nova_center = to_nova_vec3(position);
        let affected: Vec<_> = nova
            .world
            .bodies()
            .filter_map(|(h, b)| {
                if b.body_type == RigidBodyType::Dynamic {
                    let dist = (b.position - nova_center).length();
                    if dist < radius {
                        Some((h, dist))
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .collect();

        for (handle, dist) in affected {
            if let Some(body) = nova.world.get_body_mut(handle) {
                let diff = body.position - nova_center;
                if diff.length() > 0.01 {
                    let falloff = 1.0 - (dist / radius);
                    let impulse = diff.normalize() * force * falloff;
                    body.apply_impulse(impulse);
                    body.wake_up();
                }
            }
        }

        // Spawn explosion visual
        spawn_explosion_visual(&mut commands, &mut meshes, &mut materials, position, radius);

        // Trigger screen shake (intensity based on explosion force)
        let shake_intensity = (force / 1000.0).clamp(0.3, 1.0);
        screen_shake.send(ScreenShakeEvent { intensity: shake_intensity });

        stats.total_explosions += 1;
        stats.total_deleted += 1;
    }
}

// ============ AUTO DESPAWN ============

/// Remove objects after their timer expires
fn update_auto_despawn(
    mut commands: Commands,
    mut nova: ResMut<NovaWorld>,
    mut handle_to_entity: ResMut<HandleToEntity>,
    mut query: Query<(Entity, &mut AutoDespawn, Option<&PhysicsBody>)>,
    time: Res<Time>,
    mut stats: ResMut<PlaygroundStats>,
) {
    let dt = time.delta_secs();

    for (entity, mut despawn, physics_body) in query.iter_mut() {
        despawn.timer -= dt;

        if despawn.timer <= 0.0 {
            // Remove physics body if present
            if let Some(pb) = physics_body {
                nova.world.remove_body(pb.handle);
                handle_to_entity.bodies.remove(&pb.handle);
            }
            commands.entity(entity).despawn_recursive();
            stats.total_deleted += 1;
        }
    }
}

// ============ GLOWING OBJECTS ============

/// Update emissive materials for glowing objects
fn update_glowing_objects(
    glowing_query: Query<(&Glowing, &MeshMaterial3d<StandardMaterial>)>,
    mut material_assets: ResMut<Assets<StandardMaterial>>,
    time: Res<Time>,
) {
    let t = time.elapsed_secs();

    for (glowing, material_handle) in glowing_query.iter() {
        if let Some(material) = material_assets.get_mut(&material_handle.0) {
            // Pulsing glow effect
            let pulse = (t * 3.0).sin() * 0.5 + 0.5;
            let intensity = glowing.intensity * (0.5 + pulse * 0.5);

            let color = glowing.color.to_linear();
            material.emissive = LinearRgba::new(
                color.red * intensity,
                color.green * intensity,
                color.blue * intensity,
                1.0,
            );
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
    settings: Res<GameSettings>,
) {
    // Apply zone opacity from settings
    let opacity = settings.zone_opacity * 0.2; // Base opacity is 0.2, scaled by setting
    let arrow_opacity = settings.zone_opacity * 0.5;

    // Draw gravity zones - soft mint color
    for (transform, zone) in gravity_zones.iter() {
        let pos = transform.translation;

        // Draw sphere outline - soft mint with adjustable opacity
        gizmos.sphere(
            Isometry3d::from_translation(pos),
            zone.radius,
            Color::srgba(0.6, 0.9, 0.7, opacity),
        );

        // Draw gravity direction arrow - subtle
        let arrow_end = pos + zone.gravity.normalize_or_zero() * 3.0;
        gizmos.arrow(pos, arrow_end, Color::srgba(0.6, 0.9, 0.7, arrow_opacity));
    }

    // Draw force fields - soft pastel colors with adjustable opacity
    for (transform, field) in force_fields.iter() {
        let pos = transform.translation;

        let color = match field.force_type {
            ForceFieldMode::Directional(_) => Color::srgba(0.95, 0.90, 0.70, opacity), // Cream
            ForceFieldMode::Radial => Color::srgba(0.95, 0.70, 0.65, opacity),         // Coral
            ForceFieldMode::Vortex => Color::srgba(0.60, 0.75, 0.95, opacity),         // Sky blue
            ForceFieldMode::Turbulence => Color::srgba(0.80, 0.70, 0.90, opacity),     // Lavender
        };

        gizmos.sphere(Isometry3d::from_translation(pos), field.radius, color);

        // Draw direction indicator - subtle
        match field.force_type {
            ForceFieldMode::Directional(dir) => {
                gizmos.arrow(pos, pos + dir.normalize() * 3.0, Color::srgba(0.95, 0.90, 0.70, arrow_opacity));
            }
            ForceFieldMode::Vortex => {
                // Draw rotation indicator - subtle blue
                for i in 0..8 {
                    let angle = i as f32 * std::f32::consts::TAU / 8.0;
                    let start = pos + Vec3::new(angle.cos(), 0.0, angle.sin()) * field.radius * 0.5;
                    let end_angle = angle + 0.5;
                    let end =
                        pos + Vec3::new(end_angle.cos(), 0.0, end_angle.sin()) * field.radius * 0.5;
                    gizmos.line(start, end, Color::srgba(0.60, 0.75, 0.95, arrow_opacity * 0.8));
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
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    position: Vec3,
    radius: f32,
) {
    // Soft, warm white explosion for clean aesthetic
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(1.0, 0.95, 0.9, 0.4), // Soft warm white
            emissive: LinearRgba::new(3.0, 2.8, 2.5, 1.0), // Soft glow
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_translation(position).with_scale(Vec3::splat(0.5)),
        ExplosionEffect {
            timer: 0.5,
            max_radius: radius,
            start_radius: 0.5,
        },
        SpawnedObject,
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
    settings: Res<GameSettings>,
) {
    // Skip trail updates if disabled in settings
    if !settings.trail_enabled {
        return;
    }

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

        // Draw trail - soft white/gray for clean look
        if trail.points.len() > 1 {
            for i in 0..trail.points.len() - 1 {
                let alpha = i as f32 / trail.points.len() as f32;
                let color = Color::srgba(0.7, 0.75, 0.8, alpha * 0.3); // Soft gray, subtle
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
                base_color: Color::srgba(0.6, 0.9, 0.7, 0.1), // Soft mint, low opacity
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
    // Soft pastel colors for clean aesthetic
    let color = match force_type {
        ForceFieldMode::Directional(_) => Color::srgba(0.95, 0.90, 0.70, 0.1), // Cream
        ForceFieldMode::Radial => Color::srgba(0.95, 0.70, 0.65, 0.1),         // Coral
        ForceFieldMode::Vortex => Color::srgba(0.60, 0.75, 0.95, 0.1),         // Sky blue
        ForceFieldMode::Turbulence => Color::srgba(0.80, 0.70, 0.90, 0.1),     // Lavender
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

// ============ BLACK HOLE ============

/// Black hole component - super strong attractor that pulls everything in
#[derive(Component)]
pub struct BlackHole {
    pub strength: f32,
    pub radius: f32,
    pub event_horizon: f32,
}

/// Apply black hole attraction
fn apply_black_holes(
    mut nova: ResMut<NovaWorld>,
    black_holes: Query<(&Transform, &BlackHole)>,
    mut commands: Commands,
    mut handle_to_entity: ResMut<HandleToEntity>,
) {
    for (hole_transform, hole) in black_holes.iter() {
        let hole_pos = hole_transform.translation;

        let mut to_delete = Vec::new();

        for (handle, body) in nova.world.bodies_mut() {
            if body.body_type.is_static() {
                continue;
            }

            let body_pos = to_bevy_vec3(body.position);
            let diff = hole_pos - body_pos;
            let dist = diff.length();

            if dist < hole.event_horizon {
                to_delete.push(handle);
            } else if dist < hole.radius {
                let strength = hole.strength / (dist * dist).max(1.0);
                let force = to_nova_vec3(diff.normalize() * strength);
                body.apply_force(force);
            }
        }

        for handle in to_delete {
            if let Some(&entity) = handle_to_entity.bodies.get(&handle) {
                commands.entity(entity).despawn_recursive();
            }
            nova.world.remove_body(handle);
            handle_to_entity.bodies.remove(&handle);
        }
    }
}

/// Spawn a black hole
pub fn spawn_black_hole(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    position: Vec3,
    radius: f32,
    strength: f32,
) -> Entity {
    commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(radius * 0.15))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.0, 0.0, 0.0),
                emissive: LinearRgba::new(0.5, 0.0, 1.0, 1.0),
                ..default()
            })),
            Transform::from_translation(position),
            BlackHole {
                strength,
                radius,
                event_horizon: radius * 0.2,
            },
            SpawnedObject,
        ))
        .id()
}

// ============ SLOW MOTION ZONE ============

#[derive(Component)]
pub struct SlowMotionZone {
    pub radius: f32,
    pub factor: f32,
}

fn apply_slow_motion_zones(
    mut nova: ResMut<NovaWorld>,
    zones: Query<(&Transform, &SlowMotionZone)>,
) {
    for (zone_transform, zone) in zones.iter() {
        let zone_pos = zone_transform.translation;

        for (_, body) in nova.world.bodies_mut() {
            if body.body_type.is_static() {
                continue;
            }

            let body_pos = to_bevy_vec3(body.position);
            let dist = zone_pos.distance(body_pos);

            if dist < zone.radius {
                body.linear_velocity = body.linear_velocity * zone.factor;
                body.angular_velocity = body.angular_velocity * zone.factor;
            }
        }
    }
}

pub fn spawn_slow_motion_zone(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    position: Vec3,
    radius: f32,
    factor: f32,
) -> Entity {
    // Soft sky blue for clean aesthetic
    commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(radius))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(0.60, 0.80, 0.95, 0.12), // Soft sky blue
                alpha_mode: AlphaMode::Blend,
                cull_mode: None,
                ..default()
            })),
            Transform::from_translation(position),
            SlowMotionZone { radius, factor },
            ZoneVisual,
            SpawnedObject,
        ))
        .id()
}

// ============ BOUNCE PAD ============

#[derive(Component)]
pub struct BouncePad {
    pub strength: f32,
    pub size: Vec3,
}

fn apply_bounce_pads(
    mut nova: ResMut<NovaWorld>,
    pads: Query<(&Transform, &BouncePad)>,
) {
    for (pad_transform, pad) in pads.iter() {
        let pad_pos = pad_transform.translation;
        let half_size = pad.size * 0.5;

        for (_, body) in nova.world.bodies_mut() {
            if body.body_type.is_static() {
                continue;
            }

            let body_pos = to_bevy_vec3(body.position);

            let on_pad = (body_pos.x - pad_pos.x).abs() < half_size.x
                && (body_pos.z - pad_pos.z).abs() < half_size.z
                && body_pos.y > pad_pos.y
                && body_pos.y < pad_pos.y + half_size.y + 1.0;

            if on_pad && body.linear_velocity.y < 1.0 {
                body.linear_velocity.y = pad.strength;
            }
        }
    }
}

pub fn spawn_bounce_pad(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    position: Vec3,
    size: Vec3,
    strength: f32,
) -> Entity {
    // Soft blush pink for clean aesthetic
    commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(size.x, size.y, size.z))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.95, 0.80, 0.85), // Soft blush
                emissive: LinearRgba::new(0.3, 0.2, 0.25, 1.0), // Subtle glow
                metallic: 0.2,
                perceptual_roughness: 0.4,
                ..default()
            })),
            Transform::from_translation(position),
            BouncePad { strength, size },
            SpawnedObject,
        ))
        .id()
}

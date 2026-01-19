//! Arena environment setup: BIG ROOM with floor, walls, ceiling, and lights!

use bevy::prelude::*;
use nova::prelude::*;

use crate::components::{Ground, PhysicsBody, PhysicsCollider, StaticBody, Wall};
use crate::convert::to_nova_vec3;
use crate::resources::{HandleToEntity, NovaWorld};

/// Ceiling marker component
#[derive(Component)]
pub struct Ceiling;

/// Light fixture marker
#[derive(Component)]
pub struct LightFixture;

/// Setup the arena environment - A MASSIVE ROOM!
pub fn setup_arena(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut nova: ResMut<NovaWorld>,
    mut handle_to_entity: ResMut<HandleToEntity>,
) {
    // ============ ROOM DIMENSIONS - HUGE! ============
    let room_width = 500.0; // X
    let room_length = 500.0; // Z
    let room_height = 50.0; // Y
    let wall_thickness = 5.0;
    let floor_thickness = 5.0;

    // Note: Fog is added to camera in camera.rs

    // ============ FLOOR ============
    let ground_size = Vec3::new(room_width, floor_thickness, room_length);
    let ground_pos = Vec3::new(0.0, -floor_thickness / 2.0, 0.0);

    let ground_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(ground_pos))
        .build();
    let ground_handle = nova.world.insert_body(ground_body);

    let ground_collider = nova
        .world
        .create_collider(
            ground_handle,
            CollisionShape::Box(BoxShape::new(to_nova_vec3(ground_size * 0.5))),
        )
        .friction(0.7)
        .restitution(0.2)
        .build();
    let ground_collider_handle = nova.world.insert_collider(ground_collider);

    // Floor with a nice tile-like pattern color
    let ground_entity = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(ground_size.x, ground_size.y, ground_size.z))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.35, 0.38, 0.42),
                perceptual_roughness: 0.6,
                metallic: 0.1,
                ..default()
            })),
            Transform::from_translation(ground_pos),
            PhysicsBody {
                handle: ground_handle,
            },
            PhysicsCollider {
                handle: ground_collider_handle,
            },
            StaticBody,
            Ground,
        ))
        .id();

    handle_to_entity.bodies.insert(ground_handle, ground_entity);

    // ============ CEILING ============
    let ceiling_pos = Vec3::new(0.0, room_height + floor_thickness / 2.0, 0.0);

    let ceiling_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(ceiling_pos))
        .build();
    let ceiling_handle = nova.world.insert_body(ceiling_body);

    let ceiling_collider = nova
        .world
        .create_collider(
            ceiling_handle,
            CollisionShape::Box(BoxShape::new(to_nova_vec3(ground_size * 0.5))),
        )
        .friction(0.5)
        .restitution(0.1)
        .build();
    let ceiling_collider_handle = nova.world.insert_collider(ceiling_collider);

    let ceiling_entity = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(ground_size.x, ground_size.y, ground_size.z))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.9, 0.92, 0.95),
                perceptual_roughness: 0.95,
                ..default()
            })),
            Transform::from_translation(ceiling_pos),
            PhysicsBody {
                handle: ceiling_handle,
            },
            PhysicsCollider {
                handle: ceiling_collider_handle,
            },
            StaticBody,
            Ceiling,
        ))
        .id();

    handle_to_entity.bodies.insert(ceiling_handle, ceiling_entity);

    // ============ WALLS (4 sides) ============
    let wall_color = Color::srgba(0.45, 0.48, 0.52, 0.7);

    let walls = [
        // Back wall (-Z)
        (
            Vec3::new(0.0, room_height / 2.0, -room_length / 2.0 - wall_thickness / 2.0),
            Vec3::new(room_width + wall_thickness * 2.0, room_height, wall_thickness),
        ),
        // Front wall (+Z)
        (
            Vec3::new(0.0, room_height / 2.0, room_length / 2.0 + wall_thickness / 2.0),
            Vec3::new(room_width + wall_thickness * 2.0, room_height, wall_thickness),
        ),
        // Left wall (-X)
        (
            Vec3::new(-room_width / 2.0 - wall_thickness / 2.0, room_height / 2.0, 0.0),
            Vec3::new(wall_thickness, room_height, room_length),
        ),
        // Right wall (+X)
        (
            Vec3::new(room_width / 2.0 + wall_thickness / 2.0, room_height / 2.0, 0.0),
            Vec3::new(wall_thickness, room_height, room_length),
        ),
    ];

    for (pos, size) in walls {
        let wall_body = nova
            .world
            .create_body()
            .body_type(RigidBodyType::Static)
            .position(to_nova_vec3(pos))
            .build();
        let wall_handle = nova.world.insert_body(wall_body);

        let wall_collider = nova
            .world
            .create_collider(
                wall_handle,
                CollisionShape::Box(BoxShape::new(to_nova_vec3(size * 0.5))),
            )
            .friction(0.5)
            .restitution(0.3)
            .build();
        let wall_collider_handle = nova.world.insert_collider(wall_collider);

        let wall_entity = commands
            .spawn((
                Mesh3d(meshes.add(Cuboid::new(size.x, size.y, size.z))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: wall_color,
                    alpha_mode: AlphaMode::Blend,
                    perceptual_roughness: 0.7,
                    ..default()
                })),
                Transform::from_translation(pos),
                PhysicsBody {
                    handle: wall_handle,
                },
                PhysicsCollider {
                    handle: wall_collider_handle,
                },
                StaticBody,
                Wall,
            ))
            .id();

        handle_to_entity.bodies.insert(wall_handle, wall_entity);
    }

    // ============ LIGHTING - BRIGHT FOR BIG ROOM ============

    // Main directional light (sun-like)
    commands.spawn((
        DirectionalLight {
            illuminance: 30000.0,
            shadows_enabled: true,
            color: Color::srgb(1.0, 0.98, 0.95),
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.7, 0.5, 0.0)),
    ));

    // Bright ambient light for overall illumination
    commands.insert_resource(AmbientLight {
        color: Color::srgb(0.95, 0.97, 1.0),
        brightness: 800.0,
    });

    // Ceiling light fixtures - grid for big room
    let light_height = room_height - 3.0;
    let light_intensity = 2000000.0; // Brighter for bigger room

    // Sparse grid of ceiling lights (every 80 units)
    for x_idx in -3..=3 {
        for z_idx in -3..=3 {
            let x = x_idx as f32 * 80.0;
            let z = z_idx as f32 * 80.0;

            // Point light
            commands.spawn((
                PointLight {
                    color: Color::srgb(1.0, 0.98, 0.9),
                    intensity: light_intensity,
                    radius: 1.0,
                    range: 120.0,
                    shadows_enabled: false,
                    ..default()
                },
                Transform::from_translation(Vec3::new(x, light_height, z)),
                LightFixture,
            ));

            // Light fixture visual
            commands.spawn((
                Mesh3d(meshes.add(Cuboid::new(3.0, 0.5, 3.0))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: Color::srgb(1.0, 1.0, 0.95),
                    emissive: LinearRgba::new(8.0, 8.0, 7.0, 1.0),
                    ..default()
                })),
                Transform::from_translation(Vec3::new(x, light_height + 0.5, z)),
                LightFixture,
            ));
        }
    }

    // ============ FLOOR DECORATIONS ============
    // Add grid lines on the floor for visual reference (every 50 units for big room)
    let line_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.25, 0.28, 0.32),
        perceptual_roughness: 0.8,
        ..default()
    });

    // Grid lines every 50 units
    for i in -5..=5 {
        let offset = i as f32 * 50.0;

        // X-direction lines
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(room_width - 20.0, 0.02, 0.3))),
            MeshMaterial3d(line_material.clone()),
            Transform::from_translation(Vec3::new(0.0, 0.01, offset)),
        ));

        // Z-direction lines
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(0.3, 0.02, room_length - 20.0))),
            MeshMaterial3d(line_material.clone()),
            Transform::from_translation(Vec3::new(offset, 0.01, 0.0)),
        ));
    }

    // Center marker (bigger)
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(5.0, 0.05, 5.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.6, 0.3, 0.3),
            perceptual_roughness: 0.5,
            ..default()
        })),
        Transform::from_translation(Vec3::new(0.0, 0.025, 0.0)),
    ));
}

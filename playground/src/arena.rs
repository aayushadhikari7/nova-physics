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

    // Medium gray floor for better contrast with objects
    let ground_entity = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(ground_size.x, ground_size.y, ground_size.z))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.35, 0.37, 0.40), // Medium gray
                perceptual_roughness: 0.7,
                metallic: 0.02,
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
                base_color: Color::srgb(0.75, 0.77, 0.80), // Light gray ceiling
                perceptual_roughness: 0.9,
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
    // Medium gray semi-transparent walls for better contrast
    let wall_color = Color::srgba(0.55, 0.58, 0.62, 0.5);

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

    // ============ LIGHTING - SOFT & BRIGHT FOR CLEAN LOOK ============

    // Main directional light (soft, natural)
    commands.spawn((
        DirectionalLight {
            illuminance: 25000.0,  // Slightly softer
            shadows_enabled: true,
            color: Color::srgb(1.0, 0.99, 0.97), // Warm white
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.6, 0.4, 0.0)),
    ));

    // Moderate ambient for balanced illumination
    commands.insert_resource(AmbientLight {
        color: Color::srgb(0.9, 0.9, 0.92),
        brightness: 400.0, // Reduced for better contrast
    });

    // Ceiling light fixtures - clean, minimal design
    let light_height = room_height - 3.0;
    let light_intensity = 600000.0; // Reduced for better contrast

    // Sparse grid of ceiling lights (every 80 units)
    for x_idx in -3..=3 {
        for z_idx in -3..=3 {
            let x = x_idx as f32 * 80.0;
            let z = z_idx as f32 * 80.0;

            // Point light - pure white
            commands.spawn((
                PointLight {
                    color: Color::srgb(1.0, 1.0, 1.0), // Pure white
                    intensity: light_intensity,
                    radius: 1.0,
                    range: 100.0,
                    shadows_enabled: false,
                    ..default()
                },
                Transform::from_translation(Vec3::new(x, light_height, z)),
                LightFixture,
            ));

            // Light fixture visual - clean panel
            commands.spawn((
                Mesh3d(meshes.add(Cuboid::new(2.5, 0.3, 2.5))), // Thinner, sleeker
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: Color::srgb(0.95, 0.95, 0.95),
                    emissive: LinearRgba::new(1.0, 1.0, 1.0, 1.0), // Reduced glow
                    ..default()
                })),
                Transform::from_translation(Vec3::new(x, light_height + 0.3, z)),
                LightFixture,
            ));
        }
    }

    // ============ FLOOR DECORATIONS ============
    // Subtle gray grid lines for clean look
    let line_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.82, 0.84, 0.86), // Light gray - subtle
        perceptual_roughness: 0.6,
        ..default()
    });

    // Grid lines every 50 units - thinner for minimal look
    for i in -5..=5 {
        let offset = i as f32 * 50.0;

        // X-direction lines
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(room_width - 20.0, 0.015, 0.15))), // Thinner
            MeshMaterial3d(line_material.clone()),
            Transform::from_translation(Vec3::new(0.0, 0.01, offset)),
        ));

        // Z-direction lines
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(0.15, 0.015, room_length - 20.0))), // Thinner
            MeshMaterial3d(line_material.clone()),
            Transform::from_translation(Vec3::new(offset, 0.01, 0.0)),
        ));
    }

    // Center marker - subtle coral accent (matches pastel palette)
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(3.0, 0.03, 3.0))), // Smaller, thinner
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.95, 0.7, 0.65), // Soft coral
            perceptual_roughness: 0.4,
            metallic: 0.1,
            ..default()
        })),
        Transform::from_translation(Vec3::new(0.0, 0.02, 0.0)),
    ));
}

//! Object spawning system

use bevy::prelude::*;
use nova::prelude::*;
use rand::Rng;

use crate::components::{DynamicBody, PhysicsBody, PhysicsCollider, PhysicsMaterialType, SpawnedObject};
use crate::convert::{to_bevy_vec3, to_nova_vec3};
use crate::plugins::camera::PlayerCamera;
use crate::resources::{HandleToEntity, Hotbar, HotbarItem, NovaWorld, SelectedSlot};

pub struct SpawningPlugin;

impl Plugin for SpawningPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<HandleToEntity>()
            .add_systems(Update, spawn_system);
    }
}

/// Spawn colors for variety
const SPAWN_COLORS: [Color; 8] = [
    Color::srgb(0.8, 0.3, 0.3), // Red
    Color::srgb(0.3, 0.8, 0.3), // Green
    Color::srgb(0.3, 0.3, 0.8), // Blue
    Color::srgb(0.8, 0.8, 0.3), // Yellow
    Color::srgb(0.8, 0.3, 0.8), // Magenta
    Color::srgb(0.3, 0.8, 0.8), // Cyan
    Color::srgb(0.8, 0.5, 0.2), // Orange
    Color::srgb(0.6, 0.4, 0.8), // Purple
];

fn spawn_system(
    mouse: Res<ButtonInput<MouseButton>>,
    mut nova: ResMut<NovaWorld>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    camera_query: Query<(&GlobalTransform, &Camera), With<PlayerCamera>>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
    mut handle_to_entity: ResMut<HandleToEntity>,
) {
    if !mouse.just_pressed(MouseButton::Left) {
        return;
    }

    // Use get_item() for proper paging support!
    let Some(item) = hotbar.get_item(selected.index) else {
        return;
    };

    // Only handle spawnable items (shapes, presets, and special)
    if !item.is_spawnable() {
        return;
    }

    let Ok((global_transform, _camera)) = camera_query.get_single() else {
        return;
    };

    let origin = global_transform.translation();
    let direction = global_transform.forward().as_vec3();
    let spawn_pos = origin + direction * 3.0;

    let mut rng = rand::thread_rng();
    let color = SPAWN_COLORS[rng.gen_range(0..SPAWN_COLORS.len())];

    match item {
        HotbarItem::SpawnBox => {
            spawn_box(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                Vec3::splat(0.5),
                color,
                PhysicsMaterialType::Normal,
            );
        }
        HotbarItem::SpawnSphere => {
            spawn_sphere(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                0.5,
                color,
                PhysicsMaterialType::Normal,
            );
        }
        HotbarItem::SpawnCapsule => {
            spawn_capsule(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                0.3,
                0.5,
                color,
                PhysicsMaterialType::Normal,
            );
        }
        HotbarItem::SpawnCylinder => {
            spawn_cylinder(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                0.4,
                0.6,
                color,
                PhysicsMaterialType::Normal,
            );
        }
        HotbarItem::SpawnCone => {
            spawn_cone(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                0.5,
                0.8,
                color,
                PhysicsMaterialType::Normal,
            );
        }
        HotbarItem::SpawnCompound => {
            spawn_compound(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                color,
            );
        }
        HotbarItem::SpawnRandom => {
            // Randomly pick a shape
            let shape_type = rng.gen_range(0..5);
            match shape_type {
                0 => {
                    let size = rng.gen_range(0.3..0.8);
                    spawn_box(
                        &mut nova,
                        &mut commands,
                        &mut meshes,
                        &mut materials,
                        &mut handle_to_entity,
                        spawn_pos,
                        Vec3::splat(size),
                        color,
                        PhysicsMaterialType::Normal,
                    );
                }
                1 => {
                    let radius = rng.gen_range(0.3..0.7);
                    spawn_sphere(
                        &mut nova,
                        &mut commands,
                        &mut meshes,
                        &mut materials,
                        &mut handle_to_entity,
                        spawn_pos,
                        radius,
                        color,
                        PhysicsMaterialType::Normal,
                    );
                }
                2 => {
                    spawn_capsule(
                        &mut nova,
                        &mut commands,
                        &mut meshes,
                        &mut materials,
                        &mut handle_to_entity,
                        spawn_pos,
                        rng.gen_range(0.2..0.4),
                        rng.gen_range(0.3..0.6),
                        color,
                        PhysicsMaterialType::Normal,
                    );
                }
                3 => {
                    spawn_cylinder(
                        &mut nova,
                        &mut commands,
                        &mut meshes,
                        &mut materials,
                        &mut handle_to_entity,
                        spawn_pos,
                        rng.gen_range(0.3..0.5),
                        rng.gen_range(0.4..0.8),
                        color,
                        PhysicsMaterialType::Normal,
                    );
                }
                _ => {
                    spawn_cone(
                        &mut nova,
                        &mut commands,
                        &mut meshes,
                        &mut materials,
                        &mut handle_to_entity,
                        spawn_pos,
                        rng.gen_range(0.3..0.6),
                        rng.gen_range(0.5..1.0),
                        color,
                        PhysicsMaterialType::Normal,
                    );
                }
            }
        }
        HotbarItem::SpawnChain => {
            spawn_chain(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                10,
            );
        }
        // === PRESETS ===
        HotbarItem::SpawnTower => {
            crate::presets::spawn_tower(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                10,
            );
        }
        HotbarItem::SpawnPyramid => {
            crate::presets::spawn_pyramid(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                5,
            );
        }
        HotbarItem::SpawnRagdoll => {
            crate::presets::spawn_ragdoll(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos + Vec3::new(0.0, 2.0, 0.0),
            );
        }
        HotbarItem::SpawnNewtonsCradle => {
            crate::presets::spawn_newtons_cradle(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos + Vec3::new(0.0, 3.0, 0.0),
                5,
            );
        }
        HotbarItem::SpawnWreckingBall => {
            crate::presets::spawn_wrecking_ball(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos + Vec3::new(0.0, 6.0, 0.0),
                8,
            );
        }
        HotbarItem::SpawnDominos => {
            crate::presets::spawn_dominos(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                20,
                direction,
            );
        }
        HotbarItem::SpawnBridge => {
            crate::presets::spawn_bridge(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                10,
            );
        }
        HotbarItem::SpawnCatapult => {
            crate::presets::spawn_catapult(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
            );
        }
        HotbarItem::SpawnCar => {
            crate::presets::spawn_car(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos + Vec3::new(0.0, 1.0, 0.0),
            );
        }
        HotbarItem::SpawnPendulumWall => {
            crate::presets::spawn_pendulum_wall(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos + Vec3::new(0.0, 5.0, 0.0),
                5,
                3,
            );
        }
        HotbarItem::SpawnBoxRain => {
            crate::presets::spawn_box_rain(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos + Vec3::new(0.0, 10.0, 0.0),
                30,
            );
        }
        HotbarItem::SpawnSpiral => {
            crate::presets::spawn_spiral_staircase(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                20,
            );
        }
        HotbarItem::SpawnCannon => {
            crate::presets::spawn_cannon(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                direction,
            );
        }
        HotbarItem::SpawnFerrisWheel => {
            crate::presets::spawn_ferris_wheel(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos + Vec3::new(0.0, 5.0, 0.0),
                6,
            );
        }
        HotbarItem::SpawnWindmill => {
            crate::presets::spawn_windmill(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos + Vec3::new(0.0, 3.0, 0.0),
            );
        }
        // === SPECIAL ITEMS ===
        HotbarItem::GravityZone => {
            crate::plugins::effects::spawn_gravity_zone(
                &mut commands,
                &mut meshes,
                &mut materials,
                spawn_pos,
                8.0,
                Vec3::new(0.0, 15.0, 0.0), // Upward gravity
                true,
            );
        }
        HotbarItem::ForceField => {
            crate::plugins::effects::spawn_force_field(
                &mut commands,
                &mut meshes,
                &mut materials,
                spawn_pos,
                10.0,
                crate::plugins::effects::ForceFieldMode::Vortex,
                200.0,
            );
        }
        HotbarItem::Portal => {
            spawn_portal(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
            );
        }
        HotbarItem::Trampoline => {
            spawn_trampoline(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
            );
        }
        HotbarItem::Conveyor => {
            spawn_conveyor(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                direction,
            );
        }
        HotbarItem::Fan => {
            spawn_fan(
                &mut commands,
                &mut meshes,
                &mut materials,
                spawn_pos,
                Vec3::Y,
            );
        }
        _ => {}
    }
}

pub fn spawn_box(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    half_extents: Vec3,
    color: Color,
    material_type: PhysicsMaterialType,
) -> (Entity, RigidBodyHandle) {
    let nova_pos = to_nova_vec3(position);
    let nova_half = to_nova_vec3(half_extents);

    // Create Nova body
    let body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(nova_pos)
        .mass(1.0 * material_type.mass_multiplier())
        .build();
    let body_handle = nova.world.insert_body(body);

    // Create Nova collider
    let collider = nova
        .world
        .create_collider(body_handle, CollisionShape::Box(BoxShape::new(nova_half)))
        .friction(material_type.friction())
        .restitution(material_type.restitution())
        .build();
    let collider_handle = nova.world.insert_collider(collider);

    // Create Bevy entity
    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(
                half_extents.x * 2.0,
                half_extents.y * 2.0,
                half_extents.z * 2.0,
            ))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                ..default()
            })),
            Transform::from_translation(position),
            PhysicsBody {
                handle: body_handle,
            },
            PhysicsCollider {
                handle: collider_handle,
            },
            DynamicBody,
            SpawnedObject,
            material_type,
        ))
        .id();

    handle_to_entity.bodies.insert(body_handle, entity);
    handle_to_entity.colliders.insert(collider_handle, entity);

    (entity, body_handle)
}

pub fn spawn_sphere(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    radius: f32,
    color: Color,
    material_type: PhysicsMaterialType,
) -> (Entity, RigidBodyHandle) {
    let nova_pos = to_nova_vec3(position);

    // Create Nova body
    let body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(nova_pos)
        .mass(1.0 * material_type.mass_multiplier())
        .build();
    let body_handle = nova.world.insert_body(body);

    // Create Nova collider
    let collider = nova
        .world
        .create_collider(body_handle, CollisionShape::Sphere(SphereShape::new(radius)))
        .friction(material_type.friction())
        .restitution(material_type.restitution())
        .build();
    let collider_handle = nova.world.insert_collider(collider);

    // Create Bevy entity
    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(radius))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                ..default()
            })),
            Transform::from_translation(position),
            PhysicsBody {
                handle: body_handle,
            },
            PhysicsCollider {
                handle: collider_handle,
            },
            DynamicBody,
            SpawnedObject,
            material_type,
        ))
        .id();

    handle_to_entity.bodies.insert(body_handle, entity);
    handle_to_entity.colliders.insert(collider_handle, entity);

    (entity, body_handle)
}

pub fn spawn_capsule(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    radius: f32,
    half_height: f32,
    color: Color,
    material_type: PhysicsMaterialType,
) -> (Entity, RigidBodyHandle) {
    let nova_pos = to_nova_vec3(position);

    // Create Nova body
    let body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(nova_pos)
        .mass(1.0 * material_type.mass_multiplier())
        .build();
    let body_handle = nova.world.insert_body(body);

    // Create Nova collider
    let collider = nova
        .world
        .create_collider(
            body_handle,
            CollisionShape::Capsule(CapsuleShape::new(radius, half_height)),
        )
        .friction(material_type.friction())
        .restitution(material_type.restitution())
        .build();
    let collider_handle = nova.world.insert_collider(collider);

    // Create Bevy entity with capsule mesh
    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Capsule3d::new(radius, half_height * 2.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                ..default()
            })),
            Transform::from_translation(position),
            PhysicsBody {
                handle: body_handle,
            },
            PhysicsCollider {
                handle: collider_handle,
            },
            DynamicBody,
            SpawnedObject,
            material_type,
        ))
        .id();

    handle_to_entity.bodies.insert(body_handle, entity);
    handle_to_entity.colliders.insert(collider_handle, entity);

    (entity, body_handle)
}

pub fn spawn_chain(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    start_position: Vec3,
    link_count: usize,
) {
    let link_radius = 0.15;
    let link_half_height = 0.2;
    let link_spacing = 0.5;

    let mut prev_handle: Option<RigidBodyHandle> = None;

    for i in 0..link_count {
        let pos = start_position + Vec3::new(0.0, -(i as f32) * link_spacing, 0.0);
        let color = SPAWN_COLORS[i % SPAWN_COLORS.len()];

        let (_, body_handle) = spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            pos,
            link_radius,
            color,
            PhysicsMaterialType::Normal,
        );

        // Connect to previous link with ball joint
        if let Some(prev) = prev_handle {
            let anchor_a = nova::prelude::Vec3::new(0.0, -link_spacing / 2.0, 0.0);
            let anchor_b = nova::prelude::Vec3::new(0.0, link_spacing / 2.0, 0.0);
            nova.world.create_ball_joint(prev, body_handle, anchor_a, anchor_b);
        }

        prev_handle = Some(body_handle);
    }
}

/// Spawn a cylinder (using capsule physics, cylinder visual)
pub fn spawn_cylinder(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    radius: f32,
    half_height: f32,
    color: Color,
    material_type: PhysicsMaterialType,
) -> (Entity, RigidBodyHandle) {
    let nova_pos = to_nova_vec3(position);

    let body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(nova_pos)
        .mass(1.5 * material_type.mass_multiplier())
        .build();
    let body_handle = nova.world.insert_body(body);

    // Use capsule for physics (closest approximation)
    let collider = nova
        .world
        .create_collider(
            body_handle,
            CollisionShape::Capsule(CapsuleShape::new(radius, half_height)),
        )
        .friction(material_type.friction())
        .restitution(material_type.restitution())
        .build();
    let collider_handle = nova.world.insert_collider(collider);

    // Visual cylinder mesh
    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Cylinder::new(radius, half_height * 2.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                ..default()
            })),
            Transform::from_translation(position),
            PhysicsBody {
                handle: body_handle,
            },
            PhysicsCollider {
                handle: collider_handle,
            },
            DynamicBody,
            SpawnedObject,
            material_type,
        ))
        .id();

    handle_to_entity.bodies.insert(body_handle, entity);
    handle_to_entity.colliders.insert(collider_handle, entity);

    (entity, body_handle)
}

/// Spawn a cone (using sphere physics, cone visual)
pub fn spawn_cone(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    radius: f32,
    height: f32,
    color: Color,
    material_type: PhysicsMaterialType,
) -> (Entity, RigidBodyHandle) {
    let nova_pos = to_nova_vec3(position);

    let body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(nova_pos)
        .mass(1.0 * material_type.mass_multiplier())
        .build();
    let body_handle = nova.world.insert_body(body);

    // Use sphere for physics (approximation)
    let approx_radius = (radius + height / 2.0) / 2.0;
    let collider = nova
        .world
        .create_collider(
            body_handle,
            CollisionShape::Sphere(SphereShape::new(approx_radius)),
        )
        .friction(material_type.friction())
        .restitution(material_type.restitution())
        .build();
    let collider_handle = nova.world.insert_collider(collider);

    // Visual cone mesh
    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Cone::new(radius, height))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                ..default()
            })),
            Transform::from_translation(position),
            PhysicsBody {
                handle: body_handle,
            },
            PhysicsCollider {
                handle: collider_handle,
            },
            DynamicBody,
            SpawnedObject,
            material_type,
        ))
        .id();

    handle_to_entity.bodies.insert(body_handle, entity);
    handle_to_entity.colliders.insert(collider_handle, entity);

    (entity, body_handle)
}

/// Spawn a compound shape (T-shaped block - simplified physics, complex visual)
pub fn spawn_compound(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    color: Color,
) {
    let nova_pos = to_nova_vec3(position);

    // Create body with a box collider that approximates the T shape
    let body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(nova_pos)
        .mass(2.5)
        .build();
    let body_handle = nova.world.insert_body(body);

    // Use a box that roughly covers the T shape
    let half_extents = nova::prelude::Vec3::new(0.5, 0.5, 0.3);
    let collider = nova
        .world
        .create_collider(body_handle, CollisionShape::Box(BoxShape::new(half_extents)))
        .friction(0.5)
        .restitution(0.2)
        .build();
    let collider_handle = nova.world.insert_collider(collider);

    // Create visual T-shape with two boxes as children
    let entity = commands
        .spawn((
            Transform::from_translation(position),
            Visibility::default(),
            PhysicsBody {
                handle: body_handle,
            },
            PhysicsCollider {
                handle: collider_handle,
            },
            DynamicBody,
            SpawnedObject,
        ))
        .with_children(|parent| {
            // Top horizontal bar of T
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(1.0, 0.3, 0.6))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: color,
                    ..default()
                })),
                Transform::from_translation(Vec3::new(0.0, 0.35, 0.0)),
            ));
            // Vertical stem of T
            parent.spawn((
                Mesh3d(meshes.add(Cuboid::new(0.3, 0.7, 0.6))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: color,
                    ..default()
                })),
                Transform::from_translation(Vec3::new(0.0, -0.15, 0.0)),
            ));
        })
        .id();

    handle_to_entity.bodies.insert(body_handle, entity);
}

/// Spawn a portal (visual effect ring)
pub fn spawn_portal(
    _nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    _handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    // Portal is just a visual ring effect
    commands.spawn((
        Mesh3d(meshes.add(Torus::new(0.3, 1.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.5, 0.0, 1.0, 0.7),
            emissive: bevy::color::LinearRgba::new(2.0, 0.0, 4.0, 1.0),
            alpha_mode: AlphaMode::Blend,
            ..default()
        })),
        Transform::from_translation(position)
            .with_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
        SpawnedObject,
    ));
}

/// Spawn a trampoline (bouncy platform)
pub fn spawn_trampoline(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let nova_pos = to_nova_vec3(position);
    let half_extents = nova::prelude::Vec3::new(1.5, 0.1, 1.5);

    // Create static body for trampoline
    let body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(nova_pos)
        .build();
    let body_handle = nova.world.insert_body(body);

    // Very bouncy collider!
    let collider = nova
        .world
        .create_collider(body_handle, CollisionShape::Box(BoxShape::new(half_extents)))
        .friction(0.3)
        .restitution(1.5) // Super bouncy!
        .build();
    let collider_handle = nova.world.insert_collider(collider);

    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(3.0, 0.2, 3.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(1.0, 0.4, 0.1),
                ..default()
            })),
            Transform::from_translation(position),
            PhysicsBody {
                handle: body_handle,
            },
            PhysicsCollider {
                handle: collider_handle,
            },
            SpawnedObject,
        ))
        .id();

    handle_to_entity.bodies.insert(body_handle, entity);
}

/// Spawn a conveyor belt (platform that pushes objects)
pub fn spawn_conveyor(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    direction: Vec3,
) {
    let nova_pos = to_nova_vec3(position);
    let half_extents = nova::prelude::Vec3::new(2.0, 0.1, 0.8);

    let body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(nova_pos)
        .build();
    let body_handle = nova.world.insert_body(body);

    let collider = nova
        .world
        .create_collider(body_handle, CollisionShape::Box(BoxShape::new(half_extents)))
        .friction(1.0) // High friction to "grip" objects
        .restitution(0.0)
        .build();
    let collider_handle = nova.world.insert_collider(collider);

    // Create visual with stripes to show direction
    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(4.0, 0.2, 1.6))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.3, 0.3, 0.3),
                ..default()
            })),
            Transform::from_translation(position),
            PhysicsBody {
                handle: body_handle,
            },
            PhysicsCollider {
                handle: collider_handle,
            },
            ConveyorBelt {
                direction: Vec3::new(direction.x, 0.0, direction.z).normalize_or_zero(),
                speed: 5.0,
            },
            SpawnedObject,
        ))
        .id();

    handle_to_entity.bodies.insert(body_handle, entity);
}

/// Component for conveyor belt behavior
#[derive(Component)]
pub struct ConveyorBelt {
    pub direction: Vec3,
    pub speed: f32,
}

/// Spawn a fan (pushes objects upward)
pub fn spawn_fan(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    position: Vec3,
    direction: Vec3,
) {
    // Fan is a force field pointing in a direction
    crate::plugins::effects::spawn_force_field(
        commands,
        meshes,
        materials,
        position,
        6.0, // radius
        crate::plugins::effects::ForceFieldMode::Directional(direction),
        400.0, // strength
    );
}

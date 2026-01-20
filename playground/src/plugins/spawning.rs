//! Object spawning system

use bevy::color::LinearRgba;
use bevy::prelude::*;
use nova::prelude::*;
use rand::Rng;

use crate::components::{
    AutoDespawn, Breakable, DynamicBody, Explosive, Glowing, MagnetObject, PhysicsBody,
    PhysicsCollider, PhysicsMaterialType, SpawnedObject, Spinner,
};
use crate::convert::to_nova_vec3;
use crate::plugins::camera::PlayerCamera;
use crate::plugins::effects::TrailEmitter;
use crate::plugins::settings::{ColorTheme, GameSettings};
use crate::resources::{HandleToEntity, Hotbar, HotbarItem, NovaWorld, SelectedSlot};

pub struct SpawningPlugin;

impl Plugin for SpawningPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<HandleToEntity>()
            .add_systems(Update, spawn_system);
    }
}

/// Color palettes for different themes
const PASTEL_COLORS: [Color; 8] = [
    Color::srgb(0.95, 0.60, 0.55), // Coral
    Color::srgb(0.60, 0.90, 0.70), // Mint
    Color::srgb(0.60, 0.75, 0.95), // Sky blue
    Color::srgb(0.95, 0.90, 0.70), // Cream/butter
    Color::srgb(0.80, 0.70, 0.90), // Lavender
    Color::srgb(0.60, 0.90, 0.90), // Aqua
    Color::srgb(0.95, 0.75, 0.60), // Peach
    Color::srgb(0.95, 0.75, 0.80), // Blush pink
];

const VIBRANT_COLORS: [Color; 8] = [
    Color::srgb(1.0, 0.3, 0.3),  // Bright Red
    Color::srgb(0.3, 1.0, 0.3),  // Bright Green
    Color::srgb(0.3, 0.5, 1.0),  // Bright Blue
    Color::srgb(1.0, 1.0, 0.3),  // Bright Yellow
    Color::srgb(1.0, 0.3, 1.0),  // Bright Magenta
    Color::srgb(0.3, 1.0, 1.0),  // Bright Cyan
    Color::srgb(1.0, 0.6, 0.2),  // Bright Orange
    Color::srgb(1.0, 0.5, 0.7),  // Bright Pink
];

const CLASSIC_COLORS: [Color; 8] = [
    Color::srgb(0.8, 0.2, 0.2),  // Red
    Color::srgb(0.2, 0.7, 0.3),  // Green
    Color::srgb(0.2, 0.4, 0.8),  // Blue
    Color::srgb(0.9, 0.8, 0.2),  // Yellow
    Color::srgb(0.6, 0.3, 0.7),  // Purple
    Color::srgb(0.3, 0.7, 0.7),  // Teal
    Color::srgb(0.9, 0.5, 0.2),  // Orange
    Color::srgb(0.5, 0.3, 0.2),  // Brown
];

const NEON_COLORS: [Color; 8] = [
    Color::srgb(1.0, 0.1, 0.4),  // Neon Pink
    Color::srgb(0.1, 1.0, 0.5),  // Neon Green
    Color::srgb(0.2, 0.5, 1.0),  // Neon Blue
    Color::srgb(1.0, 1.0, 0.1),  // Neon Yellow
    Color::srgb(0.8, 0.1, 1.0),  // Neon Purple
    Color::srgb(0.1, 1.0, 1.0),  // Neon Cyan
    Color::srgb(1.0, 0.5, 0.1),  // Neon Orange
    Color::srgb(1.0, 0.2, 0.6),  // Neon Magenta
];

const MONOCHROME_COLORS: [Color; 8] = [
    Color::srgb(0.9, 0.9, 0.9),  // White
    Color::srgb(0.75, 0.75, 0.75), // Light Gray
    Color::srgb(0.6, 0.6, 0.6),  // Medium Light Gray
    Color::srgb(0.5, 0.5, 0.5),  // Gray
    Color::srgb(0.4, 0.4, 0.4),  // Medium Gray
    Color::srgb(0.3, 0.3, 0.3),  // Dark Gray
    Color::srgb(0.2, 0.2, 0.2),  // Darker Gray
    Color::srgb(0.15, 0.15, 0.15), // Near Black
];

/// Get color palette for the current theme
pub fn get_theme_colors(theme: ColorTheme) -> &'static [Color; 8] {
    match theme {
        ColorTheme::Pastel => &PASTEL_COLORS,
        ColorTheme::Vibrant => &VIBRANT_COLORS,
        ColorTheme::Classic => &CLASSIC_COLORS,
        ColorTheme::Neon => &NEON_COLORS,
        ColorTheme::Monochrome => &MONOCHROME_COLORS,
    }
}

/// Get material properties based on theme (neon gets emissive)
pub fn get_theme_material_properties(theme: ColorTheme) -> (f32, f32, bool) {
    // Returns (metallic, roughness, is_emissive)
    match theme {
        ColorTheme::Pastel => (0.3, 0.4, false),
        ColorTheme::Vibrant => (0.4, 0.3, false),
        ColorTheme::Classic => (0.2, 0.5, false),
        ColorTheme::Neon => (0.5, 0.2, true),  // Emissive for neon!
        ColorTheme::Monochrome => (0.6, 0.3, false),
    }
}

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
    settings: Res<GameSettings>,
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
    let spawn_pos = origin + direction * settings.spawn_distance;

    let mut rng = rand::thread_rng();
    // Use color theme from settings
    let theme_colors = get_theme_colors(settings.color_theme);
    let color = theme_colors[rng.gen_range(0..theme_colors.len())];

    match item {
        HotbarItem::SpawnBox => {
            spawn_box_with_options(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                Vec3::splat(0.5),
                color,
                PhysicsMaterialType::Normal,
                settings.color_theme,
                settings.trail_enabled,
            );
        }
        HotbarItem::SpawnSphere => {
            spawn_sphere_with_options(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                0.5,
                color,
                PhysicsMaterialType::Normal,
                settings.color_theme,
                settings.trail_enabled,
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
        // === PHYSICS OBJECTS ===
        HotbarItem::SpawnBreakable => {
            spawn_breakable(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                color,
            );
        }
        HotbarItem::SpawnExplosive => {
            spawn_explosive(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
            );
        }
        HotbarItem::SpawnSpinner => {
            spawn_spinner(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
            );
        }
        HotbarItem::SpawnMagnetObj => {
            spawn_magnet_object(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
            );
        }
        HotbarItem::SpawnGlowing => {
            spawn_glowing(
                &mut nova,
                &mut commands,
                &mut meshes,
                &mut materials,
                &mut handle_to_entity,
                spawn_pos,
                color,
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
    spawn_box_with_options(nova, commands, meshes, materials, handle_to_entity, position, half_extents, color, material_type, ColorTheme::Pastel, false)
}

/// Spawn box with full options including theme and trail support
pub fn spawn_box_with_options(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    half_extents: Vec3,
    color: Color,
    material_type: PhysicsMaterialType,
    theme: ColorTheme,
    trail_enabled: bool,
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

    // Get theme-based material properties
    let (metallic, roughness, is_emissive) = get_theme_material_properties(theme);
    let linear = color.to_linear();
    let emissive = if is_emissive {
        LinearRgba::new(linear.red * 2.0, linear.green * 2.0, linear.blue * 2.0, 1.0)
    } else {
        LinearRgba::BLACK
    };

    // Create Bevy entity with theme-based materials
    let mut entity_commands = commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(
            half_extents.x * 2.0,
            half_extents.y * 2.0,
            half_extents.z * 2.0,
        ))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: color,
            metallic,
            perceptual_roughness: roughness,
            emissive,
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
    ));

    // Add trail emitter if enabled
    if trail_enabled {
        entity_commands.insert(TrailEmitter {
            points: Vec::new(),
            max_points: 30,
            min_velocity: 3.0,
            last_pos: position,
        });
    }

    let entity = entity_commands.id();

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
    spawn_sphere_with_options(nova, commands, meshes, materials, handle_to_entity, position, radius, color, material_type, ColorTheme::Pastel, false)
}

/// Spawn sphere with full options including theme and trail support
pub fn spawn_sphere_with_options(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    radius: f32,
    color: Color,
    material_type: PhysicsMaterialType,
    theme: ColorTheme,
    trail_enabled: bool,
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

    // Get theme-based material properties
    let (metallic, roughness, is_emissive) = get_theme_material_properties(theme);
    let linear = color.to_linear();
    let emissive = if is_emissive {
        LinearRgba::new(linear.red * 2.0, linear.green * 2.0, linear.blue * 2.0, 1.0)
    } else {
        LinearRgba::BLACK
    };

    // Create Bevy entity with theme-based materials
    let mut entity_commands = commands.spawn((
        Mesh3d(meshes.add(Sphere::new(radius))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: color,
            metallic: metallic + 0.05, // Slightly more for spheres
            perceptual_roughness: roughness - 0.05, // Smoother
            emissive,
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
    ));

    // Add trail emitter if enabled
    if trail_enabled {
        entity_commands.insert(TrailEmitter {
            points: Vec::new(),
            max_points: 30,
            min_velocity: 3.0,
            last_pos: position,
        });
    }

    let entity = entity_commands.id();

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

    // Create Bevy entity with capsule mesh - polished look
    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Capsule3d::new(radius, half_height * 2.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                metallic: 0.3,
                perceptual_roughness: 0.4,
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
        let color = PASTEL_COLORS[i % PASTEL_COLORS.len()];

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

    // Visual cylinder mesh - polished look
    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Cylinder::new(radius, half_height * 2.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                metallic: 0.3,
                perceptual_roughness: 0.4,
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

    // Visual cone mesh - polished look
    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Cone::new(radius, height))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                metallic: 0.3,
                perceptual_roughness: 0.4,
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

/// Portal component for teleportation
#[derive(Component)]
pub struct PortalEntity {
    pub destination: Vec3,
    pub radius: f32,
    pub preserve_velocity: bool,
    pub cooldown: f32,
}

/// Spawn a portal (visual effect ring with teleportation)
pub fn spawn_portal(
    _nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    _handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    // Create destination offset (portals teleport 10 units forward by default)
    let destination = position + Vec3::new(10.0, 0.0, 0.0);

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
        PortalEntity {
            destination,
            radius: 1.5,
            preserve_velocity: true,
            cooldown: 0.0,
        },
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

// ============ PHYSICS OBJECTS ============

/// Spawn a breakable box that shatters on high-impulse collision
pub fn spawn_breakable(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    color: Color,
) -> Entity {
    let nova_pos = to_nova_vec3(position);
    let half_extents = nova::prelude::Vec3::new(0.5, 0.5, 0.5);

    let body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(nova_pos)
        .mass(1.5)
        .build();
    let body_handle = nova.world.insert_body(body);

    let collider = nova
        .world
        .create_collider(body_handle, CollisionShape::Box(BoxShape::new(half_extents)))
        .friction(0.5)
        .restitution(0.2)
        .build();
    let collider_handle = nova.world.insert_collider(collider);

    // Breakable box with crack lines visual
    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.6, 0.4, 0.3), // Brownish clay-like
                ..default()
            })),
            Transform::from_translation(position),
            PhysicsBody { handle: body_handle },
            PhysicsCollider { handle: collider_handle },
            DynamicBody,
            SpawnedObject,
            Breakable {
                impulse_threshold: 10.0, // Breaks when moving faster than this
                pieces: 6,
            },
        ))
        .id();

    handle_to_entity.bodies.insert(body_handle, entity);
    handle_to_entity.colliders.insert(collider_handle, entity);

    entity
}

/// Spawn an explosive that explodes on collision
pub fn spawn_explosive(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) -> Entity {
    let nova_pos = to_nova_vec3(position);

    let body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(nova_pos)
        .mass(2.0)
        .build();
    let body_handle = nova.world.insert_body(body);

    let collider = nova
        .world
        .create_collider(body_handle, CollisionShape::Sphere(SphereShape::new(0.4)))
        .friction(0.6)
        .restitution(0.1)
        .build();
    let collider_handle = nova.world.insert_collider(collider);

    // Red glowing explosive ball
    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(0.4))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.8, 0.2, 0.1),
                emissive: LinearRgba::new(2.0, 0.3, 0.1, 1.0),
                ..default()
            })),
            Transform::from_translation(position),
            PhysicsBody { handle: body_handle },
            PhysicsCollider { handle: collider_handle },
            DynamicBody,
            SpawnedObject,
            Explosive {
                radius: 8.0,
                force: 800.0,
                triggered: false,
            },
        ))
        .id();

    handle_to_entity.bodies.insert(body_handle, entity);
    handle_to_entity.colliders.insert(collider_handle, entity);

    entity
}

/// Spawn a spinner object that auto-rotates
pub fn spawn_spinner(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) -> Entity {
    let nova_pos = to_nova_vec3(position);
    let half_extents = nova::prelude::Vec3::new(1.5, 0.1, 0.3);

    let body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(nova_pos)
        .mass(5.0)
        .build();
    let body_handle = nova.world.insert_body(body);

    let collider = nova
        .world
        .create_collider(body_handle, CollisionShape::Box(BoxShape::new(half_extents)))
        .friction(0.4)
        .restitution(0.3)
        .build();
    let collider_handle = nova.world.insert_collider(collider);

    // Long rotating bar
    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(3.0, 0.2, 0.6))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.4, 0.6, 0.9),
                metallic: 0.8,
                ..default()
            })),
            Transform::from_translation(position),
            PhysicsBody { handle: body_handle },
            PhysicsCollider { handle: collider_handle },
            DynamicBody,
            SpawnedObject,
            Spinner {
                axis: Vec3::Y,
                speed: 5.0,
            },
        ))
        .id();

    handle_to_entity.bodies.insert(body_handle, entity);
    handle_to_entity.colliders.insert(collider_handle, entity);

    entity
}

/// Spawn a magnet object that attracts nearby bodies
pub fn spawn_magnet_object(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) -> Entity {
    let nova_pos = to_nova_vec3(position);

    let body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(nova_pos)
        .mass(3.0)
        .build();
    let body_handle = nova.world.insert_body(body);

    let collider = nova
        .world
        .create_collider(body_handle, CollisionShape::Sphere(SphereShape::new(0.5)))
        .friction(0.5)
        .restitution(0.2)
        .build();
    let collider_handle = nova.world.insert_collider(collider);

    // Metallic magnet sphere with red tint
    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(0.5))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.7, 0.1, 0.1),
                metallic: 1.0,
                perceptual_roughness: 0.2,
                ..default()
            })),
            Transform::from_translation(position),
            PhysicsBody { handle: body_handle },
            PhysicsCollider { handle: collider_handle },
            DynamicBody,
            SpawnedObject,
            MagnetObject {
                strength: 300.0,
                radius: 8.0,
                attract: true,
            },
        ))
        .id();

    handle_to_entity.bodies.insert(body_handle, entity);
    handle_to_entity.colliders.insert(collider_handle, entity);

    entity
}

/// Spawn a glowing object with pulsing emissive material
pub fn spawn_glowing(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    color: Color,
) -> Entity {
    let nova_pos = to_nova_vec3(position);

    let body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(nova_pos)
        .mass(1.0)
        .build();
    let body_handle = nova.world.insert_body(body);

    let collider = nova
        .world
        .create_collider(body_handle, CollisionShape::Sphere(SphereShape::new(0.4)))
        .friction(0.4)
        .restitution(0.5)
        .build();
    let collider_handle = nova.world.insert_collider(collider);

    // Glowing orb that pulses
    let linear = color.to_linear();
    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(0.4))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                emissive: LinearRgba::new(linear.red * 2.0, linear.green * 2.0, linear.blue * 2.0, 1.0),
                ..default()
            })),
            Transform::from_translation(position),
            PhysicsBody { handle: body_handle },
            PhysicsCollider { handle: collider_handle },
            DynamicBody,
            SpawnedObject,
            Glowing {
                color,
                intensity: 3.0,
            },
        ))
        .id();

    handle_to_entity.bodies.insert(body_handle, entity);
    handle_to_entity.colliders.insert(collider_handle, entity);

    entity
}

/// Spawn a projectile with auto-despawn
pub fn spawn_projectile(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    velocity: Vec3,
    lifetime: f32,
) -> Entity {
    let nova_pos = to_nova_vec3(position);
    let nova_vel = to_nova_vec3(velocity);

    let body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(nova_pos)
        .linear_velocity(nova_vel)
        .mass(0.5)
        .build();
    let body_handle = nova.world.insert_body(body);

    let collider = nova
        .world
        .create_collider(body_handle, CollisionShape::Sphere(SphereShape::new(0.2)))
        .friction(0.3)
        .restitution(0.6)
        .build();
    let collider_handle = nova.world.insert_collider(collider);

    let entity = commands
        .spawn((
            Mesh3d(meshes.add(Sphere::new(0.2))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(1.0, 0.8, 0.2),
                emissive: LinearRgba::new(2.0, 1.5, 0.3, 1.0),
                ..default()
            })),
            Transform::from_translation(position),
            PhysicsBody { handle: body_handle },
            PhysicsCollider { handle: collider_handle },
            DynamicBody,
            SpawnedObject,
            AutoDespawn { timer: lifetime },
        ))
        .id();

    handle_to_entity.bodies.insert(body_handle, entity);
    handle_to_entity.colliders.insert(collider_handle, entity);

    entity
}

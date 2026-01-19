//! Tool implementations: Gravity gun, Force gun, Explosion, Joint tool, Delete, AND SO MUCH MORE!

use bevy::prelude::*;
use nova::prelude::*;
use rand::Rng;

use crate::components::{DynamicBody, FrozenBody, PhysicsBody, PhysicsMaterialType, Trail};
use crate::convert::{to_bevy_vec3, to_nova_vec3};
use crate::plugins::camera::PlayerCamera;
use crate::plugins::console::{log_delete, log_joint, log_tool, PhysicsConsole};
use crate::plugins::spawning::{spawn_box, spawn_sphere};
use crate::resources::{
    CannonState, GrabState, HandleToEntity, Hotbar, HotbarItem, JointToolState, JointType,
    MagnetState, NovaWorld, PainterState, PlaygroundStats, ProjectileType, SelectedSlot,
    ToolSettings, PAINT_COLORS,
};

pub struct ToolsPlugin;

impl Plugin for ToolsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ToolSettings>()
            .init_resource::<MagnetState>()
            .init_resource::<CannonState>()
            .init_resource::<PainterState>()
            .init_resource::<PlaygroundStats>()
            .add_systems(
                Update,
                (
                    gravity_gun_system,
                    force_gun_system,
                    explosion_system,
                    delete_system,
                    joint_tool_system,
                    update_grabbed_object,
                    freeze_system,
                    clone_system,
                    magnet_system,
                    launch_cannon_system,
                    painter_system,
                    tool_scroll_adjust,
                ),
            );
    }
}

/// Helper to get camera ray
fn get_camera_ray(
    camera_query: &Query<(&GlobalTransform, &Camera), With<PlayerCamera>>,
) -> Option<(Vec3, Vec3)> {
    let (global_transform, _camera) = camera_query.get_single().ok()?;
    let origin = global_transform.translation();
    let direction = global_transform.forward().as_vec3();
    Some((origin, direction))
}

/// Helper to raycast into Nova world
fn raycast_nova(
    nova: &NovaWorld,
    origin: Vec3,
    direction: Vec3,
    max_dist: f32,
) -> Option<(RigidBodyHandle, ColliderHandle, f32, Vec3)> {
    let nova_origin = to_nova_vec3(origin);
    let nova_dir = to_nova_vec3(direction);

    let hit = nova
        .world
        .raycast(nova_origin, nova_dir, max_dist, QueryFilter::default())?;

    if let Some(collider) = nova.world.get_collider(hit.collider) {
        Some((
            collider.parent,
            hit.collider,
            hit.distance,
            to_bevy_vec3(hit.point),
        ))
    } else {
        None
    }
}

fn gravity_gun_system(
    mouse: Res<ButtonInput<MouseButton>>,
    mut grab_state: ResMut<GrabState>,
    mut nova: ResMut<NovaWorld>,
    camera_query: Query<(&GlobalTransform, &Camera), With<PlayerCamera>>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
    settings: Res<ToolSettings>,
    mut console: ResMut<PhysicsConsole>,
    time: Res<Time>,
) {
    if hotbar.get_item(selected.index) != Some(&HotbarItem::GravityGun) {
        return;
    }

    let Some((origin, direction)) = get_camera_ray(&camera_query) else {
        return;
    };

    if mouse.just_pressed(MouseButton::Left) {
        if grab_state.holding.is_some() {
            log_tool(&mut console, "Gravity Gun", "released object", time.elapsed_secs());
            grab_state.holding = None;
            grab_state.holding_collider = None;
        } else {
            if let Some((body_handle, collider_handle, distance, hit_point)) =
                raycast_nova(&nova, origin, direction, 50.0)
            {
                if let Some(body) = nova.world.get_body(body_handle) {
                    if body.body_type == RigidBodyType::Dynamic {
                        grab_state.holding = Some(body_handle);
                        grab_state.holding_collider = Some(collider_handle);
                        grab_state.distance = distance;
                        grab_state.grab_offset = hit_point - to_bevy_vec3(body.position);
                        log_tool(&mut console, "Gravity Gun", &format!("grabbed object at dist {:.1}m", distance), time.elapsed_secs());
                    }
                }
            }
        }
    }

    if mouse.just_pressed(MouseButton::Right) {
        if let Some(handle) = grab_state.holding {
            if let Some(body) = nova.world.get_body_mut(handle) {
                let throw_velocity = to_nova_vec3(direction * settings.grab_spring_strength);
                body.linear_velocity = throw_velocity;
                body.wake_up();
                log_tool(&mut console, "Gravity Gun", "threw object", time.elapsed_secs());
            }
            grab_state.holding = None;
            grab_state.holding_collider = None;
        }
    }
}

fn update_grabbed_object(
    grab_state: Res<GrabState>,
    mut nova: ResMut<NovaWorld>,
    camera_query: Query<(&GlobalTransform, &Camera), With<PlayerCamera>>,
    settings: Res<ToolSettings>,
) {
    let Some(handle) = grab_state.holding else {
        return;
    };

    let Some((origin, direction)) = get_camera_ray(&camera_query) else {
        return;
    };

    let target = origin + direction * grab_state.distance;

    if let Some(body) = nova.world.get_body_mut(handle) {
        let current_pos = to_bevy_vec3(body.position);
        let diff = target - current_pos;

        body.linear_velocity = to_nova_vec3(diff * settings.grab_spring_strength);
        body.angular_velocity = body.angular_velocity * 0.9;
        body.wake_up();
    }
}

fn force_gun_system(
    mouse: Res<ButtonInput<MouseButton>>,
    mut nova: ResMut<NovaWorld>,
    camera_query: Query<(&GlobalTransform, &Camera), With<PlayerCamera>>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
    settings: Res<ToolSettings>,
) {
    if hotbar.get_item(selected.index) != Some(&HotbarItem::ForceGun) {
        return;
    }

    let Some((origin, direction)) = get_camera_ray(&camera_query) else {
        return;
    };

    let push = mouse.pressed(MouseButton::Left);
    let pull = mouse.pressed(MouseButton::Right);

    if !push && !pull {
        return;
    }

    if let Some((body_handle, _, _, _)) = raycast_nova(&nova, origin, direction, 100.0) {
        if let Some(body) = nova.world.get_body_mut(body_handle) {
            if body.body_type == RigidBodyType::Dynamic {
                let force_dir = if push { direction } else { -direction };
                let force = to_nova_vec3(force_dir * settings.force_gun_strength);
                body.apply_force(force);
                body.wake_up();
            }
        }
    }
}

fn explosion_system(
    mouse: Res<ButtonInput<MouseButton>>,
    mut nova: ResMut<NovaWorld>,
    camera_query: Query<(&GlobalTransform, &Camera), With<PlayerCamera>>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
    settings: Res<ToolSettings>,
    mut stats: ResMut<PlaygroundStats>,
    mut console: ResMut<PhysicsConsole>,
    time: Res<Time>,
) {
    if hotbar.get_item(selected.index) != Some(&HotbarItem::Explosion) {
        return;
    }

    let Some((origin, direction)) = get_camera_ray(&camera_query) else {
        return;
    };

    let small = mouse.just_pressed(MouseButton::Left);
    let large = mouse.just_pressed(MouseButton::Right);

    if !small && !large {
        return;
    }

    stats.total_explosions += 1;

    let explosion_center =
        if let Some((_, _, _, hit_point)) = raycast_nova(&nova, origin, direction, 100.0) {
            hit_point
        } else {
            origin + direction * 20.0
        };

    let radius = if large {
        settings.explosion_radius * 2.0
    } else {
        settings.explosion_radius
    };
    let force = if large {
        settings.explosion_force * 2.0
    } else {
        settings.explosion_force
    };

    let nova_center = to_nova_vec3(explosion_center);

    let affected_bodies: Vec<RigidBodyHandle> = nova
        .world
        .bodies()
        .filter_map(|(handle, body)| {
            if body.body_type == RigidBodyType::Dynamic {
                let dist = (body.position - nova_center).length();
                if dist < radius {
                    Some(handle)
                } else {
                    None
                }
            } else {
                None
            }
        })
        .collect();

    let affected_count = affected_bodies.len();

    for handle in affected_bodies {
        if let Some(body) = nova.world.get_body_mut(handle) {
            let diff = body.position - nova_center;
            let dist = diff.length();
            if dist > 0.01 {
                let falloff = 1.0 - (dist / radius);
                let impulse = diff.normalize() * force * falloff;
                body.apply_impulse(impulse);
                body.wake_up();
            }
        }
    }

    let size = if large { "LARGE" } else { "small" };
    log_tool(
        &mut console,
        "Explosion",
        &format!("{} explosion at ({:.1}, {:.1}, {:.1}), {} bodies affected",
            size, explosion_center.x, explosion_center.y, explosion_center.z, affected_count),
        time.elapsed_secs(),
    );
}

fn delete_system(
    mouse: Res<ButtonInput<MouseButton>>,
    mut nova: ResMut<NovaWorld>,
    mut commands: Commands,
    camera_query: Query<(&GlobalTransform, &Camera), With<PlayerCamera>>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
    handle_to_entity: Res<HandleToEntity>,
    mut stats: ResMut<PlaygroundStats>,
    mut console: ResMut<PhysicsConsole>,
    time: Res<Time>,
) {
    if hotbar.get_item(selected.index) != Some(&HotbarItem::Delete) {
        return;
    }

    let Some((origin, direction)) = get_camera_ray(&camera_query) else {
        return;
    };

    let single = mouse.just_pressed(MouseButton::Left);
    let area = mouse.just_pressed(MouseButton::Right);

    if !single && !area {
        return;
    }

    if let Some((body_handle, _, _, hit_point)) = raycast_nova(&nova, origin, direction, 100.0) {
        if single {
            // Only delete dynamic bodies, not static world geometry
            if let Some(body) = nova.world.get_body(body_handle) {
                if body.body_type == RigidBodyType::Dynamic {
                    nova.world.remove_body(body_handle);
                    if let Some(&entity) = handle_to_entity.bodies.get(&body_handle) {
                        commands.entity(entity).despawn_recursive();
                    }
                    stats.total_deleted += 1;
                    log_delete(&mut console, 1, time.elapsed_secs());
                }
            }
        } else {
            let delete_radius = 5.0;
            let nova_center = to_nova_vec3(hit_point);

            let to_delete: Vec<RigidBodyHandle> = nova
                .world
                .bodies()
                .filter_map(|(handle, body)| {
                    if body.body_type == RigidBodyType::Dynamic {
                        let dist = (body.position - nova_center).length();
                        if dist < delete_radius {
                            Some(handle)
                        } else {
                            None
                        }
                    } else {
                        None
                    }
                })
                .collect();

            let delete_count = to_delete.len() as u32;

            for handle in to_delete {
                nova.world.remove_body(handle);
                if let Some(&entity) = handle_to_entity.bodies.get(&handle) {
                    commands.entity(entity).despawn_recursive();
                }
                stats.total_deleted += 1;
            }

            if delete_count > 0 {
                log_delete(&mut console, delete_count, time.elapsed_secs());
            }
        }
    }
}

fn joint_tool_system(
    mouse: Res<ButtonInput<MouseButton>>,
    mut nova: ResMut<NovaWorld>,
    camera_query: Query<(&GlobalTransform, &Camera), With<PlayerCamera>>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
    mut joint_state: ResMut<JointToolState>,
    mut stats: ResMut<PlaygroundStats>,
    mut console: ResMut<PhysicsConsole>,
    time: Res<Time>,
) {
    if hotbar.get_item(selected.index) != Some(&HotbarItem::JointTool) {
        return;
    }

    let Some((origin, direction)) = get_camera_ray(&camera_query) else {
        return;
    };

    if mouse.just_pressed(MouseButton::Right) {
        joint_state.first_body = None;
        return;
    }

    if mouse.just_pressed(MouseButton::Left) {
        if let Some((body_handle, _, _, hit_point)) = raycast_nova(&nova, origin, direction, 100.0)
        {
            if let Some(body) = nova.world.get_body(body_handle) {
                let local_anchor = hit_point - to_bevy_vec3(body.position);

                if let Some((first_handle, first_anchor)) = joint_state.first_body {
                    if first_handle != body_handle {
                        let anchor_a = to_nova_vec3(first_anchor);
                        let anchor_b = to_nova_vec3(local_anchor);

                        match joint_state.joint_type {
                            JointType::Ball => {
                                nova.world
                                    .create_ball_joint(first_handle, body_handle, anchor_a, anchor_b);
                            }
                            JointType::Hinge => {
                                nova.world.create_hinge_joint(
                                    first_handle,
                                    body_handle,
                                    anchor_a,
                                    anchor_b,
                                    nova::prelude::Vec3::Y,
                                );
                            }
                            JointType::Fixed => {
                                nova.world
                                    .create_fixed_joint(first_handle, body_handle, anchor_a, anchor_b);
                            }
                            JointType::Distance | JointType::Rope => {
                                let dist = (to_bevy_vec3(body.position) + local_anchor
                                    - (to_bevy_vec3(
                                        nova.world.get_body(first_handle).unwrap().position,
                                    ) + first_anchor))
                                    .length();
                                nova.world.create_distance_joint(
                                    first_handle,
                                    body_handle,
                                    anchor_a,
                                    anchor_b,
                                    dist,
                                );
                            }
                            JointType::Prismatic => {
                                let joint_data =
                                    JointData::Prismatic(PrismaticJoint::new(nova::prelude::Vec3::Y));
                                let joint = Joint::new(first_handle, body_handle, joint_data)
                                    .with_anchors(anchor_a, anchor_b);
                                nova.world.insert_joint(joint);
                            }
                            JointType::Spring => {
                                let dist = (to_bevy_vec3(body.position) + local_anchor
                                    - (to_bevy_vec3(
                                        nova.world.get_body(first_handle).unwrap().position,
                                    ) + first_anchor))
                                    .length();
                                let spring_joint =
                                    DistanceJoint::new(dist).as_spring(100.0, 5.0);
                                let joint_data = JointData::Distance(spring_joint);
                                let joint = Joint::new(first_handle, body_handle, joint_data)
                                    .with_anchors(anchor_a, anchor_b);
                                nova.world.insert_joint(joint);
                            }
                        }
                        stats.joints_created += 1;
                        log_joint(&mut console, joint_state.joint_type.name(), time.elapsed_secs());
                    }
                    joint_state.first_body = None;
                } else {
                    joint_state.first_body = Some((body_handle, local_anchor));
                }
            }
        }
    }
}

fn freeze_system(
    mouse: Res<ButtonInput<MouseButton>>,
    mut nova: ResMut<NovaWorld>,
    mut commands: Commands,
    camera_query: Query<(&GlobalTransform, &Camera), With<PlayerCamera>>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
    handle_to_entity: Res<HandleToEntity>,
    frozen_query: Query<Entity, With<FrozenBody>>,
    keyboard: Res<ButtonInput<KeyCode>>,
) {
    if hotbar.get_item(selected.index) != Some(&HotbarItem::Freeze) {
        return;
    }

    let Some((origin, direction)) = get_camera_ray(&camera_query) else {
        return;
    };

    // Right click unfreezes all
    if mouse.just_pressed(MouseButton::Right) {
        for entity in frozen_query.iter() {
            commands.entity(entity).remove::<FrozenBody>();
        }
        // Unfreeze all bodies in Nova
        let handles: Vec<_> = nova.world.bodies().map(|(h, _)| h).collect();
        for handle in handles {
            if let Some(body) = nova.world.get_body_mut(handle) {
                if body.body_type == RigidBodyType::Static {
                    // Only unfreeze if it was dynamic before (check user_data or something)
                    // For simplicity, we'll skip this
                }
            }
        }
        return;
    }

    if mouse.just_pressed(MouseButton::Left) {
        if let Some((body_handle, _, _, _)) = raycast_nova(&nova, origin, direction, 100.0) {
            if let Some(body) = nova.world.get_body_mut(body_handle) {
                if body.body_type == RigidBodyType::Dynamic {
                    // Freeze: set velocity to zero
                    body.linear_velocity = nova::prelude::Vec3::ZERO;
                    body.angular_velocity = nova::prelude::Vec3::ZERO;

                    if let Some(&entity) = handle_to_entity.bodies.get(&body_handle) {
                        commands.entity(entity).insert(FrozenBody);
                    }
                }
            }
        }
    }
}

fn clone_system(
    mouse: Res<ButtonInput<MouseButton>>,
    mut nova: ResMut<NovaWorld>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    camera_query: Query<(&GlobalTransform, &Camera), With<PlayerCamera>>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
    mut handle_to_entity: ResMut<HandleToEntity>,
    mut stats: ResMut<PlaygroundStats>,
) {
    if hotbar.get_item(selected.index) != Some(&HotbarItem::Clone) {
        return;
    }

    let Some((origin, direction)) = get_camera_ray(&camera_query) else {
        return;
    };

    if mouse.just_pressed(MouseButton::Left) {
        if let Some((body_handle, _, _, hit_point)) = raycast_nova(&nova, origin, direction, 100.0)
        {
            if let Some(body) = nova.world.get_body(body_handle) {
                if body.body_type == RigidBodyType::Dynamic {
                    // Clone at a slight offset
                    let clone_pos = to_bevy_vec3(body.position) + Vec3::new(1.0, 0.5, 0.0);

                    // Create a simple sphere clone for now
                    spawn_sphere(
                        &mut nova,
                        &mut commands,
                        &mut meshes,
                        &mut materials,
                        &mut handle_to_entity,
                        clone_pos,
                        0.5,
                        Color::srgb(0.8, 0.4, 0.8), // Purple for clones
                        PhysicsMaterialType::Normal,
                    );
                    stats.total_spawned += 1;
                }
            }
        }
    }
}

fn magnet_system(
    mouse: Res<ButtonInput<MouseButton>>,
    mut nova: ResMut<NovaWorld>,
    camera_query: Query<(&GlobalTransform, &Camera), With<PlayerCamera>>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
    mut magnet: ResMut<MagnetState>,
) {
    if hotbar.get_item(selected.index) != Some(&HotbarItem::Magnet) {
        return;
    }

    let Some((origin, direction)) = get_camera_ray(&camera_query) else {
        return;
    };

    // Right click toggles attract/repel
    if mouse.just_pressed(MouseButton::Right) {
        magnet.attract = !magnet.attract;
    }

    // Left click applies magnetic force
    if mouse.pressed(MouseButton::Left) {
        let magnet_pos = origin + direction * 5.0;
        let nova_magnet = to_nova_vec3(magnet_pos);

        let affected: Vec<RigidBodyHandle> = nova
            .world
            .bodies()
            .filter_map(|(handle, body)| {
                if body.body_type == RigidBodyType::Dynamic {
                    let dist = (body.position - nova_magnet).length();
                    if dist < magnet.radius {
                        Some(handle)
                    } else {
                        None
                    }
                } else {
                    None
                }
            })
            .collect();

        for handle in affected {
            if let Some(body) = nova.world.get_body_mut(handle) {
                let diff = nova_magnet - body.position;
                let dist = diff.length();
                if dist > 0.1 {
                    let strength = magnet.strength * (1.0 - dist / magnet.radius);
                    let dir = if magnet.attract {
                        diff.normalize()
                    } else {
                        -diff.normalize()
                    };
                    body.apply_force(dir * strength);
                    body.wake_up();
                }
            }
        }
    }
}

fn launch_cannon_system(
    mouse: Res<ButtonInput<MouseButton>>,
    mut nova: ResMut<NovaWorld>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    camera_query: Query<(&GlobalTransform, &Camera), With<PlayerCamera>>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
    mut cannon: ResMut<CannonState>,
    mut handle_to_entity: ResMut<HandleToEntity>,
    time: Res<Time>,
    mut stats: ResMut<PlaygroundStats>,
) {
    if hotbar.get_item(selected.index) != Some(&HotbarItem::LaunchCannon) {
        return;
    }

    let Some((origin, direction)) = get_camera_ray(&camera_query) else {
        return;
    };

    // Right click cycles projectile type
    if mouse.just_pressed(MouseButton::Right) {
        cannon.projectile_type = cannon.projectile_type.cycle_next();
    }

    // Left click or auto-fire
    let should_fire = mouse.just_pressed(MouseButton::Left)
        || (cannon.auto_fire
            && mouse.pressed(MouseButton::Left)
            && time.elapsed_secs() - cannon.last_fire > cannon.fire_rate);

    if should_fire {
        cannon.last_fire = time.elapsed_secs();
        let spawn_pos = origin + direction * 2.0;

        let mut rng = rand::thread_rng();

        match cannon.projectile_type {
            ProjectileType::Ball => {
                let (_, handle) = spawn_sphere(
                    &mut nova,
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    &mut handle_to_entity,
                    spawn_pos,
                    0.3,
                    Color::srgb(1.0, 0.8, 0.2),
                    PhysicsMaterialType::Normal,
                );
                if let Some(body) = nova.world.get_body_mut(handle) {
                    body.linear_velocity = to_nova_vec3(direction * cannon.power);
                }
            }
            ProjectileType::Box => {
                let (_, handle) = spawn_box(
                    &mut nova,
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    &mut handle_to_entity,
                    spawn_pos,
                    Vec3::splat(0.25),
                    Color::srgb(0.8, 0.4, 0.2),
                    PhysicsMaterialType::Normal,
                );
                if let Some(body) = nova.world.get_body_mut(handle) {
                    body.linear_velocity = to_nova_vec3(direction * cannon.power);
                }
            }
            ProjectileType::Cluster => {
                for i in 0..5 {
                    let offset = Vec3::new(
                        rng.gen_range(-0.3..0.3),
                        rng.gen_range(-0.3..0.3),
                        rng.gen_range(-0.3..0.3),
                    );
                    let (_, handle) = spawn_sphere(
                        &mut nova,
                        &mut commands,
                        &mut meshes,
                        &mut materials,
                        &mut handle_to_entity,
                        spawn_pos + offset,
                        0.15,
                        Color::srgb(1.0, 0.5, 0.0),
                        PhysicsMaterialType::Normal,
                    );
                    if let Some(body) = nova.world.get_body_mut(handle) {
                        body.linear_velocity = to_nova_vec3(direction * cannon.power);
                    }
                }
            }
            ProjectileType::Heavy => {
                let (_, handle) = spawn_sphere(
                    &mut nova,
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    &mut handle_to_entity,
                    spawn_pos,
                    0.5,
                    Color::srgb(0.3, 0.3, 0.3),
                    PhysicsMaterialType::Heavy,
                );
                if let Some(body) = nova.world.get_body_mut(handle) {
                    body.linear_velocity = to_nova_vec3(direction * cannon.power * 0.5);
                }
            }
            ProjectileType::Bouncy => {
                let (_, handle) = spawn_sphere(
                    &mut nova,
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    &mut handle_to_entity,
                    spawn_pos,
                    0.3,
                    Color::srgb(0.2, 1.0, 0.5),
                    PhysicsMaterialType::Bouncy,
                );
                if let Some(body) = nova.world.get_body_mut(handle) {
                    body.linear_velocity = to_nova_vec3(direction * cannon.power);
                }
            }
            ProjectileType::Capsule => {
                // Just use a sphere for simplicity
                let (_, handle) = spawn_sphere(
                    &mut nova,
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                    &mut handle_to_entity,
                    spawn_pos,
                    0.3,
                    Color::srgb(0.6, 0.3, 0.8),
                    PhysicsMaterialType::Normal,
                );
                if let Some(body) = nova.world.get_body_mut(handle) {
                    body.linear_velocity = to_nova_vec3(direction * cannon.power);
                }
            }
        }
        stats.total_spawned += 1;
    }
}

fn painter_system(
    mouse: Res<ButtonInput<MouseButton>>,
    camera_query: Query<(&GlobalTransform, &Camera), With<PlayerCamera>>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
    mut painter: ResMut<PainterState>,
    nova: Res<NovaWorld>,
    handle_to_entity: Res<HandleToEntity>,
    mut materials: Query<&mut MeshMaterial3d<StandardMaterial>>,
    mut material_assets: ResMut<Assets<StandardMaterial>>,
) {
    if hotbar.get_item(selected.index) != Some(&HotbarItem::Painter) {
        return;
    }

    let Some((origin, direction)) = get_camera_ray(&camera_query) else {
        return;
    };

    // Right click cycles color
    if mouse.just_pressed(MouseButton::Right) {
        painter.color_index = (painter.color_index + 1) % PAINT_COLORS.len();
    }

    // Left click paints
    if mouse.just_pressed(MouseButton::Left) {
        if let Some((body_handle, _, _, _)) = raycast_nova(&nova, origin, direction, 100.0) {
            if let Some(&entity) = handle_to_entity.bodies.get(&body_handle) {
                if let Ok(mat_handle) = materials.get_mut(entity) {
                    if let Some(mat) = material_assets.get_mut(&mat_handle.0) {
                        mat.base_color = PAINT_COLORS[painter.color_index];
                    }
                }
            }
        }
    }
}

fn tool_scroll_adjust(
    mut scroll_events: EventReader<bevy::input::mouse::MouseWheel>,
    mut grab_state: ResMut<GrabState>,
    mut cannon: ResMut<CannonState>,
    mut magnet: ResMut<MagnetState>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
) {
    for event in scroll_events.read() {
        let delta = event.y;

        match hotbar.get_item(selected.index) {
            Some(HotbarItem::GravityGun) => {
                if grab_state.holding.is_some() {
                    grab_state.distance = (grab_state.distance + delta * 0.5).clamp(2.0, 50.0);
                }
            }
            Some(HotbarItem::LaunchCannon) => {
                cannon.power = (cannon.power + delta * 5.0).clamp(10.0, 200.0);
            }
            Some(HotbarItem::Magnet) => {
                magnet.radius = (magnet.radius + delta * 1.0).clamp(2.0, 30.0);
            }
            _ => {}
        }
    }
}

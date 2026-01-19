//! Pre-built physics structures - MEGA EDITION!
//! Tower, Pyramid, Ragdoll, Newton's Cradle, Car, Ferris Wheel, AND SO MUCH MORE!

use bevy::prelude::*;
use nova::prelude::*;
use rand::Rng;

use crate::components::{PhysicsMaterialType, SpawnedObject};
use crate::convert::{to_nova_quat, to_nova_vec3};
use crate::plugins::spawning::{spawn_box, spawn_sphere};
use crate::resources::{HandleToEntity, NovaWorld};

/// Spawn a tower of stacked boxes
pub fn spawn_tower(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    base_position: Vec3,
    height: usize,
) {
    let box_size = Vec3::new(0.8, 0.4, 0.8);

    for i in 0..height {
        let pos = base_position + Vec3::new(0.0, i as f32 * box_size.y * 2.0 + box_size.y, 0.0);
        let color = Color::hsl(i as f32 * 30.0 % 360.0, 0.7, 0.5);

        spawn_box(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            pos,
            box_size * 0.5,
            color,
            PhysicsMaterialType::Normal,
        );
    }
}

/// Spawn a pyramid of boxes
pub fn spawn_pyramid(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    base_position: Vec3,
    base_size: usize,
) {
    let box_size = Vec3::splat(0.5);

    for layer in 0..base_size {
        let row_size = base_size - layer;
        let y = base_position.y + layer as f32 * box_size.y * 2.0 + box_size.y;
        let offset = (row_size as f32 - 1.0) * box_size.x;

        for i in 0..row_size {
            for j in 0..row_size {
                let x = base_position.x + i as f32 * box_size.x * 2.0 - offset;
                let z = base_position.z + j as f32 * box_size.z * 2.0 - offset;

                let color = Color::hsl((layer * 40 + i * 20 + j * 10) as f32 % 360.0, 0.6, 0.55);

                spawn_box(
                    nova,
                    commands,
                    meshes,
                    materials,
                    handle_to_entity,
                    Vec3::new(x, y, z),
                    box_size * 0.5,
                    color,
                    PhysicsMaterialType::Normal,
                );
            }
        }
    }
}

/// Spawn Newton's Cradle
pub fn spawn_newtons_cradle(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    ball_count: usize,
) {
    let ball_radius = 0.3;
    let string_length = 2.0;
    let spacing = ball_radius * 2.05;

    let anchor_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(position + Vec3::new(0.0, string_length, 0.0)))
        .build();
    let anchor_handle = nova.world.insert_body(anchor_body);

    for i in 0..ball_count {
        let x_offset = (i as f32 - (ball_count - 1) as f32 / 2.0) * spacing;
        let ball_pos = position + Vec3::new(x_offset, 0.0, 0.0);

        let actual_pos = if i == 0 {
            ball_pos + Vec3::new(-string_length * 0.7, string_length * 0.3, 0.0)
        } else {
            ball_pos
        };

        let (_, ball_handle) = spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            actual_pos,
            ball_radius,
            Color::srgb(0.8, 0.8, 0.85),
            PhysicsMaterialType::Bouncy,
        );

        let anchor_offset = nova::prelude::Vec3::new(x_offset, 0.0, 0.0);
        nova.world.create_distance_joint(
            anchor_handle,
            ball_handle,
            anchor_offset,
            nova::prelude::Vec3::ZERO,
            string_length,
        );
    }
}

/// Spawn a wrecking ball
pub fn spawn_wrecking_ball(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    anchor_position: Vec3,
    chain_length: usize,
) {
    let link_spacing = 0.4;
    let ball_radius = 1.0;

    let anchor_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(anchor_position))
        .build();
    let anchor_handle = nova.world.insert_body(anchor_body);

    let mut prev_handle = anchor_handle;

    for i in 0..chain_length {
        let link_pos = anchor_position - Vec3::new(0.0, (i + 1) as f32 * link_spacing, 0.0);

        let (_, link_handle) = spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            link_pos,
            0.1,
            Color::srgb(0.3, 0.3, 0.3),
            PhysicsMaterialType::Normal,
        );

        nova.world.create_ball_joint(
            prev_handle,
            link_handle,
            nova::prelude::Vec3::new(0.0, -link_spacing / 2.0, 0.0),
            nova::prelude::Vec3::new(0.0, link_spacing / 2.0, 0.0),
        );

        prev_handle = link_handle;
    }

    let ball_pos = anchor_position
        - Vec3::new(0.0, chain_length as f32 * link_spacing + ball_radius, 0.0);

    let (_, ball_handle) = spawn_sphere(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        ball_pos,
        ball_radius,
        Color::srgb(0.2, 0.2, 0.25),
        PhysicsMaterialType::Heavy,
    );

    nova.world.create_ball_joint(
        prev_handle,
        ball_handle,
        nova::prelude::Vec3::new(0.0, -link_spacing / 2.0, 0.0),
        nova::prelude::Vec3::new(0.0, ball_radius, 0.0),
    );
}

/// Spawn dominos
pub fn spawn_dominos(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    start_position: Vec3,
    count: usize,
    direction: Vec3,
) {
    let domino_size = Vec3::new(0.1, 0.5, 0.3);
    let spacing = 0.35;

    let dir = direction.normalize();

    for i in 0..count {
        let pos = start_position + dir * (i as f32 * spacing);
        let rotation = Quat::from_rotation_y(dir.x.atan2(dir.z));

        let tilt = if i == 0 {
            Quat::from_rotation_z(0.1)
        } else {
            Quat::IDENTITY
        };

        let color = Color::hsl(i as f32 * 15.0 % 360.0, 0.7, 0.5);

        let nova_pos = to_nova_vec3(pos + Vec3::new(0.0, domino_size.y, 0.0));

        let body = nova
            .world
            .create_body()
            .body_type(RigidBodyType::Dynamic)
            .position(nova_pos)
            .rotation(to_nova_quat(rotation * tilt))
            .mass(0.5)
            .build();
        let body_handle = nova.world.insert_body(body);

        let collider = nova
            .world
            .create_collider(
                body_handle,
                CollisionShape::Box(BoxShape::new(to_nova_vec3(domino_size * 0.5))),
            )
            .friction(0.4)
            .restitution(0.1)
            .build();
        let collider_handle = nova.world.insert_collider(collider);

        let entity = commands
            .spawn((
                Mesh3d(meshes.add(Cuboid::new(domino_size.x, domino_size.y, domino_size.z))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: color,
                    ..default()
                })),
                Transform::from_translation(pos + Vec3::new(0.0, domino_size.y, 0.0))
                    .with_rotation(rotation * tilt),
                crate::components::PhysicsBody {
                    handle: body_handle,
                },
                crate::components::PhysicsCollider {
                    handle: collider_handle,
                },
                crate::components::DynamicBody,
                SpawnedObject,
            ))
            .id();

        handle_to_entity.bodies.insert(body_handle, entity);
    }
}

/// Spawn a simple ragdoll
pub fn spawn_ragdoll(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let skin_color = Color::srgb(0.9, 0.75, 0.65);

    let torso_size = Vec3::new(0.4, 0.6, 0.2);
    let head_radius = 0.2;
    let limb_radius = 0.08;
    let upper_arm_length = 0.3;
    let lower_arm_length = 0.25;
    let upper_leg_length = 0.35;
    let lower_leg_length = 0.3;

    // Torso
    let torso_pos = position;
    let (_, torso_handle) = spawn_box(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        torso_pos,
        torso_size * 0.5,
        skin_color,
        PhysicsMaterialType::Normal,
    );

    // Head
    let head_pos = torso_pos + Vec3::new(0.0, torso_size.y / 2.0 + head_radius + 0.05, 0.0);
    let (_, head_handle) = spawn_sphere(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        head_pos,
        head_radius,
        skin_color,
        PhysicsMaterialType::Normal,
    );

    nova.world.create_ball_joint(
        torso_handle,
        head_handle,
        nova::prelude::Vec3::new(0.0, torso_size.y / 2.0, 0.0),
        nova::prelude::Vec3::new(0.0, -head_radius, 0.0),
    );

    // Arms
    for side in [-1.0_f32, 1.0] {
        let shoulder_pos = torso_pos
            + Vec3::new(
                side * (torso_size.x / 2.0 + upper_arm_length / 2.0),
                torso_size.y / 2.0 - 0.1,
                0.0,
            );
        let (_, upper_arm_handle) = spawn_box(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            shoulder_pos,
            Vec3::new(upper_arm_length / 2.0, limb_radius, limb_radius),
            skin_color,
            PhysicsMaterialType::Normal,
        );

        nova.world.create_ball_joint(
            torso_handle,
            upper_arm_handle,
            nova::prelude::Vec3::new(
                side * torso_size.x / 2.0,
                torso_size.y / 2.0 - 0.1,
                0.0,
            ),
            nova::prelude::Vec3::new(-side * upper_arm_length / 2.0, 0.0, 0.0),
        );

        let elbow_pos = shoulder_pos
            + Vec3::new(
                side * (upper_arm_length / 2.0 + lower_arm_length / 2.0),
                0.0,
                0.0,
            );
        let (_, lower_arm_handle) = spawn_box(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            elbow_pos,
            Vec3::new(lower_arm_length / 2.0, limb_radius, limb_radius),
            skin_color,
            PhysicsMaterialType::Normal,
        );

        nova.world.create_hinge_joint(
            upper_arm_handle,
            lower_arm_handle,
            nova::prelude::Vec3::new(side * upper_arm_length / 2.0, 0.0, 0.0),
            nova::prelude::Vec3::new(-side * lower_arm_length / 2.0, 0.0, 0.0),
            nova::prelude::Vec3::Z,
        );
    }

    // Legs
    for side in [-1.0_f32, 1.0] {
        let hip_pos = torso_pos
            + Vec3::new(
                side * (torso_size.x / 4.0),
                -torso_size.y / 2.0 - upper_leg_length / 2.0,
                0.0,
            );
        let (_, upper_leg_handle) = spawn_box(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            hip_pos,
            Vec3::new(limb_radius, upper_leg_length / 2.0, limb_radius),
            skin_color,
            PhysicsMaterialType::Normal,
        );

        nova.world.create_ball_joint(
            torso_handle,
            upper_leg_handle,
            nova::prelude::Vec3::new(side * torso_size.x / 4.0, -torso_size.y / 2.0, 0.0),
            nova::prelude::Vec3::new(0.0, upper_leg_length / 2.0, 0.0),
        );

        let knee_pos = hip_pos
            + Vec3::new(
                0.0,
                -(upper_leg_length / 2.0 + lower_leg_length / 2.0),
                0.0,
            );
        let (_, lower_leg_handle) = spawn_box(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            knee_pos,
            Vec3::new(limb_radius, lower_leg_length / 2.0, limb_radius),
            skin_color,
            PhysicsMaterialType::Normal,
        );

        nova.world.create_hinge_joint(
            upper_leg_handle,
            lower_leg_handle,
            nova::prelude::Vec3::new(0.0, -upper_leg_length / 2.0, 0.0),
            nova::prelude::Vec3::new(0.0, lower_leg_length / 2.0, 0.0),
            nova::prelude::Vec3::X,
        );
    }
}

/// Spawn a plank bridge
pub fn spawn_bridge(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    start_position: Vec3,
    plank_count: usize,
) {
    let plank_size = Vec3::new(1.0, 0.1, 0.6);
    let plank_spacing = 0.1;

    let anchor_a_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(start_position))
        .build();
    let anchor_a = nova.world.insert_body(anchor_a_body);

    let end_position = start_position
        + Vec3::new(
            (plank_count as f32 + 1.0) * (plank_size.x + plank_spacing),
            0.0,
            0.0,
        );
    let anchor_b_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(end_position))
        .build();
    let anchor_b = nova.world.insert_body(anchor_b_body);

    let mut prev_handle = anchor_a;

    for i in 0..plank_count {
        let pos = start_position
            + Vec3::new(
                (i as f32 + 0.5) * (plank_size.x + plank_spacing),
                0.0,
                0.0,
            );
        let color = Color::srgb(0.6, 0.4, 0.2);

        let (_, plank_handle) = spawn_box(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            pos,
            plank_size * 0.5,
            color,
            PhysicsMaterialType::Normal,
        );

        nova.world.create_hinge_joint(
            prev_handle,
            plank_handle,
            nova::prelude::Vec3::new(plank_size.x / 2.0 + plank_spacing / 2.0, 0.0, 0.0),
            nova::prelude::Vec3::new(-plank_size.x / 2.0, 0.0, 0.0),
            nova::prelude::Vec3::Z,
        );

        prev_handle = plank_handle;
    }

    nova.world.create_hinge_joint(
        prev_handle,
        anchor_b,
        nova::prelude::Vec3::new(plank_size.x / 2.0, 0.0, 0.0),
        nova::prelude::Vec3::new(-plank_spacing / 2.0, 0.0, 0.0),
        nova::prelude::Vec3::Z,
    );
}

/// Spawn a catapult
pub fn spawn_catapult(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let base_size = Vec3::new(1.5, 0.2, 1.0);
    let arm_size = Vec3::new(2.0, 0.15, 0.2);
    let bucket_size = Vec3::new(0.4, 0.3, 0.4);

    // Base (static)
    let base_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(position))
        .build();
    let base_handle = nova.world.insert_body(base_body);

    nova.world.insert_collider(
        nova.world
            .create_collider(
                base_handle,
                CollisionShape::Box(BoxShape::new(to_nova_vec3(base_size * 0.5))),
            )
            .friction(0.8)
            .build(),
    );

    // Spawn visual for base
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(base_size.x, base_size.y, base_size.z))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.4, 0.3, 0.2),
            ..default()
        })),
        Transform::from_translation(position),
        SpawnedObject,
    ));

    // Arm (dynamic, attached with hinge)
    let arm_pos = position + Vec3::new(0.0, base_size.y / 2.0 + arm_size.y / 2.0 + 0.1, 0.0);
    let (_, arm_handle) = spawn_box(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        arm_pos,
        arm_size * 0.5,
        Color::srgb(0.5, 0.35, 0.2),
        PhysicsMaterialType::Normal,
    );

    // Hinge at the center of the arm
    nova.world.create_hinge_joint(
        base_handle,
        arm_handle,
        nova::prelude::Vec3::new(0.0, base_size.y / 2.0 + 0.1, 0.0),
        nova::prelude::Vec3::ZERO,
        nova::prelude::Vec3::Z,
    );

    // Bucket at the end of the arm
    let bucket_pos = arm_pos + Vec3::new(arm_size.x / 2.0, bucket_size.y / 2.0, 0.0);
    let (_, bucket_handle) = spawn_box(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        bucket_pos,
        bucket_size * 0.5,
        Color::srgb(0.6, 0.5, 0.3),
        PhysicsMaterialType::Normal,
    );

    nova.world.create_fixed_joint(
        arm_handle,
        bucket_handle,
        nova::prelude::Vec3::new(arm_size.x / 2.0, arm_size.y / 2.0, 0.0),
        nova::prelude::Vec3::new(0.0, -bucket_size.y / 2.0, 0.0),
    );

    // Add a projectile in the bucket
    spawn_sphere(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        bucket_pos + Vec3::new(0.0, bucket_size.y / 2.0 + 0.15, 0.0),
        0.15,
        Color::srgb(0.8, 0.2, 0.2),
        PhysicsMaterialType::Heavy,
    );
}

/// Spawn a pendulum wall
pub fn spawn_pendulum_wall(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    anchor_position: Vec3,
    cols: usize,
    rows: usize,
) {
    let ball_radius = 0.25;
    let string_length = 2.0;
    let spacing_x = ball_radius * 2.5;
    let spacing_z = ball_radius * 2.5;

    for row in 0..rows {
        for col in 0..cols {
            let x_offset = (col as f32 - (cols - 1) as f32 / 2.0) * spacing_x;
            let z_offset = (row as f32 - (rows - 1) as f32 / 2.0) * spacing_z;

            let anchor_body = nova
                .world
                .create_body()
                .body_type(RigidBodyType::Static)
                .position(to_nova_vec3(
                    anchor_position + Vec3::new(x_offset, 0.0, z_offset),
                ))
                .build();
            let anchor_handle = nova.world.insert_body(anchor_body);

            let ball_pos =
                anchor_position + Vec3::new(x_offset, -string_length, z_offset);

            let color = Color::hsl((col * 50 + row * 30) as f32 % 360.0, 0.7, 0.5);

            let (_, ball_handle) = spawn_sphere(
                nova,
                commands,
                meshes,
                materials,
                handle_to_entity,
                ball_pos,
                ball_radius,
                color,
                PhysicsMaterialType::Normal,
            );

            nova.world.create_distance_joint(
                anchor_handle,
                ball_handle,
                nova::prelude::Vec3::ZERO,
                nova::prelude::Vec3::ZERO,
                string_length,
            );
        }
    }
}

/// Spawn a simple car
pub fn spawn_car(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let body_size = Vec3::new(2.0, 0.5, 1.0);
    let wheel_radius = 0.3;
    let wheel_width = 0.2;
    let wheel_offset_x = body_size.x / 2.0 - 0.3;
    let wheel_offset_z = body_size.z / 2.0 + wheel_width / 2.0;
    let wheel_offset_y = -body_size.y / 2.0;

    // Car body
    let (_, body_handle) = spawn_box(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        position,
        body_size * 0.5,
        Color::srgb(0.8, 0.2, 0.2),
        PhysicsMaterialType::Normal,
    );

    // Wheels
    let wheel_positions = [
        Vec3::new(wheel_offset_x, wheel_offset_y, wheel_offset_z),
        Vec3::new(wheel_offset_x, wheel_offset_y, -wheel_offset_z),
        Vec3::new(-wheel_offset_x, wheel_offset_y, wheel_offset_z),
        Vec3::new(-wheel_offset_x, wheel_offset_y, -wheel_offset_z),
    ];

    for wheel_pos in wheel_positions {
        let world_wheel_pos = position + wheel_pos;

        let (_, wheel_handle) = spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            world_wheel_pos,
            wheel_radius,
            Color::srgb(0.2, 0.2, 0.2),
            PhysicsMaterialType::Rubber,
        );

        // Connect wheel to body with hinge
        nova.world.create_hinge_joint(
            body_handle,
            wheel_handle,
            to_nova_vec3(wheel_pos),
            nova::prelude::Vec3::ZERO,
            nova::prelude::Vec3::Z,
        );
    }
}

/// Spawn a Ferris wheel
pub fn spawn_ferris_wheel(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    center_position: Vec3,
    car_count: usize,
) {
    let wheel_radius = 4.0;
    let hub_radius = 0.5;
    let car_size = Vec3::new(0.4, 0.6, 0.4);

    // Static center hub
    let hub_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(center_position))
        .build();
    let hub_handle = nova.world.insert_body(hub_body);

    // Visual for hub
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(hub_radius))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.5, 0.5, 0.5),
            ..default()
        })),
        Transform::from_translation(center_position),
        SpawnedObject,
    ));

    // Create spokes and cars
    for i in 0..car_count {
        let angle = (i as f32 / car_count as f32) * std::f32::consts::TAU;
        let spoke_end = center_position
            + Vec3::new(angle.cos() * wheel_radius, angle.sin() * wheel_radius, 0.0);

        // Spoke (visual only for simplicity)
        let spoke_mid = (center_position + spoke_end) / 2.0;
        let spoke_length = wheel_radius;

        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(spoke_length, 0.1, 0.1))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.4, 0.4, 0.4),
                ..default()
            })),
            Transform::from_translation(spoke_mid)
                .with_rotation(Quat::from_rotation_z(angle)),
            SpawnedObject,
        ));

        // Car at the end of the spoke
        let color = Color::hsl(i as f32 * (360.0 / car_count as f32), 0.7, 0.5);
        let (_, car_handle) = spawn_box(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            spoke_end,
            car_size * 0.5,
            color,
            PhysicsMaterialType::Normal,
        );

        // Connect car to hub with distance joint (simulates the spoke)
        nova.world.create_distance_joint(
            hub_handle,
            car_handle,
            nova::prelude::Vec3::ZERO,
            nova::prelude::Vec3::ZERO,
            wheel_radius,
        );
    }
}

/// Spawn a windmill
pub fn spawn_windmill(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let blade_count = 4;
    let blade_length = 2.5;
    let blade_width = 0.3;

    // Static center hub
    let hub_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(position))
        .build();
    let hub_handle = nova.world.insert_body(hub_body);

    // Hub visual
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.3))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.4, 0.3, 0.2),
            ..default()
        })),
        Transform::from_translation(position),
        SpawnedObject,
    ));

    // Create blades
    for i in 0..blade_count {
        let angle = (i as f32 / blade_count as f32) * std::f32::consts::TAU;
        let blade_center = position
            + Vec3::new(
                angle.cos() * blade_length / 2.0,
                angle.sin() * blade_length / 2.0,
                0.0,
            );

        let blade_size = Vec3::new(blade_length, blade_width, 0.05);

        let (_, blade_handle) = spawn_box(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            blade_center,
            blade_size * 0.5,
            Color::srgb(0.8, 0.8, 0.7),
            PhysicsMaterialType::Light,
        );

        // Connect blade to hub
        nova.world.create_hinge_joint(
            hub_handle,
            blade_handle,
            nova::prelude::Vec3::ZERO,
            to_nova_vec3(Vec3::new(-blade_length / 2.0, 0.0, 0.0)),
            nova::prelude::Vec3::Z,
        );
    }
}

/// Spawn a spiral staircase
pub fn spawn_spiral_staircase(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    base_position: Vec3,
    step_count: usize,
) {
    let step_size = Vec3::new(1.2, 0.15, 0.5);
    let step_height = 0.3;
    let radius = 1.5;
    let angle_per_step = std::f32::consts::TAU / 12.0; // 30 degrees per step

    for i in 0..step_count {
        let angle = i as f32 * angle_per_step;
        let height = i as f32 * step_height;

        let step_pos = base_position
            + Vec3::new(angle.cos() * radius, height, angle.sin() * radius);

        let rotation = Quat::from_rotation_y(-angle);

        let color = Color::hsl(i as f32 * 15.0 % 360.0, 0.5, 0.6);

        let nova_pos = to_nova_vec3(step_pos);

        let body = nova
            .world
            .create_body()
            .body_type(RigidBodyType::Dynamic)
            .position(nova_pos)
            .rotation(to_nova_quat(rotation))
            .mass(2.0)
            .build();
        let body_handle = nova.world.insert_body(body);

        let collider = nova
            .world
            .create_collider(
                body_handle,
                CollisionShape::Box(BoxShape::new(to_nova_vec3(step_size * 0.5))),
            )
            .friction(0.6)
            .build();
        let collider_handle = nova.world.insert_collider(collider);

        let entity = commands
            .spawn((
                Mesh3d(meshes.add(Cuboid::new(step_size.x, step_size.y, step_size.z))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: color,
                    ..default()
                })),
                Transform::from_translation(step_pos).with_rotation(rotation),
                crate::components::PhysicsBody {
                    handle: body_handle,
                },
                crate::components::PhysicsCollider {
                    handle: collider_handle,
                },
                crate::components::DynamicBody,
                SpawnedObject,
            ))
            .id();

        handle_to_entity.bodies.insert(body_handle, entity);
    }
}

/// Spawn raining boxes
pub fn spawn_box_rain(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    center_position: Vec3,
    count: usize,
) {
    let mut rng = rand::thread_rng();

    for _ in 0..count {
        let offset = Vec3::new(
            rng.gen_range(-5.0..5.0),
            rng.gen_range(0.0..10.0),
            rng.gen_range(-5.0..5.0),
        );

        let size = Vec3::splat(rng.gen_range(0.2..0.6));
        let color = Color::hsl(rng.gen_range(0.0..360.0), 0.7, 0.5);

        spawn_box(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            center_position + offset,
            size * 0.5,
            color,
            PhysicsMaterialType::Normal,
        );
    }
}

/// Spawn a cannon structure
pub fn spawn_cannon(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    direction: Vec3,
) {
    let barrel_length = 2.0;
    let barrel_radius = 0.25;
    let base_size = Vec3::new(1.0, 0.5, 1.0);

    // Calculate rotation to face direction
    let forward = Vec3::new(direction.x, 0.0, direction.z).normalize_or_zero();
    let rotation = if forward.length_squared() > 0.01 {
        Quat::from_rotation_arc(Vec3::Z, forward)
    } else {
        Quat::IDENTITY
    };

    // Base (static platform)
    let base_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(position))
        .build();
    let base_handle = nova.world.insert_body(base_body);

    nova.world.insert_collider(
        nova.world
            .create_collider(
                base_handle,
                CollisionShape::Box(BoxShape::new(to_nova_vec3(base_size * 0.5))),
            )
            .friction(0.8)
            .build(),
    );

    // Base visual
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(base_size.x, base_size.y, base_size.z))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.3),
            ..default()
        })),
        Transform::from_translation(position),
        SpawnedObject,
    ));

    // Barrel position
    let barrel_pos = position + Vec3::new(0.0, base_size.y / 2.0 + barrel_radius, 0.0)
        + forward * (barrel_length / 2.0);

    // Barrel (visual only - cylinder shape)
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(barrel_radius, barrel_length))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.2, 0.2),
            metallic: 0.8,
            ..default()
        })),
        Transform::from_translation(barrel_pos)
            .with_rotation(rotation * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
        SpawnedObject,
    ));

    // Spawn a cannonball ready to fire
    let ball_pos = position + Vec3::new(0.0, base_size.y / 2.0 + barrel_radius, 0.0)
        + forward * (barrel_length + 0.5);

    spawn_sphere(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        ball_pos,
        0.2,
        Color::srgb(0.1, 0.1, 0.1),
        PhysicsMaterialType::Heavy,
    );
}

//! Pre-built physics structures - MEGA EDITION!
//! Tower, Pyramid, Ragdoll, Newton's Cradle, Car, Ferris Wheel, AND SO MUCH MORE!

use bevy::prelude::*;
use nova::prelude::*;
use rand::Rng;

use crate::components::{PhysicsMaterialType, SpawnedObject};
use crate::convert::{to_nova_quat, to_nova_vec3};
use crate::plugins::spawning::{spawn_box, spawn_capsule, spawn_sphere};
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

// ============ MORE EPIC PRESETS ============

/// Spawn a bowling alley with pins
pub fn spawn_bowling(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    direction: Vec3,
) {
    let dir = direction.normalize_or_zero();
    let right = Vec3::new(-dir.z, 0.0, dir.x);

    // Bowling pins arrangement (10 pins in triangle)
    let pin_positions = [
        // Row 1 (1 pin)
        Vec3::ZERO,
        // Row 2 (2 pins)
        Vec3::new(0.3, 0.0, 0.5),
        Vec3::new(-0.3, 0.0, 0.5),
        // Row 3 (3 pins)
        Vec3::new(0.6, 0.0, 1.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(-0.6, 0.0, 1.0),
        // Row 4 (4 pins)
        Vec3::new(0.9, 0.0, 1.5),
        Vec3::new(0.3, 0.0, 1.5),
        Vec3::new(-0.3, 0.0, 1.5),
        Vec3::new(-0.9, 0.0, 1.5),
    ];

    for local_pos in pin_positions {
        let world_pos = position + dir * local_pos.z + right * local_pos.x + Vec3::Y * 0.4;

        // Spawn capsule as pin
        spawn_capsule(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            world_pos,
            0.08,
            0.3,
            Color::srgb(0.95, 0.95, 0.95),
            PhysicsMaterialType::Normal,
        );
    }

    // Bowling ball
    let ball_pos = position - dir * 5.0 + Vec3::Y * 0.3;
    spawn_sphere(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        ball_pos,
        0.25,
        Color::srgb(0.1, 0.1, 0.4),
        PhysicsMaterialType::Heavy,
    );
}

/// Spawn a seesaw
pub fn spawn_seesaw(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let plank_size = Vec3::new(6.0, 0.2, 1.0);
    let pivot_height = 0.8;

    // Static pivot/fulcrum
    let pivot_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(position))
        .build();
    let pivot_handle = nova.world.insert_body(pivot_body);

    nova.world.insert_collider(
        nova.world
            .create_collider(
                pivot_handle,
                CollisionShape::Box(BoxShape::new(nova::prelude::Vec3::new(0.3, pivot_height * 0.5, 0.5))),
            )
            .friction(0.5)
            .build(),
    );

    // Pivot visual (triangle-ish)
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.6, pivot_height, 1.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.4, 0.3, 0.2),
            ..default()
        })),
        Transform::from_translation(position + Vec3::Y * (pivot_height * 0.5)),
        SpawnedObject,
    ));

    // Plank
    let plank_pos = position + Vec3::Y * (pivot_height + 0.15);
    let plank_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(to_nova_vec3(plank_pos))
        .mass(5.0)
        .build();
    let plank_handle = nova.world.insert_body(plank_body);

    let plank_collider = nova
        .world
        .create_collider(
            plank_handle,
            CollisionShape::Box(BoxShape::new(to_nova_vec3(plank_size * 0.5))),
        )
        .friction(0.7)
        .build();
    let plank_collider_handle = nova.world.insert_collider(plank_collider);

    let plank_entity = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(plank_size.x, plank_size.y, plank_size.z))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.6, 0.4, 0.2),
                ..default()
            })),
            Transform::from_translation(plank_pos),
            crate::components::PhysicsBody { handle: plank_handle },
            crate::components::PhysicsCollider { handle: plank_collider_handle },
            crate::components::DynamicBody,
            SpawnedObject,
        ))
        .id();

    handle_to_entity.bodies.insert(plank_handle, plank_entity);

    // Connect with hinge joint
    nova.world.create_hinge_joint(
        pivot_handle,
        plank_handle,
        nova::prelude::Vec3::new(0.0, pivot_height, 0.0),
        nova::prelude::Vec3::ZERO,
        nova::prelude::Vec3::Z, // Rotate around Z axis
    );

    // Add a heavy ball on one side
    spawn_sphere(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        position + Vec3::new(-2.5, pivot_height + 1.5, 0.0),
        0.5,
        Color::srgb(0.8, 0.2, 0.2),
        PhysicsMaterialType::Heavy,
    );
}

/// Spawn a swing set
pub fn spawn_swing(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let frame_height = 4.0;
    let frame_width = 3.0;
    let rope_length = 3.0;

    // Static top bar
    let bar_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(position + Vec3::Y * frame_height))
        .build();
    let bar_handle = nova.world.insert_body(bar_body);

    nova.world.insert_collider(
        nova.world
            .create_collider(
                bar_handle,
                CollisionShape::Box(BoxShape::new(nova::prelude::Vec3::new(frame_width * 0.5, 0.1, 0.1))),
            )
            .build(),
    );

    // Top bar visual
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(frame_width, 0.2, 0.2))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.5, 0.3, 0.1),
            ..default()
        })),
        Transform::from_translation(position + Vec3::Y * frame_height),
        SpawnedObject,
    ));

    // Side poles (visual only)
    for x in [-frame_width * 0.5, frame_width * 0.5] {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(0.15, frame_height, 0.15))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.5, 0.3, 0.1),
                ..default()
            })),
            Transform::from_translation(position + Vec3::new(x, frame_height * 0.5, 0.0)),
            SpawnedObject,
        ));
    }

    // Swing seat
    let seat_pos = position + Vec3::Y * (frame_height - rope_length);
    let seat_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(to_nova_vec3(seat_pos))
        .mass(2.0)
        .build();
    let seat_handle = nova.world.insert_body(seat_body);

    let seat_collider = nova
        .world
        .create_collider(
            seat_handle,
            CollisionShape::Box(BoxShape::new(nova::prelude::Vec3::new(0.4, 0.05, 0.2))),
        )
        .friction(0.8)
        .build();
    let seat_collider_handle = nova.world.insert_collider(seat_collider);

    let seat_entity = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(0.8, 0.1, 0.4))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.2, 0.2, 0.2),
                ..default()
            })),
            Transform::from_translation(seat_pos),
            crate::components::PhysicsBody { handle: seat_handle },
            crate::components::PhysicsCollider { handle: seat_collider_handle },
            crate::components::DynamicBody,
            SpawnedObject,
        ))
        .id();

    handle_to_entity.bodies.insert(seat_handle, seat_entity);

    // Connect seat to bar with ball joint (allows swinging)
    nova.world.create_ball_joint(
        bar_handle,
        seat_handle,
        nova::prelude::Vec3::ZERO,
        nova::prelude::Vec3::new(0.0, rope_length, 0.0),
    );
}

/// Spawn a wall of boxes (for destruction!)
pub fn spawn_wall(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    width: usize,
    height: usize,
) {
    let brick_size = Vec3::new(0.8, 0.4, 0.4);

    for y in 0..height {
        let offset_x = if y % 2 == 0 { 0.0 } else { brick_size.x * 0.5 };

        for x in 0..width {
            let brick_pos = position
                + Vec3::new(
                    x as f32 * brick_size.x - (width as f32 * brick_size.x * 0.5) + offset_x,
                    y as f32 * brick_size.y + brick_size.y * 0.5,
                    0.0,
                );

            let color = if (x + y) % 2 == 0 {
                Color::srgb(0.7, 0.3, 0.2)
            } else {
                Color::srgb(0.8, 0.4, 0.3)
            };

            spawn_box(
                nova,
                commands,
                meshes,
                materials,
                handle_to_entity,
                brick_pos,
                brick_size * 0.5,
                color,
                PhysicsMaterialType::Normal,
            );
        }
    }
}

/// Spawn a sphere avalanche
pub fn spawn_avalanche(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    count: usize,
) {
    let mut rng = rand::thread_rng();

    for i in 0..count {
        let offset = Vec3::new(
            rng.gen_range(-3.0..3.0),
            i as f32 * 0.5 + rng.gen_range(0.0..2.0),
            rng.gen_range(-3.0..3.0),
        );

        let radius = rng.gen_range(0.2..0.6);
        let color = Color::hsl(rng.gen_range(0.0..60.0), 0.6, 0.5); // Warm colors

        spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            position + offset,
            radius,
            color,
            PhysicsMaterialType::Bouncy,
        );
    }
}

/// Spawn a Jenga-style tower (alternating brick layers)
pub fn spawn_jenga(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    layers: usize,
) {
    let brick_size = Vec3::new(0.8, 0.25, 0.27);
    let bricks_per_layer = 3;

    for layer in 0..layers {
        let y = position.y + layer as f32 * brick_size.y + brick_size.y * 0.5;
        let rotate_layer = layer % 2 == 1;

        for i in 0..bricks_per_layer {
            let offset = (i as f32 - 1.0) * brick_size.z;

            let (brick_pos, rotation) = if rotate_layer {
                (
                    Vec3::new(position.x + offset, y, position.z),
                    Quat::from_rotation_y(std::f32::consts::FRAC_PI_2),
                )
            } else {
                (
                    Vec3::new(position.x, y, position.z + offset),
                    Quat::IDENTITY,
                )
            };

            // Wood color with slight variation
            let color_variation = (layer * 3 + i) as f32 * 0.02;
            let color = Color::srgb(0.7 + color_variation, 0.5 + color_variation, 0.3);

            let nova_pos = to_nova_vec3(brick_pos);

            let body = nova
                .world
                .create_body()
                .body_type(RigidBodyType::Dynamic)
                .position(nova_pos)
                .rotation(to_nova_quat(rotation))
                .mass(0.3)
                .build();
            let body_handle = nova.world.insert_body(body);

            let collider = nova
                .world
                .create_collider(
                    body_handle,
                    CollisionShape::Box(BoxShape::new(to_nova_vec3(brick_size * 0.5))),
                )
                .friction(0.6)
                .restitution(0.1)
                .build();
            let collider_handle = nova.world.insert_collider(collider);

            let entity = commands
                .spawn((
                    Mesh3d(meshes.add(Cuboid::new(brick_size.x, brick_size.y, brick_size.z))),
                    MeshMaterial3d(materials.add(StandardMaterial {
                        base_color: color,
                        ..default()
                    })),
                    Transform::from_translation(brick_pos).with_rotation(rotation),
                    crate::components::PhysicsBody { handle: body_handle },
                    crate::components::PhysicsCollider { handle: collider_handle },
                    crate::components::DynamicBody,
                    SpawnedObject,
                ))
                .id();

            handle_to_entity.bodies.insert(body_handle, entity);
        }
    }
}

/// Spawn pool/billiards setup
pub fn spawn_pool_table(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let ball_radius = 0.15;
    let spacing = ball_radius * 2.1;

    // Ball colors (pool ball style)
    let ball_colors = [
        Color::srgb(0.9, 0.9, 0.1), // 1 - yellow
        Color::srgb(0.1, 0.1, 0.8), // 2 - blue
        Color::srgb(0.9, 0.2, 0.1), // 3 - red
        Color::srgb(0.5, 0.1, 0.5), // 4 - purple
        Color::srgb(0.9, 0.5, 0.1), // 5 - orange
        Color::srgb(0.1, 0.6, 0.2), // 6 - green
        Color::srgb(0.5, 0.2, 0.1), // 7 - maroon
        Color::srgb(0.1, 0.1, 0.1), // 8 - black
        Color::srgb(0.9, 0.8, 0.2), // 9 - yellow stripe
        Color::srgb(0.2, 0.2, 0.9), // 10 - blue stripe
    ];

    // Triangle formation (5 rows)
    let mut ball_idx = 0;
    for row in 0..5 {
        for col in 0..=row {
            let x = row as f32 * spacing * 0.866; // cos(30°)
            let z = (col as f32 - row as f32 * 0.5) * spacing;
            let ball_pos = position + Vec3::new(x, ball_radius, z);

            let color = ball_colors[ball_idx % ball_colors.len()];
            ball_idx += 1;

            spawn_sphere(
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
        }
    }

    // Cue ball
    let cue_pos = position - Vec3::new(3.0, -ball_radius, 0.0);
    spawn_sphere(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        cue_pos,
        ball_radius,
        Color::srgb(0.95, 0.95, 0.95),
        PhysicsMaterialType::Normal,
    );
}

/// Spawn a trebuchet (medieval siege weapon)
pub fn spawn_trebuchet(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let base_size = Vec3::new(2.0, 0.3, 1.5);
    let arm_length = 4.0;
    let arm_thickness = 0.15;
    let counterweight_radius = 0.5;

    // Static base
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

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(base_size.x, base_size.y, base_size.z))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.4, 0.3, 0.2),
            ..default()
        })),
        Transform::from_translation(position),
        SpawnedObject,
    ));

    // Pivot point (vertical support)
    let pivot_height = 1.5;
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.2, pivot_height, 0.2))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.35, 0.25, 0.15),
            ..default()
        })),
        Transform::from_translation(position + Vec3::new(0.0, base_size.y * 0.5 + pivot_height * 0.5, 0.0)),
        SpawnedObject,
    ));

    // Throwing arm (dynamic)
    let arm_pos = position + Vec3::new(0.0, base_size.y * 0.5 + pivot_height, 0.0);
    let arm_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(to_nova_vec3(arm_pos))
        .mass(2.0)
        .build();
    let arm_handle = nova.world.insert_body(arm_body);

    let arm_collider = nova
        .world
        .create_collider(
            arm_handle,
            CollisionShape::Box(BoxShape::new(nova::prelude::Vec3::new(arm_length * 0.5, arm_thickness * 0.5, arm_thickness * 0.5))),
        )
        .friction(0.5)
        .build();
    let arm_collider_handle = nova.world.insert_collider(arm_collider);

    let arm_entity = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(arm_length, arm_thickness, arm_thickness))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.5, 0.35, 0.2),
                ..default()
            })),
            Transform::from_translation(arm_pos),
            crate::components::PhysicsBody { handle: arm_handle },
            crate::components::PhysicsCollider { handle: arm_collider_handle },
            crate::components::DynamicBody,
            SpawnedObject,
        ))
        .id();

    handle_to_entity.bodies.insert(arm_handle, arm_entity);

    // Connect arm to base with hinge
    nova.world.create_hinge_joint(
        base_handle,
        arm_handle,
        nova::prelude::Vec3::new(0.0, base_size.y * 0.5 + pivot_height, 0.0),
        nova::prelude::Vec3::ZERO,
        nova::prelude::Vec3::Z,
    );

    // Counterweight on short end
    let cw_pos = arm_pos + Vec3::new(-arm_length * 0.35, -counterweight_radius - arm_thickness, 0.0);
    let (_, cw_handle) = spawn_sphere(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        cw_pos,
        counterweight_radius,
        Color::srgb(0.3, 0.3, 0.35),
        PhysicsMaterialType::Heavy,
    );

    nova.world.create_fixed_joint(
        arm_handle,
        cw_handle,
        nova::prelude::Vec3::new(-arm_length * 0.35, -arm_thickness * 0.5, 0.0),
        nova::prelude::Vec3::new(0.0, counterweight_radius, 0.0),
    );

    // Projectile on long end
    let proj_pos = arm_pos + Vec3::new(arm_length * 0.45, arm_thickness + 0.15, 0.0);
    spawn_sphere(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        proj_pos,
        0.15,
        Color::srgb(0.6, 0.4, 0.3),
        PhysicsMaterialType::Normal,
    );
}

/// Spawn a ramp/incline
pub fn spawn_ramp(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    direction: Vec3,
    length: f32,
    angle_degrees: f32,
) {
    let width = 2.0;
    let thickness = 0.2;
    let angle_rad = angle_degrees.to_radians();

    let dir = Vec3::new(direction.x, 0.0, direction.z).normalize_or_zero();
    let rotation = Quat::from_rotation_arc(Vec3::X, dir) * Quat::from_rotation_z(-angle_rad);

    // Ramp center position (elevated)
    let center_height = (length * 0.5) * angle_rad.sin();
    let ramp_pos = position + Vec3::new(0.0, center_height + thickness * 0.5, 0.0);

    let ramp_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(ramp_pos))
        .rotation(to_nova_quat(rotation))
        .build();
    let ramp_handle = nova.world.insert_body(ramp_body);

    nova.world.insert_collider(
        nova.world
            .create_collider(
                ramp_handle,
                CollisionShape::Box(BoxShape::new(nova::prelude::Vec3::new(length * 0.5, thickness * 0.5, width * 0.5))),
            )
            .friction(0.4)
            .build(),
    );

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(length, thickness, width))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.5, 0.5, 0.55),
            ..default()
        })),
        Transform::from_translation(ramp_pos).with_rotation(rotation),
        SpawnedObject,
    ));
}

/// Spawn a spinning platform
pub fn spawn_spinner_platform(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    arm_count: usize,
) {
    let arm_length = 3.0;
    let arm_width = 0.4;
    let arm_height = 0.2;

    // Static center anchor
    let center_body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(position))
        .build();
    let center_handle = nova.world.insert_body(center_body);

    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.3, 0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.4, 0.4, 0.45),
            ..default()
        })),
        Transform::from_translation(position),
        SpawnedObject,
    ));

    // Create spinning arms
    for i in 0..arm_count {
        let angle = (i as f32 / arm_count as f32) * std::f32::consts::TAU;
        let arm_center = position + Vec3::new(angle.cos() * arm_length * 0.5, 0.0, angle.sin() * arm_length * 0.5);

        let rotation = Quat::from_rotation_y(-angle);

        let arm_body = nova
            .world
            .create_body()
            .body_type(RigidBodyType::Dynamic)
            .position(to_nova_vec3(arm_center))
            .rotation(to_nova_quat(rotation))
            .mass(3.0)
            .build();
        let arm_handle = nova.world.insert_body(arm_body);

        let arm_collider = nova
            .world
            .create_collider(
                arm_handle,
                CollisionShape::Box(BoxShape::new(nova::prelude::Vec3::new(arm_length * 0.5, arm_height * 0.5, arm_width * 0.5))),
            )
            .friction(0.5)
            .build();
        let arm_collider_handle = nova.world.insert_collider(arm_collider);

        let color = Color::hsl(i as f32 * (360.0 / arm_count as f32), 0.6, 0.5);

        let arm_entity = commands
            .spawn((
                Mesh3d(meshes.add(Cuboid::new(arm_length, arm_height, arm_width))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: color,
                    ..default()
                })),
                Transform::from_translation(arm_center).with_rotation(rotation),
                crate::components::PhysicsBody { handle: arm_handle },
                crate::components::PhysicsCollider { handle: arm_collider_handle },
                crate::components::DynamicBody,
                SpawnedObject,
            ))
            .id();

        handle_to_entity.bodies.insert(arm_handle, arm_entity);

        // Connect to center with hinge (vertical axis)
        nova.world.create_hinge_joint(
            center_handle,
            arm_handle,
            nova::prelude::Vec3::ZERO,
            nova::prelude::Vec3::new(-arm_length * 0.5, 0.0, 0.0),
            nova::prelude::Vec3::Y,
        );
    }
}

/// Spawn a giant ball pit
pub fn spawn_ball_pit(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    ball_count: usize,
) {
    let mut rng = rand::thread_rng();
    let pit_radius = 4.0;

    // Create containing walls (visual only - objects can escape for fun!)
    let wall_height = 1.5;
    let wall_thickness = 0.15;

    for i in 0..8 {
        let angle = (i as f32 / 8.0) * std::f32::consts::TAU;
        let wall_pos = position + Vec3::new(
            angle.cos() * pit_radius,
            wall_height * 0.5,
            angle.sin() * pit_radius,
        );

        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(wall_thickness, wall_height, pit_radius * 0.5))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgba(0.3, 0.5, 0.8, 0.5),
                alpha_mode: bevy::prelude::AlphaMode::Blend,
                ..default()
            })),
            Transform::from_translation(wall_pos)
                .with_rotation(Quat::from_rotation_y(angle)),
            SpawnedObject,
        ));
    }

    // Spawn colorful balls
    let ball_colors = [
        Color::srgb(0.9, 0.2, 0.2), // Red
        Color::srgb(0.2, 0.9, 0.2), // Green
        Color::srgb(0.2, 0.2, 0.9), // Blue
        Color::srgb(0.9, 0.9, 0.2), // Yellow
        Color::srgb(0.9, 0.2, 0.9), // Magenta
        Color::srgb(0.2, 0.9, 0.9), // Cyan
        Color::srgb(0.9, 0.5, 0.2), // Orange
    ];

    for i in 0..ball_count {
        let angle = rng.gen_range(0.0..std::f32::consts::TAU);
        let dist = rng.gen_range(0.0..pit_radius * 0.8);
        let height = rng.gen_range(0.5..3.0) + (i as f32 * 0.1);

        let ball_pos = position + Vec3::new(
            angle.cos() * dist,
            height,
            angle.sin() * dist,
        );

        let color = ball_colors[rng.gen_range(0..ball_colors.len())];
        let radius = rng.gen_range(0.15..0.25);

        spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            ball_pos,
            radius,
            color,
            PhysicsMaterialType::Bouncy,
        );
    }
}

// ============ GOING ABSOLUTELY BONKERS ============

/// Spawn a MASSIVE domino spiral
pub fn spawn_domino_spiral(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    center: Vec3,
    domino_count: usize,
) {
    let domino_size = Vec3::new(0.08, 0.4, 0.2);
    let initial_radius = 1.0;
    let spiral_growth = 0.08;
    let angle_step = 0.15;

    for i in 0..domino_count {
        let angle = i as f32 * angle_step;
        let radius = initial_radius + i as f32 * spiral_growth;

        let x = angle.cos() * radius;
        let z = angle.sin() * radius;
        let pos = center + Vec3::new(x, domino_size.y, z);

        // Face tangent to spiral
        let tangent_angle = angle + std::f32::consts::FRAC_PI_2;
        let rotation = Quat::from_rotation_y(tangent_angle);

        // Tilt first domino to start the chain
        let tilt = if i == 0 {
            Quat::from_rotation_x(0.15)
        } else {
            Quat::IDENTITY
        };

        let color = Color::hsl((i as f32 * 5.0) % 360.0, 0.8, 0.5);

        let nova_pos = to_nova_vec3(pos);
        let body = nova.world
            .create_body()
            .body_type(RigidBodyType::Dynamic)
            .position(nova_pos)
            .rotation(to_nova_quat(rotation * tilt))
            .mass(0.3)
            .build();
        let body_handle = nova.world.insert_body(body);

        let collider = nova.world
            .create_collider(body_handle, CollisionShape::Box(BoxShape::new(to_nova_vec3(domino_size * 0.5))))
            .friction(0.5)
            .restitution(0.1)
            .build();
        let collider_handle = nova.world.insert_collider(collider);

        let entity = commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(domino_size.x, domino_size.y, domino_size.z))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                ..default()
            })),
            Transform::from_translation(pos).with_rotation(rotation * tilt),
            crate::components::PhysicsBody { handle: body_handle },
            crate::components::PhysicsCollider { handle: collider_handle },
            crate::components::DynamicBody,
            SpawnedObject,
        )).id();

        handle_to_entity.bodies.insert(body_handle, entity);
    }
}

/// Spawn a GIANT swinging pendulum of DOOM
pub fn spawn_giant_pendulum(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let chain_length = 8.0;
    let ball_radius = 1.5;

    // Static anchor point
    let anchor_body = nova.world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(position + Vec3::Y * chain_length))
        .build();
    let anchor_handle = nova.world.insert_body(anchor_body);

    // Visual anchor
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.3))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.3),
            metallic: 0.9,
            ..default()
        })),
        Transform::from_translation(position + Vec3::Y * chain_length),
        SpawnedObject,
    ));

    // Chain segments
    let segment_count = 10;
    let segment_length = chain_length / segment_count as f32;
    let mut prev_handle = anchor_handle;

    for i in 0..segment_count {
        let seg_pos = position + Vec3::Y * (chain_length - (i as f32 + 0.5) * segment_length);

        let seg_body = nova.world
            .create_body()
            .body_type(RigidBodyType::Dynamic)
            .position(to_nova_vec3(seg_pos))
            .mass(0.5)
            .build();
        let seg_handle = nova.world.insert_body(seg_body);

        let seg_collider = nova.world
            .create_collider(seg_handle, CollisionShape::Sphere(SphereShape::new(0.1)))
            .build();
        nova.world.insert_collider(seg_collider);

        // Visual chain link
        commands.spawn((
            Mesh3d(meshes.add(Cylinder::new(0.05, segment_length * 0.8))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.4, 0.4, 0.45),
                metallic: 0.8,
                ..default()
            })),
            Transform::from_translation(seg_pos),
            SpawnedObject,
        ));

        nova.world.create_ball_joint(
            prev_handle,
            seg_handle,
            nova::prelude::Vec3::new(0.0, -segment_length * 0.5, 0.0),
            nova::prelude::Vec3::new(0.0, segment_length * 0.5, 0.0),
        );

        prev_handle = seg_handle;
    }

    // GIANT BALL at the end - start it swinging!
    let ball_pos = position + Vec3::new(chain_length * 0.7, ball_radius, 0.0);
    let (_, ball_handle) = spawn_sphere(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        ball_pos,
        ball_radius,
        Color::srgb(0.8, 0.2, 0.1),
        PhysicsMaterialType::Heavy,
    );

    nova.world.create_ball_joint(
        prev_handle,
        ball_handle,
        nova::prelude::Vec3::new(0.0, -segment_length * 0.5, 0.0),
        nova::prelude::Vec3::new(0.0, ball_radius, 0.0),
    );
}

/// Spawn WRECKING BALL VS TOWER - instant chaos!
pub fn spawn_wrecking_ball_vs_tower(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    // Spawn tower first
    let tower_pos = position;
    spawn_tower(nova, commands, meshes, materials, handle_to_entity, tower_pos, 12);

    // Spawn wrecking ball aimed at tower
    let ball_anchor = position + Vec3::new(-8.0, 12.0, 0.0);
    spawn_wrecking_ball(nova, commands, meshes, materials, handle_to_entity, ball_anchor, 10);
}

/// Spawn a VOLCANO that erupts balls!
pub fn spawn_volcano(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    ball_count: usize,
) {
    let mut rng = rand::thread_rng();

    // Volcano cone (visual)
    commands.spawn((
        Mesh3d(meshes.add(Cone::new(3.0, 4.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.4, 0.25, 0.15),
            ..default()
        })),
        Transform::from_translation(position + Vec3::Y * 2.0),
        SpawnedObject,
    ));

    // ERUPTING BALLS!
    for i in 0..ball_count {
        let angle = rng.gen_range(0.0..std::f32::consts::TAU);
        let spread = rng.gen_range(0.0..0.8);
        let height = 5.0 + i as f32 * 0.3;

        let ball_pos = position + Vec3::new(
            angle.cos() * spread,
            height,
            angle.sin() * spread,
        );

        // Lava colors!
        let color = Color::hsl(rng.gen_range(0.0..40.0), 0.9, 0.5);
        let radius = rng.gen_range(0.15..0.35);

        let (entity, body_handle) = spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            ball_pos,
            radius,
            color,
            PhysicsMaterialType::Bouncy,
        );

        // Give them upward velocity!
        if let Some(body) = nova.world.get_body_mut(body_handle) {
            let vel = nova::prelude::Vec3::new(
                rng.gen_range(-3.0..3.0),
                rng.gen_range(8.0..15.0),
                rng.gen_range(-3.0..3.0),
            );
            body.linear_velocity = vel;
        }
    }
}

/// Spawn STAIRCASE OF DOOM - balls bouncing down!
pub fn spawn_staircase_of_doom(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    step_count: usize,
) {
    let step_size = Vec3::new(2.0, 0.3, 1.5);
    let step_height = 0.5;
    let step_depth = 0.8;

    // Create stairs (static)
    for i in 0..step_count {
        let step_pos = position + Vec3::new(
            i as f32 * step_depth,
            -(i as f32 * step_height),
            0.0,
        );

        let step_body = nova.world
            .create_body()
            .body_type(RigidBodyType::Static)
            .position(to_nova_vec3(step_pos))
            .build();
        let step_handle = nova.world.insert_body(step_body);

        nova.world.insert_collider(
            nova.world.create_collider(
                step_handle,
                CollisionShape::Box(BoxShape::new(to_nova_vec3(step_size * 0.5))),
            )
            .friction(0.3)
            .restitution(0.5)
            .build()
        );

        let color = Color::hsl((i as f32 * 20.0) % 360.0, 0.5, 0.6);

        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(step_size.x, step_size.y, step_size.z))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                ..default()
            })),
            Transform::from_translation(step_pos),
            SpawnedObject,
        ));
    }

    // Spawn bouncy balls at the top!
    let mut rng = rand::thread_rng();
    for i in 0..10 {
        let ball_pos = position + Vec3::new(
            rng.gen_range(-0.5..0.5),
            2.0 + i as f32 * 0.5,
            rng.gen_range(-0.3..0.3),
        );

        spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            ball_pos,
            0.25,
            Color::hsl(rng.gen_range(0.0..360.0), 0.8, 0.5),
            PhysicsMaterialType::Bouncy,
        );
    }
}

/// Spawn a PACHINKO board!
pub fn spawn_pachinko(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let board_width = 6.0;
    let board_height = 10.0;
    let peg_radius = 0.15;
    let rows = 12;
    let pegs_per_row = 8;

    // Back board (visual)
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(board_width, board_height, 0.1))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(0.2, 0.15, 0.1, 0.8),
            alpha_mode: bevy::prelude::AlphaMode::Blend,
            ..default()
        })),
        Transform::from_translation(position + Vec3::new(0.0, board_height * 0.5, -0.2)),
        SpawnedObject,
    ));

    // Create pegs
    for row in 0..rows {
        let row_offset = if row % 2 == 0 { 0.0 } else { 0.35 };
        let y = position.y + board_height - (row as f32 + 1.0) * (board_height / (rows + 1) as f32);

        for col in 0..pegs_per_row {
            let x = position.x - board_width * 0.4 + col as f32 * (board_width * 0.8 / (pegs_per_row - 1) as f32) + row_offset;
            let peg_pos = Vec3::new(x, y, position.z);

            let peg_body = nova.world
                .create_body()
                .body_type(RigidBodyType::Static)
                .position(to_nova_vec3(peg_pos))
                .build();
            let peg_handle = nova.world.insert_body(peg_body);

            nova.world.insert_collider(
                nova.world.create_collider(
                    peg_handle,
                    CollisionShape::Sphere(SphereShape::new(peg_radius)),
                )
                .friction(0.1)
                .restitution(0.8)
                .build()
            );

            commands.spawn((
                Mesh3d(meshes.add(Sphere::new(peg_radius))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: Color::srgb(0.8, 0.7, 0.2),
                    metallic: 0.9,
                    ..default()
                })),
                Transform::from_translation(peg_pos),
                SpawnedObject,
            ));
        }
    }

    // Drop some balls from the top!
    let mut rng = rand::thread_rng();
    for i in 0..8 {
        let ball_pos = position + Vec3::new(
            rng.gen_range(-1.0..1.0),
            board_height + 1.0 + i as f32 * 0.4,
            0.0,
        );

        spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            ball_pos,
            0.2,
            Color::hsl(rng.gen_range(0.0..360.0), 0.9, 0.5),
            PhysicsMaterialType::Bouncy,
        );
    }
}

/// Spawn a BALL TSUNAMI - massive wave of balls!
pub fn spawn_ball_tsunami(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    ball_count: usize,
) {
    let mut rng = rand::thread_rng();
    let wave_width = 15.0;
    let wave_height = 8.0;

    for i in 0..ball_count {
        // Arrange in a wave pattern
        let row = i / 15;
        let col = i % 15;

        let x = position.x + (col as f32 - 7.0) * 0.5;
        let y = position.y + row as f32 * 0.5 + rng.gen_range(0.0..0.3);
        let z = position.z + rng.gen_range(-0.2..0.2);

        let ball_pos = Vec3::new(x, y, z);
        let radius = rng.gen_range(0.2..0.4);

        // Ocean colors
        let color = Color::hsl(rng.gen_range(180.0..220.0), 0.7, 0.5);

        let (_, body_handle) = spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            ball_pos,
            radius,
            color,
            PhysicsMaterialType::Normal,
        );

        // Give forward momentum!
        if let Some(body) = nova.world.get_body_mut(body_handle) {
            body.linear_velocity = nova::prelude::Vec3::new(
                rng.gen_range(5.0..10.0),
                rng.gen_range(2.0..5.0),
                rng.gen_range(-1.0..1.0),
            );
        }
    }
}

/// Spawn a CHAIN REACTION setup - dominos trigger balls trigger explosions!
pub fn spawn_chain_reaction(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    // Stage 1: Domino line
    spawn_dominos(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        position,
        15,
        Vec3::X,
    );

    // Stage 2: Ramp that dominos push a ball down
    let ramp_pos = position + Vec3::new(6.0, 0.0, 0.0);
    spawn_ramp(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        ramp_pos,
        Vec3::X,
        4.0,
        20.0,
    );

    // Ball on ramp
    spawn_sphere(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        ramp_pos + Vec3::new(-1.5, 1.5, 0.0),
        0.4,
        Color::srgb(0.9, 0.3, 0.1),
        PhysicsMaterialType::Heavy,
    );

    // Stage 3: Tower to knock down
    let tower_pos = position + Vec3::new(12.0, 0.0, 0.0);
    spawn_tower(nova, commands, meshes, materials, handle_to_entity, tower_pos, 8);

    // Stage 4: Bowling pins at the end
    let pins_pos = position + Vec3::new(16.0, 0.0, 0.0);
    spawn_bowling(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        pins_pos,
        Vec3::X,
    );
}

/// Spawn a GYROSCOPE - spinning rings!
pub fn spawn_gyroscope(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let outer_radius = 2.0;
    let ring_thickness = 0.15;

    // Static center anchor
    let center_body = nova.world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(position))
        .build();
    let center_handle = nova.world.insert_body(center_body);

    // Center sphere
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.3))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.8, 0.1),
            metallic: 0.9,
            ..default()
        })),
        Transform::from_translation(position),
        SpawnedObject,
    ));

    // Create 3 nested rings with different rotation axes
    let ring_configs = [
        (outer_radius, Color::srgb(0.9, 0.2, 0.2), nova::prelude::Vec3::Y),
        (outer_radius * 0.75, Color::srgb(0.2, 0.9, 0.2), nova::prelude::Vec3::X),
        (outer_radius * 0.5, Color::srgb(0.2, 0.2, 0.9), nova::prelude::Vec3::Z),
    ];

    for (radius, color, axis) in ring_configs {
        // Create ring as a series of spheres
        let sphere_count = 16;
        let mut ring_spheres = Vec::new();

        for i in 0..sphere_count {
            let angle = (i as f32 / sphere_count as f32) * std::f32::consts::TAU;
            let local_pos = Vec3::new(angle.cos() * radius, 0.0, angle.sin() * radius);
            let world_pos = position + local_pos;

            let sphere_body = nova.world
                .create_body()
                .body_type(RigidBodyType::Dynamic)
                .position(to_nova_vec3(world_pos))
                .mass(0.2)
                .build();
            let sphere_handle = nova.world.insert_body(sphere_body);

            nova.world.insert_collider(
                nova.world.create_collider(
                    sphere_handle,
                    CollisionShape::Sphere(SphereShape::new(ring_thickness)),
                )
                .build()
            );

            commands.spawn((
                Mesh3d(meshes.add(Sphere::new(ring_thickness))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: color,
                    metallic: 0.7,
                    ..default()
                })),
                Transform::from_translation(world_pos),
                SpawnedObject,
            ));

            ring_spheres.push(sphere_handle);
        }

        // Connect spheres to center and each other with distance joints
        for (i, &handle) in ring_spheres.iter().enumerate() {
            nova.world.create_distance_joint(
                center_handle,
                handle,
                nova::prelude::Vec3::ZERO,
                nova::prelude::Vec3::ZERO,
                radius,
            );

            // Connect to next sphere in ring
            let next_handle = ring_spheres[(i + 1) % sphere_count];
            let arc_length = std::f32::consts::TAU * radius / sphere_count as f32;
            nova.world.create_distance_joint(
                handle,
                next_handle,
                nova::prelude::Vec3::ZERO,
                nova::prelude::Vec3::ZERO,
                arc_length,
            );
        }
    }
}

/// Spawn a HAMSTER WHEEL - giant running wheel!
pub fn spawn_hamster_wheel(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let wheel_radius = 3.0;
    let wheel_width = 2.0;
    let spoke_count = 12;

    // Static axle
    let axle_body = nova.world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(position))
        .build();
    let axle_handle = nova.world.insert_body(axle_body);

    // Axle visual
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.15, wheel_width * 1.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.4, 0.4, 0.45),
            metallic: 0.8,
            ..default()
        })),
        Transform::from_translation(position)
            .with_rotation(Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
        SpawnedObject,
    ));

    // Create wheel rim (dynamic, connected to axle with hinge)
    let rim_spheres_count = 24;
    let mut rim_handles = Vec::new();

    for i in 0..rim_spheres_count {
        let angle = (i as f32 / rim_spheres_count as f32) * std::f32::consts::TAU;
        let rim_pos = position + Vec3::new(angle.cos() * wheel_radius, angle.sin() * wheel_radius, 0.0);

        let rim_body = nova.world
            .create_body()
            .body_type(RigidBodyType::Dynamic)
            .position(to_nova_vec3(rim_pos))
            .mass(0.5)
            .build();
        let rim_handle = nova.world.insert_body(rim_body);

        nova.world.insert_collider(
            nova.world.create_collider(
                rim_handle,
                CollisionShape::Sphere(SphereShape::new(0.2)),
            )
            .friction(0.8)
            .build()
        );

        commands.spawn((
            Mesh3d(meshes.add(Sphere::new(0.2))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.6, 0.5, 0.3),
                ..default()
            })),
            Transform::from_translation(rim_pos),
            SpawnedObject,
        ));

        // Connect to axle
        nova.world.create_distance_joint(
            axle_handle,
            rim_handle,
            nova::prelude::Vec3::ZERO,
            nova::prelude::Vec3::ZERO,
            wheel_radius,
        );

        rim_handles.push(rim_handle);
    }

    // Connect rim pieces together
    for i in 0..rim_spheres_count {
        let next_i = (i + 1) % rim_spheres_count;
        let arc_length = std::f32::consts::TAU * wheel_radius / rim_spheres_count as f32;
        nova.world.create_distance_joint(
            rim_handles[i],
            rim_handles[next_i],
            nova::prelude::Vec3::ZERO,
            nova::prelude::Vec3::ZERO,
            arc_length,
        );
    }

    // Add some balls inside to run!
    let mut rng = rand::thread_rng();
    for _ in 0..5 {
        spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            position + Vec3::new(rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0), 0.0),
            0.3,
            Color::hsl(rng.gen_range(0.0..360.0), 0.8, 0.5),
            PhysicsMaterialType::Bouncy,
        );
    }
}

/// Spawn a CHAOS CUBE - exploding cube of balls!
pub fn spawn_chaos_cube(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    size: usize,
) {
    let mut rng = rand::thread_rng();
    let spacing = 0.6;

    for x in 0..size {
        for y in 0..size {
            for z in 0..size {
                let ball_pos = position + Vec3::new(
                    (x as f32 - size as f32 * 0.5) * spacing,
                    (y as f32) * spacing + 0.5,
                    (z as f32 - size as f32 * 0.5) * spacing,
                );

                let color = Color::hsl(rng.gen_range(0.0..360.0), 0.8, 0.5);
                let radius = rng.gen_range(0.15..0.25);

                let (_, body_handle) = spawn_sphere(
                    nova,
                    commands,
                    meshes,
                    materials,
                    handle_to_entity,
                    ball_pos,
                    radius,
                    color,
                    PhysicsMaterialType::Bouncy,
                );

                // Random explosion velocities!
                if let Some(body) = nova.world.get_body_mut(body_handle) {
                    body.linear_velocity = nova::prelude::Vec3::new(
                        rng.gen_range(-8.0..8.0),
                        rng.gen_range(5.0..15.0),
                        rng.gen_range(-8.0..8.0),
                    );
                }
            }
        }
    }
}

/// Spawn a DOUBLE PENDULUM - chaotic motion!
pub fn spawn_double_pendulum(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let arm1_length = 2.0;
    let arm2_length = 1.5;
    let ball1_radius = 0.4;
    let ball2_radius = 0.3;

    // Static anchor
    let anchor_body = nova.world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(to_nova_vec3(position))
        .build();
    let anchor_handle = nova.world.insert_body(anchor_body);

    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(0.2))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.3, 0.3, 0.3),
            ..default()
        })),
        Transform::from_translation(position),
        SpawnedObject,
    ));

    // First pendulum arm + ball (start at angle for motion)
    let ball1_pos = position + Vec3::new(arm1_length * 0.7, -arm1_length * 0.7, 0.0);
    let (_, ball1_handle) = spawn_sphere(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        ball1_pos,
        ball1_radius,
        Color::srgb(0.9, 0.3, 0.1),
        PhysicsMaterialType::Heavy,
    );

    nova.world.create_distance_joint(
        anchor_handle,
        ball1_handle,
        nova::prelude::Vec3::ZERO,
        nova::prelude::Vec3::ZERO,
        arm1_length,
    );

    // Second pendulum arm + ball
    let ball2_pos = ball1_pos + Vec3::new(arm2_length, 0.0, 0.0);
    let (_, ball2_handle) = spawn_sphere(
        nova,
        commands,
        meshes,
        materials,
        handle_to_entity,
        ball2_pos,
        ball2_radius,
        Color::srgb(0.1, 0.3, 0.9),
        PhysicsMaterialType::Normal,
    );

    nova.world.create_distance_joint(
        ball1_handle,
        ball2_handle,
        nova::prelude::Vec3::ZERO,
        nova::prelude::Vec3::ZERO,
        arm2_length,
    );
}

/// Spawn METEOR SHOWER - balls raining from above!
pub fn spawn_meteor_shower(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    count: usize,
) {
    let mut rng = rand::thread_rng();

    for i in 0..count {
        let offset = Vec3::new(
            rng.gen_range(-10.0..10.0),
            rng.gen_range(15.0..30.0) + i as f32 * 0.5,
            rng.gen_range(-10.0..10.0),
        );

        let radius = rng.gen_range(0.3..0.8);

        // Fiery meteor colors
        let color = Color::hsl(rng.gen_range(0.0..30.0), 0.9, rng.gen_range(0.4..0.6));

        let (_, body_handle) = spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            position + offset,
            radius,
            color,
            PhysicsMaterialType::Heavy,
        );

        // Downward velocity!
        if let Some(body) = nova.world.get_body_mut(body_handle) {
            body.linear_velocity = nova::prelude::Vec3::new(
                rng.gen_range(-2.0..2.0),
                rng.gen_range(-15.0..-5.0),
                rng.gen_range(-2.0..2.0),
            );
        }
    }
}

// ============ ULTRA MEGA BONKERS MODE ============

/// Spawn a BALL FOUNTAIN - continuous upward stream!
pub fn spawn_ball_fountain(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    ball_count: usize,
) {
    let mut rng = rand::thread_rng();

    // Fountain base (visual)
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(1.0, 0.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.4, 0.4, 0.5),
            metallic: 0.8,
            ..default()
        })),
        Transform::from_translation(position),
        SpawnedObject,
    ));

    // Fountain spout
    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.3, 1.5))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.5, 0.5, 0.6),
            metallic: 0.9,
            ..default()
        })),
        Transform::from_translation(position + Vec3::Y * 1.0),
        SpawnedObject,
    ));

    // Spawn balls shooting up!
    for i in 0..ball_count {
        let angle = rng.gen_range(0.0..std::f32::consts::TAU);
        let spread = rng.gen_range(0.0..0.3);
        let height = 2.0 + i as f32 * 0.2;

        let ball_pos = position + Vec3::new(
            angle.cos() * spread,
            height,
            angle.sin() * spread,
        );

        // Water colors - blues and whites
        let color = Color::hsl(rng.gen_range(190.0..220.0), 0.7, rng.gen_range(0.5..0.8));
        let radius = rng.gen_range(0.1..0.2);

        let (_, body_handle) = spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            ball_pos,
            radius,
            color,
            PhysicsMaterialType::Bouncy,
        );

        // Shoot them up!
        if let Some(body) = nova.world.get_body_mut(body_handle) {
            body.linear_velocity = nova::prelude::Vec3::new(
                rng.gen_range(-2.0..2.0),
                rng.gen_range(10.0..18.0),
                rng.gen_range(-2.0..2.0),
            );
        }
    }
}

/// Spawn FIREWORKS - exploding clusters!
pub fn spawn_fireworks(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    burst_count: usize,
) {
    let mut rng = rand::thread_rng();

    for burst in 0..burst_count {
        // Random burst center
        let burst_center = position + Vec3::new(
            rng.gen_range(-5.0..5.0),
            rng.gen_range(8.0..15.0),
            rng.gen_range(-5.0..5.0),
        );

        // Random hue for this burst
        let hue = rng.gen_range(0.0..360.0);

        // Create explosion of balls from this point
        let balls_per_burst = 20;
        for _ in 0..balls_per_burst {
            let dir = Vec3::new(
                rng.gen_range(-1.0..1.0),
                rng.gen_range(-0.5..1.0),
                rng.gen_range(-1.0..1.0),
            ).normalize_or_zero();

            let speed = rng.gen_range(5.0..12.0);
            let radius = rng.gen_range(0.08..0.15);

            // Slight hue variation within burst
            let color = Color::hsl(hue + rng.gen_range(-20.0..20.0), 0.9, 0.6);

            let (_, body_handle) = spawn_sphere(
                nova,
                commands,
                meshes,
                materials,
                handle_to_entity,
                burst_center,
                radius,
                color,
                PhysicsMaterialType::Light,
            );

            if let Some(body) = nova.world.get_body_mut(body_handle) {
                body.linear_velocity = nova::prelude::Vec3::new(
                    dir.x * speed,
                    dir.y * speed + 5.0, // Extra upward boost
                    dir.z * speed,
                );
            }
        }
    }
}

/// Spawn a BOUNCY CASTLE - walls of bouncy spheres!
pub fn spawn_bouncy_castle(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let wall_height = 4;
    let wall_length = 8;
    let ball_radius = 0.4;
    let spacing = ball_radius * 2.2;

    // Four walls
    let walls = [
        (Vec3::X, Vec3::Z),   // Front wall
        (Vec3::NEG_X, Vec3::Z), // Back wall
        (Vec3::Z, Vec3::X),   // Right wall
        (Vec3::NEG_Z, Vec3::X), // Left wall
    ];

    for (normal, along) in walls {
        let wall_center = position + normal * (wall_length as f32 * spacing * 0.5);

        for y in 0..wall_height {
            for i in 0..wall_length {
                let offset = (i as f32 - wall_length as f32 * 0.5) * spacing;
                let ball_pos = wall_center + along * offset + Vec3::Y * (y as f32 * spacing + ball_radius);

                // Rainbow castle!
                let color = Color::hsl((y * 30 + i * 15) as f32 % 360.0, 0.8, 0.6);

                // Static bouncy walls
                let body = nova.world
                    .create_body()
                    .body_type(RigidBodyType::Static)
                    .position(to_nova_vec3(ball_pos))
                    .build();
                let body_handle = nova.world.insert_body(body);

                nova.world.insert_collider(
                    nova.world.create_collider(
                        body_handle,
                        CollisionShape::Sphere(SphereShape::new(ball_radius)),
                    )
                    .friction(0.1)
                    .restitution(0.95) // SUPER BOUNCY!
                    .build()
                );

                commands.spawn((
                    Mesh3d(meshes.add(Sphere::new(ball_radius))),
                    MeshMaterial3d(materials.add(StandardMaterial {
                        base_color: color,
                        ..default()
                    })),
                    Transform::from_translation(ball_pos),
                    SpawnedObject,
                ));
            }
        }
    }

    // Add some balls inside to bounce!
    let mut rng = rand::thread_rng();
    for _ in 0..15 {
        spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            position + Vec3::new(
                rng.gen_range(-2.0..2.0),
                rng.gen_range(1.0..3.0),
                rng.gen_range(-2.0..2.0),
            ),
            0.3,
            Color::srgb(0.9, 0.9, 0.9),
            PhysicsMaterialType::Bouncy,
        );
    }
}

/// Spawn a MARBLE RUN - ramps and obstacles!
pub fn spawn_marble_run(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    // Series of ramps going down in a zigzag
    let ramp_count = 6;
    let ramp_length = 4.0;
    let ramp_drop = 1.5;

    for i in 0..ramp_count {
        let direction = if i % 2 == 0 { Vec3::X } else { Vec3::NEG_X };
        let ramp_pos = position + Vec3::new(
            if i % 2 == 0 { 0.0 } else { ramp_length * 0.8 },
            -(i as f32 * ramp_drop),
            0.0,
        );

        spawn_ramp(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            ramp_pos,
            direction,
            ramp_length,
            15.0,
        );
    }

    // Add some pegs/obstacles
    let mut rng = rand::thread_rng();
    for i in 0..20 {
        let peg_pos = position + Vec3::new(
            rng.gen_range(-1.0..ramp_length + 1.0),
            -(i as f32 * 0.4) - 0.5,
            rng.gen_range(-0.5..0.5),
        );

        let peg_body = nova.world
            .create_body()
            .body_type(RigidBodyType::Static)
            .position(to_nova_vec3(peg_pos))
            .build();
        let peg_handle = nova.world.insert_body(peg_body);

        nova.world.insert_collider(
            nova.world.create_collider(
                peg_handle,
                CollisionShape::Sphere(SphereShape::new(0.15)),
            )
            .restitution(0.7)
            .build()
        );

        commands.spawn((
            Mesh3d(meshes.add(Sphere::new(0.15))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.7, 0.6, 0.2),
                metallic: 0.9,
                ..default()
            })),
            Transform::from_translation(peg_pos),
            SpawnedObject,
        ));
    }

    // Drop marbles from the top!
    for i in 0..8 {
        spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            position + Vec3::new(rng.gen_range(-0.5..0.5), 2.0 + i as f32 * 0.5, 0.0),
            0.2,
            Color::hsl(rng.gen_range(0.0..360.0), 0.8, 0.5),
            PhysicsMaterialType::Normal,
        );
    }
}

/// Spawn BALL CANNON - rapid fire balls!
pub fn spawn_ball_cannon(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    direction: Vec3,
    ball_count: usize,
) {
    let mut rng = rand::thread_rng();
    let dir = direction.normalize_or_zero();

    // Cannon barrel (visual)
    let barrel_length = 2.0;
    let rotation = Quat::from_rotation_arc(Vec3::Z, dir);

    commands.spawn((
        Mesh3d(meshes.add(Cylinder::new(0.3, barrel_length))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.2, 0.25),
            metallic: 0.9,
            ..default()
        })),
        Transform::from_translation(position + dir * barrel_length * 0.5)
            .with_rotation(rotation * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)),
        SpawnedObject,
    ));

    // Fire balls!
    for i in 0..ball_count {
        let spread = Vec3::new(
            rng.gen_range(-0.2..0.2),
            rng.gen_range(-0.2..0.2),
            rng.gen_range(-0.2..0.2),
        );

        let ball_pos = position + dir * (barrel_length + 0.5 + i as f32 * 0.3);
        let speed = rng.gen_range(15.0..25.0);

        let color = Color::hsl(rng.gen_range(0.0..60.0), 0.9, 0.5);

        let (_, body_handle) = spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            ball_pos,
            0.2,
            color,
            PhysicsMaterialType::Heavy,
        );

        if let Some(body) = nova.world.get_body_mut(body_handle) {
            let vel_dir = (dir + spread).normalize_or_zero();
            body.linear_velocity = nova::prelude::Vec3::new(
                vel_dir.x * speed,
                vel_dir.y * speed,
                vel_dir.z * speed,
            );
        }
    }
}

/// Spawn ULTIMATE DESTRUCTION - everything at once!
pub fn spawn_ultimate_destruction(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    // Center tower
    spawn_tower(nova, commands, meshes, materials, handle_to_entity, position, 15);

    // Surrounding pyramids
    for angle in [0.0, 90.0, 180.0, 270.0_f32] {
        let rad = angle.to_radians();
        let offset = Vec3::new(rad.cos() * 8.0, 0.0, rad.sin() * 8.0);
        spawn_pyramid(nova, commands, meshes, materials, handle_to_entity, position + offset, 4);
    }

    // Wrecking balls from multiple directions
    spawn_wrecking_ball(nova, commands, meshes, materials, handle_to_entity, position + Vec3::new(-12.0, 10.0, 0.0), 8);
    spawn_wrecking_ball(nova, commands, meshes, materials, handle_to_entity, position + Vec3::new(12.0, 10.0, 0.0), 8);

    // Ball avalanche from above!
    spawn_avalanche(nova, commands, meshes, materials, handle_to_entity, position + Vec3::new(0.0, 15.0, 0.0), 50);
}

/// Spawn PENDULUM WAVE - mesmerizing wave motion!
pub fn spawn_pendulum_wave(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    pendulum_count: usize,
) {
    let base_length = 3.0;
    let length_increment = 0.1;
    let spacing = 0.5;
    let ball_radius = 0.2;

    for i in 0..pendulum_count {
        let string_length = base_length + i as f32 * length_increment;
        let x_offset = (i as f32 - pendulum_count as f32 * 0.5) * spacing;

        // Static anchor
        let anchor_body = nova.world
            .create_body()
            .body_type(RigidBodyType::Static)
            .position(to_nova_vec3(position + Vec3::new(x_offset, string_length, 0.0)))
            .build();
        let anchor_handle = nova.world.insert_body(anchor_body);

        // Pendulum ball (start at angle)
        let start_angle = std::f32::consts::FRAC_PI_4; // 45 degrees
        let ball_pos = position + Vec3::new(
            x_offset + string_length * start_angle.sin(),
            string_length * (1.0 - start_angle.cos()),
            0.0,
        );

        let color = Color::hsl((i as f32 / pendulum_count as f32) * 360.0, 0.8, 0.5);

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

/// Spawn BALL TORNADO - spinning vortex of balls!
pub fn spawn_ball_tornado(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
    ball_count: usize,
) {
    let mut rng = rand::thread_rng();
    let height = 10.0;

    for i in 0..ball_count {
        // Spiral arrangement
        let t = i as f32 / ball_count as f32;
        let angle = t * std::f32::consts::TAU * 5.0; // 5 rotations
        let radius = 0.5 + t * 3.0; // Expanding spiral
        let y = t * height;

        let ball_pos = position + Vec3::new(
            angle.cos() * radius,
            y,
            angle.sin() * radius,
        );

        let color = Color::hsl((t * 360.0) % 360.0, 0.7, 0.5);
        let ball_radius = rng.gen_range(0.1..0.25);

        let (_, body_handle) = spawn_sphere(
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

        // Give tangential velocity for spinning effect
        if let Some(body) = nova.world.get_body_mut(body_handle) {
            let tangent = Vec3::new(-angle.sin(), 0.3, angle.cos());
            let speed = 3.0 + radius * 2.0;
            body.linear_velocity = nova::prelude::Vec3::new(
                tangent.x * speed,
                tangent.y * speed + rng.gen_range(0.0..2.0),
                tangent.z * speed,
            );
        }
    }
}

// ============ SCIENCE & FUN PRESETS ============

/// Spawn BUCKYBALL (Buckminsterfullerene C60) - 60 spheres in a truncated icosahedron
pub fn spawn_buckyball(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let radius = 2.5; // Overall radius
    let ball_radius = 0.15;

    // Golden ratio for icosahedron
    let phi = (1.0 + 5.0_f32.sqrt()) / 2.0;

    // Generate vertices of a truncated icosahedron (buckyball shape)
    // We'll use a simplified approximation with 32 vertices arranged nicely
    let mut vertices = Vec::new();

    // Create vertices on sphere surface using fibonacci spiral
    let n = 32;
    for i in 0..n {
        let y = 1.0 - (i as f32 / (n - 1) as f32) * 2.0;
        let r = (1.0 - y * y).sqrt();
        let theta = phi * i as f32 * std::f32::consts::TAU;

        vertices.push(Vec3::new(
            theta.cos() * r * radius,
            y * radius,
            theta.sin() * r * radius,
        ));
    }

    let mut handles = Vec::new();

    // Spawn spheres at each vertex
    for (i, &vert) in vertices.iter().enumerate() {
        let color = Color::hsl((i as f32 / n as f32) * 60.0 + 180.0, 0.7, 0.55); // Blue-green

        let (_, handle) = spawn_sphere(
            nova,
            commands,
            meshes,
            materials,
            handle_to_entity,
            position + vert,
            ball_radius,
            color,
            PhysicsMaterialType::Normal,
        );
        handles.push(handle);
    }

    // Connect nearby vertices with distance joints
    for i in 0..handles.len() {
        for j in (i + 1)..handles.len() {
            let dist = (vertices[i] - vertices[j]).length();
            if dist < radius * 0.7 { // Connect if close enough
                nova.world.create_distance_joint(
                    handles[i],
                    handles[j],
                    nova::prelude::Vec3::ZERO,
                    nova::prelude::Vec3::ZERO,
                    dist,
                );
            }
        }
    }
}

/// Spawn DNA HELIX - double helix structure
pub fn spawn_dna_helix(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let turns = 3.0;
    let height = 8.0;
    let radius = 1.2;
    let spheres_per_strand = 24;
    let ball_radius = 0.2;

    let mut strand1_handles = Vec::new();
    let mut strand2_handles = Vec::new();

    for i in 0..spheres_per_strand {
        let t = i as f32 / spheres_per_strand as f32;
        let angle = t * turns * std::f32::consts::TAU;
        let y = t * height;

        // Strand 1 (red/orange)
        let pos1 = position + Vec3::new(angle.cos() * radius, y, angle.sin() * radius);
        let color1 = Color::hsl(t * 30.0, 0.8, 0.5); // Red to orange

        let (_, h1) = spawn_sphere(
            nova, commands, meshes, materials, handle_to_entity,
            pos1, ball_radius, color1, PhysicsMaterialType::Normal,
        );
        strand1_handles.push(h1);

        // Strand 2 (blue/purple) - opposite side
        let pos2 = position + Vec3::new(-angle.cos() * radius, y, -angle.sin() * radius);
        let color2 = Color::hsl(220.0 + t * 30.0, 0.8, 0.5); // Blue to purple

        let (_, h2) = spawn_sphere(
            nova, commands, meshes, materials, handle_to_entity,
            pos2, ball_radius, color2, PhysicsMaterialType::Normal,
        );
        strand2_handles.push(h2);

        // Connect the two strands (base pairs)
        let pair_dist = (pos1 - pos2).length();
        nova.world.create_distance_joint(
            h1, h2,
            nova::prelude::Vec3::ZERO,
            nova::prelude::Vec3::ZERO,
            pair_dist,
        );
    }

    // Connect along each strand
    for i in 1..spheres_per_strand {
        let spacing = height / spheres_per_strand as f32;
        nova.world.create_distance_joint(
            strand1_handles[i - 1], strand1_handles[i],
            nova::prelude::Vec3::ZERO, nova::prelude::Vec3::ZERO,
            spacing * 1.2,
        );
        nova.world.create_distance_joint(
            strand2_handles[i - 1], strand2_handles[i],
            nova::prelude::Vec3::ZERO, nova::prelude::Vec3::ZERO,
            spacing * 1.2,
        );
    }
}

/// Spawn SOLAR SYSTEM - planets orbiting a central sun
pub fn spawn_solar_system(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    // Sun (heavy, central)
    let sun_pos = position + Vec3::new(0.0, 3.0, 0.0);
    let (_, sun_handle) = spawn_sphere(
        nova, commands, meshes, materials, handle_to_entity,
        sun_pos, 1.0,
        Color::srgb(1.0, 0.8, 0.2), // Yellow
        PhysicsMaterialType::Heavy,
    );

    // Make sun very heavy
    if let Some(body) = nova.world.get_body_mut(sun_handle) {
        body.mass = 50.0;
        body.inv_mass = 1.0 / 50.0;
    }

    // Planets with orbital data: (distance, radius, color, orbital_speed)
    let planets = [
        (2.5, 0.15, Color::srgb(0.6, 0.6, 0.6), 4.0),   // Mercury
        (3.5, 0.25, Color::srgb(0.9, 0.7, 0.5), 3.0),   // Venus
        (4.5, 0.28, Color::srgb(0.2, 0.5, 0.8), 2.5),   // Earth
        (5.5, 0.2, Color::srgb(0.8, 0.3, 0.2), 2.0),    // Mars
        (7.0, 0.5, Color::srgb(0.8, 0.6, 0.4), 1.2),    // Jupiter
        (9.0, 0.45, Color::srgb(0.9, 0.8, 0.5), 0.9),   // Saturn
    ];

    for (i, (dist, rad, color, speed)) in planets.iter().enumerate() {
        let angle = (i as f32 / planets.len() as f32) * std::f32::consts::TAU;
        let planet_pos = sun_pos + Vec3::new(angle.cos() * dist, 0.0, angle.sin() * dist);

        let (_, planet_handle) = spawn_sphere(
            nova, commands, meshes, materials, handle_to_entity,
            planet_pos, *rad, *color, PhysicsMaterialType::Normal,
        );

        // Give orbital velocity
        if let Some(body) = nova.world.get_body_mut(planet_handle) {
            let tangent = Vec3::new(-angle.sin(), 0.0, angle.cos());
            body.linear_velocity = nova::prelude::Vec3::new(
                tangent.x * speed,
                0.0,
                tangent.z * speed,
            );
        }
    }
}

/// Spawn ATOM model - nucleus with electron shells
pub fn spawn_atom(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    // Nucleus (protons + neutrons)
    let nucleus_pos = position + Vec3::new(0.0, 2.0, 0.0);
    let mut rng = rand::thread_rng();

    // Create nucleus cluster
    for _ in 0..6 {
        let offset = Vec3::new(
            rng.gen_range(-0.2..0.2),
            rng.gen_range(-0.2..0.2),
            rng.gen_range(-0.2..0.2),
        );
        let is_proton = rng.gen_bool(0.5);
        let color = if is_proton {
            Color::srgb(0.9, 0.2, 0.2) // Red proton
        } else {
            Color::srgb(0.2, 0.2, 0.9) // Blue neutron
        };

        spawn_sphere(
            nova, commands, meshes, materials, handle_to_entity,
            nucleus_pos + offset, 0.2, color, PhysicsMaterialType::Heavy,
        );
    }

    // Electron shells
    let shells = [(1.5, 2), (2.5, 4), (3.5, 6)]; // (radius, electron_count)

    for (shell_radius, electron_count) in shells {
        for i in 0..electron_count {
            let angle = (i as f32 / electron_count as f32) * std::f32::consts::TAU;
            let tilt = rng.gen_range(0.0..std::f32::consts::PI);

            let electron_pos = nucleus_pos + Vec3::new(
                angle.cos() * tilt.sin() * shell_radius,
                tilt.cos() * shell_radius * 0.5,
                angle.sin() * tilt.sin() * shell_radius,
            );

            let (_, e_handle) = spawn_sphere(
                nova, commands, meshes, materials, handle_to_entity,
                electron_pos, 0.1,
                Color::srgb(1.0, 1.0, 0.2), // Yellow electron
                PhysicsMaterialType::Bouncy,
            );

            // Give orbital velocity
            if let Some(body) = nova.world.get_body_mut(e_handle) {
                let tangent = Vec3::new(-angle.sin(), 0.0, angle.cos());
                let speed = 5.0 / shell_radius;
                body.linear_velocity = nova::prelude::Vec3::new(
                    tangent.x * speed,
                    rng.gen_range(-1.0..1.0),
                    tangent.z * speed,
                );
            }
        }
    }
}

/// Spawn GEODESIC DOME - triangulated dome structure
pub fn spawn_geodesic_dome(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let radius = 3.0;
    let ball_radius = 0.15;

    // Generate icosahedron vertices (basis for geodesic dome)
    let phi = (1.0 + 5.0_f32.sqrt()) / 2.0;
    let vertices: Vec<Vec3> = vec![
        Vec3::new(-1.0, phi, 0.0),
        Vec3::new(1.0, phi, 0.0),
        Vec3::new(-1.0, -phi, 0.0),
        Vec3::new(1.0, -phi, 0.0),
        Vec3::new(0.0, -1.0, phi),
        Vec3::new(0.0, 1.0, phi),
        Vec3::new(0.0, -1.0, -phi),
        Vec3::new(0.0, 1.0, -phi),
        Vec3::new(phi, 0.0, -1.0),
        Vec3::new(phi, 0.0, 1.0),
        Vec3::new(-phi, 0.0, -1.0),
        Vec3::new(-phi, 0.0, 1.0),
    ].into_iter()
    .map(|v| v.normalize() * radius)
    .filter(|v| v.y >= -0.2) // Only top half for dome
    .collect();

    let mut handles = Vec::new();

    for (i, &vert) in vertices.iter().enumerate() {
        let color = Color::hsl((i as f32 / vertices.len() as f32) * 120.0 + 60.0, 0.6, 0.5);

        let (_, h) = spawn_sphere(
            nova, commands, meshes, materials, handle_to_entity,
            position + vert + Vec3::Y * radius,
            ball_radius, color, PhysicsMaterialType::Normal,
        );
        handles.push((h, vert));
    }

    // Connect vertices that are close
    for i in 0..handles.len() {
        for j in (i + 1)..handles.len() {
            let dist = (handles[i].1 - handles[j].1).length();
            if dist < radius * 0.8 {
                nova.world.create_distance_joint(
                    handles[i].0, handles[j].0,
                    nova::prelude::Vec3::ZERO, nova::prelude::Vec3::ZERO,
                    dist,
                );
            }
        }
    }
}

/// Spawn TENSEGRITY structure - floating compression structure
pub fn spawn_tensegrity(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let size = 2.0;

    // Tensegrity uses rigid struts (boxes) and tension cables (joints)
    // Classic 3-strut tensegrity prism

    // Top triangle vertices
    let top_verts = [
        Vec3::new(0.0, size, size * 0.866),
        Vec3::new(size * 0.75, size, -size * 0.433),
        Vec3::new(-size * 0.75, size, -size * 0.433),
    ];

    // Bottom triangle vertices (rotated 60 degrees)
    let bottom_verts = [
        Vec3::new(size * 0.75, 0.0, size * 0.433),
        Vec3::new(-size * 0.75, 0.0, size * 0.433),
        Vec3::new(0.0, 0.0, -size * 0.866),
    ];

    let mut top_handles = Vec::new();
    let mut bottom_handles = Vec::new();

    // Spawn corner nodes
    for (i, &v) in top_verts.iter().enumerate() {
        let (_, h) = spawn_sphere(
            nova, commands, meshes, materials, handle_to_entity,
            position + v + Vec3::Y * 2.0,
            0.2, Color::srgb(0.9, 0.3, 0.3), PhysicsMaterialType::Normal,
        );
        top_handles.push(h);
    }

    for (i, &v) in bottom_verts.iter().enumerate() {
        let (_, h) = spawn_sphere(
            nova, commands, meshes, materials, handle_to_entity,
            position + v + Vec3::Y * 2.0,
            0.2, Color::srgb(0.3, 0.3, 0.9), PhysicsMaterialType::Normal,
        );
        bottom_handles.push(h);
    }

    // Connect with distance joints (cables)
    // Top triangle
    for i in 0..3 {
        let next = (i + 1) % 3;
        let dist = (top_verts[i] - top_verts[next]).length();
        nova.world.create_distance_joint(
            top_handles[i], top_handles[next],
            nova::prelude::Vec3::ZERO, nova::prelude::Vec3::ZERO,
            dist,
        );
    }

    // Bottom triangle
    for i in 0..3 {
        let next = (i + 1) % 3;
        let dist = (bottom_verts[i] - bottom_verts[next]).length();
        nova.world.create_distance_joint(
            bottom_handles[i], bottom_handles[next],
            nova::prelude::Vec3::ZERO, nova::prelude::Vec3::ZERO,
            dist,
        );
    }

    // Diagonal struts (the magic of tensegrity!)
    for i in 0..3 {
        let dist = (top_verts[i] - bottom_verts[i]).length();
        nova.world.create_distance_joint(
            top_handles[i], bottom_handles[i],
            nova::prelude::Vec3::ZERO, nova::prelude::Vec3::ZERO,
            dist,
        );

        // Cross cables
        let next = (i + 1) % 3;
        let cross_dist = (top_verts[i] - bottom_verts[next]).length();
        nova.world.create_distance_joint(
            top_handles[i], bottom_handles[next],
            nova::prelude::Vec3::ZERO, nova::prelude::Vec3::ZERO,
            cross_dist,
        );
    }
}

/// Spawn MOBIUS CHAIN - twisted loop of connected spheres
pub fn spawn_mobius_chain(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    let major_radius = 2.5; // Ring radius
    let minor_radius = 0.5; // Tube radius
    let count = 36;
    let ball_radius = 0.15;

    let mut handles = Vec::new();

    for i in 0..count {
        let t = i as f32 / count as f32;
        let u = t * std::f32::consts::TAU; // Around the ring
        let v = t * std::f32::consts::PI; // Half twist (mobius)

        // Mobius strip parametric equations
        let x = (major_radius + minor_radius * v.cos()) * u.cos();
        let y = minor_radius * v.sin();
        let z = (major_radius + minor_radius * v.cos()) * u.sin();

        let color = Color::hsl(t * 360.0, 0.7, 0.5);

        let (_, h) = spawn_sphere(
            nova, commands, meshes, materials, handle_to_entity,
            position + Vec3::new(x, y + 3.0, z),
            ball_radius, color, PhysicsMaterialType::Normal,
        );
        handles.push(h);
    }

    // Connect in a loop
    for i in 0..count {
        let next = (i + 1) % count;
        nova.world.create_distance_joint(
            handles[i], handles[next],
            nova::prelude::Vec3::ZERO, nova::prelude::Vec3::ZERO,
            0.5,
        );
    }
}

/// Spawn PENDULUM CLOCK - classic weighted pendulum
pub fn spawn_pendulum_clock(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut HandleToEntity,
    position: Vec3,
) {
    // Clock frame (static anchor)
    let anchor_pos = position + Vec3::new(0.0, 5.0, 0.0);
    let anchor_body = nova.world.create_body()
        .body_type(nova::prelude::RigidBodyType::Static)
        .position(nova::prelude::Vec3::new(anchor_pos.x, anchor_pos.y, anchor_pos.z))
        .build();
    let anchor_handle = nova.world.insert_body(anchor_body);

    // Pendulum rod
    let rod_length = 3.0;
    let rod_pos = anchor_pos - Vec3::Y * (rod_length / 2.0);

    let (_, rod_handle) = spawn_box(
        nova, commands, meshes, materials, handle_to_entity,
        rod_pos,
        Vec3::new(0.05, rod_length / 2.0, 0.05),
        Color::srgb(0.4, 0.3, 0.2), // Brown wood
        PhysicsMaterialType::Normal,
    );

    // Heavy pendulum bob
    let bob_pos = anchor_pos - Vec3::Y * rod_length;
    let (_, bob_handle) = spawn_sphere(
        nova, commands, meshes, materials, handle_to_entity,
        bob_pos,
        0.4,
        Color::srgb(0.8, 0.7, 0.2), // Brass colored
        PhysicsMaterialType::Heavy,
    );

    // Make bob extra heavy
    if let Some(body) = nova.world.get_body_mut(bob_handle) {
        body.mass = 10.0;
        body.inv_mass = 0.1;
    }

    // Connect rod to anchor with hinge
    nova.world.create_hinge_joint(
        anchor_handle, rod_handle,
        nova::prelude::Vec3::ZERO,
        nova::prelude::Vec3::new(0.0, rod_length / 2.0, 0.0),
        nova::prelude::Vec3::new(0.0, 0.0, 1.0), // Swing on Z axis
    );

    // Connect bob to rod
    nova.world.create_fixed_joint(
        rod_handle, bob_handle,
        nova::prelude::Vec3::new(0.0, -rod_length / 2.0, 0.0),
        nova::prelude::Vec3::ZERO,
    );

    // Give initial swing
    if let Some(body) = nova.world.get_body_mut(rod_handle) {
        body.angular_velocity = nova::prelude::Vec3::new(0.0, 0.0, 1.5);
    }
}

/// Spawn BLACK HOLE - gravitational attractor (uses force field)
pub fn spawn_black_hole_preset(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    position: Vec3,
) {
    crate::plugins::effects::spawn_black_hole(
        commands,
        meshes,
        materials,
        position + Vec3::Y * 5.0,
        20.0,    // Large radius
        8000.0,  // Strong pull
    );
}

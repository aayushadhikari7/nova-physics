//! Example: Joints
//!
//! Demonstrates various joint types.

use nova::prelude::*;

fn main() {
    println!("Nova Physics Engine - Joints Example");
    println!("=====================================\n");

    let mut world = PhysicsWorld::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    // Create a static anchor point
    let anchor = world.create_body().body_type(RigidBodyType::Static).position(Vec3::new(0.0, 10.0, 0.0)).build();
    let anchor_handle = world.insert_body(anchor);

    // Create a pendulum using a ball joint
    let pendulum = world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(Vec3::new(2.0, 10.0, 0.0))
        .mass(1.0)
        .build();
    let pendulum_handle = world.insert_body(pendulum);

    world.insert_collider(
        world
            .create_collider(
                pendulum_handle,
                CollisionShape::Sphere(SphereShape::new(0.25)),
            )
            .build(),
    );

    // Create ball joint
    world.create_ball_joint(
        anchor_handle,
        pendulum_handle,
        Vec3::ZERO,
        Vec3::new(-2.0, 0.0, 0.0),
    );

    println!("Created pendulum with ball joint");

    // Create a chain using distance joints
    let chain_length = 5;
    let mut prev_handle = anchor_handle;
    let mut prev_anchor = Vec3::new(5.0, 0.0, 0.0);

    for i in 0..chain_length {
        let link = world
            .create_body()
            .body_type(RigidBodyType::Dynamic)
            .position(Vec3::new(5.0 + (i + 1) as f32, 10.0, 0.0))
            .mass(0.5)
            .build();
        let link_handle = world.insert_body(link);

        world.insert_collider(
            world
                .create_collider(link_handle, CollisionShape::Sphere(SphereShape::new(0.15)))
                .build(),
        );

        world.create_distance_joint(
            prev_handle,
            link_handle,
            prev_anchor,
            Vec3::ZERO,
            1.0,
        );

        prev_handle = link_handle;
        prev_anchor = Vec3::ZERO;
    }

    println!("Created chain with {} links using distance joints", chain_length);

    // Simulate
    let dt = 1.0 / 60.0;
    println!("\nSimulating...\n");

    for step in 0..300 {
        world.step(dt);

        if step % 60 == 0 {
            let pendulum_pos = world.get_body(pendulum_handle).unwrap().position;
            println!(
                "Time: {:.1}s | Pendulum: ({:.2}, {:.2}, {:.2})",
                step as f32 * dt,
                pendulum_pos.x,
                pendulum_pos.y,
                pendulum_pos.z
            );
        }
    }

    println!("\nSimulation complete!");
}

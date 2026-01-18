//! Example: Falling Boxes
//!
//! Demonstrates basic physics simulation with a stack of falling boxes.

use nova::prelude::*;

fn main() {
    println!("Nova Physics Engine - Falling Boxes Example");
    println!("============================================\n");

    // Create a physics world with default settings
    let mut world = PhysicsWorld::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    // Create a ground plane (static body)
    let ground = world
        .create_body()
        .body_type(RigidBodyType::Static)
        .position(Vec3::ZERO)
        .build();
    let ground_handle = world.insert_body(ground);

    world.insert_collider(
        world
            .create_collider(
                ground_handle,
                CollisionShape::Box(BoxShape::new(Vec3::new(50.0, 0.5, 50.0))),
            )
            .friction(0.5)
            .build(),
    );

    println!("Created ground plane");

    // Create a stack of boxes
    let box_count = 10;
    let mut box_handles = Vec::new();

    for i in 0..box_count {
        let body = world
            .create_body()
            .body_type(RigidBodyType::Dynamic)
            .position(Vec3::new(0.0, 1.0 + i as f32 * 1.1, 0.0))
            .mass(1.0)
            .build();
        let body_handle = world.insert_body(body);

        world.insert_collider(
            world
                .create_collider(
                    body_handle,
                    CollisionShape::Box(BoxShape::new(Vec3::splat(0.5))),
                )
                .friction(0.5)
                .restitution(0.1)
                .build(),
        );

        box_handles.push(body_handle);
    }

    println!("Created {} boxes\n", box_count);

    // Simulate for 10 seconds at 60 FPS
    let dt = 1.0 / 60.0;
    let total_steps = 600;

    println!("Simulating {} steps ({} seconds)...\n", total_steps, total_steps as f32 * dt);

    for step in 0..total_steps {
        world.step(dt);

        // Print status every second
        if step % 60 == 0 {
            let top_box = world.get_body(box_handles[box_count - 1]).unwrap();
            println!(
                "Time: {:.1}s | Top box Y: {:.3} | Bodies: {} | Contacts: {}",
                step as f32 * dt,
                top_box.position.y,
                world.body_count(),
                world.debug_info().contact_count,
            );
        }
    }

    println!("\nSimulation complete!");
    println!("\nFinal positions:");
    for (i, &handle) in box_handles.iter().enumerate() {
        let body = world.get_body(handle).unwrap();
        println!(
            "  Box {}: ({:.3}, {:.3}, {:.3})",
            i, body.position.x, body.position.y, body.position.z
        );
    }

    // Perform a raycast
    println!("\nRaycasting from above...");
    if let Some(hit) = world.raycast(
        Vec3::new(0.0, 20.0, 0.0),
        Vec3::new(0.0, -1.0, 0.0),
        100.0,
        QueryFilter::default(),
    ) {
        println!(
            "  Hit at distance {:.3}, point: ({:.3}, {:.3}, {:.3})",
            hit.distance, hit.point.x, hit.point.y, hit.point.z
        );
    }
}

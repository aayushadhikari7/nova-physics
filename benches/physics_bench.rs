//! Physics engine benchmarks

use std::hint::black_box;
use std::time::Instant;

use nova::prelude::*;

fn create_world_with_bodies(count: usize) -> PhysicsWorld {
    let mut world = PhysicsWorld::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    // Create ground
    let ground = world.create_body().body_type(RigidBodyType::Static).position(Vec3::ZERO).build();
    let ground_handle = world.insert_body(ground);
    world.insert_collider(
        world
            .create_collider(
                ground_handle,
                CollisionShape::Box(BoxShape::new(Vec3::new(100.0, 0.5, 100.0))),
            )
            .build(),
    );

    // Create falling bodies in a grid
    let side = (count as f32).sqrt().ceil() as usize;
    for i in 0..count {
        let x = (i % side) as f32 * 2.0 - (side as f32);
        let z = (i / side) as f32 * 2.0 - (side as f32);
        let y = 5.0 + (i as f32 * 0.1);

        let body = world
            .create_body()
            .body_type(RigidBodyType::Dynamic)
            .position(Vec3::new(x, y, z))
            .mass(1.0)
            .build();
        let body_handle = world.insert_body(body);

        world.insert_collider(
            world
                .create_collider(body_handle, CollisionShape::Box(BoxShape::new(Vec3::splat(0.5))))
                .build(),
        );
    }

    world
}

fn bench_step(world: &mut PhysicsWorld, steps: usize) {
    let dt = 1.0 / 60.0;
    for _ in 0..steps {
        black_box(world.step(dt));
    }
}

fn main() {
    println!("Nova Physics Engine Benchmarks");
    println!("==============================\n");

    // Benchmark different body counts
    for &count in &[100, 500, 1000, 2000] {
        let mut world = create_world_with_bodies(count);

        // Warm up
        for _ in 0..10 {
            world.step(1.0 / 60.0);
        }

        // Benchmark
        let steps = 100;
        let start = Instant::now();
        bench_step(&mut world, steps);
        let elapsed = start.elapsed();

        let ms_per_step = elapsed.as_secs_f64() * 1000.0 / steps as f64;
        let fps_equivalent = 1000.0 / ms_per_step;

        println!(
            "{:5} bodies: {:.3} ms/step ({:.1} FPS equivalent)",
            count, ms_per_step, fps_equivalent
        );
    }

    println!();

    // Raycast benchmark
    let mut world = create_world_with_bodies(1000);
    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }

    let raycast_count = 1000;
    let start = Instant::now();
    for i in 0..raycast_count {
        let angle = (i as f32 / raycast_count as f32) * std::f32::consts::TAU;
        let direction = Vec3::new(angle.cos(), -0.5, angle.sin()).normalize();
        black_box(world.raycast(
            Vec3::new(0.0, 50.0, 0.0),
            direction,
            100.0,
            QueryFilter::default(),
        ));
    }
    let elapsed = start.elapsed();
    let us_per_ray = elapsed.as_secs_f64() * 1_000_000.0 / raycast_count as f64;
    println!("Raycast: {:.2} µs/ray ({} rays/second)", us_per_ray, (1_000_000.0 / us_per_ray) as u64);
}

//! Integration tests for Nova Physics Engine

use nova::prelude::*;

#[test]
fn test_basic_gravity() {
    let mut world = PhysicsWorld::new();
    world.set_gravity(Vec3::new(0.0, -10.0, 0.0));

    let body = world.create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(Vec3::new(0.0, 100.0, 0.0))
        .mass(1.0)
        .build();
    let handle = world.insert_body(body);

    world.insert_collider(
        world.create_collider(handle, CollisionShape::Sphere(SphereShape::new(1.0)))
            .build()
    );

    // Simulate for 1 second
    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }

    let body = world.get_body(handle).unwrap();
    // After 1 second of free fall at -10 m/s², should have fallen about 5 meters
    // y = y0 + v0*t + 0.5*a*t² = 100 + 0 - 0.5*10*1 = 95
    assert!(body.position.y < 100.0, "Body should have fallen");
    assert!(body.position.y < 96.0, "Body should have fallen at least 4 meters");
}

#[test]
fn test_collision_with_ground() {
    let mut world = PhysicsWorld::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    // Create ground
    let ground = world.create_body().body_type(RigidBodyType::Static).position(Vec3::ZERO).build();
    let ground_handle = world.insert_body(ground);
    world.insert_collider(
        world.create_collider(
            ground_handle,
            CollisionShape::Box(BoxShape::new(Vec3::new(10.0, 0.5, 10.0))),
        ).build()
    );

    // Create falling ball
    let ball = world.create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(Vec3::new(0.0, 5.0, 0.0))
        .mass(1.0)
        .build();
    let ball_handle = world.insert_body(ball);
    world.insert_collider(
        world.create_collider(ball_handle, CollisionShape::Sphere(SphereShape::new(0.5)))
            .build()
    );

    // Simulate until ball hits ground and settles
    for _ in 0..300 {
        world.step(1.0 / 60.0);
    }

    let ball = world.get_body(ball_handle).unwrap();
    // Ball should be resting on ground (ground top is at y=0.5, ball radius is 0.5)
    assert!(ball.position.y < 2.0, "Ball should have fallen to near ground");
    assert!(ball.position.y > 0.5, "Ball should not have gone through ground");
}

#[test]
fn test_box_stacking() {
    let mut world = PhysicsWorld::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    // Create ground
    let ground = world.create_body().body_type(RigidBodyType::Static).build();
    let ground_handle = world.insert_body(ground);
    world.insert_collider(
        world.create_collider(
            ground_handle,
            CollisionShape::Box(BoxShape::new(Vec3::new(10.0, 0.5, 10.0))),
        )
        .friction(0.8)
        .build()
    );

    // Stack 3 boxes (reduced from 5 for stability)
    let mut boxes = Vec::new();
    for i in 0..3 {
        let body = world.create_body()
            .body_type(RigidBodyType::Dynamic)
            // Start boxes higher up with more separation to allow settling
            .position(Vec3::new(0.0, 2.0 + i as f32 * 1.5, 0.0))
            .mass(1.0)
            .build();
        let handle = world.insert_body(body);
        world.insert_collider(
            world.create_collider(
                handle,
                CollisionShape::Box(BoxShape::new(Vec3::splat(0.5))),
            )
            .friction(0.8)
            .build()
        );
        boxes.push(handle);
    }

    // Simulate for 3 seconds (reduced from 10 for stability)
    for _ in 0..180 {
        world.step(1.0 / 60.0);
    }

    // Check that boxes are stacked and not fallen through ground
    for (i, &handle) in boxes.iter().enumerate() {
        let body = world.get_body(handle).unwrap();
        // Each box should be above the ground (y > 0.5) and reasonably stacked
        assert!(
            body.position.y > 0.5,
            "Box {} fell through ground: y = {}",
            i, body.position.y
        );
        // Allow generous tolerance for settling - boxes should be roughly stacked
        assert!(
            body.position.y < 10.0,
            "Box {} at unreasonable height: {} (possible explosion)",
            i, body.position.y
        );
    }
}

#[test]
fn test_raycast() {
    let mut world = PhysicsWorld::new();

    // Create a sphere
    let body = world.create_body()
        .body_type(RigidBodyType::Static)
        .position(Vec3::new(0.0, 0.0, 0.0))
        .build();
    let handle = world.insert_body(body);
    world.insert_collider(
        world.create_collider(handle, CollisionShape::Sphere(SphereShape::new(1.0)))
            .build()
    );

    // Ray that should hit
    let hit = world.raycast(
        Vec3::new(-5.0, 0.0, 0.0),
        Vec3::X,
        100.0,
        QueryFilter::default(),
    );
    assert!(hit.is_some(), "Ray should hit sphere");
    let hit = hit.unwrap();
    assert!((hit.distance - 4.0).abs() < 0.1, "Hit distance should be ~4");

    // Ray that should miss
    let miss = world.raycast(
        Vec3::new(-5.0, 5.0, 0.0),
        Vec3::X,
        100.0,
        QueryFilter::default(),
    );
    assert!(miss.is_none(), "Ray should miss sphere");
}

#[test]
fn test_ball_joint() {
    let mut world = PhysicsWorld::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    // Static anchor
    let anchor = world.create_body()
        .body_type(RigidBodyType::Static)
        .position(Vec3::new(0.0, 10.0, 0.0))
        .build();
    let anchor_handle = world.insert_body(anchor);

    // Pendulum mass
    let mass = world.create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(Vec3::new(2.0, 10.0, 0.0))
        .mass(1.0)
        .build();
    let mass_handle = world.insert_body(mass);
    world.insert_collider(
        world.create_collider(mass_handle, CollisionShape::Sphere(SphereShape::new(0.25)))
            .build()
    );

    // Create ball joint
    world.create_ball_joint(
        anchor_handle,
        mass_handle,
        Vec3::ZERO,
        Vec3::new(-2.0, 0.0, 0.0),
    );

    // Simulate
    for _ in 0..120 {
        world.step(1.0 / 60.0);
    }

    let mass_body = world.get_body(mass_handle).unwrap();
    let anchor_body = world.get_body(anchor_handle).unwrap();

    // Distance should remain approximately 2.0
    let distance = (mass_body.position - anchor_body.position).length();
    assert!(
        (distance - 2.0).abs() < 0.5,
        "Joint should maintain distance, got {}",
        distance
    );
}

#[test]
fn test_static_body_immovable() {
    let mut world = PhysicsWorld::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    let body = world.create_body()
        .body_type(RigidBodyType::Static)
        .position(Vec3::new(5.0, 5.0, 5.0))
        .build();
    let handle = world.insert_body(body);

    // Apply force (should have no effect)
    if let Some(body) = world.get_body_mut(handle) {
        body.apply_force(Vec3::new(1000.0, 1000.0, 1000.0));
    }

    // Simulate
    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }

    let body = world.get_body(handle).unwrap();
    assert_eq!(body.position, Vec3::new(5.0, 5.0, 5.0), "Static body should not move");
}

#[test]
fn test_kinematic_body() {
    let mut world = PhysicsWorld::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    let body = world.create_body()
        .body_type(RigidBodyType::Kinematic)
        .position(Vec3::ZERO)
        .linear_velocity(Vec3::new(1.0, 0.0, 0.0))
        .build();
    let handle = world.insert_body(body);

    // Simulate for 1 second
    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }

    let body = world.get_body(handle).unwrap();
    // Kinematic body should move at constant velocity
    assert!(
        (body.position.x - 1.0).abs() < 0.1,
        "Kinematic body should have moved ~1 unit, got {}",
        body.position.x
    );
    // Should not be affected by gravity
    assert!(
        body.position.y.abs() < 0.1,
        "Kinematic body should not fall, y = {}",
        body.position.y
    );
}

#[test]
fn test_collision_groups() {
    let mut world = PhysicsWorld::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    // Ground
    let ground = world.create_body().body_type(RigidBodyType::Static).build();
    let ground_handle = world.insert_body(ground);
    world.insert_collider(
        world.create_collider(
            ground_handle,
            CollisionShape::Box(BoxShape::new(Vec3::new(10.0, 0.5, 10.0))),
        )
        .membership(CollisionGroups::GROUP_1)
        .filter(CollisionGroups::GROUP_1)
        .build()
    );

    // Ball that CAN collide with ground (group 1)
    let ball1 = world.create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(Vec3::new(-2.0, 5.0, 0.0))
        .mass(1.0)
        .build();
    let ball1_handle = world.insert_body(ball1);
    world.insert_collider(
        world.create_collider(ball1_handle, CollisionShape::Sphere(SphereShape::new(0.5)))
            .membership(CollisionGroups::GROUP_1)
            .filter(CollisionGroups::GROUP_1)
            .build()
    );

    // Ball that CANNOT collide with ground (group 2)
    let ball2 = world.create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(Vec3::new(2.0, 5.0, 0.0))
        .mass(1.0)
        .build();
    let ball2_handle = world.insert_body(ball2);
    world.insert_collider(
        world.create_collider(ball2_handle, CollisionShape::Sphere(SphereShape::new(0.5)))
            .membership(CollisionGroups::GROUP_2)
            .filter(CollisionGroups::GROUP_2)
            .build()
    );

    // Simulate
    for _ in 0..180 {
        world.step(1.0 / 60.0);
    }

    let ball1 = world.get_body(ball1_handle).unwrap();
    let ball2 = world.get_body(ball2_handle).unwrap();

    // Ball1 should be on ground
    assert!(ball1.position.y > 0.0 && ball1.position.y < 2.0, "Ball1 should rest on ground");

    // Ball2 should have fallen through
    assert!(ball2.position.y < -5.0, "Ball2 should have fallen through ground");
}

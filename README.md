# Nova Physics Engine

A production-grade 3D physics engine written in Rust.

## Features

- **Rigid Body Dynamics**: Full support for dynamic, kinematic, and static bodies
- **Collision Detection**: GJK, EPA, and SAT algorithms with broad-phase BVH acceleration
- **Multiple Shape Types**: Sphere, Capsule, Box, ConvexHull, TriMesh, and Compound shapes
- **Constraint Solver**: Sequential impulse solver with warm starting
- **Joints**: Ball, Hinge, Prismatic, Fixed, and Distance joints with motor support
- **Spatial Queries**: Raycast, shapecast, and overlap queries
- **Sleeping System**: Automatic deactivation of resting bodies for performance
- **Island Detection**: Isolated simulation groups for parallel solving

## Architecture

The engine is organized as a Cargo workspace with modular crates:

```
physics-engine/
├── nova/                    # Facade crate with re-exports
├── crates/
│   ├── nova-math/           # Math primitives (Vec3, Quat, Mat3, AABB, Isometry)
│   ├── nova-core/           # Handles, arenas, events
│   ├── nova-collision/      # Shapes, broad/narrow phase, queries
│   ├── nova-dynamics/       # Rigid body, solver, integrator
│   ├── nova-constraints/    # Joints and constraints
│   ├── nova-pipeline/       # Simulation orchestration
│   └── nova-world/          # High-level World API
├── examples/
├── benches/
└── tests/
```

## Quick Start

Add Nova to your `Cargo.toml`:

```toml
[dependencies]
nova = { path = "nova" }
```

Basic usage:

```rust
use nova::prelude::*;

fn main() {
    // Create a physics world
    let mut world = PhysicsWorld::new();
    world.set_gravity(Vec3::new(0.0, -9.81, 0.0));

    // Create a static ground plane
    let ground = world.create_body()
        .body_type(RigidBodyType::Static)
        .build();
    world.create_collider(ground)
        .shape(CollisionShape::Box(BoxShape::new(Vec3::new(50.0, 0.5, 50.0))))
        .build();

    // Create a dynamic box
    let box_body = world.create_body()
        .position(Vec3::new(0.0, 5.0, 0.0))
        .mass(1.0)
        .build();
    world.create_collider(box_body)
        .shape(CollisionShape::Box(BoxShape::new(Vec3::splat(0.5))))
        .friction(0.5)
        .restitution(0.3)
        .build();

    // Run simulation
    for _ in 0..600 {  // 10 seconds at 60 FPS
        world.step(1.0 / 60.0);
    }
}
```

## Collision Shapes

```rust
// Sphere
CollisionShape::Sphere(SphereShape::new(1.0))

// Capsule (radius, half-height)
CollisionShape::Capsule(CapsuleShape::new(0.5, 1.0))

// Box (half-extents)
CollisionShape::Box(BoxShape::new(Vec3::new(1.0, 0.5, 2.0)))

// Convex Hull (from points)
let points = vec![/* ... */];
CollisionShape::ConvexHull(ConvexHull::from_points(&points))

// Triangle Mesh (static geometry)
let vertices = vec![/* ... */];
let indices = vec![/* ... */];
CollisionShape::TriMesh(TriMesh::new(vertices, indices))
```

## Joints

```rust
// Ball joint (3 DOF rotation)
world.create_joint(body_a, body_b)
    .ball_joint()
    .local_anchor_a(Vec3::new(0.0, 1.0, 0.0))
    .local_anchor_b(Vec3::new(0.0, -1.0, 0.0))
    .build();

// Hinge joint (1 DOF rotation)
world.create_joint(body_a, body_b)
    .hinge_joint(Vec3::Y)  // axis
    .local_anchor_a(Vec3::ZERO)
    .local_anchor_b(Vec3::ZERO)
    .build();

// Distance joint (spring)
world.create_joint(body_a, body_b)
    .distance_joint(2.0)  // rest length
    .stiffness(100.0)
    .damping(5.0)
    .build();
```

## Spatial Queries

```rust
// Raycast
if let Some(hit) = world.raycast(origin, direction, max_distance, QueryFilter::default()) {
    println!("Hit {} at distance {}", hit.collider, hit.distance);
}

// Point query
let colliders = world.point_query(point, QueryFilter::default());

// AABB query
let colliders = world.aabb_query(&aabb, QueryFilter::default());
```

## Performance Targets

| Metric | Target |
|--------|--------|
| Rigid bodies (stable) | 10,000+ |
| Contacts per frame | 50,000+ |
| Frame time @ 1000 bodies | < 2ms |

## Dependencies

- `glam` - Fast SIMD math library
- `slotmap` - Generational arena for handles
- `rayon` - Parallel iteration
- `hashbrown` - Fast hash maps
- `smallvec` - Inline small vectors

## Building

```bash
# Build all crates
cargo build

# Run tests
cargo test

# Run benchmarks
cargo bench

# Run examples
cargo run --example falling_boxes
cargo run --example joints
```

## License

MIT License

# Nova Physics Engine

A production-grade 3D physics engine written in Rust, with an interactive playground for testing and experimentation.

## Features

- **Rigid Body Dynamics**: Full support for dynamic, kinematic, and static bodies
- **Collision Detection**: GJK, EPA, and SAT algorithms with broad-phase BVH acceleration
- **Multiple Shape Types**: Sphere, Capsule, Box, ConvexHull, TriMesh, and Compound shapes
- **Constraint Solver**: Sequential impulse solver with warm starting
- **Joints**: Ball, Hinge, Prismatic, Fixed, Distance, Spring, and Rope joints with motor support
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
├── playground/              # Interactive 3D physics sandbox (Bevy)
├── examples/
├── benches/
└── tests/
```

## Nova Physics Playground

An interactive 3D physics sandbox built with Bevy for testing and experimenting with the Nova physics engine.

### Running the Playground

```bash
cargo run -p nova-playground
```

### Controls

| Key | Action |
|-----|--------|
| WASD | Move camera |
| Space/Ctrl | Move up/down |
| Shift | Sprint |
| Mouse | Look around |
| 1-9 | Select hotbar slot |
| Q/Z | Previous/Next hotbar page |
| E | Open inventory |
| P | Pause/Unpause |
| [ / ] | Slow down / Speed up time |
| R | Reset scene |
| Tab | Cycle tool options |
| F1-F4 | Debug visualizations |
| F11 | Toggle fullscreen |
| Esc | Exit |

### Features

**Shapes**: Box, Sphere, Capsule, Cylinder, Cone, Compound, Random

**Tools**:
- Gravity Gun - Grab and throw objects
- Force Gun - Push/pull objects with impulse
- Explosion - Create explosive forces
- Joint Tool - Connect objects (7 joint types)
- Delete - Remove objects
- Freeze - Immobilize objects
- Clone - Duplicate objects
- Magnet - Attract/repel objects
- Launch Cannon - Fire projectiles
- Painter - Change object colors

**Presets**: Tower, Pyramid, Ragdoll, Newton's Cradle, Wrecking Ball, Dominos, Bridge, Catapult, Car, Pendulum Wall, Box Rain, Spiral Staircase, Cannon, Ferris Wheel, Windmill, Chain

**Special Zones**: Gravity Zone, Force Field (Vortex, Radial, Turbulence), Trampoline, Conveyor, Fan

**Materials**: Normal, Bouncy, Sticky, Slippery, Heavy, Light, Rubber, Metal, Ice, Wood

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
world.create_ball_joint(body_a, body_b, anchor_a, anchor_b);

// Hinge joint (1 DOF rotation)
world.create_hinge_joint(body_a, body_b, anchor_a, anchor_b, axis);

// Fixed joint (0 DOF)
world.create_fixed_joint(body_a, body_b, anchor_a, anchor_b);

// Distance joint (spring)
world.create_distance_joint(body_a, body_b, anchor_a, anchor_b, rest_length);

// Prismatic joint (1 DOF translation)
world.create_prismatic_joint(body_a, body_b, anchor_a, anchor_b, axis);
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
- `bevy` - Game engine (playground only)

## Building

```bash
# Build all crates
cargo build

# Run tests
cargo test

# Run benchmarks
cargo bench

# Run the playground
cargo run -p nova-playground

# Run examples
cargo run --example falling_boxes
cargo run --example joints
```

## License

MIT License

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

---

## Nova Physics Playground

An interactive 3D physics sandbox built with Bevy for testing and experimenting with the Nova physics engine. Features 50+ spawnable presets across 6 inventory pages, multiple tools, force field zones, and more!

### Running the Playground

```bash
cargo run -p nova-playground --release
```

### Controls

| Key | Action |
|-----|--------|
| WASD | Move camera |
| Space | Move up |
| Ctrl | Move down |
| Shift | Sprint (2.5x speed) |
| Mouse | Look around |
| Middle Mouse / Esc | Toggle cursor lock |
| 1-9 | Select hotbar slot |
| Q/E | Previous/Next inventory page |
| P | Pause/Unpause |
| [ / ] | Slow down / Speed up time |
| Backspace | Toggle 0.25x slow motion |
| R | Reset scene (clear all spawned objects) |
| Tab | Cycle tool options |
| F1 | Toggle contact points |
| F2 | Toggle joint lines |
| F3 | Toggle velocity arrows |
| F4 | Toggle AABBs |

### Tools

| Tool | Left Click | Right Click |
|------|------------|-------------|
| **Gravity Gun** | Grab/release object | Throw grabbed object |
| **Force Gun** | Push objects away | Pull objects toward |
| **Explosion** | Small explosion | Large explosion |
| **Joint Tool** | Select bodies to connect | Cancel selection |
| **Delete** | Remove single object | Area delete (10 unit radius) |
| **Freeze** | Toggle object freeze | Freeze all nearby |
| **Clone** | Duplicate object | - |
| **Magnet** | Attract nearby objects | Repel nearby objects |
| **Launch Cannon** | Fire projectile | - |
| **Painter** | Change object color | - |

---

## Inventory Pages (6 Pages, 54 Items)

### Page 1: Basic Shapes & Core Tools
Box, Sphere, Capsule, Gravity Gun, Force Gun, Explosion, Joint Tool, Delete, Freeze

### Page 2: More Tools & Shapes
Clone, Resize, Magnet, Launch Cannon, Painter, Cylinder, Cone, Compound, Random

### Page 3: Structure Presets
Chain, Tower, Pyramid, Ragdoll, Newton's Cradle, Wrecking Ball, Dominos, Bridge, Catapult

### Page 4: More Presets
Controllable Car, Pendulum Wall, Box Rain, Spiral Staircase, Ball Cannon, Ferris Wheel, Windmill, Gravity Zone, Force Field

### Page 5: Special Zones & Objects
Portal, Trampoline, Conveyor Belt, Fan, Breakable Object, Explosive, Spinner, Magnet Object, Glowing Object

### Page 6: Science & Fun (NEW!)
- **Buckyball** - C60 molecular structure with 32 connected spheres
- **DNA Helix** - Double helix with 24 base pairs
- **Solar System** - Sun with 6 orbiting planets
- **Atom** - Nucleus with 3 electron shells
- **Geodesic Dome** - Icosahedron-based dome structure
- **Tensegrity** - Floating compression structure
- **Mobius Chain** - Twisted loop of 36 connected spheres
- **Pendulum Clock** - Classic weighted pendulum mechanism
- **Black Hole** - Gravitational attractor (deletes nearby objects!)

---

## Materials

Objects can be spawned with different physics materials:

| Material | Properties |
|----------|------------|
| Normal | Standard physics |
| Bouncy | High restitution (0.9) |
| Heavy | 10x mass |
| Light | 0.1x mass |
| Rubber | High friction, medium bounce |
| Sticky | Very high friction |
| Slippery | Very low friction |

---

## Quick Start (Engine API)

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
cargo run -p nova-playground --release

# Run examples
cargo run --example falling_boxes
cargo run --example joints
```

## Recent Updates

### Physics Improvements
- **Enhanced collision detection**: 240Hz fixed timestep with up to 32 substeps (max 1/960s per step)
- **Improved solver settings**: 30 velocity iterations, 15 position iterations
- **Better joint stability**: 12 joint solver iterations with 0.95 warm start factor
- **Anti-tunneling measures**: Max velocity capped at 50 m/s, ground plane enforcement as safety net
- **Out-of-bounds cleanup**: Objects automatically deleted when leaving room bounds

### Visual Improvements
- **Balanced lighting**: Reduced brightness for better contrast
- **Cleaner arena**: Medium gray floor/walls, reduced ambient light

### UI Improvements
- **Streamlined controls**: Q/E for inventory navigation (no keyboard preset shortcuts)
- **6 inventory pages**: Organized by category with smooth FPS display

## License

MIT License

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

An interactive 3D physics sandbox built with Bevy for testing and experimenting with the Nova physics engine. Features 45+ spawnable presets, multiple tools, force field zones, and more!

### Running the Playground

```bash
cargo run -p nova-playground
```

### Basic Controls

| Key | Action |
|-----|--------|
| WASD | Move camera |
| Space | Move up |
| Ctrl | Move down |
| Shift | Sprint (2.5x speed, affects all movement) |
| Mouse | Look around |
| Middle Mouse / Esc | Toggle cursor lock |
| 1-9 | Select hotbar slot |
| Q/Z | Previous/Next hotbar page |
| E | Open inventory |
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

## Preset Spawning Keys

### Letter Keys - Basic Presets

| Key | Preset | Description |
|-----|--------|-------------|
| T | Tower | Stack of 10 colored boxes |
| Y | Pyramid | 5-layer pyramid of boxes |
| U | Newton's Cradle | Classic physics toy (5 balls) |
| I | Wrecking Ball | Heavy ball on chain |
| O | Dominos | Row of 20 falling dominos |
| H | Ragdoll | Jointed humanoid figure |
| J | Bridge | Plank bridge with hinge joints |
| K | Catapult | Working catapult with projectile |
| L | Pendulum Wall | Grid of swinging pendulums |
| N | Controllable Car | Drivable vehicle (F to enter/exit) |
| M | Ferris Wheel | Rotating wheel with cars |
| Q | Jenga Tower | 15-layer alternating brick tower |
| X | Pool Table | Billiards triangle + cue ball |
| Z | Trebuchet | Medieval siege weapon |
| C | Ramp | Angled ramp (25 degrees) |

### Symbol Keys - More Presets

| Key | Preset | Description |
|-----|--------|-------------|
| ` (Backquote) | Bowling Alley | 10 pins + bowling ball |
| 0 | Seesaw | Working seesaw with heavy ball |
| - (Minus) | Swing Set | Swing with ball joint |
| = (Equal) | Brick Wall | 8x6 destructible wall |
| \ (Backslash) | Ball Avalanche | 40 bouncy spheres |
| ' (Quote) | Spinning Platform | 4-arm rotating platform |
| ] | Ball Pit | 50 colorful bouncy balls |
| , (Comma) | Windmill | 4-blade spinning windmill |
| . (Period) | Spiral Staircase | 20-step spiral stairs |
| / (Slash) | Box Rain | 30 random falling boxes |

### Numpad - BONKERS MODE!

| Key | Preset | Description |
|-----|--------|-------------|
| Numpad 1 | Domino Spiral | 100 dominos in spiral pattern |
| Numpad 2 | Giant Pendulum | Massive swinging pendulum of doom |
| Numpad 3 | Wrecking Ball vs Tower | Tower + wrecking ball combo |
| Numpad 4 | Volcano | Erupting balls with upward velocity |
| Numpad 5 | Staircase of Doom | Stairs with bouncing balls |
| Numpad 6 | Pachinko | Japanese pinball board |
| Numpad 7 | Ball Tsunami | 100 balls with forward momentum |
| Numpad 8 | Chain Reaction | Dominos → Ramp → Tower → Bowling |
| Numpad 9 | Gyroscope | 3 nested spinning rings |
| Numpad 0 | Hamster Wheel | Giant running wheel |
| Numpad + | Chaos Cube | 125 balls exploding outward |
| Numpad - | Double Pendulum | Chaotic motion demonstration |
| Numpad * | Meteor Shower | 30 heavy balls raining down |
| Numpad / | Ball Fountain | 50 balls shooting upward |
| Numpad . | Fireworks | 5 colorful exploding bursts |
| Numpad Enter | Bouncy Castle | Rainbow walls with bouncy balls |

### Navigation Keys - ULTRA MODE!

| Key | Preset | Description |
|-----|--------|-------------|
| Insert | Marble Run | Zigzag ramps with obstacles |
| Delete | Ball Cannon | Fires 20 balls forward |
| Home | Ultimate Destruction | Tower + Pyramids + Wrecking Balls + Avalanche |
| End | Pendulum Wave | 20 pendulums with varying lengths |
| Page Up | Ball Tornado | 80 balls in spinning vortex |

### Quick Spawn (F-Keys)

| Key | Action |
|-----|--------|
| F5 | Spawn box |
| F6 | Spawn bouncy sphere |
| F7 | Spawn heavy ball |
| F8 | Spawn chain |

---

## Force Field Zones

| Key | Zone | Description |
|-----|------|-------------|
| G | Gravity Zone | Reverse gravity (upward force) |
| V | Vortex | Spinning force field |
| B | Radial Push | Pushes objects away |
| F9 | Turbulence | Random chaotic forces |
| F10 | Upward Wind | Strong upward force |
| F11 | Black Hole | Super strong attractor (deletes nearby objects!) |
| F12 | Slow Motion Zone | Slows objects passing through |
| ; (Semicolon) | Bounce Pad | Launches objects upward |

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
cargo run -p nova-playground

# Run examples
cargo run --example falling_boxes
cargo run --example joints
```

## License

MIT License

//! Vehicle control system - motors and steering for cars

use bevy::prelude::*;
use nova::prelude::*;

use crate::components::{PhysicsBody, Vehicle, VehicleType, Wheel};
use crate::convert::{to_bevy_vec3, to_nova_vec3};
use crate::resources::NovaWorld;

pub struct VehiclePlugin;

impl Plugin for VehiclePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ActiveVehicle>()
            .add_systems(Update, (
                vehicle_enter_exit_system,
                vehicle_motor_system,
                vehicle_steering_system,
            ));
    }
}

/// Resource tracking which vehicle the player is controlling
#[derive(Resource, Default)]
pub struct ActiveVehicle {
    /// The vehicle entity currently being controlled (if any)
    pub entity: Option<Entity>,
    /// The body handle of the vehicle
    pub body_handle: Option<RigidBodyHandle>,
    /// Current throttle input (-1 to 1)
    pub throttle: f32,
    /// Current steering input (-1 to 1)
    pub steering: f32,
    /// Motor strength
    pub motor_force: f32,
    /// Maximum steering angle in radians
    pub max_steer_angle: f32,
}

impl ActiveVehicle {
    pub fn new() -> Self {
        Self {
            entity: None,
            body_handle: None,
            throttle: 0.0,
            steering: 0.0,
            motor_force: 500.0,
            max_steer_angle: 0.5, // ~28 degrees
        }
    }
}

/// Enter/exit vehicles with F key
fn vehicle_enter_exit_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut active_vehicle: ResMut<ActiveVehicle>,
    nova: Res<NovaWorld>,
    vehicles: Query<(Entity, &PhysicsBody, &Transform), With<Vehicle>>,
    camera_query: Query<&GlobalTransform, With<crate::plugins::camera::PlayerCamera>>,
) {
    if !keyboard.just_pressed(KeyCode::KeyF) {
        return;
    }

    // If already in a vehicle, exit it
    if active_vehicle.entity.is_some() {
        active_vehicle.entity = None;
        active_vehicle.body_handle = None;
        active_vehicle.throttle = 0.0;
        active_vehicle.steering = 0.0;
        return;
    }

    // Try to enter nearest vehicle
    let Ok(camera_transform) = camera_query.get_single() else {
        return;
    };

    let camera_pos = camera_transform.translation();
    let mut closest: Option<(Entity, RigidBodyHandle, f32)> = None;

    for (entity, physics_body, transform) in vehicles.iter() {
        let distance = camera_pos.distance(transform.translation);
        if distance < 5.0 {
            // Within 5 units
            if closest.is_none() || distance < closest.unwrap().2 {
                closest = Some((entity, physics_body.handle, distance));
            }
        }
    }

    if let Some((entity, handle, _)) = closest {
        active_vehicle.entity = Some(entity);
        active_vehicle.body_handle = Some(handle);
    }
}

/// Apply motor torque to powered wheels
fn vehicle_motor_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut active_vehicle: ResMut<ActiveVehicle>,
    mut nova: ResMut<NovaWorld>,
    wheels: Query<(&PhysicsBody, &Wheel)>,
) {
    // Only process if we're in a vehicle
    if active_vehicle.entity.is_none() {
        return;
    }

    // Read throttle input (W/S or Up/Down)
    let mut throttle = 0.0;
    if keyboard.pressed(KeyCode::KeyW) || keyboard.pressed(KeyCode::ArrowUp) {
        throttle += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyS) || keyboard.pressed(KeyCode::ArrowDown) {
        throttle -= 1.0;
    }

    // Handbrake with Space
    let handbrake = keyboard.pressed(KeyCode::Space);

    active_vehicle.throttle = throttle;

    // Apply torque to powered wheels
    for (physics_body, wheel) in wheels.iter() {
        if !wheel.powered {
            continue;
        }

        if let Some(body) = nova.world.get_body_mut(physics_body.handle) {
            if handbrake {
                // Apply braking by reducing angular velocity
                body.angular_velocity = body.angular_velocity * 0.9;
            } else if throttle.abs() > 0.01 {
                // Apply motor torque around the wheel's rotation axis (Z in local space)
                let torque = nova::prelude::Vec3::new(0.0, 0.0, throttle * active_vehicle.motor_force);
                body.apply_torque(torque);
            }
        }
    }
}

/// Apply steering angle to steering wheels
fn vehicle_steering_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut active_vehicle: ResMut<ActiveVehicle>,
    mut nova: ResMut<NovaWorld>,
    wheels: Query<(&PhysicsBody, &Wheel)>,
) {
    // Only process if we're in a vehicle
    if active_vehicle.entity.is_none() {
        return;
    }

    // Read steering input (A/D or Left/Right)
    let mut steering = 0.0;
    if keyboard.pressed(KeyCode::KeyA) || keyboard.pressed(KeyCode::ArrowLeft) {
        steering += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyD) || keyboard.pressed(KeyCode::ArrowRight) {
        steering -= 1.0;
    }

    active_vehicle.steering = steering;

    // Apply steering force to steering wheels
    // Since we can't directly change hinge joint angles, we apply a sideways torque
    // to simulate steering effect
    for (physics_body, wheel) in wheels.iter() {
        if !wheel.steering {
            continue;
        }

        if let Some(body) = nova.world.get_body_mut(physics_body.handle) {
            if steering.abs() > 0.01 {
                // Apply a torque around Y axis to simulate steering
                let steer_torque = nova::prelude::Vec3::new(0.0, steering * 200.0, 0.0);
                body.apply_torque(steer_torque);
            }
        }
    }

    // Also apply a turning force to the main vehicle body for better handling
    if let Some(body_handle) = active_vehicle.body_handle {
        if let Some(body) = nova.world.get_body_mut(body_handle) {
            let speed = body.linear_velocity.length();
            if speed > 1.0 && steering.abs() > 0.01 {
                // Apply yaw torque proportional to speed
                let yaw_torque = steering * speed * 50.0;
                body.apply_torque(nova::prelude::Vec3::new(0.0, yaw_torque, 0.0));
            }
        }
    }
}

/// Spawn a controllable vehicle with proper components
pub fn spawn_controllable_car(
    nova: &mut NovaWorld,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    handle_to_entity: &mut crate::resources::HandleToEntity,
    position: Vec3,
) -> Entity {
    let body_size = Vec3::new(2.0, 0.5, 1.0);
    let wheel_radius = 0.35;
    let wheel_width = 0.25;
    let wheel_offset_x = body_size.x / 2.0 - 0.2;
    let wheel_offset_z = body_size.z / 2.0 + wheel_width / 2.0 + 0.1;
    let wheel_offset_y = -body_size.y / 2.0 - wheel_radius * 0.3;

    // Create car body
    let body = nova
        .world
        .create_body()
        .body_type(RigidBodyType::Dynamic)
        .position(to_nova_vec3(position))
        .mass(10.0)
        .build();
    let body_handle = nova.world.insert_body(body);

    let half_extents = to_nova_vec3(body_size * 0.5);
    let collider = nova
        .world
        .create_collider(body_handle, CollisionShape::Box(BoxShape::new(half_extents)))
        .friction(0.3)
        .restitution(0.1)
        .build();
    let collider_handle = nova.world.insert_collider(collider);

    // Spawn car body entity with Vehicle component
    let car_entity = commands
        .spawn((
            Mesh3d(meshes.add(Cuboid::new(body_size.x, body_size.y, body_size.z))),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: Color::srgb(0.8, 0.1, 0.1),
                metallic: 0.7,
                perceptual_roughness: 0.3,
                ..default()
            })),
            Transform::from_translation(position),
            PhysicsBody { handle: body_handle },
            crate::components::PhysicsCollider { handle: collider_handle },
            crate::components::DynamicBody,
            crate::components::SpawnedObject,
            Vehicle {
                vehicle_type: VehicleType::Car,
            },
        ))
        .id();

    handle_to_entity.bodies.insert(body_handle, car_entity);

    // Wheel configurations: (local_pos, is_powered, is_steering)
    let wheel_configs = [
        (Vec3::new(wheel_offset_x, wheel_offset_y, wheel_offset_z), false, true),   // Front right - steering
        (Vec3::new(wheel_offset_x, wheel_offset_y, -wheel_offset_z), false, true),  // Front left - steering
        (Vec3::new(-wheel_offset_x, wheel_offset_y, wheel_offset_z), true, false),  // Rear right - powered
        (Vec3::new(-wheel_offset_x, wheel_offset_y, -wheel_offset_z), true, false), // Rear left - powered
    ];

    for (local_pos, powered, steering) in wheel_configs {
        let world_pos = position + local_pos;

        // Create wheel body
        let wheel_body = nova
            .world
            .create_body()
            .body_type(RigidBodyType::Dynamic)
            .position(to_nova_vec3(world_pos))
            .mass(1.0)
            .build();
        let wheel_handle = nova.world.insert_body(wheel_body);

        let wheel_collider = nova
            .world
            .create_collider(wheel_handle, CollisionShape::Sphere(SphereShape::new(wheel_radius)))
            .friction(1.2) // High friction for grip
            .restitution(0.1)
            .build();
        let wheel_collider_handle = nova.world.insert_collider(wheel_collider);

        // Visual wheel (cylinder rotated to be sideways)
        let wheel_color = if powered {
            Color::srgb(0.15, 0.15, 0.15) // Dark for rear (powered)
        } else {
            Color::srgb(0.25, 0.25, 0.25) // Lighter for front (steering)
        };

        let wheel_entity = commands
            .spawn((
                Mesh3d(meshes.add(Sphere::new(wheel_radius))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: wheel_color,
                    metallic: 0.1,
                    perceptual_roughness: 0.9,
                    ..default()
                })),
                Transform::from_translation(world_pos),
                PhysicsBody { handle: wheel_handle },
                crate::components::PhysicsCollider { handle: wheel_collider_handle },
                crate::components::DynamicBody,
                crate::components::SpawnedObject,
                Wheel { powered, steering },
            ))
            .id();

        handle_to_entity.bodies.insert(wheel_handle, wheel_entity);

        // Connect wheel to body with hinge joint (rotates around Z axis - sideways)
        nova.world.create_hinge_joint(
            body_handle,
            wheel_handle,
            to_nova_vec3(local_pos),
            nova::prelude::Vec3::ZERO,
            nova::prelude::Vec3::Z, // Rotation axis
        );
    }

    car_entity
}

//! First-person camera controller

use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::prelude::*;
use bevy::window::{CursorGrabMode, PrimaryWindow};

use crate::resources::CameraSettings;

pub struct CameraPlugin;

impl Plugin for CameraPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CameraSettings>()
            .add_systems(Startup, setup_camera)
            .add_systems(Update, (camera_look, camera_move, toggle_cursor_grab));
    }
}

/// Marker for the player camera
#[derive(Component)]
pub struct PlayerCamera {
    pub pitch: f32,
    pub yaw: f32,
}

impl Default for PlayerCamera {
    fn default() -> Self {
        Self {
            pitch: 0.0,
            yaw: 0.0,
        }
    }
}

fn setup_camera(mut commands: Commands, mut windows: Query<&mut Window, With<PrimaryWindow>>) {
    // Spawn the camera at a good starting position for the BIG ROOM
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 8.0, 35.0).looking_at(Vec3::ZERO, Vec3::Y),
        PlayerCamera::default(),
        // Fog for distance culling
        bevy::pbr::DistanceFog {
            color: Color::srgba(0.15, 0.17, 0.2, 1.0),
            directional_light_color: Color::srgba(1.0, 0.95, 0.85, 0.5),
            directional_light_exponent: 30.0,
            falloff: bevy::pbr::FogFalloff::Linear {
                start: 80.0,
                end: 350.0,
            },
        },
    ));

    // Lock and hide cursor for FPS controls
    if let Ok(mut window) = windows.get_single_mut() {
        window.cursor_options.grab_mode = CursorGrabMode::Locked;
        window.cursor_options.visible = false;
    }
}

fn camera_look(
    mut camera: Query<(&mut Transform, &mut PlayerCamera)>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    settings: Res<CameraSettings>,
    windows: Query<&Window, With<PrimaryWindow>>,
) {
    let Ok((mut transform, mut player_camera)) = camera.get_single_mut() else {
        return;
    };

    // Don't move camera if cursor is unlocked (visible)
    if let Ok(window) = windows.get_single() {
        if window.cursor_options.visible {
            return; // Cursor is free, don't move camera!
        }
    }

    let delta = mouse_motion.delta;

    if delta != Vec2::ZERO {
        player_camera.yaw -= delta.x * settings.sensitivity;
        player_camera.pitch -= delta.y * settings.sensitivity;

        // Clamp pitch to prevent flipping
        player_camera.pitch = player_camera.pitch.clamp(-1.5, 1.5);

        transform.rotation = Quat::from_euler(
            EulerRot::YXZ,
            player_camera.yaw,
            player_camera.pitch,
            0.0,
        );
    }
}

// Room bounds for camera collision (must match arena.rs)
pub const ROOM_HALF_WIDTH: f32 = 250.0;  // Half of 500
pub const ROOM_HALF_LENGTH: f32 = 250.0; // Half of 500
pub const ROOM_HEIGHT: f32 = 50.0;
pub const PLAYER_MARGIN: f32 = 2.0; // Keep player this far from walls

fn camera_move(
    mut camera: Query<&mut Transform, With<PlayerCamera>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    settings: Res<CameraSettings>,
) {
    let Ok(mut transform) = camera.get_single_mut() else {
        return;
    };

    let mut velocity = Vec3::ZERO;

    // Get camera-relative directions (horizontal plane)
    let forward = transform.forward();
    let forward_flat = Vec3::new(forward.x, 0.0, forward.z).normalize_or_zero();
    let right = transform.right();
    let right_flat = Vec3::new(right.x, 0.0, right.z).normalize_or_zero();

    // WASD movement
    if keyboard.pressed(KeyCode::KeyW) {
        velocity += forward_flat;
    }
    if keyboard.pressed(KeyCode::KeyS) {
        velocity -= forward_flat;
    }
    if keyboard.pressed(KeyCode::KeyD) {
        velocity += right_flat;
    }
    if keyboard.pressed(KeyCode::KeyA) {
        velocity -= right_flat;
    }

    // Check sprint first (affects all movement including vertical)
    let sprint = keyboard.pressed(KeyCode::ShiftLeft) || keyboard.pressed(KeyCode::ShiftRight);
    let speed_mult = if sprint { settings.sprint_multiplier } else { 1.0 };

    // Vertical movement (also affected by sprint!)
    if keyboard.pressed(KeyCode::Space) {
        velocity.y += 1.0;
    }
    if keyboard.pressed(KeyCode::ControlLeft) || keyboard.pressed(KeyCode::ControlRight) {
        velocity.y -= 1.0;
    }

    // Normalize and apply speed
    if velocity != Vec3::ZERO {
        velocity = velocity.normalize();
    }

    let speed = settings.speed * speed_mult;

    transform.translation += velocity * speed * time.delta_secs();

    // CLAMP position to stay inside room bounds (solid walls!)
    let bounds_x = ROOM_HALF_WIDTH - PLAYER_MARGIN;
    let bounds_z = ROOM_HALF_LENGTH - PLAYER_MARGIN;
    let min_y = 1.0; // Don't go below floor
    let max_y = ROOM_HEIGHT - PLAYER_MARGIN;

    transform.translation.x = transform.translation.x.clamp(-bounds_x, bounds_x);
    transform.translation.z = transform.translation.z.clamp(-bounds_z, bounds_z);
    transform.translation.y = transform.translation.y.clamp(min_y, max_y);
}

fn toggle_cursor_grab(
    keyboard: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    mut windows: Query<&mut Window, With<PrimaryWindow>>,
) {
    // Escape OR Middle Mouse Button toggles cursor
    let should_toggle =
        keyboard.just_pressed(KeyCode::Escape) || mouse.just_pressed(MouseButton::Middle);

    if should_toggle {
        if let Ok(mut window) = windows.get_single_mut() {
            match window.cursor_options.grab_mode {
                CursorGrabMode::None => {
                    window.cursor_options.grab_mode = CursorGrabMode::Locked;
                    window.cursor_options.visible = false;
                }
                _ => {
                    window.cursor_options.grab_mode = CursorGrabMode::None;
                    window.cursor_options.visible = true;
                }
            }
        }
    }
}

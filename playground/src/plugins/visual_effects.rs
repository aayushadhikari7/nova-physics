//! Visual effects plugin - screen shake, vignette, and settings application

use bevy::core_pipeline::bloom::Bloom;
use bevy::prelude::*;
use rand::Rng;

use crate::plugins::camera::PlayerCamera;
use crate::plugins::settings::GameSettings;

pub struct VisualEffectsPlugin;

impl Plugin for VisualEffectsPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ScreenShake>()
            .add_event::<ScreenShakeEvent>()
            .add_systems(
                Update,
                (
                    apply_visual_settings,
                    handle_screen_shake_events,
                    update_screen_shake,
                ),
            );
    }
}

// ============ SCREEN SHAKE ============

/// Resource tracking current screen shake state
#[derive(Resource, Default)]
pub struct ScreenShake {
    pub trauma: f32,        // Current trauma level (0-1)
    pub offset: Vec3,       // Current camera offset
    pub rotation_offset: f32, // Current rotation offset
}

/// Event to trigger screen shake
#[derive(Event)]
pub struct ScreenShakeEvent {
    pub intensity: f32, // 0.0 - 1.0
}

/// Handle incoming screen shake events
fn handle_screen_shake_events(
    mut events: EventReader<ScreenShakeEvent>,
    mut shake: ResMut<ScreenShake>,
    settings: Res<GameSettings>,
) {
    if !settings.explosion_screen_shake || settings.reduce_motion {
        events.clear();
        return;
    }

    for event in events.read() {
        // Add trauma (capped at 1.0)
        shake.trauma = (shake.trauma + event.intensity * settings.screen_shake_intensity).min(1.0);
    }
}

/// Update screen shake effect
fn update_screen_shake(
    mut shake: ResMut<ScreenShake>,
    mut camera: Query<&mut Transform, With<PlayerCamera>>,
    time: Res<Time>,
    settings: Res<GameSettings>,
) {
    // Decay trauma over time
    let decay_rate = 2.0; // Trauma decays in ~0.5 seconds
    shake.trauma = (shake.trauma - decay_rate * time.delta_secs()).max(0.0);

    // Remove previous offset
    if let Ok(mut transform) = camera.get_single_mut() {
        transform.translation -= shake.offset;
        // Reset rotation (simplified - just affects translation)
    }

    if shake.trauma > 0.0 && settings.explosion_screen_shake && !settings.reduce_motion {
        let mut rng = rand::thread_rng();

        // Shake amount increases with trauma squared for snappy feel
        let shake_amount = shake.trauma * shake.trauma;

        // Max offset based on intensity setting (subtle for clean aesthetic)
        let max_offset = 0.15 * settings.screen_shake_intensity;
        let max_rotation = 0.02 * settings.screen_shake_intensity;

        // Generate random offset using perlin-like smoothing
        let t = time.elapsed_secs() * 15.0;
        shake.offset = Vec3::new(
            (t * 1.1).sin() * rng.gen_range(-1.0..1.0),
            (t * 1.3).cos() * rng.gen_range(-1.0..1.0),
            (t * 0.9).sin() * rng.gen_range(-0.5..0.5),
        ) * max_offset
            * shake_amount;

        shake.rotation_offset = (t * 1.7).sin() * max_rotation * shake_amount;

        // Apply offset to camera
        if let Ok(mut transform) = camera.get_single_mut() {
            transform.translation += shake.offset;
        }
    } else {
        shake.offset = Vec3::ZERO;
        shake.rotation_offset = 0.0;
    }
}

// ============ VISUAL SETTINGS APPLICATION ============

/// Apply visual settings to camera and post-processing
fn apply_visual_settings(
    settings: Res<GameSettings>,
    mut bloom_query: Query<&mut Bloom, With<PlayerCamera>>,
    mut fog_query: Query<&mut bevy::pbr::DistanceFog, With<PlayerCamera>>,
) {
    if !settings.is_changed() {
        return;
    }

    // Apply bloom settings
    if let Ok(mut bloom) = bloom_query.get_single_mut() {
        if settings.bloom {
            bloom.intensity = settings.bloom_intensity;
        } else {
            bloom.intensity = 0.0;
        }
    }

    // Apply fog settings
    if let Ok(mut fog) = fog_query.get_single_mut() {
        if settings.fog_enabled {
            // Scale fog distances based on density setting
            let base_start = 120.0;
            let base_end = 400.0;
            let density_factor = 2.0 - settings.fog_density; // Higher density = closer fog

            fog.falloff = bevy::pbr::FogFalloff::Linear {
                start: base_start * density_factor,
                end: base_end * density_factor,
            };
            // Make fog visible
            fog.color = Color::srgba(0.92, 0.94, 0.96, 1.0);
        } else {
            // Push fog far away to effectively disable it
            fog.falloff = bevy::pbr::FogFalloff::Linear {
                start: 10000.0,
                end: 10001.0,
            };
        }
    }
}

// ============ HELPER FUNCTIONS ============

/// Trigger a screen shake from anywhere (public API helper)
#[allow(dead_code)]
pub fn trigger_screen_shake(writer: &mut EventWriter<ScreenShakeEvent>, intensity: f32) {
    writer.send(ScreenShakeEvent { intensity });
}

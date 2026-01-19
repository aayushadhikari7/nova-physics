//! Time control: pause, slow-mo, speed up

use bevy::prelude::*;

use crate::resources::NovaWorld;

pub struct TimeControlPlugin;

impl Plugin for TimeControlPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, time_control_system);
    }
}

fn time_control_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut nova: ResMut<NovaWorld>,
) {
    // P - Toggle pause
    if keyboard.just_pressed(KeyCode::KeyP) {
        nova.paused = !nova.paused;
    }

    // Backspace - Toggle 0.25x slow-mo
    if keyboard.just_pressed(KeyCode::Backspace) {
        if (nova.time_scale - 0.25).abs() < 0.01 {
            nova.time_scale = 1.0;
        } else {
            nova.time_scale = 0.25;
        }
    }

    // [ - Slow down
    if keyboard.just_pressed(KeyCode::BracketLeft) {
        nova.time_scale = (nova.time_scale * 0.5).max(0.1);
    }

    // ] - Speed up
    if keyboard.just_pressed(KeyCode::BracketRight) {
        nova.time_scale = (nova.time_scale * 2.0).min(4.0);
    }
}

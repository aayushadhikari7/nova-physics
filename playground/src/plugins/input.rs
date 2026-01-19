//! Input handling for hotbar selection and tool actions - MEGA EDITION!

use bevy::prelude::*;
use bevy::window::{MonitorSelection, PrimaryWindow, WindowMode};

use crate::resources::{
    CannonState, GrabState, Hotbar, InventoryState, JointToolState, MagnetState, PainterState,
    SelectedSlot, PAINT_COLORS,
};

pub struct InputPlugin;

impl Plugin for InputPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SelectedSlot>()
            .init_resource::<GrabState>()
            .init_resource::<JointToolState>()
            .init_resource::<InventoryState>()
            .add_systems(
                Update,
                (
                    hotbar_selection,
                    hotbar_paging,
                    adjust_grab_distance,
                    cycle_joint_type,
                    cycle_projectile_type,
                    cycle_paint_color,
                    toggle_magnet_mode,
                    toggle_inventory,
                    toggle_fullscreen,
                    exit_on_escape,
                ),
            );
    }
}

/// Exit application when Escape is pressed
fn exit_on_escape(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut exit: EventWriter<AppExit>,
) {
    if keyboard.just_pressed(KeyCode::Escape) {
        exit.send(AppExit::Success);
    }
}

fn hotbar_selection(keyboard: Res<ButtonInput<KeyCode>>, mut selected: ResMut<SelectedSlot>) {
    let keys = [
        KeyCode::Digit1,
        KeyCode::Digit2,
        KeyCode::Digit3,
        KeyCode::Digit4,
        KeyCode::Digit5,
        KeyCode::Digit6,
        KeyCode::Digit7,
        KeyCode::Digit8,
        KeyCode::Digit9,
    ];

    for (i, key) in keys.iter().enumerate() {
        if keyboard.just_pressed(*key) {
            selected.index = i;
        }
    }
}

fn hotbar_paging(keyboard: Res<ButtonInput<KeyCode>>, mut hotbar: ResMut<Hotbar>) {
    // Q and E to page through hotbar
    if keyboard.just_pressed(KeyCode::KeyQ) {
        hotbar.prev_page();
    }
    if keyboard.just_pressed(KeyCode::KeyZ) {
        hotbar.next_page();
    }

    // Also support mouse side buttons would go here if we had access to them
}

fn adjust_grab_distance(
    mut scroll_events: EventReader<bevy::input::mouse::MouseWheel>,
    mut grab_state: ResMut<GrabState>,
    hotbar: Res<Hotbar>,
    selected: Res<SelectedSlot>,
) {
    // Only adjust when gravity gun is selected and not handled by tool_scroll_adjust
    if hotbar.get_item(selected.index) != Some(&crate::resources::HotbarItem::GravityGun) {
        return;
    }

    for event in scroll_events.read() {
        if grab_state.holding.is_some() {
            grab_state.distance = (grab_state.distance + event.y * 0.5).clamp(2.0, 50.0);
        }
    }
}

fn cycle_joint_type(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut joint_state: ResMut<JointToolState>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
) {
    if hotbar.get_item(selected.index) == Some(&crate::resources::HotbarItem::JointTool) {
        if keyboard.just_pressed(KeyCode::Tab) {
            joint_state.joint_type = joint_state.joint_type.cycle_next();
        }
    }
}

fn cycle_projectile_type(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut cannon: ResMut<CannonState>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
) {
    if hotbar.get_item(selected.index) == Some(&crate::resources::HotbarItem::LaunchCannon) {
        if keyboard.just_pressed(KeyCode::Tab) {
            cannon.projectile_type = cannon.projectile_type.cycle_next();
        }
        // Toggle auto-fire with F
        if keyboard.just_pressed(KeyCode::KeyF) {
            cannon.auto_fire = !cannon.auto_fire;
        }
    }
}

fn cycle_paint_color(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut painter: ResMut<PainterState>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
) {
    if hotbar.get_item(selected.index) == Some(&crate::resources::HotbarItem::Painter) {
        if keyboard.just_pressed(KeyCode::Tab) {
            painter.color_index = (painter.color_index + 1) % PAINT_COLORS.len();
        }
    }
}

fn toggle_magnet_mode(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut magnet: ResMut<MagnetState>,
    selected: Res<SelectedSlot>,
    hotbar: Res<Hotbar>,
) {
    if hotbar.get_item(selected.index) == Some(&crate::resources::HotbarItem::Magnet) {
        if keyboard.just_pressed(KeyCode::Tab) {
            magnet.attract = !magnet.attract;
        }
    }
}

fn toggle_inventory(keyboard: Res<ButtonInput<KeyCode>>, mut inventory: ResMut<InventoryState>) {
    if keyboard.just_pressed(KeyCode::KeyE) {
        inventory.open = !inventory.open;
    }
}

/// Toggle fullscreen mode with F11
fn toggle_fullscreen(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut windows: Query<&mut Window, With<PrimaryWindow>>,
) {
    if keyboard.just_pressed(KeyCode::F11) {
        if let Ok(mut window) = windows.get_single_mut() {
            window.mode = match window.mode {
                WindowMode::Windowed => WindowMode::BorderlessFullscreen(MonitorSelection::Current),
                _ => WindowMode::Windowed,
            };
        }
    }
}

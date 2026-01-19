//! Hotbar and inventory UI - MEGA EDITION!

use bevy::prelude::*;

use crate::resources::{
    CannonState, DebugVisuals, GrabState, Hotbar, HotbarItem, InventoryState, JointToolState,
    MagnetState, NovaWorld, PainterState, PlaygroundStats, SelectedSlot,
};

pub struct InventoryPlugin;

impl Plugin for InventoryPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Hotbar>()
            .init_resource::<PlaygroundStats>()
            .add_systems(Startup, setup_ui)
            .add_systems(
                Update,
                (
                    update_ui,
                    update_inventory_panel,
                    update_stats,
                    inventory_item_click,
                ),
            );
    }
}

#[derive(Component)]
struct HudRoot;

#[derive(Component)]
struct BodyCountText;

#[derive(Component)]
struct FpsText;

#[derive(Component)]
struct TimeScaleText;

#[derive(Component)]
struct ToolInfoText;

#[derive(Component)]
struct HotbarSlot(usize);

#[derive(Component)]
struct HelpText;

#[derive(Component)]
struct Crosshair;

#[derive(Component)]
struct PageIndicator;

#[derive(Component)]
struct InventoryPanel;

#[derive(Component)]
struct InventoryCategory(usize);

#[derive(Component)]
struct InventoryItem(HotbarItem);

#[derive(Component)]
struct StatsPanel;

#[derive(Component)]
struct SpawnedCountText;

#[derive(Component)]
struct DeletedCountText;

#[derive(Component)]
struct JointCountText;

#[derive(Component)]
struct ColliderCountText;

const SLOT_SIZE: f32 = 50.0;
const SLOT_GAP: f32 = 4.0;
const HOTBAR_BG: Color = Color::srgba(0.1, 0.1, 0.1, 0.8);
const SLOT_BG: Color = Color::srgba(0.2, 0.2, 0.2, 0.9);
const SLOT_SELECTED: Color = Color::srgba(0.4, 0.6, 0.4, 0.95);
const TEXT_COLOR: Color = Color::srgba(1.0, 1.0, 1.0, 1.0);
const PANEL_BG: Color = Color::srgba(0.05, 0.05, 0.05, 0.95);
const CATEGORY_ACTIVE: Color = Color::srgba(0.3, 0.5, 0.3, 1.0);
const CATEGORY_INACTIVE: Color = Color::srgba(0.15, 0.15, 0.15, 1.0);

fn setup_ui(mut commands: Commands) {
    // Root UI container
    commands
        .spawn((
            Node {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                flex_direction: FlexDirection::Column,
                justify_content: JustifyContent::SpaceBetween,
                ..default()
            },
            HudRoot,
        ))
        .with_children(|parent| {
            // Top bar - stats
            parent
                .spawn(Node {
                    width: Val::Percent(100.0),
                    padding: UiRect::all(Val::Px(10.0)),
                    flex_direction: FlexDirection::Row,
                    justify_content: JustifyContent::SpaceBetween,
                    ..default()
                })
                .with_children(|parent| {
                    // Left stats
                    parent
                        .spawn(Node {
                            flex_direction: FlexDirection::Row,
                            column_gap: Val::Px(20.0),
                            ..default()
                        })
                        .with_children(|parent| {
                            parent.spawn((
                                Text::new("Bodies: 0"),
                                TextFont {
                                    font_size: 16.0,
                                    ..default()
                                },
                                TextColor(TEXT_COLOR),
                                BodyCountText,
                            ));
                            parent.spawn((
                                Text::new("FPS: 60"),
                                TextFont {
                                    font_size: 16.0,
                                    ..default()
                                },
                                TextColor(TEXT_COLOR),
                                FpsText,
                            ));
                            parent.spawn((
                                Text::new("Time: 1.0x"),
                                TextFont {
                                    font_size: 16.0,
                                    ..default()
                                },
                                TextColor(TEXT_COLOR),
                                TimeScaleText,
                            ));
                        });

                    // Right - detailed stats panel
                    parent
                        .spawn((
                            Node {
                                flex_direction: FlexDirection::Row,
                                column_gap: Val::Px(15.0),
                                padding: UiRect::all(Val::Px(5.0)),
                                ..default()
                            },
                            BackgroundColor(Color::srgba(0.1, 0.1, 0.1, 0.5)),
                            StatsPanel,
                        ))
                        .with_children(|parent| {
                            parent.spawn((
                                Text::new("Spawned: 0"),
                                TextFont {
                                    font_size: 12.0,
                                    ..default()
                                },
                                TextColor(Color::srgba(0.5, 0.8, 0.5, 1.0)),
                                SpawnedCountText,
                            ));
                            parent.spawn((
                                Text::new("Deleted: 0"),
                                TextFont {
                                    font_size: 12.0,
                                    ..default()
                                },
                                TextColor(Color::srgba(0.8, 0.5, 0.5, 1.0)),
                                DeletedCountText,
                            ));
                            parent.spawn((
                                Text::new("Joints: 0"),
                                TextFont {
                                    font_size: 12.0,
                                    ..default()
                                },
                                TextColor(Color::srgba(0.5, 0.5, 0.8, 1.0)),
                                JointCountText,
                            ));
                            parent.spawn((
                                Text::new("Colliders: 0"),
                                TextFont {
                                    font_size: 12.0,
                                    ..default()
                                },
                                TextColor(Color::srgba(0.8, 0.8, 0.5, 1.0)),
                                ColliderCountText,
                            ));
                            parent.spawn((
                                Text::new("[F1-F4]"),
                                TextFont {
                                    font_size: 12.0,
                                    ..default()
                                },
                                TextColor(Color::srgba(0.6, 0.6, 0.6, 1.0)),
                            ));
                        });
                });

            // Center - crosshair
            parent
                .spawn(Node {
                    width: Val::Percent(100.0),
                    height: Val::Percent(100.0),
                    position_type: PositionType::Absolute,
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::Center,
                    ..default()
                })
                .with_children(|parent| {
                    parent.spawn((
                        Text::new("+"),
                        TextFont {
                            font_size: 24.0,
                            ..default()
                        },
                        TextColor(Color::srgba(1.0, 1.0, 1.0, 0.8)),
                        Crosshair,
                    ));
                });

            // Bottom section
            parent
                .spawn(Node {
                    width: Val::Percent(100.0),
                    flex_direction: FlexDirection::Column,
                    align_items: AlignItems::Center,
                    padding: UiRect::bottom(Val::Px(20.0)),
                    row_gap: Val::Px(10.0),
                    ..default()
                })
                .with_children(|parent| {
                    // Tool info (above hotbar)
                    parent.spawn((
                        Text::new(""),
                        TextFont {
                            font_size: 14.0,
                            ..default()
                        },
                        TextColor(Color::srgba(0.8, 0.8, 0.3, 1.0)),
                        ToolInfoText,
                    ));

                    // Hotbar container with page indicator
                    parent
                        .spawn(Node {
                            flex_direction: FlexDirection::Column,
                            align_items: AlignItems::Center,
                            row_gap: Val::Px(5.0),
                            ..default()
                        })
                        .with_children(|parent| {
                            // Page indicator
                            parent.spawn((
                                Text::new("Page 1/5 (Q/Z to change)"),
                                TextFont {
                                    font_size: 12.0,
                                    ..default()
                                },
                                TextColor(Color::srgba(0.6, 0.6, 0.6, 1.0)),
                                PageIndicator,
                            ));

                            // Hotbar
                            parent
                                .spawn((
                                    Node {
                                        padding: UiRect::all(Val::Px(4.0)),
                                        column_gap: Val::Px(SLOT_GAP),
                                        flex_direction: FlexDirection::Row,
                                        ..default()
                                    },
                                    BackgroundColor(HOTBAR_BG),
                                ))
                                .with_children(|parent| {
                                    for i in 0..9 {
                                        parent
                                            .spawn((
                                                Node {
                                                    width: Val::Px(SLOT_SIZE),
                                                    height: Val::Px(SLOT_SIZE),
                                                    justify_content: JustifyContent::Center,
                                                    align_items: AlignItems::Center,
                                                    flex_direction: FlexDirection::Column,
                                                    ..default()
                                                },
                                                BackgroundColor(SLOT_BG),
                                                HotbarSlot(i),
                                            ))
                                            .with_children(|parent| {
                                                // Slot number
                                                parent.spawn((
                                                    Text::new(format!("{}", i + 1)),
                                                    TextFont {
                                                        font_size: 10.0,
                                                        ..default()
                                                    },
                                                    TextColor(Color::srgba(0.5, 0.5, 0.5, 1.0)),
                                                ));
                                                // Item name (will be updated)
                                                parent.spawn((
                                                    Text::new(""),
                                                    TextFont {
                                                        font_size: 11.0,
                                                        ..default()
                                                    },
                                                    TextColor(TEXT_COLOR),
                                                ));
                                            });
                                    }
                                });
                        });

                    // Help text - more comprehensive
                    parent.spawn((
                        Text::new(
                            "WASD: Move | Space/Ctrl: Up/Down | Shift: Fast | E: Inventory | Q/Z: Page | P: Pause | [/]: Speed | R: Reset | Tab: Cycle | F1-F4: Debug | F11: Fullscreen | Esc: Exit",
                        ),
                        TextFont {
                            font_size: 11.0,
                            ..default()
                        },
                        TextColor(Color::srgba(0.6, 0.6, 0.6, 1.0)),
                        HelpText,
                    ));
                });
        });

    // Create inventory panel (hidden by default)
    spawn_inventory_panel(&mut commands);
}

fn spawn_inventory_panel(commands: &mut Commands) {
    commands
        .spawn((
            Node {
                width: Val::Percent(100.0),
                height: Val::Percent(100.0),
                position_type: PositionType::Absolute,
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                display: Display::None,
                ..default()
            },
            BackgroundColor(Color::srgba(0.0, 0.0, 0.0, 0.5)),
            InventoryPanel,
        ))
        .with_children(|parent| {
            // Main panel
            parent
                .spawn((
                    Node {
                        width: Val::Px(600.0),
                        height: Val::Px(500.0),
                        flex_direction: FlexDirection::Column,
                        padding: UiRect::all(Val::Px(10.0)),
                        ..default()
                    },
                    BackgroundColor(PANEL_BG),
                ))
                .with_children(|parent| {
                    // Title
                    parent.spawn((
                        Text::new("INVENTORY"),
                        TextFont {
                            font_size: 24.0,
                            ..default()
                        },
                        TextColor(TEXT_COLOR),
                    ));

                    // Categories
                    parent
                        .spawn(Node {
                            flex_direction: FlexDirection::Row,
                            column_gap: Val::Px(5.0),
                            margin: UiRect::vertical(Val::Px(10.0)),
                            ..default()
                        })
                        .with_children(|parent| {
                            let categories = ["Shapes", "Tools", "Presets", "Materials"];
                            for (i, cat) in categories.iter().enumerate() {
                                parent.spawn((
                                    Text::new(*cat),
                                    TextFont {
                                        font_size: 14.0,
                                        ..default()
                                    },
                                    TextColor(TEXT_COLOR),
                                    Node {
                                        padding: UiRect::axes(Val::Px(15.0), Val::Px(8.0)),
                                        ..default()
                                    },
                                    BackgroundColor(if i == 0 {
                                        CATEGORY_ACTIVE
                                    } else {
                                        CATEGORY_INACTIVE
                                    }),
                                    InventoryCategory(i),
                                ));
                            }
                        });

                    // Items grid
                    parent
                        .spawn(Node {
                            flex_direction: FlexDirection::Row,
                            flex_wrap: FlexWrap::Wrap,
                            column_gap: Val::Px(8.0),
                            row_gap: Val::Px(8.0),
                            padding: UiRect::all(Val::Px(10.0)),
                            ..default()
                        })
                        .with_children(|parent| {
                            // Add all shapes
                            let shapes = [
                                HotbarItem::SpawnBox,
                                HotbarItem::SpawnSphere,
                                HotbarItem::SpawnCapsule,
                                HotbarItem::SpawnCylinder,
                                HotbarItem::SpawnCone,
                                HotbarItem::SpawnCompound,
                                HotbarItem::SpawnRandom,
                            ];
                            for item in shapes {
                                spawn_inventory_item(parent, item);
                            }

                            // Add tools
                            let tools = [
                                HotbarItem::GravityGun,
                                HotbarItem::ForceGun,
                                HotbarItem::Explosion,
                                HotbarItem::JointTool,
                                HotbarItem::Delete,
                                HotbarItem::Freeze,
                                HotbarItem::Clone,
                                HotbarItem::Magnet,
                                HotbarItem::LaunchCannon,
                                HotbarItem::Painter,
                            ];
                            for item in tools {
                                spawn_inventory_item(parent, item);
                            }
                        });

                    // Instructions
                    parent.spawn((
                        Text::new("Click to add to hotbar | Press E to close"),
                        TextFont {
                            font_size: 12.0,
                            ..default()
                        },
                        TextColor(Color::srgba(0.5, 0.5, 0.5, 1.0)),
                        Node {
                            margin: UiRect::top(Val::Auto),
                            ..default()
                        },
                    ));
                });
        });
}

fn spawn_inventory_item(parent: &mut ChildBuilder, item: HotbarItem) {
    parent
        .spawn((
            Node {
                width: Val::Px(80.0),
                height: Val::Px(60.0),
                flex_direction: FlexDirection::Column,
                justify_content: JustifyContent::Center,
                align_items: AlignItems::Center,
                ..default()
            },
            BackgroundColor(SLOT_BG),
            InventoryItem(item),
            Interaction::None,
        ))
        .with_children(|child| {
            child.spawn((
                Text::new(item.name()),
                TextFont {
                    font_size: 12.0,
                    ..default()
                },
                TextColor(TEXT_COLOR),
            ));
        });
}

fn update_inventory_panel(
    inventory_state: Res<InventoryState>,
    mut panel: Query<&mut Node, With<InventoryPanel>>,
) {
    if let Ok(mut node) = panel.get_single_mut() {
        node.display = if inventory_state.open {
            Display::Flex
        } else {
            Display::None
        };
    }
}

fn inventory_item_click(
    mut interaction_query: Query<
        (&Interaction, &InventoryItem, &mut BackgroundColor),
        Changed<Interaction>,
    >,
    mut hotbar: ResMut<Hotbar>,
    selected: Res<SelectedSlot>,
) {
    for (interaction, item, mut bg) in interaction_query.iter_mut() {
        match *interaction {
            Interaction::Pressed => {
                // Add item to selected hotbar slot - use absolute index for paging!
                if let Some(abs_index) = hotbar.get_absolute_index(selected.index) {
                    if abs_index < hotbar.items.len() {
                        hotbar.items[abs_index] = item.0;
                    }
                }
                *bg = BackgroundColor(SLOT_SELECTED);
            }
            Interaction::Hovered => {
                *bg = BackgroundColor(Color::srgba(0.3, 0.3, 0.3, 1.0));
            }
            Interaction::None => {
                *bg = BackgroundColor(SLOT_BG);
            }
        }
    }
}

fn update_stats(
    nova: Res<NovaWorld>,
    stats: Res<PlaygroundStats>,
    mut stat_texts: Query<
        (
            Option<&SpawnedCountText>,
            Option<&DeletedCountText>,
            Option<&JointCountText>,
            Option<&ColliderCountText>,
            &mut Text,
        ),
        Or<(
            With<SpawnedCountText>,
            With<DeletedCountText>,
            With<JointCountText>,
            With<ColliderCountText>,
        )>,
    >,
) {
    for (spawned, deleted, joint, collider, mut text) in stat_texts.iter_mut() {
        if spawned.is_some() {
            **text = format!("Spawned: {}", stats.total_spawned);
        } else if deleted.is_some() {
            **text = format!("Deleted: {}", stats.total_deleted);
        } else if joint.is_some() {
            **text = format!("Joints: {}", nova.world.joints().count());
        } else if collider.is_some() {
            **text = format!("Colliders: {}", nova.world.colliders().count());
        }
    }
}

#[allow(clippy::type_complexity)]
fn update_ui(
    nova: Res<NovaWorld>,
    hotbar: Res<Hotbar>,
    selected: Res<SelectedSlot>,
    joint_state: Res<JointToolState>,
    grab_state: Res<GrabState>,
    cannon_state: Res<CannonState>,
    magnet_state: Res<MagnetState>,
    painter_state: Res<PainterState>,
    debug_visuals: Res<DebugVisuals>,
    time: Res<Time>,
    mut hud_texts: Query<
        (
            Option<&BodyCountText>,
            Option<&FpsText>,
            Option<&TimeScaleText>,
            Option<&ToolInfoText>,
            Option<&PageIndicator>,
            &mut Text,
        ),
        Or<(
            With<BodyCountText>,
            With<FpsText>,
            With<TimeScaleText>,
            With<ToolInfoText>,
            With<PageIndicator>,
        )>,
    >,
    mut slots: Query<(&HotbarSlot, &mut BackgroundColor, &Children)>,
    mut slot_texts: Query<
        &mut Text,
        (
            Without<BodyCountText>,
            Without<FpsText>,
            Without<TimeScaleText>,
            Without<ToolInfoText>,
            Without<PageIndicator>,
            Without<HotbarSlot>,
        ),
    >,
) {
    // Build tool info string - use get_item() for proper paging
    let selected_item = hotbar.get_item(selected.index);
    let tool_info = match selected_item {
        Some(HotbarItem::JointTool) => {
            if joint_state.first_body.is_some() {
                format!(
                    "[Joint: {}] - Click second body",
                    joint_state.joint_type.name()
                )
            } else {
                format!("[Joint: {}] - Tab to cycle", joint_state.joint_type.name())
            }
        }
        Some(HotbarItem::GravityGun) => {
            if grab_state.holding.is_some() {
                format!(
                    "[Grab] HOLDING - Dist: {:.1}m | RMB: Throw | Scroll: Distance",
                    grab_state.distance
                )
            } else {
                "[Grab] LMB: Grab/Release | RMB: Throw | Scroll: Distance".to_string()
            }
        }
        Some(HotbarItem::ForceGun) => "[Force] LMB: Push | RMB: Pull".to_string(),
        Some(HotbarItem::Explosion) => "[Explosion] LMB: Small | RMB: Large".to_string(),
        Some(HotbarItem::Delete) => "[Delete] LMB: Single | RMB: Area".to_string(),
        Some(HotbarItem::Freeze) => "[Freeze] LMB: Toggle freeze".to_string(),
        Some(HotbarItem::Clone) => "[Clone] LMB: Clone object".to_string(),
        Some(HotbarItem::Magnet) => {
            let mode = if magnet_state.attract {
                "ATTRACT"
            } else {
                "REPEL"
            };
            format!("[Magnet: {}] LMB: Apply | Tab: Toggle mode", mode)
        }
        Some(HotbarItem::LaunchCannon) => {
            let auto = if cannon_state.auto_fire { " AUTO" } else { "" };
            format!(
                "[Cannon: {}{}] LMB: Fire | Tab: Type | F: Auto",
                cannon_state.projectile_type.name(),
                auto
            )
        }
        Some(HotbarItem::Painter) => {
            let color_name = match painter_state.color_index {
                0 => "Red",
                1 => "Orange",
                2 => "Yellow",
                3 => "Green",
                4 => "Blue",
                5 => "Purple",
                6 => "Pink",
                7 => "White",
                8 => "Black",
                9 => "Rainbow",
                _ => "Unknown",
            };
            format!("[Paint: {}] LMB: Paint | Tab: Color", color_name)
        }
        Some(item) if item.is_spawnable() => {
            format!("[Spawn: {}] LMB: Place", item.name())
        }
        _ => "".to_string(),
    };

    // Add debug info if any visualization is on
    let debug_suffix = if debug_visuals.show_contacts
        || debug_visuals.show_joints
        || debug_visuals.show_velocities
        || debug_visuals.show_aabbs
    {
        format!(
            " | Debug: {}{}{}{}",
            if debug_visuals.show_contacts { "C" } else { "" },
            if debug_visuals.show_joints { "J" } else { "" },
            if debug_visuals.show_velocities { "V" } else { "" },
            if debug_visuals.show_aabbs { "A" } else { "" },
        )
    } else {
        String::new()
    };

    // Update all HUD text elements
    for (body_count, fps, time_scale, tool, page, mut text) in hud_texts.iter_mut() {
        if body_count.is_some() {
            **text = format!("Bodies: {}", nova.world.body_count());
        } else if fps.is_some() {
            let fps_val = 1.0 / time.delta_secs();
            **text = format!("FPS: {:.0}", fps_val);
        } else if time_scale.is_some() {
            **text = if nova.paused {
                "PAUSED".to_string()
            } else {
                format!("Time: {:.2}x", nova.time_scale)
            };
        } else if tool.is_some() {
            **text = format!("{}{}", tool_info, debug_suffix);
        } else if page.is_some() {
            **text = format!(
                "Page {}/{} (Q/Z to change)",
                hotbar.current_page + 1,
                hotbar.total_pages()
            );
        }
    }

    // Update hotbar slots - use get_item() for proper paging
    for (slot, mut bg, children) in slots.iter_mut() {
        let is_selected = slot.0 == selected.index;
        *bg = if is_selected {
            BackgroundColor(SLOT_SELECTED)
        } else {
            BackgroundColor(SLOT_BG)
        };

        // Update item name in slot (second child text) - use get_item for paging!
        if let Some(item) = hotbar.get_item(slot.0) {
            // The second child should be the item name text
            if children.len() >= 2 {
                if let Ok(mut text) = slot_texts.get_mut(children[1]) {
                    **text = item.short_name().to_string();
                }
            }
        } else {
            // Empty slot on this page
            if children.len() >= 2 {
                if let Ok(mut text) = slot_texts.get_mut(children[1]) {
                    **text = "".to_string();
                }
            }
        }
    }
}

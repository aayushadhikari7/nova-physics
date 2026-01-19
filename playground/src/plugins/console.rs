//! Physics Console - Event log with timestamps
//!
//! Press ~ (Backquote) to toggle the console visibility

use bevy::prelude::*;
use std::collections::VecDeque;

pub struct ConsolePlugin;

impl Plugin for ConsolePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsConsole>()
            .add_systems(Startup, setup_console_ui)
            .add_systems(
                Update,
                (
                    toggle_console,
                    update_console_display,
                    log_physics_stats,
                    cleanup_old_logs,
                ),
            );
    }
}

/// Maximum number of log entries to keep (prevents memory leaks)
const MAX_LOG_ENTRIES: usize = 500;
/// How often to log stats (in seconds)
const STATS_LOG_INTERVAL: f32 = 2.0;

/// A single log entry
#[derive(Clone)]
pub struct LogEntry {
    pub timestamp: f32,
    pub category: LogCategory,
    pub message: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogCategory {
    Physics,
    Spawn,
    Delete,
    Joint,
    Collision,
    Zone,
    Tool,
    System,
}

impl LogCategory {
    fn color(&self) -> Color {
        match self {
            Self::Physics => Color::srgb(0.7, 0.7, 1.0),
            Self::Spawn => Color::srgb(0.5, 1.0, 0.5),
            Self::Delete => Color::srgb(1.0, 0.5, 0.5),
            Self::Joint => Color::srgb(1.0, 0.8, 0.3),
            Self::Collision => Color::srgb(1.0, 0.6, 0.6),
            Self::Zone => Color::srgb(0.6, 0.8, 1.0),
            Self::Tool => Color::srgb(0.8, 0.6, 1.0),
            Self::System => Color::srgb(0.6, 0.6, 0.6),
        }
    }

    fn prefix(&self) -> &'static str {
        match self {
            Self::Physics => "[PHY]",
            Self::Spawn => "[SPW]",
            Self::Delete => "[DEL]",
            Self::Joint => "[JNT]",
            Self::Collision => "[COL]",
            Self::Zone => "[ZON]",
            Self::Tool => "[TOL]",
            Self::System => "[SYS]",
        }
    }
}

/// Resource containing all log entries and console state
#[derive(Resource)]
pub struct PhysicsConsole {
    pub visible: bool,
    pub entries: VecDeque<LogEntry>,
    pub filter: Option<LogCategory>,
    pub last_stats_time: f32,
    pub paused: bool,
    // Stats tracking
    pub collision_count: u32,
    pub peak_bodies: u32,
    pub peak_velocity: f32,
    pub total_momentum: f32,
}

impl Default for PhysicsConsole {
    fn default() -> Self {
        Self {
            visible: false,
            entries: VecDeque::with_capacity(MAX_LOG_ENTRIES),
            filter: None,
            last_stats_time: 0.0,
            paused: false,
            collision_count: 0,
            peak_bodies: 0,
            peak_velocity: 0.0,
            total_momentum: 0.0,
        }
    }
}

impl PhysicsConsole {
    /// Log a message to the console
    pub fn log(&mut self, category: LogCategory, message: impl Into<String>, time: f32) {
        if self.paused {
            return;
        }

        let entry = LogEntry {
            timestamp: time,
            category,
            message: message.into(),
        };

        self.entries.push_back(entry);

        // Prevent memory leak by removing old entries
        while self.entries.len() > MAX_LOG_ENTRIES {
            self.entries.pop_front();
        }
    }

    /// Get filtered entries for display
    pub fn filtered_entries(&self) -> impl Iterator<Item = &LogEntry> {
        self.entries.iter().filter(move |e| {
            self.filter
                .map(|f| e.category == f)
                .unwrap_or(true)
        })
    }

    /// Clear all entries
    pub fn clear(&mut self) {
        self.entries.clear();
    }
}

// UI Components
#[derive(Component)]
struct ConsolePanel;

#[derive(Component)]
struct ConsoleText;

#[derive(Component)]
struct ConsoleStatsText;

fn setup_console_ui(mut commands: Commands) {
    // Console panel (hidden by default)
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: Val::Px(10.0),
                top: Val::Px(50.0),
                width: Val::Px(500.0),
                height: Val::Px(400.0),
                flex_direction: FlexDirection::Column,
                padding: UiRect::all(Val::Px(10.0)),
                display: Display::None,
                overflow: Overflow::clip(),
                ..default()
            },
            BackgroundColor(Color::srgba(0.05, 0.05, 0.1, 0.9)),
            ConsolePanel,
        ))
        .with_children(|parent| {
            // Header
            parent.spawn((
                Text::new("PHYSICS CONSOLE (~)"),
                TextFont {
                    font_size: 16.0,
                    ..default()
                },
                TextColor(Color::srgb(0.8, 0.8, 1.0)),
            ));

            // Stats bar
            parent.spawn((
                Text::new("Bodies: 0 | Collisions: 0 | Peak Vel: 0.0"),
                TextFont {
                    font_size: 12.0,
                    ..default()
                },
                TextColor(Color::srgb(0.6, 0.6, 0.6)),
                Node {
                    margin: UiRect::vertical(Val::Px(5.0)),
                    ..default()
                },
                ConsoleStatsText,
            ));

            // Scrollable log area
            parent
                .spawn(Node {
                    flex_direction: FlexDirection::Column,
                    overflow: Overflow::clip_y(),
                    height: Val::Percent(100.0),
                    ..default()
                })
                .with_children(|parent| {
                    parent.spawn((
                        Text::new(""),
                        TextFont {
                            font_size: 11.0,
                            ..default()
                        },
                        TextColor(Color::srgb(0.8, 0.8, 0.8)),
                        ConsoleText,
                    ));
                });

            // Footer with controls
            parent.spawn((
                Text::new("C: Clear | P: Pause | 1-8: Filter categories"),
                TextFont {
                    font_size: 10.0,
                    ..default()
                },
                TextColor(Color::srgb(0.4, 0.4, 0.4)),
                Node {
                    margin: UiRect::top(Val::Px(5.0)),
                    ..default()
                },
            ));
        });
}

fn toggle_console(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut console: ResMut<PhysicsConsole>,
    mut panel: Query<&mut Node, With<ConsolePanel>>,
) {
    // Toggle visibility with ~ (Backquote)
    if keyboard.just_pressed(KeyCode::Backquote) {
        console.visible = !console.visible;
        if let Ok(mut node) = panel.get_single_mut() {
            node.display = if console.visible {
                Display::Flex
            } else {
                Display::None
            };
        }
    }

    // Console controls (only when visible)
    if console.visible {
        // C - Clear
        if keyboard.just_pressed(KeyCode::KeyC) && !keyboard.pressed(KeyCode::ControlLeft) {
            console.clear();
        }

        // P - Pause/Resume logging
        if keyboard.just_pressed(KeyCode::KeyP) {
            console.paused = !console.paused;
        }

        // Number keys to filter categories
        if keyboard.just_pressed(KeyCode::Digit1) {
            console.filter = Some(LogCategory::Physics);
        } else if keyboard.just_pressed(KeyCode::Digit2) {
            console.filter = Some(LogCategory::Spawn);
        } else if keyboard.just_pressed(KeyCode::Digit3) {
            console.filter = Some(LogCategory::Delete);
        } else if keyboard.just_pressed(KeyCode::Digit4) {
            console.filter = Some(LogCategory::Joint);
        } else if keyboard.just_pressed(KeyCode::Digit5) {
            console.filter = Some(LogCategory::Collision);
        } else if keyboard.just_pressed(KeyCode::Digit6) {
            console.filter = Some(LogCategory::Zone);
        } else if keyboard.just_pressed(KeyCode::Digit7) {
            console.filter = Some(LogCategory::Tool);
        } else if keyboard.just_pressed(KeyCode::Digit8) {
            console.filter = Some(LogCategory::System);
        } else if keyboard.just_pressed(KeyCode::Digit0) {
            console.filter = None; // Show all
        }
    }
}

fn update_console_display(
    console: Res<PhysicsConsole>,
    mut text_query: Query<&mut Text, With<ConsoleText>>,
    mut stats_query: Query<&mut Text, (With<ConsoleStatsText>, Without<ConsoleText>)>,
) {
    if !console.visible {
        return;
    }

    // Update log text
    if let Ok(mut text) = text_query.get_single_mut() {
        let mut output = String::new();

        // Show last 30 entries (most recent at bottom)
        let entries: Vec<_> = console.filtered_entries().collect();
        let start = entries.len().saturating_sub(30);

        for entry in entries.iter().skip(start) {
            let time_str = format!("{:>8.2}s", entry.timestamp);
            let prefix = entry.category.prefix();
            output.push_str(&format!("{} {} {}\n", time_str, prefix, entry.message));
        }

        if console.paused {
            output.push_str("\n[PAUSED]");
        }

        if let Some(filter) = console.filter {
            output.push_str(&format!("\n[Filter: {:?}]", filter));
        }

        **text = output;
    }

    // Update stats text
    if let Ok(mut text) = stats_query.get_single_mut() {
        **text = format!(
            "Bodies: {} | Collisions: {} | Peak Vel: {:.1} | Momentum: {:.1}",
            console.peak_bodies, console.collision_count, console.peak_velocity, console.total_momentum
        );
    }
}

fn log_physics_stats(
    mut console: ResMut<PhysicsConsole>,
    nova: Res<crate::resources::NovaWorld>,
    time: Res<Time>,
) {
    let elapsed = time.elapsed_secs();

    // Update peak bodies
    let body_count = nova.world.body_count() as u32;
    if body_count > console.peak_bodies {
        console.peak_bodies = body_count;
    }

    // Calculate velocities and momentum
    let mut max_velocity = 0.0f32;
    let mut total_momentum = 0.0f32;

    for (_, body) in nova.world.bodies() {
        let vel = body.linear_velocity;
        let speed = (vel.x * vel.x + vel.y * vel.y + vel.z * vel.z).sqrt();

        if speed > max_velocity {
            max_velocity = speed;
        }

        // momentum = mass * velocity
        total_momentum += body.mass * speed;
    }

    console.peak_velocity = console.peak_velocity.max(max_velocity);
    console.total_momentum = total_momentum;

    // Periodic stats logging
    if elapsed - console.last_stats_time > STATS_LOG_INTERVAL {
        console.last_stats_time = elapsed;

        if !console.paused {
            let joint_count = nova.world.joints().count();
            let collider_count = nova.world.colliders().count();

            console.log(
                LogCategory::Physics,
                format!(
                    "Stats: {} bodies, {} joints, {} colliders, vel={:.1}, p={:.1}",
                    body_count, joint_count, collider_count, max_velocity, total_momentum
                ),
                elapsed,
            );
        }
    }
}

fn cleanup_old_logs(mut console: ResMut<PhysicsConsole>) {
    // Extra safety: ensure we never exceed capacity
    while console.entries.len() > MAX_LOG_ENTRIES {
        console.entries.pop_front();
    }
}

// ============ PUBLIC LOGGING FUNCTIONS ============

/// Log a spawn event
pub fn log_spawn(console: &mut PhysicsConsole, item: &str, pos: bevy::math::Vec3, time: f32) {
    console.log(
        LogCategory::Spawn,
        format!("Spawned {} at ({:.1}, {:.1}, {:.1})", item, pos.x, pos.y, pos.z),
        time,
    );
}

/// Log a delete event
pub fn log_delete(console: &mut PhysicsConsole, count: u32, time: f32) {
    console.log(
        LogCategory::Delete,
        format!("Deleted {} object(s)", count),
        time,
    );
}

/// Log a joint creation
pub fn log_joint(console: &mut PhysicsConsole, joint_type: &str, time: f32) {
    console.log(
        LogCategory::Joint,
        format!("Created {} joint", joint_type),
        time,
    );
}

/// Log a tool action
pub fn log_tool(console: &mut PhysicsConsole, tool: &str, action: &str, time: f32) {
    console.log(
        LogCategory::Tool,
        format!("{}: {}", tool, action),
        time,
    );
}

/// Log a zone event
pub fn log_zone(console: &mut PhysicsConsole, zone_type: &str, action: &str, time: f32) {
    console.log(
        LogCategory::Zone,
        format!("{} {}", zone_type, action),
        time,
    );
}

/// Log a collision event
pub fn log_collision(console: &mut PhysicsConsole, info: &str, time: f32) {
    console.collision_count += 1;
    console.log(LogCategory::Collision, info.to_string(), time);
}

/// Log a system event
pub fn log_system(console: &mut PhysicsConsole, msg: &str, time: f32) {
    console.log(LogCategory::System, msg.to_string(), time);
}

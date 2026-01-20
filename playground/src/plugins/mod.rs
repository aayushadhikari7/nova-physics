//! Plugin modules for the physics playground - MEGA EDITION!

pub mod camera;
pub mod console;
pub mod debug;
pub mod effects;
pub mod input;
pub mod inventory;
pub mod physics;
pub mod settings;
pub mod spawning;
pub mod time_control;
pub mod tools;
pub mod vehicle;
pub mod visual_effects;

pub use camera::CameraPlugin;
pub use console::ConsolePlugin;
pub use debug::DebugPlugin;
pub use effects::EffectsPlugin;
pub use input::InputPlugin;
pub use inventory::InventoryPlugin;
pub use physics::PhysicsPlugin;
pub use settings::SettingsPlugin;
pub use spawning::SpawningPlugin;
pub use time_control::TimeControlPlugin;
pub use tools::ToolsPlugin;
pub use vehicle::VehiclePlugin;
pub use visual_effects::VisualEffectsPlugin;

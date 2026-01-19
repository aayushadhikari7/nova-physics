//! Shared state resources for the physics playground

use bevy::prelude::*;
use nova::prelude::{ColliderHandle, PhysicsWorld, RigidBodyHandle};

/// Resource wrapping the Nova physics world
#[derive(Resource)]
pub struct NovaWorld {
    pub world: PhysicsWorld,
    pub time_scale: f32,
    pub paused: bool,
}

impl Default for NovaWorld {
    fn default() -> Self {
        let mut world = PhysicsWorld::new();
        world.set_gravity(nova::prelude::Vec3::new(0.0, -9.81, 0.0));
        Self {
            world,
            time_scale: 1.0,
            paused: false,
        }
    }
}

/// Currently selected tool/item in the hotbar
#[derive(Resource, Default)]
pub struct SelectedSlot {
    pub index: usize,
}

/// Hotbar item types - JAM PACKED WITH FEATURES!
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum HotbarItem {
    // === SHAPES ===
    SpawnBox,
    SpawnSphere,
    SpawnCapsule,
    SpawnCylinder,
    SpawnCone,
    SpawnCompound,
    SpawnRandom,

    // === TOOLS ===
    GravityGun,
    ForceGun,
    Explosion,
    JointTool,
    Delete,
    Freeze,
    Clone,
    Resize,
    Magnet,
    LaunchCannon,
    Painter,

    // === PRESETS ===
    SpawnChain,
    SpawnTower,
    SpawnPyramid,
    SpawnRagdoll,
    SpawnNewtonsCradle,
    SpawnWreckingBall,
    SpawnDominos,
    SpawnBridge,
    SpawnCatapult,
    SpawnCar,
    SpawnPendulumWall,
    SpawnBoxRain,
    SpawnSpiral,
    SpawnCannon,
    SpawnFerrisWheel,
    SpawnWindmill,

    // === SPECIAL ===
    GravityZone,
    ForceField,
    Portal,
    Trampoline,
    Conveyor,
    Fan,
}

impl HotbarItem {
    pub fn name(&self) -> &'static str {
        match self {
            // Shapes
            Self::SpawnBox => "Box",
            Self::SpawnSphere => "Sphere",
            Self::SpawnCapsule => "Capsule",
            Self::SpawnCylinder => "Cylinder",
            Self::SpawnCone => "Cone",
            Self::SpawnCompound => "Compound",
            Self::SpawnRandom => "Random",
            // Tools
            Self::GravityGun => "Grab",
            Self::ForceGun => "Force",
            Self::Explosion => "Explode",
            Self::JointTool => "Joint",
            Self::Delete => "Delete",
            Self::Freeze => "Freeze",
            Self::Clone => "Clone",
            Self::Resize => "Resize",
            Self::Magnet => "Magnet",
            Self::LaunchCannon => "Cannon",
            Self::Painter => "Paint",
            // Presets
            Self::SpawnChain => "Chain",
            Self::SpawnTower => "Tower",
            Self::SpawnPyramid => "Pyramid",
            Self::SpawnRagdoll => "Ragdoll",
            Self::SpawnNewtonsCradle => "Cradle",
            Self::SpawnWreckingBall => "Wrecker",
            Self::SpawnDominos => "Dominos",
            Self::SpawnBridge => "Bridge",
            Self::SpawnCatapult => "Catapult",
            Self::SpawnCar => "Car",
            Self::SpawnPendulumWall => "Pendulums",
            Self::SpawnBoxRain => "Box Rain",
            Self::SpawnSpiral => "Spiral",
            Self::SpawnCannon => "Launcher",
            Self::SpawnFerrisWheel => "Ferris",
            Self::SpawnWindmill => "Windmill",
            // Special
            Self::GravityZone => "Grav Zone",
            Self::ForceField => "Field",
            Self::Portal => "Portal",
            Self::Trampoline => "Trampoline",
            Self::Conveyor => "Conveyor",
            Self::Fan => "Fan",
        }
    }

    pub fn is_tool(&self) -> bool {
        matches!(
            self,
            Self::GravityGun
                | Self::ForceGun
                | Self::Explosion
                | Self::JointTool
                | Self::Delete
                | Self::Freeze
                | Self::Clone
                | Self::Resize
                | Self::Magnet
                | Self::LaunchCannon
                | Self::Painter
        )
    }

    pub fn is_spawn(&self) -> bool {
        matches!(
            self,
            Self::SpawnBox
                | Self::SpawnSphere
                | Self::SpawnCapsule
                | Self::SpawnCylinder
                | Self::SpawnCone
                | Self::SpawnCompound
                | Self::SpawnRandom
        )
    }

    pub fn is_preset(&self) -> bool {
        matches!(
            self,
            Self::SpawnChain
                | Self::SpawnTower
                | Self::SpawnPyramid
                | Self::SpawnRagdoll
                | Self::SpawnNewtonsCradle
                | Self::SpawnWreckingBall
                | Self::SpawnDominos
                | Self::SpawnBridge
                | Self::SpawnCatapult
                | Self::SpawnCar
                | Self::SpawnPendulumWall
                | Self::SpawnBoxRain
                | Self::SpawnSpiral
                | Self::SpawnCannon
                | Self::SpawnFerrisWheel
                | Self::SpawnWindmill
        )
    }

    pub fn category(&self) -> &'static str {
        if self.is_spawn() {
            "Shapes"
        } else if self.is_tool() {
            "Tools"
        } else if self.is_preset() {
            "Presets"
        } else {
            "Special"
        }
    }

    /// Returns true if this is a spawnable item (shape, preset, or special)
    pub fn is_spawnable(&self) -> bool {
        self.is_spawn() || self.is_preset() || self.is_special()
    }

    pub fn is_special(&self) -> bool {
        matches!(
            self,
            Self::GravityZone
                | Self::ForceField
                | Self::Portal
                | Self::Trampoline
                | Self::Conveyor
                | Self::Fan
        )
    }

    /// Returns a shorter display name for compact UI
    pub fn short_name(&self) -> &'static str {
        match self {
            // Shapes - keep short
            Self::SpawnBox => "Box",
            Self::SpawnSphere => "Sph",
            Self::SpawnCapsule => "Cap",
            Self::SpawnCylinder => "Cyl",
            Self::SpawnCone => "Cone",
            Self::SpawnCompound => "Comp",
            Self::SpawnRandom => "Rand",
            // Tools
            Self::GravityGun => "Grab",
            Self::ForceGun => "Force",
            Self::Explosion => "Boom",
            Self::JointTool => "Joint",
            Self::Delete => "Del",
            Self::Freeze => "Frz",
            Self::Clone => "Clone",
            Self::Resize => "Size",
            Self::Magnet => "Mag",
            Self::LaunchCannon => "Fire",
            Self::Painter => "Paint",
            // Presets
            Self::SpawnChain => "Chain",
            Self::SpawnTower => "Tower",
            Self::SpawnPyramid => "Pyra",
            Self::SpawnRagdoll => "Rag",
            Self::SpawnNewtonsCradle => "Cradl",
            Self::SpawnWreckingBall => "Wreck",
            Self::SpawnDominos => "Domns",
            Self::SpawnBridge => "Brdge",
            Self::SpawnCatapult => "Ctplt",
            Self::SpawnCar => "Car",
            Self::SpawnPendulumWall => "Pend",
            Self::SpawnBoxRain => "Rain",
            Self::SpawnSpiral => "Spirl",
            Self::SpawnCannon => "Lnchr",
            Self::SpawnFerrisWheel => "Ferrs",
            Self::SpawnWindmill => "Mill",
            // Special
            Self::GravityZone => "GZone",
            Self::ForceField => "Field",
            Self::Portal => "Port",
            Self::Trampoline => "Tramp",
            Self::Conveyor => "Conv",
            Self::Fan => "Fan",
        }
    }
}

/// The hotbar containing available items
#[derive(Resource)]
pub struct Hotbar {
    pub items: Vec<HotbarItem>,
    pub current_page: usize,
    pub items_per_page: usize,
}

impl Default for Hotbar {
    fn default() -> Self {
        Self {
            items: vec![
                // Page 1: Basic shapes + core tools
                HotbarItem::SpawnBox,
                HotbarItem::SpawnSphere,
                HotbarItem::SpawnCapsule,
                HotbarItem::GravityGun,
                HotbarItem::ForceGun,
                HotbarItem::Explosion,
                HotbarItem::JointTool,
                HotbarItem::Delete,
                HotbarItem::Freeze,
                // Page 2: More tools
                HotbarItem::Clone,
                HotbarItem::Resize,
                HotbarItem::Magnet,
                HotbarItem::LaunchCannon,
                HotbarItem::Painter,
                HotbarItem::SpawnCylinder,
                HotbarItem::SpawnCone,
                HotbarItem::SpawnCompound,
                HotbarItem::SpawnRandom,
                // Page 3: Presets
                HotbarItem::SpawnChain,
                HotbarItem::SpawnTower,
                HotbarItem::SpawnPyramid,
                HotbarItem::SpawnRagdoll,
                HotbarItem::SpawnNewtonsCradle,
                HotbarItem::SpawnWreckingBall,
                HotbarItem::SpawnDominos,
                HotbarItem::SpawnBridge,
                HotbarItem::SpawnCatapult,
                // Page 4: More presets
                HotbarItem::SpawnCar,
                HotbarItem::SpawnPendulumWall,
                HotbarItem::SpawnBoxRain,
                HotbarItem::SpawnSpiral,
                HotbarItem::SpawnCannon,
                HotbarItem::SpawnFerrisWheel,
                HotbarItem::SpawnWindmill,
                HotbarItem::GravityZone,
                HotbarItem::ForceField,
                // Page 5: Special
                HotbarItem::Portal,
                HotbarItem::Trampoline,
                HotbarItem::Conveyor,
                HotbarItem::Fan,
            ],
            current_page: 0,
            items_per_page: 9,
        }
    }
}

impl Hotbar {
    pub fn current_page_items(&self) -> &[HotbarItem] {
        let start = self.current_page * self.items_per_page;
        let end = (start + self.items_per_page).min(self.items.len());
        &self.items[start..end]
    }

    pub fn total_pages(&self) -> usize {
        (self.items.len() + self.items_per_page - 1) / self.items_per_page
    }

    pub fn page_count(&self) -> usize {
        self.total_pages()
    }

    pub fn next_page(&mut self) {
        self.current_page = (self.current_page + 1) % self.page_count();
    }

    pub fn prev_page(&mut self) {
        if self.current_page == 0 {
            self.current_page = self.page_count() - 1;
        } else {
            self.current_page -= 1;
        }
    }

    pub fn get_absolute_index(&self, slot: usize) -> Option<usize> {
        let idx = self.current_page * self.items_per_page + slot;
        if idx < self.items.len() {
            Some(idx)
        } else {
            None
        }
    }

    pub fn get_item(&self, slot: usize) -> Option<&HotbarItem> {
        self.get_absolute_index(slot).and_then(|i| self.items.get(i))
    }
}

/// State for the gravity gun tool
#[derive(Resource, Default)]
pub struct GrabState {
    pub holding: Option<RigidBodyHandle>,
    pub holding_collider: Option<ColliderHandle>,
    pub distance: f32,
    pub grab_offset: bevy::math::Vec3,
}

/// State for joint creation tool
#[derive(Resource, Default)]
pub struct JointToolState {
    pub first_body: Option<(RigidBodyHandle, bevy::math::Vec3)>,
    pub joint_type: JointType,
}

/// Available joint types for the joint tool
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum JointType {
    #[default]
    Ball,
    Hinge,
    Fixed,
    Distance,
    Prismatic,
    Spring,
    Rope,
}

impl JointType {
    pub fn name(&self) -> &'static str {
        match self {
            Self::Ball => "Ball",
            Self::Hinge => "Hinge",
            Self::Fixed => "Fixed",
            Self::Distance => "Distance",
            Self::Prismatic => "Slider",
            Self::Spring => "Spring",
            Self::Rope => "Rope",
        }
    }

    pub fn cycle_next(&self) -> Self {
        match self {
            Self::Ball => Self::Hinge,
            Self::Hinge => Self::Fixed,
            Self::Fixed => Self::Distance,
            Self::Distance => Self::Prismatic,
            Self::Prismatic => Self::Spring,
            Self::Spring => Self::Rope,
            Self::Rope => Self::Ball,
        }
    }
}

/// State for resize tool
#[derive(Resource, Default)]
pub struct ResizeState {
    pub target: Option<RigidBodyHandle>,
    pub original_scale: f32,
}

/// State for magnet tool
#[derive(Resource)]
pub struct MagnetState {
    pub active: bool,
    pub strength: f32,
    pub radius: f32,
    pub attract: bool, // true = attract, false = repel
}

impl Default for MagnetState {
    fn default() -> Self {
        Self {
            active: false,
            strength: 500.0,
            radius: 10.0,
            attract: true,
        }
    }
}

/// State for launch cannon
#[derive(Resource)]
pub struct CannonState {
    pub power: f32,
    pub projectile_type: ProjectileType,
    pub auto_fire: bool,
    pub fire_rate: f32,
    pub last_fire: f32,
}

impl Default for CannonState {
    fn default() -> Self {
        Self {
            power: 50.0,
            projectile_type: ProjectileType::Ball,
            auto_fire: false,
            fire_rate: 0.2,
            last_fire: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ProjectileType {
    #[default]
    Ball,
    Box,
    Capsule,
    Cluster, // Multiple small balls
    Heavy,   // Slow but massive
    Bouncy,  // High restitution
}

impl ProjectileType {
    pub fn cycle_next(&self) -> Self {
        match self {
            Self::Ball => Self::Box,
            Self::Box => Self::Capsule,
            Self::Capsule => Self::Cluster,
            Self::Cluster => Self::Heavy,
            Self::Heavy => Self::Bouncy,
            Self::Bouncy => Self::Ball,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Ball => "Ball",
            Self::Box => "Box",
            Self::Capsule => "Capsule",
            Self::Cluster => "Cluster",
            Self::Heavy => "Heavy",
            Self::Bouncy => "Bouncy",
        }
    }
}

/// Painter tool state
#[derive(Resource)]
pub struct PainterState {
    pub color_index: usize,
}

impl Default for PainterState {
    fn default() -> Self {
        Self { color_index: 0 }
    }
}

pub const PAINT_COLORS: [Color; 12] = [
    Color::srgb(1.0, 0.2, 0.2),   // Red
    Color::srgb(1.0, 0.5, 0.0),   // Orange
    Color::srgb(1.0, 1.0, 0.2),   // Yellow
    Color::srgb(0.2, 1.0, 0.2),   // Green
    Color::srgb(0.2, 1.0, 1.0),   // Cyan
    Color::srgb(0.2, 0.2, 1.0),   // Blue
    Color::srgb(0.8, 0.2, 1.0),   // Purple
    Color::srgb(1.0, 0.4, 0.7),   // Pink
    Color::srgb(1.0, 1.0, 1.0),   // White
    Color::srgb(0.3, 0.3, 0.3),   // Dark Gray
    Color::srgb(0.6, 0.4, 0.2),   // Brown
    Color::srgb(0.0, 0.0, 0.0),   // Black
];

/// Debug visualization toggles
#[derive(Resource)]
pub struct DebugVisuals {
    pub show_contacts: bool,
    pub show_joints: bool,
    pub show_velocities: bool,
    pub show_aabbs: bool,
    pub show_forces: bool,
    pub show_mass_centers: bool,
    pub show_sleeping: bool,
}

impl Default for DebugVisuals {
    fn default() -> Self {
        Self {
            show_contacts: false,
            show_joints: true,
            show_velocities: false,
            show_aabbs: false,
            show_forces: false,
            show_mass_centers: false,
            show_sleeping: true,
        }
    }
}

/// Mapping from Nova handles to Bevy entities
#[derive(Resource, Default)]
pub struct HandleToEntity {
    pub bodies: bevy::utils::HashMap<RigidBodyHandle, Entity>,
    pub colliders: bevy::utils::HashMap<ColliderHandle, Entity>,
}

/// Camera movement settings
#[derive(Resource)]
pub struct CameraSettings {
    pub speed: f32,
    pub sprint_multiplier: f32,
    pub sensitivity: f32,
    pub fov: f32,
}

impl Default for CameraSettings {
    fn default() -> Self {
        Self {
            speed: 10.0,
            sprint_multiplier: 2.5,
            sensitivity: 0.003,
            fov: 75.0,
        }
    }
}

/// Tool settings
#[derive(Resource)]
pub struct ToolSettings {
    pub grab_spring_strength: f32,
    pub force_gun_strength: f32,
    pub explosion_radius: f32,
    pub explosion_force: f32,
    pub freeze_all_mode: bool,
}

impl Default for ToolSettings {
    fn default() -> Self {
        Self {
            grab_spring_strength: 50.0,
            force_gun_strength: 500.0,
            explosion_radius: 5.0,
            explosion_force: 1000.0,
            freeze_all_mode: false,
        }
    }
}

/// Spawning options
#[derive(Resource)]
pub struct SpawnSettings {
    pub default_mass: f32,
    pub default_size: f32,
    pub random_colors: bool,
    pub random_sizes: bool,
    pub spawn_velocity: bool,
}

impl Default for SpawnSettings {
    fn default() -> Self {
        Self {
            default_mass: 1.0,
            default_size: 0.5,
            random_colors: true,
            random_sizes: false,
            spawn_velocity: false,
        }
    }
}

/// Gravity zone entity data
#[derive(Resource, Default)]
pub struct GravityZones {
    pub zones: Vec<GravityZoneData>,
}

#[derive(Clone)]
pub struct GravityZoneData {
    pub center: bevy::math::Vec3,
    pub radius: f32,
    pub gravity: bevy::math::Vec3,
    pub falloff: bool,
}

/// Force field entity data
#[derive(Resource, Default)]
pub struct ForceFields {
    pub fields: Vec<ForceFieldData>,
}

#[derive(Clone)]
pub struct ForceFieldData {
    pub center: bevy::math::Vec3,
    pub radius: f32,
    pub force: bevy::math::Vec3,
    pub field_type: ForceFieldType,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ForceFieldType {
    Directional,  // Push in one direction
    Radial,       // Push away from center
    Vortex,       // Spin around center
    Turbulence,   // Random forces
}

/// Statistics tracking
#[derive(Resource, Default)]
pub struct PlaygroundStats {
    pub total_spawned: u32,
    pub total_deleted: u32,
    pub total_explosions: u32,
    pub joints_created: u32,
    pub max_bodies: u32,
    pub session_time: f32,
}

/// Inventory panel state
#[derive(Resource, Default)]
pub struct InventoryState {
    pub open: bool,
    pub selected_category: usize,
}

pub const INVENTORY_CATEGORIES: [&str; 4] = ["Shapes", "Tools", "Presets", "Special"];

/// Slow-mo effect state
#[derive(Resource)]
pub struct SlowMoEffect {
    pub target_scale: f32,
    pub lerp_speed: f32,
}

impl Default for SlowMoEffect {
    fn default() -> Self {
        Self {
            target_scale: 1.0,
            lerp_speed: 5.0,
        }
    }
}

/// Trail effect settings
#[derive(Resource)]
pub struct TrailSettings {
    pub enabled: bool,
    pub min_velocity: f32,
    pub max_points: usize,
}

impl Default for TrailSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            min_velocity: 5.0,
            max_points: 20,
        }
    }
}

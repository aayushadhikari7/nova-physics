//! ECS components for the physics playground - MEGA EDITION!

use bevy::prelude::*;
use nova::prelude::{ColliderHandle, JointHandle, RigidBodyHandle};

/// Marker for any entity spawned by the user that should be removed on scene reset
/// This includes all spawned objects, zones, visual effects, presets, etc.
#[derive(Component)]
pub struct SpawnedObject;

/// Links a Bevy entity to a Nova rigid body
#[derive(Component)]
pub struct PhysicsBody {
    pub handle: RigidBodyHandle,
}

/// Links a Bevy entity to a Nova collider
#[derive(Component)]
pub struct PhysicsCollider {
    pub handle: ColliderHandle,
}

/// Links a Bevy entity to a Nova joint
#[derive(Component)]
pub struct PhysicsJoint {
    pub handle: JointHandle,
}

/// Marker for static bodies (floor, walls)
#[derive(Component)]
pub struct StaticBody;

/// Marker for dynamic bodies that can be interacted with
#[derive(Component)]
pub struct DynamicBody;

/// Marker for kinematic bodies
#[derive(Component)]
pub struct KinematicBody;

/// Marker for the ground plane
#[derive(Component)]
pub struct Ground;

/// Marker for arena walls
#[derive(Component)]
pub struct Wall;

/// Marker for frozen bodies (velocity set to zero)
#[derive(Component)]
pub struct FrozenBody;

/// Visual indicator for sleeping bodies
#[derive(Component)]
pub struct SleepingIndicator;

/// Trail effect for fast-moving objects
#[derive(Component)]
pub struct Trail {
    pub points: Vec<Vec3>,
    pub max_points: usize,
    pub color: Color,
}

impl Default for Trail {
    fn default() -> Self {
        Self {
            points: Vec::new(),
            max_points: 20,
            color: Color::srgba(1.0, 1.0, 1.0, 0.5),
        }
    }
}

/// Material type for spawned objects
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PhysicsMaterialType {
    #[default]
    Normal,
    Bouncy,
    Sticky,
    Slippery,
    Heavy,
    Light,
    Rubber,
    Metal,
    Ice,
    Wood,
}

impl PhysicsMaterialType {
    pub fn friction(&self) -> f32 {
        match self {
            Self::Normal => 0.5,
            Self::Bouncy => 0.3,
            Self::Sticky => 1.0,
            Self::Slippery | Self::Ice => 0.05,
            Self::Heavy | Self::Metal => 0.6,
            Self::Light => 0.4,
            Self::Rubber => 0.9,
            Self::Wood => 0.5,
        }
    }

    pub fn restitution(&self) -> f32 {
        match self {
            Self::Normal => 0.3,
            Self::Bouncy | Self::Rubber => 0.95,
            Self::Sticky => 0.0,
            Self::Slippery | Self::Ice => 0.5,
            Self::Heavy | Self::Metal => 0.1,
            Self::Light => 0.6,
            Self::Wood => 0.3,
        }
    }

    pub fn mass_multiplier(&self) -> f32 {
        match self {
            Self::Normal => 1.0,
            Self::Bouncy => 1.0,
            Self::Sticky => 1.0,
            Self::Slippery | Self::Ice => 1.0,
            Self::Heavy | Self::Metal => 10.0,
            Self::Light => 0.1,
            Self::Rubber => 0.8,
            Self::Wood => 0.6,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Self::Normal => "Normal",
            Self::Bouncy => "Bouncy",
            Self::Sticky => "Sticky",
            Self::Slippery => "Slippery",
            Self::Heavy => "Heavy",
            Self::Light => "Light",
            Self::Rubber => "Rubber",
            Self::Metal => "Metal",
            Self::Ice => "Ice",
            Self::Wood => "Wood",
        }
    }

    pub fn all() -> &'static [Self] {
        &[
            Self::Normal,
            Self::Bouncy,
            Self::Sticky,
            Self::Slippery,
            Self::Heavy,
            Self::Light,
            Self::Rubber,
            Self::Metal,
            Self::Ice,
            Self::Wood,
        ]
    }
}

/// Breakable object marker with threshold
#[derive(Component)]
pub struct Breakable {
    pub impulse_threshold: f32,
    pub pieces: u32,
}

impl Default for Breakable {
    fn default() -> Self {
        Self {
            impulse_threshold: 50.0,
            pieces: 4,
        }
    }
}

/// Gravity zone component
#[derive(Component)]
pub struct GravityZone {
    pub gravity: Vec3,
    pub radius: f32,
    pub falloff: bool,
}

/// Force field component
#[derive(Component)]
pub struct ForceField {
    pub force: Vec3,
    pub radius: f32,
    pub field_type: ForceFieldType,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ForceFieldType {
    #[default]
    Directional,
    Radial,
    Vortex,
    Turbulence,
}

/// Portal component (teleports objects)
#[derive(Component)]
pub struct Portal {
    pub destination: Vec3,
    pub preserve_velocity: bool,
}

/// Trampoline component (bounces objects)
#[derive(Component)]
pub struct Trampoline {
    pub bounce_multiplier: f32,
}

impl Default for Trampoline {
    fn default() -> Self {
        Self {
            bounce_multiplier: 2.0,
        }
    }
}

/// Conveyor belt component
#[derive(Component)]
pub struct Conveyor {
    pub direction: Vec3,
    pub speed: f32,
}

/// Fan component (applies wind force)
#[derive(Component)]
pub struct Fan {
    pub direction: Vec3,
    pub strength: f32,
    pub range: f32,
}

/// Explosive component (explodes on impact)
#[derive(Component)]
pub struct Explosive {
    pub radius: f32,
    pub force: f32,
    pub triggered: bool,
}

impl Default for Explosive {
    fn default() -> Self {
        Self {
            radius: 5.0,
            force: 500.0,
            triggered: false,
        }
    }
}

/// Glowing effect for special objects
#[derive(Component)]
pub struct Glowing {
    pub color: Color,
    pub intensity: f32,
}

/// Spinner component (constantly rotates)
#[derive(Component)]
pub struct Spinner {
    pub axis: Vec3,
    pub speed: f32,
}

/// Magnet component (attracts/repels nearby objects)
#[derive(Component)]
pub struct MagnetObject {
    pub strength: f32,
    pub radius: f32,
    pub attract: bool,
}

/// Projectile marker
#[derive(Component)]
pub struct Projectile {
    pub lifetime: f32,
    pub spawn_time: f32,
}

/// Clone source marker
#[derive(Component)]
pub struct CloneSource;

/// Highlighted object (when hovered)
#[derive(Component)]
pub struct Highlighted;

/// Selected object (for multi-select operations)
#[derive(Component)]
pub struct Selected;

/// Object scale for resize tool
#[derive(Component)]
pub struct ObjectScale {
    pub original: Vec3,
    pub current: Vec3,
}

impl Default for ObjectScale {
    fn default() -> Self {
        Self {
            original: Vec3::ONE,
            current: Vec3::ONE,
        }
    }
}

/// Velocity display for debug
#[derive(Component)]
pub struct VelocityArrow;

/// Object info display
#[derive(Component)]
pub struct ObjectInfo {
    pub show_velocity: bool,
    pub show_mass: bool,
    pub show_position: bool,
}

/// Auto-despawn after time
#[derive(Component)]
pub struct AutoDespawn {
    pub timer: f32,
}

/// Chain link marker
#[derive(Component)]
pub struct ChainLink {
    pub index: usize,
}

/// Ragdoll part marker
#[derive(Component)]
pub struct RagdollPart {
    pub part_type: RagdollPartType,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RagdollPartType {
    Head,
    Torso,
    UpperArm,
    LowerArm,
    UpperLeg,
    LowerLeg,
    Hand,
    Foot,
}

/// Vehicle marker
#[derive(Component)]
pub struct Vehicle {
    pub vehicle_type: VehicleType,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VehicleType {
    Car,
    Tank,
    Helicopter,
    Boat,
}

/// Wheel marker for vehicles
#[derive(Component)]
pub struct Wheel {
    pub powered: bool,
    pub steering: bool,
}

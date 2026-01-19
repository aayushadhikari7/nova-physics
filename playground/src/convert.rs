//! Nova <-> Bevy math conversion utilities

use bevy::prelude::*;
use nova::prelude::{Quat as NovaQuat, Vec3 as NovaVec3};

/// Convert Nova Vec3 to Bevy Vec3
#[inline]
pub fn to_bevy_vec3(v: NovaVec3) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}

/// Convert Bevy Vec3 to Nova Vec3
#[inline]
pub fn to_nova_vec3(v: Vec3) -> NovaVec3 {
    NovaVec3::new(v.x, v.y, v.z)
}

/// Convert Nova Quat to Bevy Quat
#[inline]
pub fn to_bevy_quat(q: NovaQuat) -> Quat {
    Quat::from_xyzw(q.x, q.y, q.z, q.w)
}

/// Convert Bevy Quat to Nova Quat
#[inline]
pub fn to_nova_quat(q: Quat) -> NovaQuat {
    NovaQuat::from_xyzw(q.x, q.y, q.z, q.w)
}

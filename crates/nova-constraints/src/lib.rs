//! Nova Constraints - Joints and constraints for the Nova Physics Engine
//!
//! Provides various joint types for connecting rigid bodies.

mod ball_joint;
mod distance_joint;
mod fixed_joint;
mod hinge_joint;
mod joint;
mod prismatic_joint;
mod solver;

pub use ball_joint::BallJoint;
pub use distance_joint::DistanceJoint;
pub use fixed_joint::FixedJoint;
pub use hinge_joint::HingeJoint;
pub use joint::{Joint, JointData, JointLimits, JointMotor, MotorModel};
pub use prismatic_joint::PrismaticJoint;
pub use solver::JointSolver;

// Re-export core types
pub use nova_core::{JointHandle, RigidBodyHandle};
pub use nova_math::{Isometry, Mat3, Quat, Vec3};

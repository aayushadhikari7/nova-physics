//! Nova Pipeline - Physics simulation orchestration
//!
//! Manages the simulation loop and parallel execution.

mod pipeline;
mod step;

pub use pipeline::{PhysicsPipeline, PipelineConfig};
pub use step::SimulationStep;

// Re-export core types
pub use nova_core::{ColliderHandle, JointHandle, RigidBodyHandle};
pub use nova_math::Vec3;

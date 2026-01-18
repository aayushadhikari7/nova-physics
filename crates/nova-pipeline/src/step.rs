//! Simulation step tracking

/// Represents a single simulation step
#[derive(Debug, Clone, Copy)]
pub struct SimulationStep {
    /// Step number (monotonically increasing)
    pub number: u64,
    /// Time delta for this step
    pub dt: f32,
    /// Accumulated time
    pub time: f64,
    /// Number of substeps used
    pub substeps: u32,
}

impl SimulationStep {
    pub fn new(number: u64, dt: f32, time: f64, substeps: u32) -> Self {
        Self {
            number,
            dt,
            time,
            substeps,
        }
    }
}

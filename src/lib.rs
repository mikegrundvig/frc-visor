//! Visor: AprilTag vision for an FRC robot's coprocessor. Each camera has a capture thread
//! ([`capture`]) and a detect thread ([`pipeline`]) joined by a one-frame slot ([`latest`]); frames
//! go to the robot as [`wire`] datagrams stamped with robot time ([`clock`]).

pub mod capture;
pub mod clock;
pub mod config;
pub mod image;
pub mod latest;
pub mod napa;
pub mod pipeline;
pub mod wire;

//! Visor: AprilTag vision for an FRC robot's coprocessor. Each camera has a capture thread
//! ([`capture`]) and a detect thread ([`pipeline`]) joined by a one-frame slot ([`latest`]); frames
//! go to the robot as [`wire`] datagrams stamped with robot time ([`clock`]), each with the
//! exposure and gain it was taken at ([`settings`]) and how bright it is ([`brightness`]).

pub mod brightness;
pub mod cameras;
pub mod capture;
pub mod clock;
pub mod config;
pub mod image;
pub mod latest;
pub mod lens;
pub mod napa;
pub mod pipeline;
pub mod pose;
pub mod settings;
pub mod wire;

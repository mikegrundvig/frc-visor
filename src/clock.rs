//! Robot time for a frame.
//!
//! The robot program answers NTP requests with its own clock plus [`EPOCH_NANOS`], and chrony keeps
//! this computer's wall clock on that. So robot time is wall time minus the epoch. A frame's capture
//! time comes from the monotonic clock (the kernel stamps frames with it), so each frame is moved
//! onto the wall clock with the two clocks' current difference, read here.

/// What the robot adds to its clock when it answers NTP: 2026-01-01T00:00:00Z. Any fixed value
/// works, since both ends agree on it; a recent date keeps the Pi's logs and chrony unsurprised.
pub const EPOCH_NANOS: i64 = 1_767_225_600 * 1_000_000_000;

/// The computer's clocks, behind a trait so tests can set them.
pub trait Clocks {
    /// `CLOCK_MONOTONIC`, the clock V4L2 stamps frames with.
    fn monotonic_nanos(&self) -> i64;
    /// `CLOCK_REALTIME`, which chrony keeps at robot time plus the epoch.
    fn wall_nanos(&self) -> i64;
    /// Whether chrony says the wall clock is synced.
    fn synced(&self) -> bool;
}

/// What to add to a monotonic time to get robot time, as of now.
///
/// The wall clock is read between two monotonic reads, and paired with their midpoint, so the
/// time between reads can't bias it by more than half the gap: tens of nanoseconds.
pub fn monotonic_to_robot(clocks: &impl Clocks) -> i64 {
    let before = clocks.monotonic_nanos();
    let wall = clocks.wall_nanos();
    let after = clocks.monotonic_nanos();
    wall - EPOCH_NANOS - (before + (after - before) / 2)
}

/// The real clocks.
pub struct SystemClocks;

impl Clocks for SystemClocks {
    fn monotonic_nanos(&self) -> i64 {
        read_clock(libc::CLOCK_MONOTONIC)
    }

    fn wall_nanos(&self) -> i64 {
        read_clock(libc::CLOCK_REALTIME)
    }

    fn synced(&self) -> bool {
        // chrony keeps the kernel's sync status: TIME_ERROR, or STA_UNSYNC set, means not synced.
        // A mode of 0 only reads.
        let mut status: libc::timex = unsafe { std::mem::zeroed() };
        let state = unsafe { libc::ntp_adjtime(&mut status) };
        state != libc::TIME_ERROR && status.status & libc::STA_UNSYNC == 0
    }
}

fn read_clock(clock: libc::clockid_t) -> i64 {
    let mut time = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // Can't fail for these two clocks on Linux.
    unsafe { libc::clock_gettime(clock, &mut time) };
    time.tv_sec * 1_000_000_000 + time.tv_nsec
}

#[cfg(test)]
pub mod tests {
    use super::*;
    use std::cell::Cell;

    /// Clocks a test sets. Each monotonic read advances the monotonic clock by `step`.
    pub struct FakeClocks {
        pub monotonic: Cell<i64>,
        pub step: i64,
        pub wall: i64,
        pub synced: bool,
    }

    impl Clocks for FakeClocks {
        fn monotonic_nanos(&self) -> i64 {
            let now = self.monotonic.get();
            self.monotonic.set(now + self.step);
            now
        }
        fn wall_nanos(&self) -> i64 {
            self.wall
        }
        fn synced(&self) -> bool {
            self.synced
        }
    }

    #[test]
    fn robot_time_is_the_wall_clock_less_the_epoch() {
        // (monotonic, step between reads, wall clock, robot time at the midpoint read)
        let cases = [
            (
                5_000_000_000,
                0,
                EPOCH_NANOS + 42_000_000_000,
                42_000_000_000,
            ),
            // The wall clock was read halfway between two monotonic reads 100 ns apart.
            (
                5_000_000_000,
                100,
                EPOCH_NANOS + 42_000_000_000,
                42_000_000_000,
            ),
            // An unsynced clock still converts; the frame's flag says not to trust it.
            (1_000, 10, 0, -EPOCH_NANOS),
        ];
        for (monotonic, step, wall, robot_at_midpoint) in cases {
            let clocks = FakeClocks {
                monotonic: Cell::new(monotonic),
                step,
                wall,
                synced: true,
            };
            let offset = monotonic_to_robot(&clocks);
            let midpoint = monotonic + step / 2;
            assert_eq!(midpoint + offset, robot_at_midpoint);
        }
    }

    #[test]
    fn the_real_clocks_read() {
        let clocks = SystemClocks;
        let a = clocks.monotonic_nanos();
        let b = clocks.monotonic_nanos();
        assert!(b >= a);
        // Later than 2026: this machine's wall clock is a real date.
        assert!(clocks.wall_nanos() > EPOCH_NANOS);
        // Synced or not depends on the machine; it only has to answer.
        let _ = clocks.synced();
    }
}

//! Each camera's live view, over HTTP on port [`PORT`]: its frames made smaller, with each tag
//! outlined and a line of the camera's numbers, as an MJPEG stream a browser shows in an `<img>`.
//! Pitwall's camera card shows it.
//!
//! The view is made on demand and never at detection's expense. With no one watching, the detect
//! thread checks one flag a frame and nothing else runs. While someone watches, the view's own
//! thread asks for a frame at the view's rate; the detect thread hands over the next one after
//! sending it to the robot, without waiting ([`feed`]); and the view's thread makes the picture
//! ([`picture`]) at a lower priority ([`view`]). The server ([`server`]) sends each watcher the
//! newest picture whenever its connection is ready, so a slow watcher skips pictures.

pub mod feed;
pub mod picture;
pub mod server;
pub mod view;

/// The live view's port: inside FRC's 5800 to 5810 for teams, beside Spotter's 5808.
pub const PORT: u16 = 5809;

/// How much lower than detection's the view's threads' priority is: nice 10.
const NICENESS: libc::c_int = 10;

/// Runs the calling thread at nice 10: Linux gives each thread its own niceness. Says so if it
/// can't, and runs on at detection's.
fn lower_priority(what: &str) {
    // SAFETY: setpriority reads nothing of ours, and gettid can't fail.
    let result =
        unsafe { libc::setpriority(libc::PRIO_PROCESS, libc::gettid() as libc::id_t, NICENESS) };
    if result != 0 {
        log::warn!(
            "{what} runs at detection's priority: {}",
            std::io::Error::last_os_error()
        );
    }
}

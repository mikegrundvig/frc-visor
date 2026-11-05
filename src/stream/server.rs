//! The live views over HTTP, on axum: `/` lists the cameras, each with its view, and
//! `/stream/<camera>` is one camera's view as an MJPEG stream (`multipart/x-mixed-replace`), which
//! a browser's `<img>` shows as it comes. It runs on a thread of its own, on a single-threaded
//! tokio runtime, at a lower priority than detection.
//!
//! A watcher is sent the newest picture whenever its connection is ready for one, so a slow
//! connection skips pictures rather than queueing them, and a stalled one is closed once a write
//! has waited [`Options::write_timeout`]. Watchers are capped ([`Options::most_watchers`]), since
//! each costs the board's network.

use std::convert::Infallible;
use std::fmt::Write as _;
use std::io;
use std::net::SocketAddr;
use std::pin::Pin;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::Duration;

use axum::Router;
use axum::body::{Body, Bytes};
use axum::extract::{Path, State};
use axum::http::{StatusCode, header};
use axum::response::{Html, IntoResponse, Response};
use axum::routing::get;
use log::{info, warn};
use percent_encoding::{AsciiSet, NON_ALPHANUMERIC, utf8_percent_encode};
use socket2::SockRef;
use tokio::net::{TcpListener, TcpStream};
use tokio_io_timeout::TimeoutStream;

use crate::stream::view::View;

/// The boundary between pictures in a stream.
const BOUNDARY: &str = "frame";

/// A camera's name as one part of an address: everything but letters, digits and `-_.~` escaped.
const SEGMENT: &AsciiSet = &NON_ALPHANUMERIC
    .remove(b'-')
    .remove(b'_')
    .remove(b'.')
    .remove(b'~');

/// What a watcher may cost.
#[derive(Debug, Clone, Copy)]
pub struct Options {
    /// How long a write to a watcher may wait before its connection is closed: it's stopped
    /// reading.
    pub write_timeout: Duration,
    /// The most watchers at once, across every camera; more are turned away.
    pub most_watchers: usize,
    /// The socket's send buffer for each watcher, in bytes: a few pictures, so a stalled watcher
    /// fills it, and is found, in seconds.
    pub send_buffer: usize,
}

impl Default for Options {
    fn default() -> Options {
        Options {
            write_timeout: Duration::from_secs(5),
            most_watchers: 8,
            send_buffer: 256 * 1024,
        }
    }
}

/// Serves the views on `listener` until `stop` is set, on this thread. Returns at once, saying why,
/// if the runtime can't start.
pub fn run(
    listener: std::net::TcpListener,
    views: Arc<Vec<View>>,
    options: Options,
    stop: &AtomicBool,
) {
    super::lower_priority("the live view's server");
    let runtime = match tokio::runtime::Builder::new_current_thread()
        .enable_io()
        .enable_time()
        .build()
    {
        Ok(runtime) => runtime,
        Err(error) => {
            warn!("the live view can't start: {error}");
            return;
        }
    };
    runtime.block_on(async {
        let served = serve(listener, views, options);
        let stopped = async {
            while !stop.load(Ordering::Relaxed) {
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
        };
        tokio::select! {
            result = served => if let Err(error) = result {
                warn!("the live view stopped: {error}");
            },
            () = stopped => {}
        }
    });
    // Dropping the runtime ends every watcher's connection.
}

async fn serve(
    listener: std::net::TcpListener,
    views: Arc<Vec<View>>,
    options: Options,
) -> io::Result<()> {
    listener.set_nonblocking(true)?;
    let listener = Watchers {
        listener: TcpListener::from_std(listener)?,
        options,
    };
    info!(
        "live view on http://{}/ (/stream/<camera>)",
        listener.listener.local_addr()?
    );
    let shared = Arc::new(Shared {
        views,
        watching: AtomicUsize::new(0),
        most: options.most_watchers,
    });
    let app = Router::new()
        .route("/", get(index))
        .route("/stream/{camera}", get(stream))
        .with_state(shared);
    axum::serve(listener, app).await
}

/// What every request reads.
struct Shared {
    views: Arc<Vec<View>>,
    watching: AtomicUsize,
    most: usize,
}

/// The page at `/`: each camera's view, by name.
async fn index(State(shared): State<Arc<Shared>>) -> Html<String> {
    let mut page = String::from(
        "<!doctype html>\n<html lang=\"en\"><head><meta charset=\"utf-8\">\
         <meta name=\"viewport\" content=\"width=device-width\"><title>Visor</title>\
         <style>body{font-family:sans-serif;background:#111;color:#eee;margin:1em}\
         img{max-width:100%;background:#000}</style></head><body>\n<h1>Visor's cameras</h1>\n",
    );
    for view in shared.views.iter() {
        let href = stream_path(&view.name);
        let name = escaped(&view.name);
        let _ = write!(
            page,
            "<h2><a href=\"{href}\">{name}</a></h2>\n<img src=\"{href}\" alt=\"{name}'s live view\">\n"
        );
    }
    page.push_str("</body></html>\n");
    Html(page)
}

/// A camera's stream's address on this server: `/stream/front-left`.
pub fn stream_path(camera: &str) -> String {
    format!("/stream/{}", utf8_percent_encode(camera, SEGMENT))
}

/// Text safe in HTML, in an element or an attribute.
fn escaped(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for c in text.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&#39;"),
            _ => out.push(c),
        }
    }
    out
}

/// A camera's stream: each new picture as a part, for as long as the watcher stays.
async fn stream(State(shared): State<Arc<Shared>>, Path(camera): Path<String>) -> Response {
    let Some(view) = shared.views.iter().find(|view| view.name == camera) else {
        let names: Vec<&str> = shared.views.iter().map(|v| v.name.as_str()).collect();
        let why = format!("no camera {camera:?}: Visor has {}\n", names.join(", "));
        return (StatusCode::NOT_FOUND, why).into_response();
    };
    let Some(seat) = Seat::take(&shared) else {
        let why = format!(
            "{} watchers already, the most Visor serves: close another view\n",
            shared.most
        );
        return (StatusCode::SERVICE_UNAVAILABLE, why).into_response();
    };
    let pictures = view.watch();
    let parts = futures_util::stream::unfold((pictures, seat), |(mut pictures, seat)| async move {
        loop {
            // An error: Visor is stopping.
            pictures.changed().await.ok()?;
            let picture = pictures.borrow_and_update().clone();
            if let Some(jpeg) = picture {
                return Some((Ok::<Bytes, Infallible>(part(&jpeg)), (pictures, seat)));
            }
        }
    });
    Response::builder()
        .header(
            header::CONTENT_TYPE,
            format!("multipart/x-mixed-replace; boundary={BOUNDARY}"),
        )
        .header(header::CACHE_CONTROL, "no-store")
        .body(Body::from_stream(parts))
        .expect("a fixed set of valid headers")
}

/// One picture as a part of the stream.
fn part(jpeg: &Bytes) -> Bytes {
    let head = format!(
        "--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
        jpeg.len()
    );
    let mut bytes = Vec::with_capacity(head.len() + jpeg.len() + 2);
    bytes.extend_from_slice(head.as_bytes());
    bytes.extend_from_slice(jpeg);
    bytes.extend_from_slice(b"\r\n");
    Bytes::from(bytes)
}

/// A watcher's place among the most allowed, given back when its stream ends.
struct Seat(Arc<Shared>);

impl Seat {
    fn take(shared: &Arc<Shared>) -> Option<Seat> {
        let taken = shared
            .watching
            .try_update(Ordering::AcqRel, Ordering::Acquire, |n| {
                (n < shared.most).then_some(n + 1)
            });
        taken.ok().map(|_| Seat(Arc::clone(shared)))
    }
}

impl Drop for Seat {
    fn drop(&mut self) {
        self.0.watching.fetch_sub(1, Ordering::AcqRel);
    }
}

/// The listener: each connection with a small send buffer, and a write timeout.
struct Watchers {
    listener: TcpListener,
    options: Options,
}

impl axum::serve::Listener for Watchers {
    type Io = Pin<Box<TimeoutStream<TcpStream>>>;
    type Addr = SocketAddr;

    async fn accept(&mut self) -> (Self::Io, SocketAddr) {
        loop {
            match self.listener.accept().await {
                Ok((connection, address)) => {
                    let _ = connection.set_nodelay(true);
                    let _ =
                        SockRef::from(&connection).set_send_buffer_size(self.options.send_buffer);
                    let mut timed = TimeoutStream::new(connection);
                    timed.set_write_timeout(Some(self.options.write_timeout));
                    return (Box::pin(timed), address);
                }
                // Out of file descriptors, say: wait, rather than spin.
                Err(error) => {
                    warn!("the live view can't accept a watcher: {error}");
                    tokio::time::sleep(Duration::from_millis(100)).await;
                }
            }
        }
    }

    fn local_addr(&self) -> io::Result<SocketAddr> {
        self.listener.local_addr()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_camera_s_name_is_escaped_in_its_address_and_on_the_page() {
        let cases = [
            ("front-left", "/stream/front-left"),
            ("Front Left", "/stream/Front%20Left"),
            ("a/b?c#d", "/stream/a%2Fb%3Fc%23d"),
            (
                "front \"left\" \\ 1",
                "/stream/front%20%22left%22%20%5C%201",
            ),
        ];
        for (name, path) in cases {
            assert_eq!(stream_path(name), path);
        }
        assert_eq!(
            escaped("<b>\"Tom's\" & co</b>"),
            "&lt;b&gt;&quot;Tom&#39;s&quot; &amp; co&lt;/b&gt;"
        );
    }

    #[test]
    fn a_part_is_a_picture_between_boundaries() {
        let part = part(&Bytes::from_static(b"JPEG"));
        assert_eq!(
            &part[..],
            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 4\r\n\r\nJPEG\r\n"
        );
    }
}

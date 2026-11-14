//! The live views over HTTP, on hyper and axum: `/` lists the cameras, each with its view,
//! `/stream/<camera>` is one camera's view as an MJPEG stream (`multipart/x-mixed-replace`), which
//! a browser's `<img>` shows as it comes, and `/frame/<camera>` is its next frame at full size, as
//! detection read it ([`frame`]). It runs on a thread of its own, on a single-threaded tokio
//! runtime, at a lower priority than detection.
//!
//! A watcher is sent the newest picture whenever its connection is ready for one, so a slow
//! connection skips pictures rather than queueing them, and a stalled one is closed once a write
//! has waited [`Options::write_timeout`]. A connection that sends no request within
//! [`Options::header_timeout`] is closed too, so idle sockets can't use up Visor's files.
//! Connections and watchers are capped ([`Options::most_connections`],
//! [`Options::most_watchers`]), since each costs the board's network and files, with a few kept for
//! the robot controller's address ([`Options::kept_for_controller`]), so a laptop's open pages
//! can't shut the robot out. A request must be addressed to the board, by an IP address or its own
//! name ([`addressed_here`]), so a web page whose name is pointed at the board (DNS rebinding) can't
//! read its pictures.

use std::convert::Infallible;
use std::fmt::Write as _;
use std::io;
use std::net::{IpAddr, SocketAddr};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::time::Duration;

use axum::body::{Body, Bytes};
use axum::extract::{ConnectInfo, Path, Request, State};
use axum::http::{HeaderValue, StatusCode, header};
use axum::middleware::{self, Next};
use axum::response::{Html, IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Extension, Router};
use hyper_util::rt::{TokioIo, TokioTimer};
use hyper_util::service::TowerToHyperService;
use log::{info, warn};
use percent_encoding::{AsciiSet, NON_ALPHANUMERIC, utf8_percent_encode};
use socket2::SockRef;
use tokio::net::{TcpListener, TcpStream};
use tokio_io_timeout::TimeoutStream;

use crate::settings::Tried;
use crate::stream::view::View;

/// The boundary between pictures in a stream.
const BOUNDARY: &str = "frame";

/// The least time between one camera's frames at full size. A ThriftyCam's is two megabytes, so
/// asked for as fast as they're answered they could take the robot's whole network.
const STILL_EVERY: Duration = Duration::from_millis(500);

/// How long a request for a frame waits for the camera's next one before saying there's none.
const STILL_WAIT: Duration = Duration::from_secs(2);

/// The header a frame's facts come in: what it was taken at and the tags Visor found in it.
pub const ABOUT_HEADER: &str = "x-visor-frame";

/// A camera's name as one part of an address: everything but letters, digits and `-_.~` escaped.
const SEGMENT: &AsciiSet = &NON_ALPHANUMERIC
    .remove(b'-')
    .remove(b'_')
    .remove(b'.')
    .remove(b'~');

/// What the live view's watchers may cost.
#[derive(Debug, Clone, Copy)]
pub struct Options {
    /// How long a write to a watcher may wait before its connection is closed: it's stopped
    /// reading.
    pub write_timeout: Duration,
    /// How long a connection may take to send a request's headers, the first or the next, before
    /// it's closed.
    pub header_timeout: Duration,
    /// The most connections at once from addresses other than the controller's; more are closed as
    /// they're accepted. Each holds a file, and Visor's are limited (`LimitNOFILE=`,
    /// deploy/visor.service).
    pub most_connections: usize,
    /// The most watchers at once, across every camera, from addresses other than the
    /// controller's; more are turned away.
    pub most_watchers: usize,
    /// How many more connections, and watchers, the robot controller's address may have past the
    /// most: room for a page and its stream whoever else watches.
    pub kept_for_controller: usize,
    /// The socket's send buffer for each watcher, in bytes: a few pictures, so a stalled watcher
    /// fills it, and is found, in seconds.
    pub send_buffer: usize,
    /// Whether an exposure and gain may be tried from here (`try_settings` in the configuration).
    pub try_settings: bool,
}

impl Default for Options {
    fn default() -> Options {
        Options {
            write_timeout: Duration::from_secs(5),
            header_timeout: Duration::from_secs(5),
            most_connections: 32,
            most_watchers: 8,
            kept_for_controller: 2,
            send_buffer: 256 * 1024,
            try_settings: false,
        }
    }
}

/// Serves the views on `listener` until `stop` is set, on this thread, keeping places for
/// `controller`, the robot controller's address. Returns at once, saying why, if the runtime
/// can't start.
pub fn run(
    listener: std::net::TcpListener,
    views: Arc<Vec<View>>,
    options: Options,
    controller: IpAddr,
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
        let served = serve(listener, views, options, controller);
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
    controller: IpAddr,
) -> io::Result<()> {
    listener.set_nonblocking(true)?;
    let listener = TcpListener::from_std(listener)?;
    info!(
        "live view on http://{}/ (/stream/<camera>, /frame/<camera>)",
        listener.local_addr()?
    );
    let shared = Arc::new(Shared {
        stills: views
            .iter()
            .map(|_| tokio::sync::Mutex::new(None))
            .collect(),
        try_settings: options.try_settings,
        views,
        watchers: Places::new(
            options.most_watchers,
            options.kept_for_controller,
            controller,
        ),
    });
    let names = Arc::new(own_names());
    let app = Router::new()
        .route("/", get(index))
        .route("/stream/{camera}", get(stream))
        .route("/frame/{camera}", get(frame))
        .route("/settings/{camera}", post(settings))
        .with_state(shared)
        .layer(middleware::from_fn_with_state(names, only_addressed_here));
    let connections = Places::new(
        options.most_connections,
        options.kept_for_controller,
        controller,
    );
    let mut http = hyper::server::conn::http1::Builder::new();
    http.timer(TokioTimer::new())
        .header_read_timeout(options.header_timeout);
    loop {
        let (connection, address) = accept(&listener).await;
        // Past the most: closed at once, said nowhere, as a flood of them would flood the log.
        let Some(place) = connections.take(address.ip()) else {
            continue;
        };
        let io = TokioIo::new(Box::pin(watched(connection, &options)));
        let service = TowerToHyperService::new(app.clone().layer(Extension(ConnectInfo(address))));
        let served = http.serve_connection(io, service);
        tokio::spawn(async move {
            // A connection that ends badly (timed out, reset) is the watcher's business.
            let _ = served.await;
            drop(place);
        });
    }
}

/// The next connection. An error (out of files, say) is said and waited out, rather than spun on.
async fn accept(listener: &TcpListener) -> (TcpStream, SocketAddr) {
    loop {
        match listener.accept().await {
            Ok(accepted) => return accepted,
            Err(error) => {
                warn!("the live view can't accept a watcher: {error}");
                tokio::time::sleep(Duration::from_millis(100)).await;
            }
        }
    }
}

/// A connection with a small send buffer and a write timeout.
fn watched(connection: TcpStream, options: &Options) -> TimeoutStream<TcpStream> {
    let _ = connection.set_nodelay(true);
    let _ = SockRef::from(&connection).set_send_buffer_size(options.send_buffer);
    let mut timed = TimeoutStream::new(connection);
    timed.set_write_timeout(Some(options.write_timeout));
    timed
}

/// The board's own names, which a request may be addressed to besides an IP address: `localhost`,
/// its host name, and that name on the local network (`.local`).
fn own_names() -> Vec<String> {
    let mut names = vec!["localhost".to_string()];
    match std::fs::read_to_string("/proc/sys/kernel/hostname") {
        Ok(name) if !name.trim().is_empty() => {
            let name = name.trim().to_ascii_lowercase();
            names.push(format!("{name}.local"));
            names.push(name);
        }
        Ok(_) => {}
        Err(error) => warn!("the live view answers by address only: no host name ({error})"),
    }
    names
}

/// Whether a request's `Host` names this board: an IP address, or one of its own `names`. A
/// request with none (HTTP/1.0) is answered: a page whose name points at the board always sends
/// one.
pub fn addressed_here(host: Option<&HeaderValue>, names: &[String]) -> bool {
    let Some(host) = host else {
        return true;
    };
    let Ok(host) = host.to_str() else {
        return false;
    };
    let host = host.trim().to_ascii_lowercase();
    // An IPv6 address is in brackets, with the port after them.
    if let Some(rest) = host.strip_prefix('[') {
        let Some((address, _port)) = rest.split_once(']') else {
            return false;
        };
        return address.parse::<std::net::Ipv6Addr>().is_ok();
    }
    let name = host
        .split_once(':')
        .map_or(host.as_str(), |(name, _port)| name);
    name.parse::<std::net::Ipv4Addr>().is_ok() || names.iter().any(|own| own == name)
}

/// Answers only requests addressed to this board ([`addressed_here`]); others are told so.
async fn only_addressed_here(
    State(names): State<Arc<Vec<String>>>,
    request: Request,
    next: Next,
) -> Response {
    if !addressed_here(request.headers().get(header::HOST), &names) {
        let why = "Visor answers requests addressed to its board, by address or name\n";
        return (StatusCode::MISDIRECTED_REQUEST, why).into_response();
    }
    next.run(request).await
}

/// What every request reads.
struct Shared {
    views: Arc<Vec<View>>,
    watchers: Arc<Places>,
    /// For each view, when its last frame at full size was made: one request at a time holds it.
    stills: Vec<tokio::sync::Mutex<Option<tokio::time::Instant>>>,
    try_settings: bool,
}

/// The page at `/`: each camera's view, by name.
async fn index(State(shared): State<Arc<Shared>>) -> Html<String> {
    let mut page = String::from(
        "<!doctype html>\n<html lang=\"en\"><head><meta charset=\"utf-8\">\
         <meta name=\"viewport\" content=\"width=device-width\"><title>Visor</title>\
         <style>body{font-family:sans-serif;background:#111;color:#eee;margin:1em}\
         img{max-width:100%;background:#000}</style></head><body>\n<h1>Visor's cameras</h1>\n",
    );
    if shared.try_settings {
        page.push_str(TRY_SCRIPT);
    }
    for view in shared.views.iter() {
        let href = stream_path(&view.name);
        let full = frame_path(&view.name);
        let name = escaped(&view.name);
        let _ = write!(
            page,
            "<h2><a href=\"{href}\">{name}</a></h2>\n<img src=\"{href}\" alt=\"{name}'s live view\">\n\
             <p><a href=\"{full}\">one frame at full size, as detection read it (PGM)</a></p>\n"
        );
        if shared.try_settings {
            let to = settings_path(&view.name);
            let _ = writeln!(
                page,
                "<form onsubmit=\"return trySettings(this, '{to}')\">exposure, microseconds \
                 <input name=\"exposure\" size=\"7\" inputmode=\"numeric\"> gain \
                 <input name=\"gain\" size=\"4\" inputmode=\"numeric\"> <button>try</button> \
                 <output name=\"said\"></output></form>"
            );
        }
    }
    page.push_str("</body></html>\n");
    Html(page)
}

/// A camera's stream's address on this server: `/stream/front-left`.
pub fn stream_path(camera: &str) -> String {
    format!("/stream/{}", utf8_percent_encode(camera, SEGMENT))
}

/// The page's script where settings can be tried: posts what a camera's form holds, and shows the
/// answer beside it.
const TRY_SCRIPT: &str = "<script>function trySettings(form, to) {\n\
  const asked = [];\n\
  if (form.exposure.value) asked.push('exposure_us=' + encodeURIComponent(form.exposure.value));\n\
  if (form.gain.value) asked.push('gain=' + encodeURIComponent(form.gain.value));\n\
  fetch(to + '?' + asked.join('&'), {method: 'POST'})\n\
    .then(answer => answer.text()).then(text => { form.said.value = text; })\n\
    .catch(error => { form.said.value = String(error); });\n\
  return false;\n\
}</script>\n";

/// Where a camera's settings are tried on this server: `/settings/front-left`.
pub fn settings_path(camera: &str) -> String {
    format!("/settings/{}", utf8_percent_encode(camera, SEGMENT))
}

/// A camera's full-size frame's address on this server: `/frame/front-left`.
pub fn frame_path(camera: &str) -> String {
    format!("/frame/{}", utf8_percent_encode(camera, SEGMENT))
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
async fn stream(
    State(shared): State<Arc<Shared>>,
    ConnectInfo(from): ConnectInfo<SocketAddr>,
    Path(camera): Path<String>,
) -> Response {
    let Some(view) = shared.views.iter().find(|view| view.name == camera) else {
        return no_such_camera(&shared, &camera);
    };
    let Some(seat) = shared.watchers.take(from.ip()) else {
        return no_seat(&shared);
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

/// A camera's next frame at full size: the pixels detection read, as a binary PGM, with what it
/// was taken at and the tags found in it as JSON in the [`ABOUT_HEADER`] header. Made on the view's
/// thread from the frame detection hands over after sending it, as a picture is, so asking never
/// holds detection up. One request at a time for each camera, [`STILL_EVERY`] apart, and each takes
/// a watcher's place while it waits.
async fn frame(
    State(shared): State<Arc<Shared>>,
    ConnectInfo(from): ConnectInfo<SocketAddr>,
    Path(camera): Path<String>,
) -> Response {
    let Some(at) = shared.views.iter().position(|view| view.name == camera) else {
        return no_such_camera(&shared, &camera);
    };
    let Some(_seat) = shared.watchers.take(from.ip()) else {
        return no_seat(&shared);
    };
    let view = &shared.views[at];
    let mut last = shared.stills[at].lock().await;
    if let Some(last) = *last {
        tokio::time::sleep_until(last + STILL_EVERY).await;
    }
    let mut stills = view.ask_still();
    let made = tokio::time::timeout(STILL_WAIT, stills.changed()).await;
    *last = Some(tokio::time::Instant::now());
    let still = match made {
        Ok(Ok(())) => stills.borrow_and_update().clone(),
        // No frame in time, or Visor is stopping.
        _ => None,
    };
    view.forget_still();
    let Some(still) = still else {
        let why = format!(
            "no frame from {camera:?} in {} s: the camera is sending none\n",
            STILL_WAIT.as_secs()
        );
        return (StatusCode::SERVICE_UNAVAILABLE, why).into_response();
    };
    let mut response = Response::builder()
        .header(header::CONTENT_TYPE, "image/x-portable-graymap")
        .header(header::CACHE_CONTROL, "no-store")
        .body(Body::from(still.pgm.clone()))
        .expect("a fixed set of valid headers");
    // ASCII by its making; were it ever not, the frame goes without it.
    if let Ok(about) = HeaderValue::from_str(&still.about) {
        response.headers_mut().insert(ABOUT_HEADER, about);
    }
    response
}

/// Tries an exposure and gain on a running camera: `POST /settings/<camera>?exposure_us=1500&gain=2`,
/// either or both. Only where the configuration says `try_settings = true`, which a bench's does and
/// a robot's never does. The camera's capture thread sets them before its next frame, held to the
/// camera's own range, and every frame after says what's in effect: the view's line, a full-size
/// frame's facts, the frames sent to the robot. They last until the camera is opened again; what
/// lasts is the configuration. A page from another site can't ask ([`same_origin`]).
async fn settings(
    State(shared): State<Arc<Shared>>,
    Path(camera): Path<String>,
    request: Request,
) -> Response {
    let Some(view) = shared.views.iter().find(|view| view.name == camera) else {
        return no_such_camera(&shared, &camera);
    };
    if !shared.try_settings {
        let why = "settings can't be tried here: this Visor's configuration doesn't say \
                   try_settings = true. On a robot, a camera's settings are the robot program's\n";
        return (StatusCode::FORBIDDEN, why).into_response();
    }
    if !same_origin(request.headers()) {
        let why = "settings are tried from Visor's own page, not another site's\n";
        return (StatusCode::FORBIDDEN, why).into_response();
    }
    let tried = match tried(request.uri().query().unwrap_or("")) {
        Ok(tried) => tried,
        Err(why) => return (StatusCode::BAD_REQUEST, format!("{why}\n")).into_response(),
    };
    view.trial.ask(tried);
    let said = |name: &str, value: Option<String>| value.map(|v| format!("{name} {v}"));
    let asked: Vec<String> = [
        said("exposure_us", tried.exposure_us.map(|us| us.to_string())),
        said("gain", tried.gain.map(|gain| gain.to_string())),
    ]
    .into_iter()
    .flatten()
    .collect();
    let answer = format!(
        "{camera}: trying {}, until the camera is opened again. The view's line says what's in effect\n",
        asked.join(" and ")
    );
    (StatusCode::ACCEPTED, answer).into_response()
}

/// What a request's query asks to try: `exposure_us` (1 to 10,000,000, as the configuration's) and
/// `gain` (the camera's own units), either or both, and nothing else.
pub fn tried(query: &str) -> Result<Tried, String> {
    let mut tried = Tried::default();
    for pair in query.split('&').filter(|pair| !pair.is_empty()) {
        let (name, value) = pair.split_once('=').unwrap_or((pair, ""));
        match name {
            "exposure_us" => match value.parse::<u32>() {
                Ok(us) if (1..=10_000_000).contains(&us) => tried.exposure_us = Some(us),
                _ => {
                    return Err(format!(
                        "exposure_us is microseconds, 1 to 10000000, not {value:?}"
                    ));
                }
            },
            "gain" => match value.parse::<i64>() {
                Ok(gain) => tried.gain = Some(gain),
                Err(_) => return Err(format!("gain is a whole number, not {value:?}")),
            },
            other => {
                return Err(format!(
                    "{other:?} can't be tried: exposure_us and gain can"
                ));
            }
        }
    }
    if tried == Tried::default() {
        return Err("give exposure_us, gain, or both: ?exposure_us=1500&gain=2".to_string());
    }
    Ok(tried)
}

/// Whether a request comes from Visor's own page, or from no page at all (a command line): its
/// `Origin`, when it has one, names the host it was sent to. A page from another site that posts
/// here has its own.
pub fn same_origin(headers: &axum::http::HeaderMap) -> bool {
    let Some(origin) = headers.get(header::ORIGIN) else {
        return true;
    };
    let (Ok(origin), Some(Ok(host))) = (
        origin.to_str(),
        headers.get(header::HOST).map(HeaderValue::to_str),
    ) else {
        return false;
    };
    origin
        .split_once("://")
        .is_some_and(|(_, from)| from.eq_ignore_ascii_case(host))
}

/// The answer to a request for a camera Visor hasn't got: the ones it has.
fn no_such_camera(shared: &Shared, camera: &str) -> Response {
    let names: Vec<&str> = shared.views.iter().map(|v| v.name.as_str()).collect();
    let why = format!("no camera {camera:?}: Visor has {}\n", names.join(", "));
    (StatusCode::NOT_FOUND, why).into_response()
}

/// The answer when every watcher's place is taken.
fn no_seat(shared: &Shared) -> Response {
    let why = format!(
        "{} watchers already, the most Visor serves: close another view\n",
        shared.watchers.most
    );
    (StatusCode::SERVICE_UNAVAILABLE, why).into_response()
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

/// Places for connections or watchers: at most `most` taken by anyone, and `kept` more that only
/// the controller's address may take.
struct Places {
    taken: AtomicUsize,
    most: usize,
    kept: usize,
    controller: IpAddr,
}

impl Places {
    fn new(most: usize, kept: usize, controller: IpAddr) -> Arc<Places> {
        Arc::new(Places {
            taken: AtomicUsize::new(0),
            most,
            kept,
            controller: controller.to_canonical(),
        })
    }

    /// A place for a connection or watcher from `from`, or none if they're all taken.
    fn take(self: &Arc<Places>, from: IpAddr) -> Option<Place> {
        let most = if from.to_canonical() == self.controller {
            self.most + self.kept
        } else {
            self.most
        };
        let taken = self
            .taken
            .try_update(Ordering::AcqRel, Ordering::Acquire, |n| {
                (n < most).then_some(n + 1)
            });
        taken.ok().map(|_| Place(Arc::clone(self)))
    }
}

/// A place taken, given back when its connection or stream ends.
struct Place(Arc<Places>);

impl Drop for Place {
    fn drop(&mut self) {
        self.0.taken.fetch_sub(1, Ordering::AcqRel);
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
            assert_eq!(frame_path(name), path.replacen("/stream/", "/frame/", 1));
        }
        assert_eq!(
            escaped("<b>\"Tom's\" & co</b>"),
            "&lt;b&gt;&quot;Tom&#39;s&quot; &amp; co&lt;/b&gt;"
        );
    }

    #[test]
    fn a_request_is_addressed_here_by_an_ip_address_or_the_board_s_own_name() {
        let names = ["localhost".to_string(), "visor-front.local".to_string()];
        // (the request's Host, whether it's answered)
        let rows = [
            (None, true),
            (Some("10.12.34.11"), true),
            (Some("10.12.34.11:5809"), true),
            (Some("[fe80::1]:5809"), true),
            (Some("[::1]"), true),
            (Some("LOCALHOST:5809"), true),
            (Some("visor-front.local"), true),
            (Some("visor.example.com"), false),
            (Some("10.12.34.11.example.com"), false),
            (Some("[visor]:5809"), false),
            (Some("[::1"), false),
            (Some(""), false),
        ];
        for (host, answered) in rows {
            let value = host.map(|h| HeaderValue::from_str(h).unwrap());
            assert_eq!(addressed_here(value.as_ref(), &names), answered, "{host:?}");
        }
    }

    #[test]
    fn a_part_is_a_picture_between_boundaries() {
        let part = part(&Bytes::from_static(b"JPEG"));
        assert_eq!(
            &part[..],
            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 4\r\n\r\nJPEG\r\n"
        );
    }

    #[test]
    fn a_trial_asks_for_an_exposure_a_gain_or_both_and_nothing_else() {
        let both = Tried {
            exposure_us: Some(1500),
            gain: Some(2),
        };
        assert_eq!(tried("exposure_us=1500&gain=2"), Ok(both));
        assert_eq!(tried("gain=2&exposure_us=1500"), Ok(both));
        assert_eq!(tried("gain=-3").unwrap().gain, Some(-3));
        assert_eq!(tried("exposure_us=10000000").unwrap().gain, None);
        // (the query, part of why it's refused)
        let refused = [
            ("", "give exposure_us, gain, or both"),
            ("exposure_us=0", "1 to 10000000"),
            ("exposure_us=10000001", "1 to 10000000"),
            ("exposure_us=fast", "1 to 10000000"),
            ("gain=", "whole number"),
            ("gain=1.5", "whole number"),
            ("brightness=3", "\"brightness\" can't be tried"),
            ("exposure_us=1500&reset", "\"reset\" can't be tried"),
        ];
        for (query, why) in refused {
            let error = tried(query).unwrap_err();
            assert!(error.contains(why), "{query:?}: {error}");
        }
    }

    #[test]
    fn a_request_from_another_site_s_page_is_not_from_here() {
        // (the request's Origin, its Host, whether it's from here)
        let rows = [
            (None, Some("10.12.34.11:5809"), true),
            (
                Some("http://10.12.34.11:5809"),
                Some("10.12.34.11:5809"),
                true,
            ),
            (
                Some("http://VISOR-1.local:5809"),
                Some("visor-1.local:5809"),
                true,
            ),
            (Some("http://evil.example"), Some("10.12.34.11:5809"), false),
            (Some("http://10.12.34.11"), Some("10.12.34.11:5809"), false),
            (Some("null"), Some("10.12.34.11:5809"), false),
            (Some("http://10.12.34.11:5809"), None, false),
        ];
        for (origin, host, here) in rows {
            let mut headers = axum::http::HeaderMap::new();
            if let Some(origin) = origin {
                headers.insert(header::ORIGIN, HeaderValue::from_static(origin));
            }
            if let Some(host) = host {
                headers.insert(header::HOST, HeaderValue::from_static(host));
            }
            assert_eq!(same_origin(&headers), here, "{origin:?} to {host:?}");
        }
    }
}

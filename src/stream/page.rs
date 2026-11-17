//! The live view's page: a small application of three files, compiled into Visor, with no web
//! toolchain and nothing loaded from anywhere else, since a robot's network has no internet. The
//! page reads everything it shows from the status document (`GET /status`) and the cameras'
//! streams and corners.

use axum::http::header;
use axum::response::{IntoResponse, Response};

/// One of the page's files: its bytes and its type.
#[derive(Debug, Clone, Copy)]
pub struct File {
    pub body: &'static str,
    pub content_type: &'static str,
}

pub const INDEX: File = File {
    body: include_str!("page/index.html"),
    content_type: "text/html; charset=utf-8",
};

pub const STYLE: File = File {
    body: include_str!("page/page.css"),
    content_type: "text/css; charset=utf-8",
};

pub const SCRIPT: File = File {
    body: include_str!("page/page.js"),
    content_type: "text/javascript; charset=utf-8",
};

/// A file as an answer: its type, and not kept, so a new Visor's page is never mixed with an old
/// one's.
pub async fn file(file: File) -> Response {
    (
        [
            (header::CONTENT_TYPE, file.content_type),
            (header::CACHE_CONTROL, "no-store"),
        ],
        file.body,
    )
        .into_response()
}

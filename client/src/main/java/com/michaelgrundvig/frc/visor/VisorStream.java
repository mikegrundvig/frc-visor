package com.michaelgrundvig.frc.visor;

import java.net.URLEncoder;
import java.nio.charset.StandardCharsets;

/**
 * Visor's live view: each camera's frames, made smaller, with its tags outlined, as an MJPEG stream
 * a browser shows in an {@code <img>}. Visor serves it on its coprocessor, on {@link #PORT}, while
 * anyone watches (Visor's docs/live-view.md). The robot program only names the address: the video
 * goes from the coprocessor straight to the browser.
 */
public final class VisorStream {
  /** The live view's port on each coprocessor. */
  public static final int PORT = 5809;

  private VisorStream() {}

  /**
   * A camera's stream: {@code http://10.12.34.11:5809/stream/front%20left}.
   *
   * @param host the coprocessor's address, as its frames come from ({@link VisorFrame#sender})
   * @param camera the camera's name in Visor's configuration
   */
  public static String url(String host, String camera) {
    String server = host.contains(":") ? "[" + host + "]" : host;
    return "http://" + server + ":" + PORT + "/stream/" + segment(camera);
  }

  /** A name as one part of an address's path: everything but letters, digits and "-_.*" escaped. */
  static String segment(String name) {
    // Form encoding, but with a space as %20, as a path has it.
    return URLEncoder.encode(name, StandardCharsets.UTF_8).replace("+", "%20");
  }
}

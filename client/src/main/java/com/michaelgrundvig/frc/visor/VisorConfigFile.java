package com.michaelgrundvig.frc.visor;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.jspecify.annotations.Nullable;

/**
 * Visor's configuration file (deploy/visor.example.toml), written for one coprocessor: what a
 * robot's deploy puts beside the program. Each setting is refused here as Visor would refuse it
 * (Visor's src/config.rs), so a robot definition Visor can't run fails as the robot program builds
 * its configuration, not when the board checks it ({@code visor check}). The file always names the
 * port the robot receives frames on ({@link VisorReceiver#DEFAULT_PORT} unless {@link #port} says).
 *
 * <pre>{@code
 * VisorConfigFile file = VisorConfigFile.forTeam(1234);
 * file.camera("front-left", 0, "/dev/v4l/by-id/usb-cam-video-index0", 1600, 1304)
 *     .fps(60)
 *     .exposureMicros(1000)
 *     .gain(40)
 *     .stream(0.5, 10, 60)
 *     .lens(1600, 1304, 1229.7, 1229.9, 801.2, 650.4, -0.31, 0.12, 0.0004, -0.0002, -0.02);
 * String toml = file.toToml();
 * }</pre>
 */
public final class VisorConfigFile {
  /** The highest team number Visor takes: its address, 10.TE.AM.2, has room for no more. */
  static final int MOST_TEAM = 25_599;

  /** A camera's frame rate when its entry gives none: Visor's own. */
  static final int VISOR_FPS = 60;

  private final int team;
  private int port = VisorReceiver.DEFAULT_PORT;
  private double tagSizeMeters;
  private final List<Camera> cameras = new ArrayList<>();

  private VisorConfigFile(int team) {
    this.team = team;
  }

  /** Frames to the robot controller of this team, at 10.TE.AM.2. */
  public static VisorConfigFile forTeam(int team) {
    within("a team number", team, 1, MOST_TEAM);
    return new VisorConfigFile(team);
  }

  /** The UDP port the robot receives frames on: {@link VisorReceiver#DEFAULT_PORT} unless set. */
  public VisorConfigFile port(int port) {
    within("the robot's UDP port", port, 1, 65_535);
    this.port = port;
    return this;
  }

  /**
   * The side of the tags' black square, in metres, more than 0 and less than 10: the field's, so
   * every camera's poses are solved at the size of the tags it sees. Without one, Visor's own
   * default, an FRC tag's 6.5 inches, which is there for files written by hand.
   */
  public VisorConfigFile tagSizeMeters(double meters) {
    if (!(meters > 0 && meters < 10)) {
      throw new IllegalArgumentException(
          "a tag's size is its black square in metres, more than 0 and less than 10, not "
              + meters);
    }
    this.tagSizeMeters = meters;
    return this;
  }

  /**
   * Adds a camera, in the order it's written.
   *
   * @param name for Visor's logs
   * @param index sent in every frame: the camera's place in the robot's list of cameras
   * @param device its V4L2 device as {@code visor cameras} names it: a {@code /dev/v4l/by-id/} path
   *     for a camera with a USB serial; for one with none, its port's {@code /dev/v4l/by-path/}
   *     path
   * @param width the frame size it runs at
   * @param height the frame size it runs at
   */
  public Camera camera(String name, int index, String device, int width, int height) {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a camera needs a name");
    }
    within("a camera's index", index, 0, 255);
    within("a frame's width", width, 1, 65_535);
    within("a frame's height", height, 1, 65_535);
    for (Camera other : cameras) {
      if (other.name.equals(name)) {
        throw new IllegalArgumentException("two cameras are named \"" + name + "\"");
      }
      if (other.index == index) {
        throw new IllegalArgumentException("two cameras have index " + index);
      }
    }
    Camera camera = new Camera(name, index, device, width, height);
    cameras.add(camera);
    return camera;
  }

  /** Refuses {@code value} unless it's from {@code least} to {@code most}, naming {@code what}. */
  private static void within(String what, long value, long least, long most) {
    if (value < least || value > most) {
      throw new IllegalArgumentException(
          what + " is from " + least + " to " + most + ", not " + value);
    }
  }

  /**
   * One camera's entry. Unset settings are left out, so Visor's defaults apply: for the exposure
   * and gain, the camera's own.
   */
  public static final class Camera {
    /**
     * The controls {@link #exposureMicros} and {@link #gain} set, by every name V4L2's drivers have
     * given them, each with the setting to give instead: as a control too, either would undo the
     * setting, and its live tuning. Visor refuses them (its src/config.rs).
     */
    static final Map<String, String> SET_BY_EXPOSURE_AND_GAIN =
        Map.of(
            "auto_exposure", "exposureMicros(..)",
            "exposure_auto", "exposureMicros(..)",
            "exposure_time_absolute", "exposureMicros(..)",
            "exposure_absolute", "exposureMicros(..)",
            "gain", "gain(..)");

    /** The most a time offset can be in microseconds: Visor works in nanoseconds, in 64 bits. */
    static final long MOST_OFFSET_MICROS = Long.MAX_VALUE / 1000;

    private final String name;
    private final int index;
    private final String device;
    private final int width;
    private final int height;
    private int fps;
    private int minTagPixels;
    private long timeOffsetMicros;
    private long exposureMicros;
    private @Nullable Long gain;
    private double streamScale;
    private int streamFps;
    private int streamQuality;
    private final Map<String, Long> controls = new LinkedHashMap<>();
    private double[] lens = new double[0];

    private Camera(String name, int index, String device, int width, int height) {
      this.name = Objects.requireNonNull(name);
      this.index = index;
      this.device = Objects.requireNonNull(device);
      this.width = width;
      this.height = height;
    }

    /** Frames a second to ask the camera for: 1 to 1000, and no fewer than its live view's. */
    public Camera fps(int fps) {
      within("a camera's frame rate", fps, 1, 1000);
      if (streamFps > fps) {
        throw new IllegalArgumentException(
            "a camera's frame rate is at least its live view's, " + streamFps + ", not " + fps);
      }
      this.fps = fps;
      return this;
    }

    /** The smallest tag NAPA looks for, in pixels: 8 to 30000. */
    public Camera minTagPixels(int pixels) {
      within("the smallest tag", pixels, 8, 30_000);
      this.minTagPixels = pixels;
      return this;
    }

    /**
     * Added to every capture time: the spin test's result. Visor works in nanoseconds, so at most
     * about 292 years either way.
     */
    public Camera timeOffsetMicros(long micros) {
      within("a time offset in microseconds", micros, -MOST_OFFSET_MICROS, MOST_OFFSET_MICROS);
      this.timeOffsetMicros = micros;
      return this;
    }

    /**
     * A fixed exposure, in microseconds: Visor turns the camera's automatic exposure off and sets
     * this, every time it opens the camera. Without one, the camera's own automatic exposure.
     */
    public Camera exposureMicros(long micros) {
      within("an exposure in microseconds", micros, 1, 10_000_000);
      this.exposureMicros = micros;
      return this;
    }

    /**
     * The camera's {@code gain} control, in its own units ({@code visor cameras} lists its range).
     * Without one, the camera's default.
     */
    public Camera gain(long gain) {
      // Frames carry the gain as a 32-bit number, as V4L2 sets it.
      within("a gain", gain, 0, Integer.MAX_VALUE);
      this.gain = gain;
      return this;
    }

    /**
     * The camera's live view ({@link VisorStream}). Without one, Visor's own: half the camera's
     * size, 10 a second, quality 60.
     *
     * @param scale its size as a share of the camera's, across and down, 0.05 to 1: 0.5 is half
     * @param fps pictures a second while someone watches, at most the camera's
     * @param quality JPEG quality, 1 to 100
     */
    public Camera stream(double scale, int fps, int quality) {
      if (!(scale >= 0.05 && scale <= 1)) {
        throw new IllegalArgumentException(
            "a live view's scale is a share of the camera's size, 0.05 to 1, not " + scale);
      }
      within("a live view's rate", fps, 1, this.fps > 0 ? this.fps : VISOR_FPS);
      within("a JPEG quality", quality, 1, 100);
      this.streamScale = scale;
      this.streamFps = fps;
      this.streamQuality = quality;
      return this;
    }

    /**
     * A V4L2 control, by the name {@code v4l2-ctl --list-ctrls} prints. Visor sets them after the
     * exposure and gain, in the order given, which matters where one control depends on another.
     * Not the exposure or gain controls themselves: {@link #exposureMicros} and {@link #gain} set
     * those.
     */
    public Camera control(String control, long value) {
      if (control.isBlank()) {
        throw new IllegalArgumentException("a control needs a name");
      }
      String setting = SET_BY_EXPOSURE_AND_GAIN.get(control);
      if (setting != null) {
        throw new IllegalArgumentException(
            "control " + control + " is set by " + setting + ": give that instead");
      }
      controls.put(control, value);
      return this;
    }

    /**
     * The lens calibration ({@code tools/calibrate.py}): the frame size it was made at, which is
     * the camera's, positive focal lengths and the centre in pixels, and OpenCV's distortion
     * coefficients (0, 4, 5 or 8).
     */
    public Camera lens(
        int lensWidth,
        int lensHeight,
        double fx,
        double fy,
        double cx,
        double cy,
        double... distortion) {
      double[] values = new double[6 + distortion.length];
      values[0] = lensWidth;
      values[1] = lensHeight;
      values[2] = fx;
      values[3] = fy;
      values[4] = cx;
      values[5] = cy;
      System.arraycopy(distortion, 0, values, 6, distortion.length);
      for (double value : values) {
        if (!Double.isFinite(value)) {
          throw new IllegalArgumentException("a lens value is a number, not " + value);
        }
      }
      if (lensWidth != width || lensHeight != height) {
        throw new IllegalArgumentException(
            "a lens calibrated at "
                + lensWidth
                + "x"
                + lensHeight
                + " is for another size than the"
                + " camera's, "
                + width
                + "x"
                + height
                + ": calibrate at the size it runs");
      }
      if (!(fx > 0 && fy > 0)) {
        throw new IllegalArgumentException("focal lengths are positive, not " + fx + ", " + fy);
      }
      if (!List.of(0, 4, 5, 8).contains(distortion.length)) {
        throw new IllegalArgumentException(
            "OpenCV's distortion has 0, 4, 5 or 8 coefficients, not " + distortion.length);
      }
      this.lens = values;
      return this;
    }

    private void write(StringBuilder out) {
      out.append("\n[[camera]]\n");
      out.append("name = ").append(quoted(name)).append('\n');
      out.append("index = ").append(index).append('\n');
      out.append("device = ").append(quoted(device)).append('\n');
      out.append("width = ").append(width).append('\n');
      out.append("height = ").append(height).append('\n');
      if (fps > 0) {
        out.append("fps = ").append(fps).append('\n');
      }
      if (minTagPixels > 0) {
        out.append("min_tag_px = ").append(minTagPixels).append('\n');
      }
      if (timeOffsetMicros != 0) {
        out.append("time_offset_us = ").append(timeOffsetMicros).append('\n');
      }
      if (exposureMicros > 0) {
        out.append("exposure_us = ").append(exposureMicros).append('\n');
      }
      Long gain = this.gain;
      if (gain != null) {
        out.append("gain = ").append(gain).append('\n');
      }
      if (streamFps > 0) {
        out.append("stream_scale = ").append(number(streamScale)).append('\n');
        out.append("stream_fps = ").append(streamFps).append('\n');
        out.append("stream_quality = ").append(streamQuality).append('\n');
      }
      if (!controls.isEmpty()) {
        out.append("[camera.controls]\n");
        controls.forEach(
            (control, value) ->
                out.append(quoted(control)).append(" = ").append(value).append('\n'));
      }
      if (lens.length > 0) {
        out.append("[camera.lens]\n");
        out.append("width = ").append((long) lens[0]).append('\n');
        out.append("height = ").append((long) lens[1]).append('\n');
        String[] names = {"fx", "fy", "cx", "cy"};
        for (int i = 0; i < names.length; i++) {
          out.append(names[i]).append(" = ").append(number(lens[2 + i])).append('\n');
        }
        out.append("distortion = [");
        for (int i = 6; i < lens.length; i++) {
          out.append(i > 6 ? ", " : "").append(number(lens[i]));
        }
        out.append("]\n");
      }
    }
  }

  /** The file's text. */
  public String toToml() {
    StringBuilder out = new StringBuilder();
    out.append(
        "# Visor's configuration, written by the robot's deploy: edits here are overwritten.\n");
    out.append("team = ").append(team).append('\n');
    out.append("port = ").append(port).append('\n');
    if (tagSizeMeters > 0) {
      out.append("tag_size_m = ").append(number(tagSizeMeters)).append('\n');
    }
    for (Camera camera : cameras) {
      camera.write(out);
    }
    return out.toString();
  }

  /** A TOML basic string. */
  static String quoted(String text) {
    StringBuilder out = new StringBuilder("\"");
    for (int i = 0; i < text.length(); i++) {
      char c = text.charAt(i);
      switch (c) {
        case '"' -> out.append("\\\"");
        case '\\' -> out.append("\\\\");
        case '\n' -> out.append("\\n");
        case '\t' -> out.append("\\t");
        default -> {
          if (c < 0x20 || c == 0x7f) {
            out.append(String.format("\\u%04x", (int) c));
          } else {
            out.append(c);
          }
        }
      }
    }
    return out.append('"').toString();
  }

  /** A TOML float, exactly: Java's shortest form that reads back to the same double. */
  static String number(double value) {
    return Double.toString(value);
  }
}

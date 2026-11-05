package com.michaelgrundvig.frc.visor;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.jspecify.annotations.Nullable;

/**
 * Visor's configuration file (deploy/visor.example.toml), written for one coprocessor: what a
 * robot's deploy puts beside the program. Visor checks it as it reads it ({@code visor check}), so
 * this writes what it's given and leaves the judging to Visor.
 *
 * <pre>{@code
 * VisorConfigFile file = VisorConfigFile.forTeam(1234);
 * file.camera("front-left", 0, "/dev/v4l/by-id/usb-cam-video-index0", 1600, 1304)
 *     .fps(60)
 *     .exposureMicros(1000)
 *     .gain(40)
 *     .lens(1600, 1304, 1229.7, 1229.9, 801.2, 650.4, -0.31, 0.12, 0.0004, -0.0002, -0.02);
 * String toml = file.toToml();
 * }</pre>
 */
public final class VisorConfigFile {
  private final int team;
  private final List<Camera> cameras = new ArrayList<>();

  private VisorConfigFile(int team) {
    this.team = team;
  }

  /** Frames to the robot controller of this team, at 10.TE.AM.2. */
  public static VisorConfigFile forTeam(int team) {
    if (team <= 0) {
      throw new IllegalArgumentException("a team number is positive, not " + team);
    }
    return new VisorConfigFile(team);
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
    Camera camera = new Camera(name, index, device, width, height);
    cameras.add(camera);
    return camera;
  }

  /**
   * One camera's entry. Unset settings are left out, so Visor's defaults apply: for the exposure
   * and gain, the camera's own.
   */
  public static final class Camera {
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
    private final Map<String, Long> controls = new LinkedHashMap<>();
    private double[] lens = new double[0];

    private Camera(String name, int index, String device, int width, int height) {
      this.name = Objects.requireNonNull(name);
      this.index = index;
      this.device = Objects.requireNonNull(device);
      this.width = width;
      this.height = height;
    }

    /** Frames a second to ask the camera for. */
    public Camera fps(int fps) {
      this.fps = fps;
      return this;
    }

    /** The smallest tag NAPA looks for, in pixels. */
    public Camera minTagPixels(int pixels) {
      this.minTagPixels = pixels;
      return this;
    }

    /** Added to every capture time: the spin test's result. */
    public Camera timeOffsetMicros(long micros) {
      this.timeOffsetMicros = micros;
      return this;
    }

    /**
     * A fixed exposure, in microseconds: Visor turns the camera's automatic exposure off and sets
     * this, every time it opens the camera. Without one, the camera's own automatic exposure.
     */
    public Camera exposureMicros(long micros) {
      if (micros <= 0) {
        throw new IllegalArgumentException("an exposure is a positive time, not " + micros);
      }
      this.exposureMicros = micros;
      return this;
    }

    /**
     * The camera's {@code gain} control, in its own units ({@code visor cameras} lists its range).
     * Without one, the camera's default.
     */
    public Camera gain(long gain) {
      this.gain = gain;
      return this;
    }

    /**
     * A V4L2 control, by the name {@code v4l2-ctl --list-ctrls} prints. Visor sets them after the
     * exposure and gain, in the order given, which matters where one control depends on another.
     */
    public Camera control(String control, long value) {
      controls.put(Objects.requireNonNull(control), value);
      return this;
    }

    /**
     * The lens calibration ({@code tools/calibrate.py}): the frame size it was made at, focal
     * lengths and centre in pixels, and OpenCV's distortion coefficients (0, 4, 5 or 8).
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

package com.michaelgrundvig.frc.visor;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.util.Properties;

/**
 * The Visor release this client belongs to. Visor and the robot program deploy together, so a frame
 * carrying another version ID came from a coprocessor a deploy missed.
 */
public final class VisorVersion {
  /** What the build wrote into the jar: the version, and the release archive's SHA-256. */
  private static final Properties PROPERTIES = read();

  /** The version, year.month.day: Visor's, from its Cargo.toml when this client was built. */
  public static final String VERSION = PROPERTIES.getProperty("version", "");

  /** The version as a number, as frames carry it: 2026.11.2 is 20261102. */
  public static final int ID = id(VERSION);

  /**
   * The SHA-256 of this version's release archive ({@link VisorPack#releaseName}), in lowercase
   * hex: the archive published beside this client, which a robot's build stages and its boards
   * install only when it has this hash. Empty in a client built without one (a developer's own),
   * which nothing installs from.
   */
  public static final String RELEASE_SHA256 = PROPERTIES.getProperty("release-sha256", "");

  private VisorVersion() {}

  /** A version's ID: year, month and day as one number. */
  static int id(String version) {
    String[] parts = version.split("\\.", -1);
    if (parts.length != 3) {
      throw new IllegalArgumentException("a version is year.month.day: " + version);
    }
    return Integer.parseInt(parts[0]) * 10_000
        + Integer.parseInt(parts[1]) * 100
        + Integer.parseInt(parts[2]);
  }

  private static Properties read() {
    try (InputStream in = VisorVersion.class.getResourceAsStream("version.properties")) {
      if (in == null) {
        throw new IllegalStateException("version.properties is missing from the client's jar");
      }
      Properties properties = new Properties();
      properties.load(in);
      return properties;
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }
}

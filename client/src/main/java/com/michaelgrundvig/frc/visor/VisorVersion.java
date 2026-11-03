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
  /** The version, year.month.day: Visor's, from its Cargo.toml when this client was built. */
  public static final String VERSION = read();

  /** The version as a number, as frames carry it: 2026.11.2 is 20261102. */
  public static final int ID = id(VERSION);

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

  private static String read() {
    try (InputStream in = VisorVersion.class.getResourceAsStream("version.properties")) {
      if (in == null) {
        throw new IllegalStateException("version.properties is missing from the client's jar");
      }
      Properties properties = new Properties();
      properties.load(in);
      return properties.getProperty("version", "");
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }
}

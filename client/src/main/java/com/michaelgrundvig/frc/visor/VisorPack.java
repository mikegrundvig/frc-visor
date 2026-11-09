package com.michaelgrundvig.frc.visor;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Arrays;
import java.util.HexFormat;
import java.util.List;

/**
 * Visor's Spotter pack ({@code spotter/visor} in each release), by the names a robot program gives
 * Spotter's manager: the values its status reports, its actions and their responses, and the
 * release archive its install takes. The pack's own files say what each one is; this class is the
 * one place the robot's side names them, and the client's tests run it against the pack.
 */
public final class VisorPack {
  /** The pack's name, as a board's definition lists it. */
  public static final String NAME = "visor";

  /** The version of the Visor installed, on disk; empty when there's none. */
  public static final String VERSION = "visor.status.version";

  /** The SHA-256 of the configuration on disk ({@link #configHash}); empty when there's none. */
  public static final String CONFIG = "visor.status.config";

  /**
   * The version of the Visor running, as it said when it started; empty while none runs. It differs
   * from {@link #VERSION} while a program put in place hasn't been started: a restart that failed.
   */
  public static final String RUNNING_VERSION = "visor.status.running-version";

  /** The SHA-256 of the configuration Visor runs, as {@link #CONFIG}; empty while none runs. */
  public static final String RUNNING_CONFIG = "visor.status.running-config";

  /** The cameras plugged in, by the names a configuration gives them, comma-separated. */
  public static final String CAMERAS = "visor.status.cameras";

  /** Every status value, in the order {@link #report} takes them. */
  public static final List<String> STATUS =
      List.of(VERSION, CONFIG, RUNNING_VERSION, RUNNING_CONFIG, CAMERAS);

  /** Visor's log, its journal. */
  public static final String LOG = "visor.log";

  /** The action that installs a release: its input is the archive ({@link #releaseName}). */
  public static final String INSTALL = "visor.install";

  /** The install's response: the version it installed. */
  public static final String INSTALLED = "version";

  /** The action that puts a configuration in place, once Visor's check of it passes. */
  public static final String CONFIGURE = "visor.configure";

  /** The configure action's response: the SHA-256 of the configuration it put in place. */
  public static final String CONFIGURED = "config";

  private VisorPack() {}

  /**
   * What a board's status says: what's on disk, what's running, and the cameras plugged in.
   *
   * @param version the Visor installed; empty when there's none
   * @param config its configuration's SHA-256; empty when there's none
   * @param runningVersion the Visor running; empty while none runs
   * @param runningConfig the SHA-256 of the configuration it runs; empty while none runs
   * @param cameras the cameras plugged in, by their names
   */
  public record Report(
      String version,
      String config,
      String runningVersion,
      String runningConfig,
      List<String> cameras) {
    public Report {
      cameras = List.copyOf(cameras);
    }
  }

  /** A status report, from its values' texts, in {@link #STATUS}'s order. */
  public static Report report(
      String version, String config, String runningVersion, String runningConfig, String cameras) {
    List<String> plugged = cameras.isEmpty() ? List.of() : Arrays.asList(cameras.split(",", -1));
    return new Report(version, config, runningVersion, runningConfig, plugged);
  }

  /** A release's archive, as its release workflow names it and its install takes it. */
  public static String releaseName(String version) {
    return "visor-" + version + "-linux-arm64.tar.gz";
  }

  /** A configuration's SHA-256 as the status reports it: of its UTF-8 bytes, in lowercase hex. */
  public static String configHash(String config) {
    try {
      byte[] digest =
          MessageDigest.getInstance("SHA-256").digest(config.getBytes(StandardCharsets.UTF_8));
      return HexFormat.of().formatHex(digest);
    } catch (NoSuchAlgorithmException e) {
      throw new IllegalStateException("every Java has SHA-256", e);
    }
  }
}

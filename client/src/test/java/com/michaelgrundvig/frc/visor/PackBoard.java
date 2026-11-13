package com.michaelgrundvig.frc.visor;

import static org.assertj.core.api.Assertions.assertThat;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.PosixFilePermissions;
import java.util.Map;

/**
 * A coprocessor as Visor's pack sees it, in a folder of a test's: Visor in {@code opt-visor}, the
 * service's runtime folder in {@code run}, and in place of systemd, a restart that starts the
 * program installed (it writes the runtime file, as Visor does as it starts, under the main process
 * ID {@value #MAIN_PID}), or goes as {@link #restart} says. The pack's scripts run directly, as
 * Spotter's agent runs them, with the variables that point them here.
 */
final class PackBoard {
  /** The service's main process ID, while it runs. */
  static final String MAIN_PID = "4242";

  private static final ObjectMapper JSON = new ObjectMapper();

  final Path root;
  final Path visor;
  final Path run;
  final Path restarts;
  private final Path pack;
  private final Path restart;
  private final Path mainPid;

  /** What a pack script did: its exit status, its standard output and its standard error. */
  record Ran(int exit, String out, String err) {}

  PackBoard(Path root, Path pack) throws IOException {
    this.root = root;
    this.pack = pack;
    this.visor = Files.createDirectories(root.resolve("opt-visor"));
    this.run = Files.createDirectories(root.resolve("run"));
    this.restarts = root.resolve("restarts");
    // Starting Visor writes what it runs, as src/running.rs does: its version, from the program,
    // and its configuration's SHA-256.
    this.restart =
        executable(
            root.resolve("restart"),
            """
            #!/bin/sh
            set -eu
            if [ -e "%2$s/restart-cut-off" ]; then
              # The run is cut off here, before the restart: the old process runs on.
              kill -KILL "$PPID"
              exit 1
            fi
            echo restarted >> "%1$s"
            if [ -e "%2$s/restart-fails" ]; then
              # Stopped, and the new one doesn't start: nothing runs.
              rm -f "%5$s/running"
              echo "Job for visor.service failed" >&2
              exit 1
            fi
            version=$("%3$s/visor" version | cut -d ' ' -f 2)
            config=$(sha256sum "%3$s/visor.toml" | cut -d ' ' -f 1)
            printf 'pid=%4$s\\nversion=%%s\\nconfig=%%s\\n' "$version" "$config" > "%5$s/running"
            """
                .formatted(restarts, root, visor, MAIN_PID, run));
    this.mainPid = executable(root.resolve("main-pid"), "#!/bin/sh\necho " + MAIN_PID + "\n");
  }

  /** Visor's program, as the pack sees it: its version, and a check refusing what says "bad". */
  static String program(String version) {
    return """
    #!/bin/sh
    case "$1" in
      version) echo "visor %s (version ID 1)" ;;
      check)
        if grep -q bad "$3"; then echo "$3: no, this is bad" >&2; exit 1; fi
        echo "$3: 1 camera" ;;
      cameras) echo usb-Thrifty_Cam_0001-video-index0 ;;
    esac
    """
        .formatted(version);
  }

  /**
   * A board running Visor {@code version} on {@code config}, as an install and restart leave it.
   */
  void runs(String version, String config) throws IOException, InterruptedException {
    executable(visor.resolve("visor"), program(version));
    Files.writeString(visor.resolve("visor.toml"), config);
    assertThat(script("restart", new byte[0]).exit()).isZero();
    Files.deleteIfExists(restarts);
  }

  /** How the next restart goes. */
  enum Restart {
    /** Visor starts on what's in place. */
    STARTS,
    /** The old process stops, and the new one doesn't start, as for a unit that won't. */
    FAILS,
    /**
     * The run is cut off before the restart (a timeout, the agent stopped): the old one runs on.
     */
    CUT_OFF
  }

  void restart(Restart how) throws IOException {
    Files.deleteIfExists(root.resolve("restart-fails"));
    Files.deleteIfExists(root.resolve("restart-cut-off"));
    switch (how) {
      case STARTS -> {}
      case FAILS -> Files.writeString(root.resolve("restart-fails"), "");
      case CUT_OFF -> Files.writeString(root.resolve("restart-cut-off"), "");
    }
  }

  /** How many times Visor was restarted. */
  long restarted() throws IOException {
    return Files.exists(restarts) ? Files.readAllLines(restarts).size() : 0;
  }

  /** The pack's status now. */
  VisorPack.Report status() throws IOException, InterruptedException {
    Ran ran = script("status", new byte[0]);
    assertThat(ran.exit()).as(ran.err()).isZero();
    JsonNode said = JSON.readTree(ran.out());
    return VisorPack.report(
        said.get("version").asText(),
        said.get("config").asText(),
        said.get("running-version").asText(),
        said.get("running-config").asText(),
        said.get("cameras").asText());
  }

  /** Runs one of the pack's scripts with {@code input} on its standard input. */
  Ran script(String name, byte[] input) throws IOException, InterruptedException {
    return script(name, input, Map.of());
  }

  /** The same, with more variables. */
  Ran script(String name, byte[] input, Map<String, String> more)
      throws IOException, InterruptedException {
    Path program = name.equals("restart") ? restart : pack.resolve(name);
    Path in = Files.createTempFile(root, "input", "");
    Files.write(in, input);
    ProcessBuilder builder =
        new ProcessBuilder(program.toString()).directory(pack.toFile()).redirectInput(in.toFile());
    builder.environment().putAll(environment());
    builder.environment().putAll(more);
    Process process = builder.start();
    // Its output is small: read one after the other.
    String out = new String(process.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
    String err = new String(process.getErrorStream().readAllBytes(), StandardCharsets.UTF_8);
    int exit = process.waitFor();
    Files.delete(in);
    return new Ran(exit, out, err);
  }

  /** The variables that point the pack at this board, as an agent's environment gives them. */
  Map<String, String> environment() {
    return Map.of(
        "VISOR_DIR", visor.toString(),
        "VISOR_RUN", run.toString(),
        "VISOR_MAIN_PID", mainPid.toString(),
        "VISOR_RESTART", restart.toString(),
        "VISOR_BY_ID", root.resolve("by-id").toString(),
        "VISOR_BY_PATH", root.resolve("by-path").toString(),
        "VISOR_SYSFS", root.resolve("sys").toString());
  }

  static Path executable(Path file, String text) throws IOException {
    Files.writeString(file, text);
    Files.setPosixFilePermissions(file, PosixFilePermissions.fromString("rwxr-xr-x"));
    return file;
  }
}

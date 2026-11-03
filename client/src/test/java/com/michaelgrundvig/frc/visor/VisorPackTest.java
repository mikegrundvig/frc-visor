package com.michaelgrundvig.frc.visor;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.spotter.manager.Board;
import com.michaelgrundvig.frc.spotter.manager.BoardDefinition;
import com.michaelgrundvig.frc.spotter.manager.Level;
import com.michaelgrundvig.frc.spotter.manager.Manager;
import com.michaelgrundvig.frc.spotter.manager.Recorder;
import com.michaelgrundvig.frc.spotter.manager.RobotSignals;
import com.michaelgrundvig.frc.spotter.manager.Run;
import com.michaelgrundvig.frc.spotter.manager.Settings;
import com.michaelgrundvig.frc.spotter.manager.Value;
import java.io.File;
import java.io.IOException;
import java.net.ServerSocket;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.PosixFilePermissions;
import java.security.MessageDigest;
import java.util.HexFormat;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * Visor's Spotter pack (spotter/visor) on Spotter's real agent, driven by its real manager as the
 * robot program drives it: the robot pushes the pack, reads what the board has, installs a release
 * and configures it, and the pack refuses what would break the board. The board is this computer:
 * the agent runs from its published jar with its files under a folder of the test's, and Visor's
 * program is a script that answers {@code version} and {@code check} as Visor does.
 */
class VisorPackTest {
  @TempDir Path work;

  private @Nullable Process agent;
  private volatile boolean stopping;
  private @Nullable Thread supervisor;
  private @Nullable Manager manager;

  /** Visor's program, as the pack sees it: its version, and a check refusing what says "bad". */
  private static String program(String version) {
    return """
        #!/bin/sh
        case "$1" in
          version) echo "visor %s (version ID 1)" ;;
          check)
            if grep -q bad "$3"; then echo "$3: no, this is bad" >&2; exit 1; fi
            echo "$3: 1 camera" ;;
        esac
        """
        .formatted(version);
  }

  @AfterEach
  void stop() throws InterruptedException {
    if (manager != null) {
      manager.close();
    }
    stopping = true;
    if (supervisor != null) {
      supervisor.interrupt();
      supervisor.join(5_000);
    }
    if (agent != null) {
      agent.destroy();
      agent.waitFor();
    }
  }

  @Test
  void aBoardIsBroughtUpToDateAndRefusesWhatWouldBreakIt() throws Exception {
    Path visor = Files.createDirectories(work.resolve("opt-visor"));
    Path devices = Files.createDirectories(work.resolve("by-id"));
    for (String device :
        List.of(
            "usb-Thrifty_Cam_0001-video-index0",
            "usb-Thrifty_Cam_0001-video-index1",
            "usb-Other_Cam_X-video-index0")) {
      Files.createFile(devices.resolve(device));
    }
    Path restarts = work.resolve("restarts");
    Path restart = executable("restart", "#!/bin/sh\necho restarted >> " + restarts + "\n");
    Path packs = Files.createDirectories(work.resolve("packs/visor"));
    try (Stream<Path> files = Files.list(Path.of(System.getProperty("visor.pack")))) {
      for (Path file : files.toList()) {
        Files.copy(file, packs.resolve(file.getFileName()), StandardCopyOption.COPY_ATTRIBUTES);
      }
    }
    int port = freePort();
    startAgent(port, visor, devices, restart);

    Manager spotter =
        new Manager(
            new RobotSignals(() -> false, () -> false, System::nanoTime),
            List.of(new BoardDefinition("127.0.0.1:" + port, List.of("visor"))),
            Settings.DEFAULTS.withPacks(work.resolve("packs")),
            Recorder.NONE);
    manager = spotter;
    spotter.start();
    Board board = spotter.boards().get(0);
    Value version = board.value("visor.status.version");
    Value config = board.value("visor.status.config");
    Value cameras = board.value("visor.status.cameras");

    // Pushed the pack, the board says it has no Visor yet, and which cameras are plugged in.
    await(spotter, version::available, "the pack's status");
    assertThat(version.text()).as("no Visor yet").isEmpty();
    assertThat(config.text()).isEmpty();
    assertThat(cameras.text())
        .isEqualTo("usb-Other_Cam_X-video-index0,usb-Thrifty_Cam_0001-video-index0");

    // A release installs, and Visor is restarted on it.
    Run installed = run(spotter, board.run("visor.install", release(program("2099.1.2"))));
    assertThat(installed.level()).as(installed.reason()).isEqualTo(Level.OK);
    assertThat(Objects.requireNonNull(installed.response("version")).text()).isEqualTo("2099.1.2");
    await(spotter, () -> version.text().equals("2099.1.2"), "the installed version");
    assertThat(restarts).content().isEqualTo("restarted\n");

    // A configuration Visor accepts is put in place.
    String toml = "team = 1234\n";
    Run configured = run(spotter, board.run("visor.configure", toml));
    assertThat(configured.level()).as(configured.reason()).isEqualTo(Level.OK);
    String hash = sha256(toml);
    await(spotter, () -> config.text().equals(hash), "the configuration's hash");
    assertThat(visor.resolve("visor.toml")).content().isEqualTo(toml);

    // One it refuses leaves the board as it was, and says why.
    Run refused = run(spotter, board.run("visor.configure", "bad = true\n"));
    assertThat(refused.level()).isEqualTo(Level.FAILING);
    assertThat(refused.log())
        .anySatisfy(line -> assertThat(line.getMessage()).contains("no, this is bad"));
    assertThat(visor.resolve("visor.toml")).content().isEqualTo(toml);
    assertThat(visor.resolve("visor.toml.new")).doesNotExist();

    // A release whose program won't run on the board is never put in place.
    Run broken = run(spotter, board.run("visor.install", release("#!/bin/sh\nexit 3\n")));
    assertThat(broken.level()).isEqualTo(Level.FAILING);
    assertThat(broken.log())
        .anySatisfy(line -> assertThat(line.getMessage()).contains("doesn't run here"));
    assertThat(visor.resolve("visor")).content().isEqualTo(program("2099.1.2"));
    assertThat(restarts)
        .content()
        .as("restarted for the two that went in")
        .isEqualTo("restarted\nrestarted\n");
  }

  /**
   * The agent, as systemd runs it on a board: started again whenever it exits, as it does after a
   * push to read the packs pushed.
   */
  private void startAgent(int port, Path visor, Path devices, Path restart) throws IOException {
    Path root = Files.createDirectories(work.resolve("root"));
    Files.createDirectories(root.resolve("var/lib/frc-spotter"));
    Path log = work.resolve("agent.log");
    ProcessBuilder builder =
        new ProcessBuilder(
                ProcessHandle.current().info().command().orElse("java"),
                "-Xmx128m",
                "-jar",
                System.getProperty("visor.spotterAgent"),
                "--root=" + root,
                "--controller=127.0.0.1",
                "--port=" + port)
            .redirectErrorStream(true)
            .redirectOutput(ProcessBuilder.Redirect.appendTo(log.toFile()));
    builder.environment().put("VISOR_DIR", visor.toString());
    builder.environment().put("VISOR_DEVICES", devices.toString());
    builder.environment().put("VISOR_RESTART", restart.toString());
    agent = builder.start();
    Thread thread =
        new Thread(
            () -> {
              while (!stopping) {
                try {
                  Process running = Objects.requireNonNull(agent);
                  running.waitFor();
                  if (!stopping) {
                    Thread.sleep(200);
                    agent = builder.start();
                  }
                } catch (InterruptedException | IOException e) {
                  return;
                }
              }
            },
            "agent-supervisor");
    thread.setDaemon(true);
    thread.start();
    supervisor = thread;
  }

  /** A release archive as the release workflow makes one: the program, as {@code ./visor}. */
  private byte[] release(String program) throws IOException, InterruptedException {
    Path folder = Files.createTempDirectory(work, "release");
    Path visor = folder.resolve("visor");
    Files.writeString(visor, program);
    Files.setPosixFilePermissions(visor, PosixFilePermissions.fromString("rwxr-xr-x"));
    Path archive = work.resolve(folder.getFileName() + ".tar.gz");
    Process tar =
        new ProcessBuilder("tar", "-czf", archive.toString(), "-C", folder.toString(), ".")
            .inheritIO()
            .start();
    assertThat(tar.waitFor()).isZero();
    return Files.readAllBytes(archive);
  }

  private Path executable(String name, String text) throws IOException {
    Path file = work.resolve(name);
    Files.writeString(file, text);
    Files.setPosixFilePermissions(file, PosixFilePermissions.fromString("rwxr-xr-x"));
    return file;
  }

  /** Keeps the manager up to date, as the robot loop does, until the run is done. */
  private Run run(Manager spotter, Run run) throws Exception {
    await(spotter, run::done, run.action());
    return run;
  }

  private void await(Manager spotter, BooleanSupplier done, String what) throws Exception {
    long until = System.nanoTime() + 60_000_000_000L;
    while (System.nanoTime() < until) {
      spotter.update();
      if (done.getAsBoolean()) {
        return;
      }
      Thread.sleep(20);
    }
    throw new AssertionError("waited a minute for " + what + "; the agent said:\n" + agentLog());
  }

  private String agentLog() throws IOException {
    File log = work.resolve("agent.log").toFile();
    return log.exists() ? Files.readString(log.toPath()) : "(nothing)";
  }

  private static String sha256(String text) throws Exception {
    return HexFormat.of()
        .formatHex(
            MessageDigest.getInstance("SHA-256").digest(text.getBytes(StandardCharsets.UTF_8)));
  }

  private static int freePort() throws IOException {
    try (ServerSocket socket = new ServerSocket(0)) {
      return socket.getLocalPort();
    }
  }
}

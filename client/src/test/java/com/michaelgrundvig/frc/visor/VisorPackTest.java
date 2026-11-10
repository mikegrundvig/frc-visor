package com.michaelgrundvig.frc.visor;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.spotter.manager.Board;
import com.michaelgrundvig.frc.spotter.manager.BoardDefinition;
import com.michaelgrundvig.frc.spotter.manager.Level;
import com.michaelgrundvig.frc.spotter.manager.Manager;
import com.michaelgrundvig.frc.spotter.manager.Recorder;
import com.michaelgrundvig.frc.spotter.manager.RobotSignals;
import com.michaelgrundvig.frc.spotter.manager.Run;
import com.michaelgrundvig.frc.spotter.manager.Settings;
import com.michaelgrundvig.frc.spotter.manager.Value;
import com.michaelgrundvig.frc.visor.PackBoard.Ran;
import com.michaelgrundvig.frc.visor.PackBoard.Restart;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.attribute.PosixFilePermissions;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * Visor's Spotter pack (spotter/visor), by the names {@link VisorPack} gives the robot. Its scripts
 * run directly on a board in a folder of the test's ({@link PackBoard}), one row each for what an
 * install, a configuration or a page of the log can meet; and once on Spotter's real agent, driven
 * by its real manager as the robot program drives it.
 */
class VisorPackTest {
  private static final String OLD = "2099.1.1";
  private static final String NEW = "2099.1.2";
  private static final String CONFIG = "team = 1234\n";

  /** The agent's own report of the port it serves on, as it starts. */
  private static final Pattern SERVING = Pattern.compile("serving protocol 2 on port (\\d+)");

  @TempDir Path work;

  private @Nullable Process agent;
  private volatile boolean stopping;
  private @Nullable Thread supervisor;
  private @Nullable Manager manager;

  /** What a script's run is to end in. */
  private enum Exit {
    OK,
    FAILS;

    static Exit of(Ran ran) {
      return ran.exit() == 0 ? OK : FAILS;
    }
  }

  /** A release archive, made in a folder of the test's. */
  private interface Archive {
    byte[] make(Path folder) throws Exception;
  }

  /** What the install is given for an archive. */
  private interface Input {
    byte[] of(byte[] archive);
  }

  /** As the robot gives it: the archive after the line naming its own SHA-256. */
  private static final Input NAMED =
      archive -> VisorPack.installInput(archive, VisorPack.sha256(archive));

  /**
   * An install's row: the archive, how the restart goes, what the install ends in and says, then
   * the Visor on disk, the one running, and how many restarts there were.
   */
  private record InstallRow(
      String why,
      Archive archive,
      Input input,
      Restart restart,
      Exit exit,
      String says,
      String installed,
      String running,
      long restarts) {}

  @Test
  void anInstallChangesTheBoardOnlyWhenItsReleaseRunsThere() throws Exception {
    Path outside = work.resolve("outside");
    Path ranOutside = work.resolve("outside.ran");
    PackBoard.executable(outside, "#!/bin/sh\ntouch " + ranOutside + "\necho visor 2099.9.9\n");
    // A program that says it ran, in an archive the robot didn't name.
    String says = "#!/bin/sh\ntouch " + ranOutside + "\necho visor " + NEW + "\n";
    List<InstallRow> rows =
        List.of(
            new InstallRow(
                "a release installs, and Visor runs it",
                folder -> release(folder, Map.of("visor", PackBoard.program(NEW))),
                NAMED,
                Restart.STARTS,
                Exit.OK,
                "installed visor " + NEW,
                NEW,
                NEW,
                1),
            new InstallRow(
                "the same version again: put in place and restarted, so a board whose restart"
                    + " failed is put right",
                folder -> release(folder, Map.of("visor", PackBoard.program(OLD))),
                NAMED,
                Restart.STARTS,
                Exit.OK,
                "installed visor " + OLD,
                OLD,
                OLD,
                1),
            new InstallRow(
                "no visor in the archive",
                folder -> release(folder, Map.of("README.md", "a release\n")),
                NAMED,
                Restart.STARTS,
                Exit.FAILS,
                "the archive has no visor program",
                OLD,
                OLD,
                0),
            new InstallRow(
                "a program whose version fails: it doesn't run on this board",
                folder -> release(folder, Map.of("visor", "#!/bin/sh\nexit 3\n")),
                NAMED,
                Restart.STARTS,
                Exit.FAILS,
                "doesn't run here",
                OLD,
                OLD,
                0),
            new InstallRow(
                "an archive cut short",
                folder -> {
                  byte[] whole = release(folder, Map.of("visor", PackBoard.program(NEW)));
                  return Arrays.copyOf(whole, whole.length / 2);
                },
                NAMED,
                Restart.STARTS,
                Exit.FAILS,
                "can't be read",
                OLD,
                OLD,
                0),
            new InstallRow(
                "a visor that's a link to a program outside the archive: refused before anything"
                    + " runs",
                folder -> release(folder, Map.of("visor", "->" + outside)),
                NAMED,
                Restart.STARTS,
                Exit.FAILS,
                "only files and folders",
                OLD,
                OLD,
                0),
            new InstallRow(
                "a link beside the program",
                folder -> release(folder, Map.of("visor", PackBoard.program(NEW), "etc", "->/etc")),
                NAMED,
                Restart.STARTS,
                Exit.FAILS,
                "only files and folders",
                OLD,
                OLD,
                0),
            new InstallRow(
                "an archive other than the one the robot names: refused before anything in it"
                    + " runs",
                folder -> release(folder, Map.of("visor", says)),
                archive ->
                    VisorPack.installInput(
                        archive, VisorPack.sha256("another".getBytes(StandardCharsets.UTF_8))),
                Restart.STARTS,
                Exit.FAILS,
                "isn't the release the robot names",
                OLD,
                OLD,
                0),
            new InstallRow(
                "an archive alone, with no SHA-256 named before it",
                folder -> release(folder, Map.of("visor", says)),
                archive -> archive,
                Restart.STARTS,
                Exit.FAILS,
                "names no SHA-256",
                OLD,
                OLD,
                0),
            new InstallRow(
                "a SHA-256 cut short",
                folder -> release(folder, Map.of("visor", says)),
                archive -> {
                  byte[] line = "sha256=abc\n".getBytes(StandardCharsets.US_ASCII);
                  byte[] input = Arrays.copyOf(line, line.length + archive.length);
                  System.arraycopy(archive, 0, input, line.length, archive.length);
                  return input;
                },
                Restart.STARTS,
                Exit.FAILS,
                "isn't 64 lowercase hex digits",
                OLD,
                OLD,
                0),
            new InstallRow(
                "a restart that fails once the program is in place: the status says none runs",
                folder -> release(folder, Map.of("visor", PackBoard.program(NEW))),
                NAMED,
                Restart.FAILS,
                Exit.FAILS,
                "installed visor " + NEW,
                NEW,
                "",
                1),
            new InstallRow(
                "an install cut off once the program is in place: the old one runs on, and the"
                    + " status says so",
                folder -> release(folder, Map.of("visor", PackBoard.program(NEW))),
                NAMED,
                Restart.CUT_OFF,
                Exit.FAILS,
                "installed visor " + NEW,
                NEW,
                OLD,
                0));
    for (int n = 0; n < rows.size(); n++) {
      InstallRow row = rows.get(n);
      PackBoard board = board("install-" + n);
      board.runs(OLD, CONFIG);
      // What an install cut off while it unpacked left: the next one starts afresh.
      Files.createDirectories(board.visor.resolve(".install"));
      Files.writeString(board.visor.resolve(".install/visor"), "#!/bin/sh\nexit 9\n");
      board.restart(row.restart());
      byte[] archive = row.archive().make(Files.createDirectories(work.resolve("release-" + n)));
      Ran ran = board.script("install", row.input().of(archive));

      assertThat(Exit.of(ran)).as(row.why() + ": " + ran).isEqualTo(row.exit());
      assertThat(ran.err()).as(row.why()).contains(row.says());
      VisorPack.Report status = board.status();
      assertThat(status.version()).as(row.why()).isEqualTo(row.installed());
      assertThat(status.runningVersion()).as(row.why()).isEqualTo(row.running());
      assertThat(board.restarted()).as(row.why()).isEqualTo(row.restarts());
      assertThat(ranOutside).as(row.why() + ": ran nothing outside the archive").doesNotExist();
      assertThat(board.visor.resolve(".install")).as(row.why() + ": left nothing").doesNotExist();
      if (row.exit() == Exit.OK) {
        assertThat(ran.out())
            .as(row.why())
            .isEqualTo("{\"version\": \"" + row.installed() + "\"}\n");
        assertThat(permissions(board.visor.resolve("visor"))).isEqualTo("rwxr-xr-x");
      }
    }
  }

  /**
   * A configuration's row: the configuration, how the restart goes, what it ends in and says, and
   * then the configuration on disk and the one running (empty: none runs).
   */
  private record ConfigureRow(
      String why,
      String config,
      Restart restart,
      Exit exit,
      String says,
      String onDisk,
      String running) {}

  @Test
  void aConfigurationIsPutInPlaceOnlyOnceVisorAcceptsIt() throws Exception {
    String next = "team = 5678\n";
    List<ConfigureRow> rows =
        List.of(
            new ConfigureRow(
                "one Visor accepts, and runs",
                next,
                Restart.STARTS,
                Exit.OK,
                "1 camera",
                next,
                next),
            new ConfigureRow(
                "one it refuses leaves the board as it was, and says why",
                "bad = true\n",
                Restart.STARTS,
                Exit.FAILS,
                "no, this is bad",
                CONFIG,
                CONFIG),
            new ConfigureRow(
                "a restart that fails once it's in place: none runs",
                next,
                Restart.FAILS,
                Exit.FAILS,
                "visor.service failed",
                next,
                ""),
            new ConfigureRow(
                "cut off once it's in place: the old configuration runs on",
                next,
                Restart.CUT_OFF,
                Exit.FAILS,
                "1 camera",
                next,
                CONFIG));
    for (int n = 0; n < rows.size(); n++) {
      ConfigureRow row = rows.get(n);
      PackBoard board = board("configure-" + n);
      board.runs(OLD, CONFIG);
      board.restart(row.restart());
      Ran ran = board.script("configure", row.config().getBytes(StandardCharsets.UTF_8));

      assertThat(Exit.of(ran)).as(row.why() + ": " + ran).isEqualTo(row.exit());
      assertThat(ran.err()).as(row.why()).contains(row.says());
      VisorPack.Report status = board.status();
      assertThat(status.config()).as(row.why()).isEqualTo(VisorPack.configHash(row.onDisk()));
      String running = row.running().isEmpty() ? "" : VisorPack.configHash(row.running());
      assertThat(status.runningConfig()).as(row.why()).isEqualTo(running);
      assertThat(board.visor.resolve("visor.toml")).content().isEqualTo(row.onDisk());
      assertThat(board.visor.resolve("visor.toml.new")).doesNotExist();
      // Readable by Visor, which runs as its own user, though the agent writes it under umask 0077.
      assertThat(permissions(board.visor.resolve("visor.toml"))).isEqualTo("rw-r--r--");
      if (row.exit() == Exit.OK) {
        assertThat(ran.out())
            .isEqualTo("{\"config\": \"" + VisorPack.configHash(row.onDisk()) + "\"}\n");
      }
    }
  }

  /** A page of the log's row: what's asked, journalctl itself, what it ends in, and the page. */
  private record JournalRow(
      String why, Map<String, String> asked, String journalctl, Exit exit, List<String> page) {}

  @Test
  void aPageOfTheLogFailsWhenJournalctlDoes() throws Exception {
    // A journal of five entries, oldest first, and newest first with --reverse, as journalctl's.
    String five =
        """
        #!/bin/sh
        for argument; do
          if [ "$argument" = --reverse ]; then printf '5\\n4\\n3\\n2\\n1\\n'; exit 0; fi
        done
        printf '1\\n2\\n3\\n4\\n5\\n'
        """;
    String endless = "#!/bin/sh\nexec yes again\n";
    String refuses = "#!/bin/sh\necho 'Failed to seek to cursor: Invalid argument' >&2\nexit 1\n";
    List<JournalRow> rows =
        List.of(
            new JournalRow(
                "the latest, oldest first",
                Map.of("SPOTTER_LIMIT", "2"),
                five,
                Exit.OK,
                List.of("4", "5")),
            new JournalRow(
                "a journal longer than the page: journalctl is stopped once the page is full,"
                    + " which is no failure",
                Map.of("SPOTTER_LIMIT", "3"),
                endless,
                Exit.OK,
                List.of("again", "again", "again")),
            new JournalRow(
                "a cursor journalctl refuses: the page fails, rather than coming back empty",
                Map.of("SPOTTER_FROM", "after", "SPOTTER_CURSOR", "nonsense"),
                refuses,
                Exit.FAILS,
                List.of()),
            new JournalRow(
                "journalctl failing on the latest page", Map.of(), refuses, Exit.FAILS, List.of()));
    for (int n = 0; n < rows.size(); n++) {
      JournalRow row = rows.get(n);
      PackBoard board = board("journal-" + n);
      Path bin = Files.createDirectories(board.root.resolve("bin"));
      PackBoard.executable(bin.resolve("journalctl"), row.journalctl());
      Map<String, String> environment = new HashMap<>(row.asked());
      environment.put("PATH", bin + ":" + System.getenv("PATH"));
      Ran ran = board.script("journal", new byte[0], environment);

      assertThat(Exit.of(ran)).as(row.why() + ": " + ran).isEqualTo(row.exit());
      List<String> page = ran.out().isEmpty() ? List.of() : List.of(ran.out().split("\n"));
      assertThat(page).as(row.why()).isEqualTo(row.page());
    }
  }

  @Test
  void aStatusReportIsReadAsThePackWritesIt() {
    assertThat(VisorPack.report(NEW, "abc", "", "", ""))
        .isEqualTo(new VisorPack.Report(NEW, "abc", "", "", List.of()));
    assertThat(VisorPack.report(NEW, "abc", NEW, "abc", "one,two").cameras())
        .containsExactly("one", "two");
    assertThat(VisorPack.releaseName(NEW)).isEqualTo("visor-2099.1.2-linux-arm64.tar.gz");
  }

  @Test
  void aConfigurationsIdIsTheStartOfItsHash() {
    // sha256sum's answers, as the pack reports them, and the IDs Visor's tests give them.
    record Row(String config, String hash, int id) {}
    List<Row> rows =
        List.of(
            new Row(
                "", "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", 0xe3b0c442),
            new Row(
                CONFIG,
                "6a5d27df0fe1c29c85dde1a3e70dca2aaf0c32305ae3598a612142d37193f7a0",
                0x6a5d27df),
            new Row(
                "team = 5678\n",
                "a6738b08f08e4e5bc22144449d0ac3af5e8423ce86c6b3a06c0cf13927497663",
                0xa6738b08));
    for (Row row : rows) {
      assertThat(VisorPack.configHash(row.config())).isEqualTo(row.hash());
      assertThat(VisorPack.configId(row.hash())).as(row.config()).isEqualTo(row.id());
    }
    for (String notAHash : List.of("", "6a5d27df", rows.get(1).hash().toUpperCase(Locale.ROOT))) {
      assertThatThrownBy(() -> VisorPack.configId(notAHash))
          .isInstanceOf(IllegalArgumentException.class);
    }
  }

  @Test
  void anInstallsInputNamesTheArchivesHashFirst() {
    byte[] archive = {0x1f, (byte) 0x8b, 10, 0};
    String hash = VisorPack.sha256(archive);
    byte[] input = VisorPack.installInput(archive, hash);
    byte[] line = ("sha256=" + hash + "\n").getBytes(StandardCharsets.US_ASCII);
    assertThat(Arrays.copyOf(input, line.length)).isEqualTo(line);
    assertThat(Arrays.copyOfRange(input, line.length, input.length)).isEqualTo(archive);
    assertThatThrownBy(() -> VisorPack.installInput(archive, "abc"))
        .isInstanceOf(IllegalArgumentException.class);
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
  void onSpottersAgentABoardIsBroughtUpToDate() throws Exception {
    PackBoard board = board("agent");
    Path packs = Files.createDirectories(work.resolve("packs/visor"));
    try (Stream<Path> files = Files.list(Path.of(System.getProperty("visor.pack")))) {
      for (Path file : files.toList()) {
        Files.copy(file, packs.resolve(file.getFileName()), StandardCopyOption.COPY_ATTRIBUTES);
      }
    }
    int port = startAgent(board);

    Manager spotter =
        new Manager(
            new RobotSignals(() -> false, () -> false, System::nanoTime),
            List.of(new BoardDefinition("127.0.0.1:" + port, List.of(VisorPack.NAME))),
            Settings.DEFAULTS.withPacks(work.resolve("packs")),
            Recorder.NONE);
    manager = spotter;
    spotter.start();
    Board on = spotter.boards().get(0);
    List<Value> values = new ArrayList<>();
    for (String id : VisorPack.STATUS) {
      values.add(on.value(id));
    }
    BooleanSupplier reported = () -> values.stream().allMatch(Value::available);

    // Pushed the pack (the agent restarts to read it), the board says it has no Visor yet.
    await(spotter, reported, "the pack's status");
    assertThat(report(values)).isEqualTo(new VisorPack.Report("", "", "", "", List.of()));

    // A release installs, and Visor runs it.
    byte[] archive =
        release(
            Files.createDirectories(work.resolve("release")),
            Map.of("visor", PackBoard.program(NEW)));
    Files.writeString(board.visor.resolve("visor.toml"), CONFIG);
    Run installed =
        run(
            spotter,
            on.run(VisorPack.INSTALL, VisorPack.installInput(archive, VisorPack.sha256(archive))));
    assertThat(installed.level()).as(installed.reason()).isEqualTo(Level.OK);
    assertThat(Objects.requireNonNull(installed.response(VisorPack.INSTALLED)).text())
        .isEqualTo(NEW);
    await(spotter, () -> report(values).runningVersion().equals(NEW), "the running version");
    assertThat(report(values).version()).isEqualTo(NEW);
    assertThat(report(values).cameras()).containsExactly("usb-Thrifty_Cam_0001-video-index0");

    // A configuration Visor accepts is put in place, and runs.
    String next = "team = 5678\n";
    String hash = VisorPack.configHash(next);
    Run configured = run(spotter, on.run(VisorPack.CONFIGURE, next));
    assertThat(configured.level()).as(configured.reason()).isEqualTo(Level.OK);
    assertThat(Objects.requireNonNull(configured.response(VisorPack.CONFIGURED)).text())
        .isEqualTo(hash);
    await(spotter, () -> report(values).runningConfig().equals(hash), "the running configuration");
    assertThat(report(values).config()).isEqualTo(hash);
  }

  private static VisorPack.Report report(List<Value> values) {
    return VisorPack.report(
        values.get(0).text(),
        values.get(1).text(),
        values.get(2).text(),
        values.get(3).text(),
        values.get(4).text());
  }

  private PackBoard board(String name) throws IOException {
    return new PackBoard(
        Files.createDirectories(work.resolve(name)), Path.of(System.getProperty("visor.pack")));
  }

  /**
   * The agent, as systemd runs it on a board: started again whenever it exits, as it does after a
   * push to read the packs pushed. It's first given any free port (0), and says which it has; each
   * restart takes that one again.
   */
  private int startAgent(PackBoard board) throws Exception {
    Path root = Files.createDirectories(work.resolve("root"));
    Files.createDirectories(root.resolve("var/lib/frc-spotter"));
    Path log = work.resolve("agent.log");
    agent = agent(root, board, log, 0).start();
    long until = System.nanoTime() + 60_000_000_000L;
    int port = 0;
    while (port == 0 && System.nanoTime() < until) {
      Matcher said = SERVING.matcher(agentLog());
      if (said.find()) {
        port = Integer.parseInt(said.group(1));
      } else {
        Thread.sleep(20);
      }
    }
    assertThat(port).as("the agent's port; it said:\n" + agentLog()).isPositive();
    ProcessBuilder again = agent(root, board, log, port);
    Thread thread =
        new Thread(
            () -> {
              while (!stopping) {
                try {
                  Process running = Objects.requireNonNull(agent);
                  running.waitFor();
                  if (!stopping) {
                    Thread.sleep(200);
                    agent = again.start();
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
    return port;
  }

  private static ProcessBuilder agent(Path root, PackBoard board, Path log, int port) {
    ProcessBuilder builder =
        new ProcessBuilder(
                "/bin/sh",
                "-c",
                // With the umask its systemd unit gives it (UMask=0077): what a pack writes is its
                // own unless the pack says otherwise.
                "umask 077; exec \"$0\" -Xmx128m -jar \"$1\" --root=\"$2\" --controller=127.0.0.1"
                    + " --port=\"$3\"",
                ProcessHandle.current().info().command().orElse("java"),
                System.getProperty("visor.spotterAgent"),
                root.toString(),
                Integer.toString(port))
            .redirectErrorStream(true)
            .redirectOutput(ProcessBuilder.Redirect.appendTo(log.toFile()));
    builder.environment().putAll(board.environment());
    return builder;
  }

  /**
   * A release archive as the release workflow makes one, of the files given by name: each one's
   * text, or {@code ->target} for a link to {@code target}.
   */
  private static byte[] release(Path folder, Map<String, String> files)
      throws IOException, InterruptedException {
    Path contents = Files.createDirectories(folder.resolve("contents"));
    for (Map.Entry<String, String> file : files.entrySet()) {
      Path path = contents.resolve(file.getKey());
      if (file.getValue().startsWith("->")) {
        Files.createSymbolicLink(path, Path.of(file.getValue().substring(2)));
      } else {
        PackBoard.executable(path, file.getValue());
      }
    }
    Path archive = folder.resolve("release.tar.gz");
    Process tar =
        new ProcessBuilder("tar", "-czf", archive.toString(), "-C", contents.toString(), ".")
            .inheritIO()
            .start();
    assertThat(tar.waitFor()).isZero();
    return Files.readAllBytes(archive);
  }

  private static String permissions(Path file) throws IOException {
    return PosixFilePermissions.toString(Files.getPosixFilePermissions(file));
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
    Path log = work.resolve("agent.log");
    return Files.exists(log) ? Files.readString(log) : "(nothing)";
  }
}

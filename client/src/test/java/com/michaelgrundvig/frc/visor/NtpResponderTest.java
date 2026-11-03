package com.michaelgrundvig.frc.visor;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.concurrent.TimeUnit;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class NtpResponderTest {
  /** 1900 to 1970, in seconds. */
  private static final long NTP_TO_UNIX = 2_208_988_800L;

  /** A client's request: version 4, client mode, with its own transmit time. */
  private static ByteBuffer request(long transmit) {
    ByteBuffer request = ByteBuffer.allocate(48);
    request.put(0, (byte) ((4 << 3) | 3));
    request.put(2, (byte) -3);
    request.putLong(40, transmit);
    return request;
  }

  @Test
  void robotTimeBecomesNtpTime() {
    record Case(long robotNanos, long seconds, long fraction) {}
    long epochSeconds = 1_767_225_600L + NTP_TO_UNIX;
    Case[] cases = {
      new Case(0, epochSeconds, 0),
      new Case(500_000_000L, epochSeconds, 1L << 31),
      new Case(90_250_000_000L, epochSeconds + 90, 1L << 30),
    };
    for (Case c : cases) {
      long ntp = NtpResponder.ntpTime(c.robotNanos());
      assertThat(ntp >>> 32).isEqualTo(c.seconds());
      assertThat(ntp & 0xffffffffL).isEqualTo(c.fraction());
    }
  }

  @Test
  void aRequestGetsTheRobotsTimesAndItsOwnTimeBack() {
    ByteBuffer reply = ByteBuffer.allocate(48);
    long theirs = 0x1234_5678_9abc_def0L;
    assertThat(NtpResponder.reply(request(theirs), 1_000_000_000L, 1_000_050_000L, reply)).isTrue();
    assertThat(reply.remaining()).isEqualTo(48);
    assertThat(reply.get(0)).as("version 4, server").isEqualTo((byte) ((4 << 3) | 4));
    assertThat(reply.get(1)).as("stratum").isEqualTo((byte) 1);
    assertThat(reply.get(2)).as("their poll").isEqualTo((byte) -3);
    assertThat(reply.getLong(24)).as("their transmit time, as sent").isEqualTo(theirs);
    assertThat(reply.getLong(32)).isEqualTo(NtpResponder.ntpTime(1_000_000_000L));
    assertThat(reply.getLong(40)).isEqualTo(NtpResponder.ntpTime(1_000_050_000L));
  }

  @Test
  void onlyAClientsRequestIsAnswered() {
    ByteBuffer server = request(1);
    server.put(0, (byte) ((4 << 3) | 4));
    ByteBuffer noVersion = request(1);
    noVersion.put(0, (byte) 3);
    for (ByteBuffer request : new ByteBuffer[] {server, noVersion, ByteBuffer.allocate(47)}) {
      assertThat(NtpResponder.reply(request, 0, 0, ByteBuffer.allocate(48))).isFalse();
    }
  }

  @Test
  void itAnswersOverUdp() throws IOException {
    try (NtpResponder responder = NtpResponder.start(0, () -> 42_000_000_000L);
        DatagramChannel client = DatagramChannel.open()) {
      client.connect(new InetSocketAddress("127.0.0.1", responder.port()));
      client.write(request(7));
      ByteBuffer reply = ByteBuffer.allocate(64);
      client.read(reply);
      reply.flip();
      assertThat(reply.remaining()).isEqualTo(48);
      assertThat(reply.getLong(24)).isEqualTo(7);
      assertThat(reply.getLong(40)).isEqualTo(NtpResponder.ntpTime(42_000_000_000L));
      assertThat(responder.answered()).isEqualTo(1);
    }
  }

  /**
   * Real chrony asks the responder once (its query-only mode, which never sets a clock) and says
   * how far this computer's clock is from the time it was given. Runs where VISOR_CHRONYD names a
   * chronyd: it's not in CI.
   */
  @Test
  void chronyAcceptsItsAnswers(@TempDir Path dir) throws Exception {
    String chronyd = System.getProperty("visor.chronyd");
    assumeTrue(chronyd != null, "VISOR_CHRONYD names no chronyd");
    // The robot's clock, made two seconds ahead of this computer's wall clock.
    long aheadNanos = 2_000_000_000L;
    try (NtpResponder responder =
        NtpResponder.start(
            0,
            () -> {
              Instant now = Instant.now();
              long wall = now.getEpochSecond() * 1_000_000_000L + now.getNano();
              return wall - NtpResponder.EPOCH_NANOS + aheadNanos;
            })) {
      Path config = dir.resolve("chrony.conf");
      Files.writeString(
          config,
          "server 127.0.0.1 port "
              + responder.port()
              + " iburst minpoll -3\nuser "
              + System.getProperty("user.name")
              + "\n");
      // -Q: query only, never touching the clock.
      Process chrony =
          new ProcessBuilder(chronyd, "-Q", "-n", "-f", config.toString())
              .redirectErrorStream(true)
              .start();
      assertThat(chrony.waitFor(20, TimeUnit.SECONDS)).as("chronyd -Q finishes").isTrue();
      String output = new String(chrony.getInputStream().readAllBytes(), StandardCharsets.UTF_8);
      Matcher wrong = Pattern.compile("wrong by (-?[0-9.]+) seconds").matcher(output);
      assertThat(wrong.find()).as(output).isTrue();
      // chrony says how far this clock is behind the source: the two seconds, to a millisecond.
      assertThat(Math.abs(Double.parseDouble(wrong.group(1)))).isCloseTo(2.0, within(0.001));
    }
  }
}

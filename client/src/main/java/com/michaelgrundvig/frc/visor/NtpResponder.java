package com.michaelgrundvig.frc.visor;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.ClosedChannelException;
import java.nio.channels.DatagramChannel;
import java.util.function.LongSupplier;

/**
 * Answers NTP requests with the robot's clock, so chrony on each coprocessor keeps the
 * coprocessor's clock on the robot's, and Visor can stamp frames with robot time.
 *
 * <p>The time it gives is the robot's clock plus {@link #EPOCH_NANOS} (2026-01-01T00:00:00Z), as
 * nanoseconds since 1970: so the coprocessor's wall clock becomes robot time plus that, and Visor
 * subtracts it again. On the robot, the clock is {@code RobotController::getTime}.
 *
 * <p>It's the least an NTP server can be: chrony on the coprocessor does all the filtering and
 * drift tracking. One thread, its own UDP port; a request gets the robot's time at its arrival and
 * at the reply's sending.
 */
public final class NtpResponder implements AutoCloseable {
  /** The port chrony asks on, by default. */
  public static final int DEFAULT_PORT = 5806;

  /** What the robot adds to its clock: 2026-01-01T00:00:00Z, in nanoseconds since 1970. */
  public static final long EPOCH_NANOS = 1_767_225_600L * 1_000_000_000L;

  /** NTP counts seconds from 1900; Unix time from 1970. */
  private static final long NTP_TO_UNIX_SECONDS = 2_208_988_800L;

  static final int PACKET_BYTES = 48;
  private static final int MODE_CLIENT = 3;
  private static final int MODE_SERVER = 4;

  /** About a microsecond: 2^-20 s. */
  private static final byte PRECISION = -20;

  private static final int REFERENCE_ID = 0x56495352; // "VISR"

  private final DatagramChannel channel;
  private final Thread thread;
  private volatile long answered;
  private volatile long failures;

  private NtpResponder(DatagramChannel channel, LongSupplier robotNanos) {
    this.channel = channel;
    thread = new Thread(() -> serve(robotNanos), "visor-ntp");
    thread.setDaemon(true);
    thread.start();
  }

  /**
   * Answers on a UDP port (0 for any free one), on a thread of its own.
   *
   * @param robotNanos the robot's clock, in nanoseconds: {@code RobotController::getTime}
   */
  public static NtpResponder start(int port, LongSupplier robotNanos) {
    try {
      DatagramChannel channel = DatagramChannel.open();
      channel.bind(new InetSocketAddress(port));
      return new NtpResponder(channel, robotNanos);
    } catch (IOException e) {
      throw new UncheckedIOException("opening UDP port " + port + " for NTP", e);
    }
  }

  /** The port it answers on. */
  public int port() {
    try {
      return ((InetSocketAddress) channel.getLocalAddress()).getPort();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  /** Requests answered. */
  public long answered() {
    return answered;
  }

  /** Requests left unanswered because the socket failed or the robot's clock threw. */
  public long failures() {
    return failures;
  }

  @Override
  public void close() {
    try {
      channel.close();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  private void serve(LongSupplier robotNanos) {
    ByteBuffer request = ByteBuffer.allocateDirect(512).order(ByteOrder.BIG_ENDIAN);
    ByteBuffer reply = ByteBuffer.allocateDirect(PACKET_BYTES).order(ByteOrder.BIG_ENDIAN);
    RepeatedErrors errors = new RepeatedErrors();
    while (channel.isOpen()) {
      try {
        if (answer(robotNanos, request, reply)) {
          errors.succeeded();
        }
      } catch (ClosedChannelException e) {
        return;
      } catch (IOException | RuntimeException e) {
        // The socket failed, or the robot's clock threw: this request goes unanswered, and chrony
        // asks again. Once the errors keep coming, each waits.
        failures++;
        errors.failed();
      }
    }
  }

  /**
   * Answers the next request. False for one that gets no answer: it isn't a client's request.
   *
   * @throws IOException if the socket fails
   * @throws RuntimeException if the robot's clock does
   */
  private boolean answer(LongSupplier robotNanos, ByteBuffer request, ByteBuffer reply)
      throws IOException {
    request.clear();
    SocketAddress from = channel.receive(request);
    long received = robotNanos.getAsLong();
    request.flip();
    if (from == null || !reply(request, received, robotNanos.getAsLong(), reply)) {
      return false;
    }
    channel.send(reply, from);
    answered++;
    return true;
  }

  /**
   * Writes the answer to a request into {@code reply} (position 0 to 48, ready to send), from the
   * robot's time when it arrived and when the answer leaves. False for anything that isn't a
   * client's request, which gets no answer.
   */
  static boolean reply(
      ByteBuffer request, long receivedNanos, long sendingNanos, ByteBuffer reply) {
    int start = request.position();
    if (request.remaining() < PACKET_BYTES) {
      return false;
    }
    int first = request.get(start);
    int version = (first >> 3) & 0x7;
    if ((first & 0x7) != MODE_CLIENT || version < 1) {
      return false;
    }
    reply.clear();
    // No leap second warning, the request's version, a server's answer.
    reply.put(0, (byte) ((version << 3) | MODE_SERVER));
    // Stratum 1: a primary source (chrony refuses 0 and 16).
    reply.put(1, (byte) 1);
    reply.put(2, request.get(start + 2));
    reply.put(3, PRECISION);
    reply.putInt(4, 0); // root delay
    reply.putInt(8, 0); // root dispersion
    reply.putInt(12, REFERENCE_ID);
    long received = ntpTime(receivedNanos);
    reply.putLong(16, received); // reference time: the clock is always "just set"
    reply.putLong(24, request.getLong(start + 40)); // the request's own transmit time, as sent
    reply.putLong(32, received);
    reply.putLong(40, ntpTime(sendingNanos));
    reply.position(0).limit(PACKET_BYTES);
    return true;
  }

  /** The robot's time as NTP writes it: seconds since 1900, then the fraction in 2^-32 s. */
  static long ntpTime(long robotNanos) {
    long unixNanos = robotNanos + EPOCH_NANOS;
    long seconds = Math.floorDiv(unixNanos, 1_000_000_000L) + NTP_TO_UNIX_SECONDS;
    long nanos = Math.floorMod(unixNanos, 1_000_000_000L);
    long fraction = (nanos << 32) / 1_000_000_000L;
    return (seconds << 32) | fraction;
  }
}

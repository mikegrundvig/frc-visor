package com.michaelgrundvig.frc.visor;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.net.StandardSocketOptions;
import java.nio.ByteBuffer;
import java.nio.channels.ClosedChannelException;
import java.nio.channels.DatagramChannel;
import java.util.Arrays;
import org.jspecify.annotations.Nullable;

/**
 * Receives Visor's frames and hands them to the robot loop.
 *
 * <p>{@link #listen} opens a UDP port and a thread of its own, so a slow loop never leaves
 * datagrams waiting in the socket. The thread decodes each one straight into a slot of a fixed
 * ring, and the loop takes them with {@link #poll}, oldest first, as many as arrived since it last
 * asked:
 *
 * <pre>{@code
 * VisorFrame frame = new VisorFrame(); // once
 * while (receiver.poll(frame)) {
 *   // use frame before the next poll
 * }
 * }</pre>
 *
 * <p>Nothing allocates once running. Frames from another Visor version (a coprocessor a deploy
 * missed) and datagrams that aren't frames are counted and dropped; frames from an unsynced clock
 * are counted and delivered, their flag says so.
 *
 * <p>{@link #inMemory} makes one with no socket and no thread, fed by {@link #offer}: a simulation
 * can send frames through exactly the path real ones take.
 */
public final class VisorReceiver implements AutoCloseable {
  /** The port Visor sends to by default. */
  public static final int DEFAULT_PORT = 5807;

  /**
   * The most frames that wait for the loop, past which new ones are dropped: about a quarter second
   * of four cameras at 60 fps.
   */
  public static final int RING_FRAMES = 64;

  private final FrameRing ring = new FrameRing(RING_FRAMES);
  private final @Nullable DatagramChannel channel;
  private final @Nullable Thread thread;
  // Each camera's last sequence number, or -1 before its first frame. The writer's alone.
  private final long[] lastSequence = new long[256];
  // For offer(VisorFrame): the frame's bytes, before they're decoded again.
  private final ByteBuffer encoded = ByteBuffer.allocate(VisorWire.MAX_BYTES);

  // Counts, each written by the writer alone and read by anyone.
  private volatile long frames;
  private volatile long malformed;
  private volatile long otherVersion;
  private volatile int lastOtherVersionId;
  private volatile int lastOtherVersionCamera = -1;
  private volatile long dropped;
  private volatile long gaps;
  private volatile long restarts;
  private volatile long unsynced;

  private VisorReceiver(@Nullable DatagramChannel channel) {
    Arrays.fill(lastSequence, -1);
    this.channel = channel;
    if (channel == null) {
      thread = null;
      return;
    }
    Thread receiving = new Thread(() -> receive(channel), "visor-receiver");
    receiving.setDaemon(true);
    thread = receiving;
    receiving.start();
  }

  /** Receives frames on a UDP port (0 for any free one), on a thread of its own. */
  public static VisorReceiver listen(int port) {
    try {
      DatagramChannel channel = DatagramChannel.open();
      channel.setOption(StandardSocketOptions.SO_REUSEADDR, true);
      // Room for a burst: about a second of four cameras' largest frames.
      channel.setOption(StandardSocketOptions.SO_RCVBUF, 256 * VisorWire.MAX_BYTES);
      channel.bind(new InetSocketAddress(port));
      return new VisorReceiver(channel);
    } catch (IOException e) {
      throw new UncheckedIOException("opening UDP port " + port + " for Visor", e);
    }
  }

  /** A receiver with no socket, fed by {@link #offer}: for a simulation, or a test. */
  public static VisorReceiver inMemory() {
    return new VisorReceiver(null);
  }

  /** The port it listens on, or -1 for one in memory. */
  public int port() {
    if (channel == null) {
      return -1;
    }
    try {
      return ((InetSocketAddress) channel.getLocalAddress()).getPort();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  /**
   * Copies the oldest frame not yet taken into {@code into}.
   *
   * @return false when there's none
   */
  public boolean poll(VisorFrame into) {
    return ring.poll(into);
  }

  /**
   * Takes a datagram's bytes, from its position to its limit, as if it had arrived on the port. For
   * a receiver {@link #inMemory}; one thread only.
   */
  public void offer(ByteBuffer datagram) {
    requireInMemory();
    accept(datagram, null);
  }

  /**
   * Takes a frame as if Visor had sent it: it's encoded and decoded again, exactly as real ones
   * are. For a receiver {@link #inMemory}; one thread only.
   */
  public void offer(VisorFrame frame) {
    requireInMemory();
    encoded.clear();
    VisorWire.encode(frame, encoded);
    encoded.flip();
    accept(encoded, null);
  }

  /** Frames delivered to the ring. */
  public long frames() {
    return frames;
  }

  /** Datagrams that weren't Visor frames. */
  public long malformed() {
    return malformed;
  }

  /** Frames dropped because they came from another Visor version. */
  public long otherVersion() {
    return otherVersion;
  }

  /** The last other version ID seen, or 0 for none: what the stale coprocessor runs. */
  public int lastOtherVersionId() {
    return lastOtherVersionId;
  }

  /**
   * The camera index the last other-version frame named, or -1 for none: which of the robot's
   * cameras the stale coprocessor runs.
   */
  public int lastOtherVersionCamera() {
    return lastOtherVersionCamera;
  }

  /** Frames dropped because the loop fell behind and the ring was full. */
  public long dropped() {
    return dropped;
  }

  /**
   * Frames missing between ones that arrived: lost on the network, or dropped on the coprocessor.
   */
  public long gaps() {
    return gaps;
  }

  /** How often a camera's sequence started again: Visor restarted. */
  public long restarts() {
    return restarts;
  }

  /** Frames delivered whose coprocessor clock wasn't synced to the robot's. */
  public long unsynced() {
    return unsynced;
  }

  /** The receive thread, for tests that measure it. */
  @Nullable Thread thread() {
    return thread;
  }

  @Override
  public void close() {
    if (channel == null) {
      return;
    }
    try {
      // Closing wakes the thread from its receive, and it ends.
      channel.close();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  private void receive(DatagramChannel channel) {
    ByteBuffer buffer = ByteBuffer.allocateDirect(VisorWire.MAX_BYTES + 1);
    while (channel.isOpen()) {
      buffer.clear();
      SocketAddress from;
      try {
        from = channel.receive(buffer);
      } catch (ClosedChannelException e) {
        return;
      } catch (IOException e) {
        // A datagram that couldn't be read is one malformed frame; the socket goes on.
        malformed++;
        continue;
      }
      buffer.flip();
      // The channel gives the same address object while the sender is the same: no allocation.
      accept(buffer, from instanceof InetSocketAddress inet ? inet.getAddress() : null);
    }
  }

  /**
   * Decodes one datagram into the ring, with the address it came from, and counts what it was. The
   * writer's alone.
   */
  private void accept(ByteBuffer datagram, @Nullable InetAddress sender) {
    VisorFrame slot = ring.claim();
    if (slot == null) {
      dropped++;
      return;
    }
    if (VisorWire.decode(datagram, slot) != VisorWire.Decoded.FRAME) {
      malformed++;
      return;
    }
    slot.sender = sender;
    if (slot.versionId != VisorVersion.ID) {
      lastOtherVersionId = slot.versionId;
      lastOtherVersionCamera = slot.camera;
      otherVersion++;
      return;
    }
    long last = lastSequence[slot.camera];
    long sequence = slot.sequence;
    if (last >= 0 && sequence > last + 1) {
      gaps += sequence - last - 1;
    } else if (last >= 0 && sequence <= last) {
      restarts++;
    }
    lastSequence[slot.camera] = sequence;
    if (!slot.clockSynced) {
      unsynced++;
    }
    frames++;
    ring.publish();
  }

  private void requireInMemory() {
    if (channel != null) {
      throw new IllegalStateException("this receiver listens on a port: offer is for inMemory()");
    }
  }
}

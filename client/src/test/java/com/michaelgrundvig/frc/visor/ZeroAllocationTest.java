package com.michaelgrundvig.frc.visor;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import org.junit.jupiter.api.Test;

/**
 * The client runs beside the robot's 20 ms loop, so once running it allocates nothing: garbage it
 * made would be garbage the loop pays for.
 */
class ZeroAllocationTest {
  private static final com.sun.management.ThreadMXBean THREADS =
      (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();

  @Test
  void theLoopsSideAllocatesNothing() throws IOException {
    VisorReceiver receiver = VisorReceiver.inMemory();
    VisorFrame sent = VisorWireTest.built(0, 1);
    ByteBuffer golden = VisorWireTest.golden();
    VisorFrame frame = new VisorFrame();
    CameraToTag pose = new CameraToTag();
    // Warm up first: the first runs load classes and compile.
    run(receiver, sent, golden, frame, pose, 20_000);
    long before = THREADS.getCurrentThreadAllocatedBytes();
    run(receiver, sent, golden, frame, pose, 20_000);
    long allocated = THREADS.getCurrentThreadAllocatedBytes() - before;
    // Per frame: a few hundred bytes once (the coverage agent's probes, the compiler's work) can
    // show up in a run, but anything allocated per frame would be at least 16 bytes 20,000 times.
    assertThat(allocated / 20_000).as("bytes allocated per frame (%d in all)", allocated).isZero();
  }

  /** What a robot loop and a simulation do with each frame. */
  private static void run(
      VisorReceiver receiver,
      VisorFrame sent,
      ByteBuffer golden,
      VisorFrame frame,
      CameraToTag pose,
      int times) {
    double sum = 0;
    for (int i = 0; i < times; i++) {
      sent.sequence = i;
      receiver.offer(sent);
      while (receiver.poll(frame)) {
        for (int tag = 0; tag < frame.tagCount(); tag++) {
          if (frame.posesSolved(tag)) {
            frame.cameraToTag(tag, Candidate.BEST, pose);
            sum += pose.x() + frame.ambiguity(tag) + frame.cornerX(tag, 2);
          }
        }
      }
      VisorWire.decode(golden, frame);
    }
    assertThat(sum).isPositive();
  }

  @Test
  void theReceiveThreadAllocatesNothingPerFrame() throws IOException, InterruptedException {
    try (VisorReceiver receiver = VisorReceiver.listen(0);
        DatagramChannel visor = DatagramChannel.open()) {
      visor.connect(new InetSocketAddress("127.0.0.1", receiver.port()));
      ByteBuffer bytes = ByteBuffer.allocate(VisorWire.MAX_BYTES);
      VisorFrame frame = new VisorFrame();
      send(visor, bytes, receiver, frame, 0, 2_000);
      long thread = java.util.Objects.requireNonNull(receiver.thread()).threadId();
      long before = THREADS.getThreadAllocatedBytes(thread);
      send(visor, bytes, receiver, frame, 2_000, 2_000);
      long perFrame = (THREADS.getThreadAllocatedBytes(thread) - before) / 2_000;
      assertThat(perFrame).as("bytes the receive thread allocates per frame").isZero();
    }
  }

  /** Sends frames one at a time, each taken before the next goes. */
  private static void send(
      DatagramChannel visor,
      ByteBuffer bytes,
      VisorReceiver receiver,
      VisorFrame frame,
      long first,
      int count)
      throws IOException, InterruptedException {
    for (long sequence = first; sequence < first + count; sequence++) {
      bytes.clear();
      VisorWire.encode(VisorWireTest.built(sequence, 0), bytes);
      bytes.flip();
      visor.write(bytes);
      assertThat(VisorReceiverTest.pollWithin(receiver, frame)).isTrue();
    }
  }
}

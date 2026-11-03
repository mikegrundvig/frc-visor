package com.michaelgrundvig.frc.visor;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class VisorReceiverTest {
  /** Waits up to five seconds for a frame. */
  static boolean pollWithin(VisorReceiver receiver, VisorFrame into) throws InterruptedException {
    long deadline = System.nanoTime() + 5_000_000_000L;
    while (System.nanoTime() < deadline) {
      if (receiver.poll(into)) {
        return true;
      }
      Thread.sleep(1);
    }
    return false;
  }

  @Test
  void framesArriveOverUdpOldestFirst() throws IOException, InterruptedException {
    try (VisorReceiver receiver = VisorReceiver.listen(0);
        DatagramChannel visor = DatagramChannel.open()) {
      InetSocketAddress robot = new InetSocketAddress("127.0.0.1", receiver.port());
      ByteBuffer bytes = ByteBuffer.allocate(VisorWire.MAX_BYTES);
      for (long sequence = 0; sequence < 3; sequence++) {
        bytes.clear();
        VisorWire.encode(VisorWireTest.built(sequence, 2), bytes);
        bytes.flip();
        visor.send(bytes, robot);
      }
      VisorFrame frame = new VisorFrame();
      for (long sequence = 0; sequence < 3; sequence++) {
        assertThat(pollWithin(receiver, frame)).as("frame %d arrives", sequence).isTrue();
        VisorWireTest.assertSame(frame, VisorWireTest.built(sequence, 2));
      }
      assertThat(receiver.poll(frame)).isFalse();
      assertThat(receiver.frames()).isEqualTo(3);
    }
  }

  @Test
  void closingStopsTheThread() throws InterruptedException {
    VisorReceiver receiver = VisorReceiver.listen(0);
    Thread thread = java.util.Objects.requireNonNull(receiver.thread());
    receiver.close();
    thread.join(5_000);
    assertThat(thread.isAlive()).isFalse();
  }

  /** What arrives, in order: a frame's camera, sequence and version ID, and its clock. */
  private record Sent(int camera, long sequence, int versionId, boolean synced) {
    static Sent frame(int camera, long sequence) {
      return new Sent(camera, sequence, VisorVersion.ID, true);
    }
  }

  /** What the counts should be after. */
  private record Counts(
      long frames, long gaps, long restarts, long otherVersion, long unsynced, long delivered) {}

  private record Scenario(String name, List<Sent> sent, Counts counts) {}

  @Test
  void whatArrivesIsCountedAndOnlyTheRobotsVersionIsDelivered() {
    List<Scenario> scenarios =
        List.of(
            new Scenario(
                "in order, two cameras",
                List.of(Sent.frame(0, 0), Sent.frame(1, 0), Sent.frame(0, 1), Sent.frame(1, 1)),
                new Counts(4, 0, 0, 0, 0, 4)),
            new Scenario(
                "two frames lost on one camera",
                List.of(Sent.frame(0, 10), Sent.frame(0, 13)),
                new Counts(2, 2, 0, 0, 0, 2)),
            new Scenario(
                "Visor restarted",
                List.of(Sent.frame(0, 500), Sent.frame(0, 0), Sent.frame(0, 1)),
                new Counts(3, 0, 1, 0, 0, 3)),
            new Scenario(
                "a coprocessor the deploy missed",
                List.of(Sent.frame(0, 0), new Sent(1, 0, 20250101, true)),
                new Counts(1, 0, 0, 1, 0, 1)),
            new Scenario(
                "a clock not synced yet",
                List.of(new Sent(0, 0, VisorVersion.ID, false), Sent.frame(0, 1)),
                new Counts(2, 0, 0, 0, 1, 2)));
    for (Scenario scenario : scenarios) {
      VisorReceiver receiver = VisorReceiver.inMemory();
      for (Sent sent : scenario.sent()) {
        VisorFrame frame = VisorWireTest.built(sent.sequence(), sent.camera());
        frame.setHeader(sent.sequence(), 1600, 1304, 0, sent.camera(), sent.synced(), false);
        frame.setVersionId(sent.versionId());
        receiver.offer(frame);
      }
      List<Long> delivered = new ArrayList<>();
      VisorFrame frame = new VisorFrame();
      while (receiver.poll(frame)) {
        delivered.add(frame.sequence());
      }
      Counts counts =
          new Counts(
              receiver.frames(),
              receiver.gaps(),
              receiver.restarts(),
              receiver.otherVersion(),
              receiver.unsynced(),
              delivered.size());
      assertThat(counts).as(scenario.name()).isEqualTo(scenario.counts());
    }
  }

  @Test
  void theOtherVersionIsRemembered() {
    VisorReceiver receiver = VisorReceiver.inMemory();
    VisorFrame stale = VisorWireTest.built(0, 0);
    stale.setVersionId(20251225);
    receiver.offer(stale);
    assertThat(receiver.lastOtherVersionId()).isEqualTo(20251225);
  }

  @Test
  void garbageIsCountedNotDelivered() {
    VisorReceiver receiver = VisorReceiver.inMemory();
    receiver.offer(ByteBuffer.wrap(new byte[] {1, 2, 3}));
    receiver.offer(ByteBuffer.allocate(VisorWire.HEADER_BYTES));
    assertThat(receiver.malformed()).isEqualTo(2);
    assertThat(receiver.poll(new VisorFrame())).isFalse();
  }

  @Test
  void aLoopThatFallsBehindLosesTheNewestFrames() {
    VisorReceiver receiver = VisorReceiver.inMemory();
    for (long sequence = 0; sequence < 70; sequence++) {
      receiver.offer(VisorWireTest.built(sequence, 0));
    }
    assertThat(receiver.frames()).isEqualTo(64);
    assertThat(receiver.dropped()).isEqualTo(6);
    VisorFrame frame = new VisorFrame();
    assertThat(receiver.poll(frame)).isTrue();
    assertThat(frame.sequence()).as("the oldest kept").isZero();
    // Room again: the next one is taken.
    receiver.offer(VisorWireTest.built(70, 0));
    assertThat(receiver.frames()).isEqualTo(65);
  }

  @Test
  void onlyAReceiverInMemoryTakesOffers() {
    try (VisorReceiver receiver = VisorReceiver.listen(0)) {
      assertThatThrownBy(() -> receiver.offer(new VisorFrame()))
          .isInstanceOf(IllegalStateException.class)
          .hasMessageContaining("inMemory");
      assertThat(VisorReceiver.inMemory().port()).isEqualTo(-1);
    }
  }
}

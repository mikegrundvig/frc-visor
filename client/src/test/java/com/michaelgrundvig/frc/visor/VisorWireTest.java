package com.michaelgrundvig.frc.visor;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.io.IOException;
import java.nio.BufferOverflowException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;

class VisorWireTest {
  /** The frame Visor's own tests encode, byte for byte (testdata/golden-frame.bin). */
  static ByteBuffer golden() throws IOException {
    Path file = Path.of(System.getProperty("visor.testdata"), "golden-frame.bin");
    return ByteBuffer.wrap(Files.readAllBytes(file));
  }

  /** A frame with one tag solved and one not, built as a simulation would. */
  static VisorFrame built(long sequence, int camera) {
    VisorFrame frame = new VisorFrame();
    frame.setHeader(sequence, 1600, 1304, 987_654_321L, camera, true, false);
    int solved = frame.addTag(21, 1, 88.5f, new float[] {10, 20, 30, 21, 29, 31, 9, 30});
    frame.setPoses(
        solved,
        new float[] {3.0f, 0.1f, -0.2f},
        new float[] {0.5f, -0.25f, 4.0f},
        0.2f,
        new float[] {-2.9f, 0.3f, 0.1f},
        new float[] {0.5f, -0.25f, 3.9f},
        0.8f);
    frame.addTag(400, 0, 12f, new float[] {1, 2, 3, 4, 5, 6, 7, 8});
    frame.setExposure(1300);
    frame.setGain(55);
    frame.setBrightness(41, 187, 0.0042);
    frame.setHealth(3_900, 4_294_967_295L);
    return frame;
  }

  @Test
  void theGoldenFileDecodesFieldByField() throws IOException {
    VisorFrame frame = new VisorFrame();
    assertThat(VisorWire.decode(golden(), frame)).isEqualTo(VisorWire.Decoded.FRAME);

    assertThat(frame.versionId()).isEqualTo(20261102);
    assertThat(frame.sequence()).isEqualTo(4_000_000_001L);
    assertThat(frame.width()).isEqualTo(1600);
    assertThat(frame.height()).isEqualTo(1304);
    assertThat(frame.captureNanos()).isEqualTo(1_234_567_890_123L);
    assertThat(frame.camera()).isEqualTo(3);
    assertThat(frame.clockSynced()).isTrue();
    assertThat(frame.dequeueTime()).isFalse();
    assertThat(frame.exposureKnown()).isTrue();
    assertThat(frame.exposureMicros()).isEqualTo(1000);
    assertThat(frame.gainKnown()).isTrue();
    assertThat(frame.gain()).isEqualTo(40);
    assertThat(frame.autoExposure()).isFalse();
    assertThat(frame.medianBrightness()).isEqualTo(37);
    assertThat(frame.highBrightness()).isEqualTo(201);
    assertThat(frame.clippedShare()).isEqualTo(0.0125);
    assertThat(frame.detectMicros()).isEqualTo(4250);
    assertThat(frame.dropped()).isEqualTo(17);
    assertThat(frame.tagCount()).isEqualTo(2);

    assertThat(frame.tagId(0)).isEqualTo(7);
    assertThat(frame.hamming(0)).isZero();
    assertThat(frame.decisionMargin(0)).isEqualTo(61.25f);
    assertThat(corners(frame, 0))
        .containsExactly(100.5f, 200.25f, 180, 201, 179.75f, 280.5f, 99, 279);
    assertThat(frame.posesSolved(0)).isTrue();
    assertThat(pose(frame, 0, Candidate.BEST))
        .containsExactly(0.125f, -2.5f, 0.0625f, -0.5f, 0.25f, 3.75f);
    assertThat(pose(frame, 0, Candidate.OTHER))
        .containsExactly(-0.25f, 2, -0.5f, -0.5f, 0.25f, 3.5f);
    assertThat(frame.error(0, Candidate.BEST)).isEqualTo(0.1875f);
    assertThat(frame.error(0, Candidate.OTHER)).isEqualTo(1.5f);
    assertThat(frame.ambiguity(0)).isEqualTo(0.125);

    assertThat(frame.tagId(1)).isEqualTo(586);
    assertThat(frame.hamming(1)).isEqualTo(2);
    assertThat(frame.decisionMargin(1)).isEqualTo(12.5f);
    assertThat(corners(frame, 1))
        .containsExactly(1500.125f, 1200, 1580, 1201.5f, 1579, 1290.25f, 1499.5f, 1289);
    assertThat(frame.posesSolved(1)).isFalse();
  }

  @Test
  void theGoldenFileEncodesBackExactly() throws IOException {
    ByteBuffer golden = golden();
    VisorFrame frame = new VisorFrame();
    VisorWire.decode(golden, frame);
    ByteBuffer out = ByteBuffer.allocate(VisorWire.MAX_BYTES);
    VisorWire.encode(frame, out);
    out.flip();
    assertThat(out).isEqualTo(golden);
  }

  @Test
  void framesSurviveARoundTripWhateverTheBufferIsLike() {
    // Byte order and a starting position the frame doesn't begin at: neither may matter.
    for (ByteOrder order : List.of(ByteOrder.BIG_ENDIAN, ByteOrder.LITTLE_ENDIAN)) {
      for (int offset : new int[] {0, 7}) {
        VisorFrame sent = built(4_294_967_295L, 255);
        ByteBuffer buffer = ByteBuffer.allocate(VisorWire.MAX_BYTES + offset).order(order);
        buffer.position(offset);
        VisorWire.encode(sent, buffer);
        assertThat(buffer.position())
            .isEqualTo(offset + VisorWire.HEADER_BYTES + 2 * VisorWire.TAG_BYTES);
        buffer.flip().position(offset);

        VisorFrame received = new VisorFrame();
        assertThat(VisorWire.decode(buffer, received)).isEqualTo(VisorWire.Decoded.FRAME);
        assertThat(buffer.position()).as("decode leaves the buffer as it was").isEqualTo(offset);
        assertSame(received, sent);
      }
    }
  }

  /** The camera choosing its own exposure, with no gain control: neither is known. */
  @Test
  void settingsTheCameraDoesntReportArriveUnknown() {
    VisorFrame sent = built(1, 0);
    sent.setHeader(1, 1600, 1304, 5, 0, true, false);
    sent.setAutoExposure();
    sent.setBrightness(255, 255, 1);
    ByteBuffer buffer = ByteBuffer.allocate(VisorWire.MAX_BYTES);
    VisorWire.encode(sent, buffer);
    buffer.flip();

    VisorFrame received = built(9, 9);
    VisorWire.decode(buffer, received);
    assertThat(received.autoExposure()).isTrue();
    assertThat(received.exposureKnown()).isFalse();
    assertThat(received.exposureMicros()).isZero();
    assertThat(received.gainKnown()).isFalse();
    assertThat(received.clippedShare()).isEqualTo(1.0);
    assertThatThrownBy(() -> sent.setBrightness(256, 0, 0)).hasMessageContaining("0 to 255");
    assertThatThrownBy(() -> sent.setBrightness(0, 0, 1.5)).hasMessageContaining("0 to 1");
    assertThatThrownBy(() -> sent.setExposure(0)).hasMessageContaining("microseconds");
    assertThatThrownBy(() -> sent.setHealth(-1, 0)).hasMessageContaining("microseconds");
    assertThatThrownBy(() -> sent.setHealth(0, 1L << 32)).hasMessageContaining("2^32");
  }

  @Test
  void aFullFrameFitsOnePacket() {
    VisorFrame frame = new VisorFrame();
    frame.setHeader(0, 1600, 1304, 0, 0, true, false);
    for (int i = 0; i < VisorFrame.MAX_TAGS; i++) {
      frame.addTag(i, 0, 1, new float[8]);
    }
    ByteBuffer buffer = ByteBuffer.allocate(VisorWire.MAX_BYTES);
    VisorWire.encode(frame, buffer);
    assertThat(buffer.position()).isEqualTo(VisorWire.MAX_BYTES).isLessThanOrEqualTo(1472);
    assertThatThrownBy(() -> frame.addTag(99, 0, 1, new float[8]))
        .isInstanceOf(IllegalStateException.class)
        .hasMessageContaining("at most 14");
    assertThatThrownBy(() -> VisorWire.encode(frame, ByteBuffer.allocate(100)))
        .isInstanceOf(BufferOverflowException.class);
  }

  @Test
  void datagramsThatArentFramesAreNamed() throws IOException {
    byte[] good = golden().array();
    byte[] wrongMagic = good.clone();
    wrongMagic[0] ^= (byte) 0xff;
    byte[] tooManyTags = good.clone();
    tooManyTags[26] = 15;
    record Case(String name, byte[] bytes, VisorWire.Decoded expected) {}
    List<Case> cases =
        List.of(
            new Case("empty", new byte[0], VisorWire.Decoded.TOO_SHORT),
            new Case("header cut", Arrays.copyOf(good, 47), VisorWire.Decoded.TOO_SHORT),
            new Case("wrong magic", wrongMagic, VisorWire.Decoded.NOT_VISOR),
            new Case("too many tags", tooManyTags, VisorWire.Decoded.TOO_MANY_TAGS),
            new Case(
                "a tag cut off",
                Arrays.copyOf(good, good.length - 1),
                VisorWire.Decoded.WRONG_LENGTH),
            new Case(
                "a byte too many",
                Arrays.copyOf(good, good.length + 1),
                VisorWire.Decoded.WRONG_LENGTH));
    for (Case c : cases) {
      assertThat(VisorWire.decode(ByteBuffer.wrap(c.bytes()), new VisorFrame()))
          .as(c.name())
          .isEqualTo(c.expected());
    }
  }

  @Test
  void readingPastTheTagsIsRefused() throws IOException {
    VisorFrame frame = new VisorFrame();
    VisorWire.decode(golden(), frame);
    assertThatThrownBy(() -> frame.tagId(2)).isInstanceOf(IndexOutOfBoundsException.class);
    assertThatThrownBy(() -> frame.cornerX(0, 4)).isInstanceOf(IndexOutOfBoundsException.class);
    assertThatThrownBy(() -> frame.rvec(0, Candidate.BEST, 3))
        .isInstanceOf(IndexOutOfBoundsException.class);
    assertThatThrownBy(() -> frame.cameraToTag(1, Candidate.BEST, new CameraToTag()))
        .isInstanceOf(IllegalStateException.class)
        .hasMessageContaining("586");
  }

  static float[] corners(VisorFrame frame, int tag) {
    float[] values = new float[8];
    for (int corner = 0; corner < 4; corner++) {
      values[2 * corner] = frame.cornerX(tag, corner);
      values[2 * corner + 1] = frame.cornerY(tag, corner);
    }
    return values;
  }

  static float[] pose(VisorFrame frame, int tag, Candidate candidate) {
    float[] values = new float[6];
    for (int axis = 0; axis < 3; axis++) {
      values[axis] = frame.rvec(tag, candidate, axis);
      values[3 + axis] = frame.tvec(tag, candidate, axis);
    }
    return values;
  }

  /** Every field of two frames is equal. */
  static void assertSame(VisorFrame actual, VisorFrame expected) {
    assertThat(actual.versionId()).isEqualTo(expected.versionId());
    assertThat(actual.sequence()).isEqualTo(expected.sequence());
    assertThat(actual.width()).isEqualTo(expected.width());
    assertThat(actual.height()).isEqualTo(expected.height());
    assertThat(actual.captureNanos()).isEqualTo(expected.captureNanos());
    assertThat(actual.camera()).isEqualTo(expected.camera());
    assertThat(actual.clockSynced()).isEqualTo(expected.clockSynced());
    assertThat(actual.dequeueTime()).isEqualTo(expected.dequeueTime());
    assertThat(actual.exposureKnown()).isEqualTo(expected.exposureKnown());
    assertThat(actual.exposureMicros()).isEqualTo(expected.exposureMicros());
    assertThat(actual.gainKnown()).isEqualTo(expected.gainKnown());
    assertThat(actual.gain()).isEqualTo(expected.gain());
    assertThat(actual.autoExposure()).isEqualTo(expected.autoExposure());
    assertThat(actual.medianBrightness()).isEqualTo(expected.medianBrightness());
    assertThat(actual.highBrightness()).isEqualTo(expected.highBrightness());
    assertThat(actual.clippedShare()).isEqualTo(expected.clippedShare());
    assertThat(actual.detectMicros()).isEqualTo(expected.detectMicros());
    assertThat(actual.dropped()).isEqualTo(expected.dropped());
    assertThat(actual.tagCount()).isEqualTo(expected.tagCount());
    for (int tag = 0; tag < expected.tagCount(); tag++) {
      assertThat(actual.tagId(tag)).isEqualTo(expected.tagId(tag));
      assertThat(actual.hamming(tag)).isEqualTo(expected.hamming(tag));
      assertThat(actual.decisionMargin(tag)).isEqualTo(expected.decisionMargin(tag));
      assertThat(corners(actual, tag)).containsExactly(corners(expected, tag));
      assertThat(actual.posesSolved(tag)).isEqualTo(expected.posesSolved(tag));
      if (expected.posesSolved(tag)) {
        for (Candidate candidate : Candidate.values()) {
          assertThat(pose(actual, tag, candidate)).containsExactly(pose(expected, tag, candidate));
          assertThat(actual.error(tag, candidate)).isEqualTo(expected.error(tag, candidate));
        }
      }
    }
  }
}

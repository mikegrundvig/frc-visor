package com.michaelgrundvig.frc.visor;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Visor's datagram: one frame, little-endian, fixed sizes, no padding; a 48-byte header and 96
 * bytes per tag. The layout is the repository's docs/wire.md, and testdata/golden-frame.bin holds
 * one frame both sides' tests check. Decoding and encoding allocate nothing.
 */
public final class VisorWire {
  /** "VISR" in byte order. */
  public static final int MAGIC = 0x52534956;

  /** The header's size, in bytes. */
  public static final int HEADER_BYTES = 48;

  /** Each tag's size, in bytes. */
  public static final int TAG_BYTES = 96;

  /** The largest frame, in bytes: inside one Ethernet packet's UDP payload. */
  public static final int MAX_BYTES = HEADER_BYTES + VisorFrame.MAX_TAGS * TAG_BYTES;

  private static final int FLAG_CLOCK_SYNCED = 1;
  private static final int FLAG_DEQUEUE_TIME = 2;
  private static final int FLAG_EXPOSURE = 4;
  private static final int FLAG_GAIN = 8;
  private static final int FLAG_AUTO_EXPOSURE = 16;
  private static final int TAG_FLAG_POSES = 1;

  private VisorWire() {}

  /** What a datagram turned out to be. */
  public enum Decoded {
    /** A frame, now in the frame given. */
    FRAME,
    /** Shorter than a frame's header. */
    TOO_SHORT,
    /** Not Visor's: the magic number is wrong. */
    NOT_VISOR,
    /** More tags than a frame carries. */
    TOO_MANY_TAGS,
    /** Not the length its tag count says. */
    WRONG_LENGTH
  }

  /**
   * Reads a frame from a datagram's bytes, from the buffer's position to its limit, into {@code
   * into}. The buffer is left as it was. Anything but {@link Decoded#FRAME} leaves {@code into}
   * partly written.
   */
  public static Decoded decode(ByteBuffer datagram, VisorFrame into) {
    int start = datagram.position();
    int length = datagram.remaining();
    if (length < HEADER_BYTES) {
      return Decoded.TOO_SHORT;
    }
    if (i32(datagram, start) != MAGIC) {
      return Decoded.NOT_VISOR;
    }
    int tagCount = Byte.toUnsignedInt(datagram.get(start + 26));
    if (tagCount > VisorFrame.MAX_TAGS) {
      return Decoded.TOO_MANY_TAGS;
    }
    if (length != HEADER_BYTES + tagCount * TAG_BYTES) {
      return Decoded.WRONG_LENGTH;
    }
    // The bytes don't say where they came from: the receiver sets it.
    into.sender = null;
    into.versionId = i32(datagram, start + 4);
    into.sequence = Integer.toUnsignedLong(i32(datagram, start + 8));
    into.width = u16(datagram, start + 12);
    into.height = u16(datagram, start + 14);
    into.captureNanos = i64(datagram, start + 16);
    into.camera = Byte.toUnsignedInt(datagram.get(start + 24));
    int flags = datagram.get(start + 25);
    into.clockSynced = (flags & FLAG_CLOCK_SYNCED) != 0;
    into.dequeueTime = (flags & FLAG_DEQUEUE_TIME) != 0;
    into.exposureKnown = (flags & FLAG_EXPOSURE) != 0;
    into.gainKnown = (flags & FLAG_GAIN) != 0;
    into.autoExposure = (flags & FLAG_AUTO_EXPOSURE) != 0;
    into.exposureMicros = Integer.toUnsignedLong(i32(datagram, start + 28));
    into.gain = i32(datagram, start + 32);
    into.medianBrightness = Byte.toUnsignedInt(datagram.get(start + 36));
    into.highBrightness = Byte.toUnsignedInt(datagram.get(start + 37));
    into.clipped = u16(datagram, start + 38);
    into.detectMicros = Integer.toUnsignedLong(i32(datagram, start + 40));
    into.dropped = Integer.toUnsignedLong(i32(datagram, start + 44));
    into.tagCount = tagCount;
    for (int tag = 0; tag < tagCount; tag++) {
      int at = start + HEADER_BYTES + tag * TAG_BYTES;
      into.ids[tag] = u16(datagram, at);
      into.hamming[tag] = Byte.toUnsignedInt(datagram.get(at + 2));
      into.posesSolved[tag] = (datagram.get(at + 3) & TAG_FLAG_POSES) != 0;
      into.margins[tag] = f32(datagram, at + 4);
      for (int i = 0; i < VisorFrame.CORNER_VALUES; i++) {
        into.corners[tag * VisorFrame.CORNER_VALUES + i] = f32(datagram, at + 8 + 4 * i);
      }
      for (int i = 0; i < VisorFrame.POSE_VALUES; i++) {
        into.poses[tag * VisorFrame.POSE_VALUES + i] = f32(datagram, at + 40 + 4 * i);
      }
    }
    return Decoded.FRAME;
  }

  /**
   * Writes a frame's bytes into {@code out} from its position, and moves the position past them.
   * For a simulation feeding a receiver, and for tests.
   *
   * @throws java.nio.BufferOverflowException if {@code out} hasn't room
   */
  public static void encode(VisorFrame frame, ByteBuffer out) {
    int start = out.position();
    int length = HEADER_BYTES + frame.tagCount * TAG_BYTES;
    if (out.remaining() < length) {
      throw new java.nio.BufferOverflowException();
    }
    putI32(out, start, MAGIC);
    putI32(out, start + 4, frame.versionId);
    putI32(out, start + 8, (int) frame.sequence);
    putU16(out, start + 12, frame.width);
    putU16(out, start + 14, frame.height);
    putI64(out, start + 16, frame.captureNanos);
    out.put(start + 24, (byte) frame.camera);
    int flags = 0;
    if (frame.clockSynced) {
      flags |= FLAG_CLOCK_SYNCED;
    }
    if (frame.dequeueTime) {
      flags |= FLAG_DEQUEUE_TIME;
    }
    if (frame.exposureKnown) {
      flags |= FLAG_EXPOSURE;
    }
    if (frame.gainKnown) {
      flags |= FLAG_GAIN;
    }
    if (frame.autoExposure) {
      flags |= FLAG_AUTO_EXPOSURE;
    }
    out.put(start + 25, (byte) flags);
    out.put(start + 26, (byte) frame.tagCount);
    out.put(start + 27, (byte) 0);
    // Without its flag, a value's bytes are zero.
    putI32(out, start + 28, frame.exposureKnown ? (int) frame.exposureMicros : 0);
    putI32(out, start + 32, frame.gainKnown ? frame.gain : 0);
    out.put(start + 36, (byte) frame.medianBrightness);
    out.put(start + 37, (byte) frame.highBrightness);
    putU16(out, start + 38, frame.clipped);
    putI32(out, start + 40, (int) frame.detectMicros);
    putI32(out, start + 44, (int) frame.dropped);
    for (int tag = 0; tag < frame.tagCount; tag++) {
      int at = start + HEADER_BYTES + tag * TAG_BYTES;
      putU16(out, at, frame.ids[tag]);
      out.put(at + 2, (byte) frame.hamming[tag]);
      out.put(at + 3, (byte) (frame.posesSolved[tag] ? TAG_FLAG_POSES : 0));
      putF32(out, at + 4, frame.margins[tag]);
      for (int i = 0; i < VisorFrame.CORNER_VALUES; i++) {
        putF32(out, at + 8 + 4 * i, frame.corners[tag * VisorFrame.CORNER_VALUES + i]);
      }
      for (int i = 0; i < VisorFrame.POSE_VALUES; i++) {
        float value = frame.posesSolved[tag] ? frame.poses[tag * VisorFrame.POSE_VALUES + i] : 0f;
        putF32(out, at + 40 + 4 * i, value);
      }
    }
    out.position(start + length);
  }

  // Little-endian reads and writes at absolute positions, whatever the buffer's own byte order,
  // without changing the buffer or making a view of it (which would allocate).

  private static boolean little(ByteBuffer b) {
    return b.order() == ByteOrder.LITTLE_ENDIAN;
  }

  private static int u16(ByteBuffer b, int at) {
    short value = b.getShort(at);
    return Short.toUnsignedInt(little(b) ? value : Short.reverseBytes(value));
  }

  private static int i32(ByteBuffer b, int at) {
    int value = b.getInt(at);
    return little(b) ? value : Integer.reverseBytes(value);
  }

  private static long i64(ByteBuffer b, int at) {
    long value = b.getLong(at);
    return little(b) ? value : Long.reverseBytes(value);
  }

  private static float f32(ByteBuffer b, int at) {
    return Float.intBitsToFloat(i32(b, at));
  }

  private static void putU16(ByteBuffer b, int at, int value) {
    short bits = (short) value;
    b.putShort(at, little(b) ? bits : Short.reverseBytes(bits));
  }

  private static void putI32(ByteBuffer b, int at, int value) {
    b.putInt(at, little(b) ? value : Integer.reverseBytes(value));
  }

  private static void putI64(ByteBuffer b, int at, long value) {
    b.putLong(at, little(b) ? value : Long.reverseBytes(value));
  }

  private static void putF32(ByteBuffer b, int at, float value) {
    putI32(b, at, Float.floatToRawIntBits(value));
  }
}

package com.michaelgrundvig.frc.visor;

import java.util.concurrent.atomic.AtomicLong;
import org.jspecify.annotations.Nullable;

/**
 * A fixed ring of frames between one writer (the receive thread) and one reader (the robot loop),
 * with no locks and no allocation. The writer decodes straight into a free slot and then publishes
 * it; the reader copies the oldest published slot out and frees it. When the reader falls behind
 * and the ring is full, the writer drops new frames.
 */
final class FrameRing {
  private final VisorFrame[] slots;
  private final int mask;
  // The next slot to read, and the next to write. Each is written by one side only.
  private final AtomicLong head = new AtomicLong();
  private final AtomicLong tail = new AtomicLong();

  /** A ring of {@code capacity} frames, a power of two. */
  FrameRing(int capacity) {
    if (capacity < 1 || Integer.bitCount(capacity) != 1) {
      throw new IllegalArgumentException("a power of two: " + capacity);
    }
    slots = new VisorFrame[capacity];
    for (int i = 0; i < capacity; i++) {
      slots[i] = new VisorFrame();
    }
    mask = capacity - 1;
  }

  /** The writer's next slot to fill, or null when the ring is full. */
  @Nullable VisorFrame claim() {
    long next = tail.get();
    if (next - head.get() >= slots.length) {
      return null;
    }
    return slots[(int) (next & mask)];
  }

  /** Hands the claimed slot to the reader. */
  void publish() {
    // A release store: the slot's contents are visible to the reader before the new tail is.
    tail.lazySet(tail.get() + 1);
  }

  /** Copies the oldest published frame into {@code into} and frees its slot; false if none. */
  boolean poll(VisorFrame into) {
    long next = head.get();
    if (next >= tail.get()) {
      return false;
    }
    into.copyFrom(slots[(int) (next & mask)]);
    head.lazySet(next + 1);
    return true;
  }
}

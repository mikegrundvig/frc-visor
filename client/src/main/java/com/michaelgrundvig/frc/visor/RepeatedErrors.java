package com.michaelgrundvig.frc.visor;

/**
 * Paces a socket's thread through errors that keep coming: a socket that fails every call would
 * otherwise have its loop retry at once, forever, and burn a core the robot loop needs. The first
 * {@link #FREE} errors in a row cost nothing, as one bad datagram shouldn't slow the next; past
 * them, each waits {@link #FIRST_PAUSE_MILLIS}, doubling to {@link #MOST_PAUSE_MILLIS}. A success
 * starts the count again. One thread's own.
 */
final class RepeatedErrors {
  /** Errors in a row that cost no wait. */
  static final int FREE = 10;

  /** The first wait past them. */
  static final long FIRST_PAUSE_MILLIS = 10;

  /** The longest wait: a socket that recovers is back within a second. */
  static final long MOST_PAUSE_MILLIS = 1000;

  private int inARow;

  /** One more error: waits as long as {@link #pauseMillis} says for this many in a row. */
  void failed() {
    if (inARow < Integer.MAX_VALUE) {
      inARow++;
    }
    long pause = pauseMillis(inARow);
    if (pause == 0) {
      return;
    }
    try {
      Thread.sleep(pause);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  /** A call that worked: the count starts again. */
  void succeeded() {
    inARow = 0;
  }

  /** How long the {@code inARow}th error in a row waits, in milliseconds. */
  static long pauseMillis(int inARow) {
    if (inARow <= FREE) {
      return 0;
    }
    int doublings = Math.min(inARow - FREE - 1, 30);
    return Math.min(MOST_PAUSE_MILLIS, FIRST_PAUSE_MILLIS << doublings);
  }
}

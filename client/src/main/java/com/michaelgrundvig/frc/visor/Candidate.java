package com.michaelgrundvig.frc.visor;

/**
 * Which of a tag's two poses. A flat square seen by one camera fits two, mirror images about the
 * line of sight; Visor sends both, the one whose corners fit better first.
 */
public enum Candidate {
  /** The pose whose corners fit better. */
  BEST,
  /** The other. */
  OTHER
}

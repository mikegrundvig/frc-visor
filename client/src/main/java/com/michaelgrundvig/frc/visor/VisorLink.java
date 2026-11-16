package com.michaelgrundvig.frc.visor;

/**
 * A camera's link state, as every frame carries it in one byte (the repository's docs/wire.md,
 * offset 27): its USB link's speed class, the pixel format Visor runs it in, whether it looks able
 * to do better than its link, and whether Visor tried to recover it and it stayed slow. 0 is "not
 * known" throughout, so a frame from a Visor before this byte reads as unknown.
 *
 * <p>Bits 0 to 2 are the speed, 3 and 4 the format, 5 the headroom, 6 the recovery; bit 7 is 0.
 */
public final class VisorLink {
  private static final int SPEED_MASK = 0b111;
  private static final int FORMAT_SHIFT = 3;
  private static final int FORMAT_MASK = 0b11;
  private static final int HEADROOM = 1 << 5;
  private static final int RECOVERY = 1 << 6;

  private VisorLink() {}

  /** The USB link's speed class. */
  public enum Speed {
    /** Not known: not on USB, unreadable, or an older Visor. */
    UNKNOWN,
    /** USB 1's low or full speed, 12 Mb/s or less: far too slow for a full-size camera. */
    LOW_OR_FULL,
    /** USB 2's high speed, 480 Mb/s. */
    HIGH,
    /** USB 3's SuperSpeed, 5000 Mb/s. */
    SUPER,
    /** Faster than SuperSpeed. */
    FASTER
  }

  /** The pixel format Visor runs the camera in. */
  public enum Format {
    /** Not known: an older Visor. */
    UNKNOWN,
    /** 8-bit mono, the luma itself. */
    GREY,
    /** 4:2:2 colour; Visor takes the luma. */
    YUYV,
    /** Motion JPEG; Visor decodes the luma of each frame. */
    MJPG
  }

  /** Whether the camera looks able to do better than the link it's on. */
  public enum Headroom {
    /** At its best, or not known. */
    NONE_KNOWN,
    /** Below SuperSpeed while it reports USB 2.10 or later: a USB 3 camera on USB 2. */
    MAY_DO_BETTER
  }

  /** Whether Visor tried to bring the camera back to SuperSpeed since it last appeared. */
  public enum Recovery {
    /** Not tried, or tried and it worked. */
    NOT_TRIED,
    /**
     * A USB reset or a power cycle of its port was tried and left it below SuperSpeed: its USB 3
     * contacts are probably not connecting, which only reseating its plug or changing its cable
     * fixes. Visor tries nothing more until it's replugged or restarted.
     */
    TRIED_STILL_SLOW
  }

  /** The speed a link state byte says. */
  public static Speed speed(int link) {
    return switch (link & SPEED_MASK) {
      case 1 -> Speed.LOW_OR_FULL;
      case 2 -> Speed.HIGH;
      case 3 -> Speed.SUPER;
      case 4 -> Speed.FASTER;
      default -> Speed.UNKNOWN;
    };
  }

  /** The format a link state byte says. */
  public static Format format(int link) {
    return switch ((link >> FORMAT_SHIFT) & FORMAT_MASK) {
      case 1 -> Format.GREY;
      case 2 -> Format.YUYV;
      case 3 -> Format.MJPG;
      default -> Format.UNKNOWN;
    };
  }

  /** The headroom a link state byte says. */
  public static Headroom headroom(int link) {
    return (link & HEADROOM) != 0 ? Headroom.MAY_DO_BETTER : Headroom.NONE_KNOWN;
  }

  /** The recovery a link state byte says. */
  public static Recovery recovery(int link) {
    return (link & RECOVERY) != 0 ? Recovery.TRIED_STILL_SLOW : Recovery.NOT_TRIED;
  }

  /** The byte for a link state. */
  public static int of(Speed speed, Format format, Headroom headroom, Recovery recovery) {
    // Each speed's and format's place in its enum is its value on the wire.
    int link = speed.ordinal() | format.ordinal() << FORMAT_SHIFT;
    if (headroom == Headroom.MAY_DO_BETTER) {
      link |= HEADROOM;
    }
    if (recovery == Recovery.TRIED_STILL_SLOW) {
      link |= RECOVERY;
    }
    return link;
  }
}

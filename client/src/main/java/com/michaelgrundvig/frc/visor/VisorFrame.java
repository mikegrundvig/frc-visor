package com.michaelgrundvig.frc.visor;

/**
 * One frame from one of Visor's cameras, and the tags in it.
 *
 * <p>A frame is mutable and meant to be reused: the receiver copies each frame into one the caller
 * keeps ({@link VisorReceiver#poll}), so receiving frames allocates nothing. Read what you need
 * before the next call fills it again.
 *
 * <p>Tags are numbered from 0 to {@link #tagCount()} - 1. Corners are in OpenCV's order (the
 * printed tag's top-left, top-right, bottom-right, bottom-left), in pixels, in pixel-centre
 * coordinates, raw (not undistorted). Each tag's two poses, when Visor solved them, are in OpenCV's
 * axes as sent; {@link #cameraToTag} gives them in WPILib's.
 */
public final class VisorFrame {
  /** The most tags a frame carries. */
  public static final int MAX_TAGS = 15;

  /** x and y for each of four corners. */
  static final int CORNER_VALUES = 8;

  /** Per tag: each pose's rotation (3) and translation (3), then the two errors. */
  static final int POSE_VALUES = 14;

  private static final int ERRORS = 12;

  int versionId = VisorVersion.ID;
  long sequence;
  int width;
  int height;
  long captureNanos;
  int camera;
  boolean clockSynced;
  boolean dequeueTime;
  int tagCount;
  final int[] ids = new int[MAX_TAGS];
  final int[] hamming = new int[MAX_TAGS];
  final float[] margins = new float[MAX_TAGS];
  final boolean[] posesSolved = new boolean[MAX_TAGS];
  final float[] corners = new float[MAX_TAGS * CORNER_VALUES];
  final float[] poses = new float[MAX_TAGS * POSE_VALUES];
  // Working space for setPoses from WPILib's axes, so it allocates nothing.
  private final float[] bestRvec = new float[3];
  private final float[] bestTvec = new float[3];
  private final float[] otherRvec = new float[3];
  private final float[] otherTvec = new float[3];

  /** Visor's version ID: 2026.11.2 is 20261102. */
  public int versionId() {
    return versionId;
  }

  /** Counts this camera's frames from Visor's start; a gap is a dropped frame. */
  public long sequence() {
    return sequence;
  }

  /** The image's width, in pixels. */
  public int width() {
    return width;
  }

  /** The image's height, in pixels. */
  public int height() {
    return height;
  }

  /** When the image was captured, in nanoseconds on the robot's clock. */
  public long captureNanos() {
    return captureNanos;
  }

  /** The camera's index in Visor's configuration, unique on the robot. */
  public int camera() {
    return camera;
  }

  /**
   * Whether the coprocessor's clock was synced to the robot's; if not, the capture time is wrong.
   */
  public boolean clockSynced() {
    return clockSynced;
  }

  /** Whether the capture time is when Visor got the frame, because the driver gave no timestamp. */
  public boolean dequeueTime() {
    return dequeueTime;
  }

  /** How many tags are in the frame. */
  public int tagCount() {
    return tagCount;
  }

  /** A tag's ID. */
  public int tagId(int tag) {
    return ids[checked(tag)];
  }

  /** How many of a tag's payload bits NAPA corrected. */
  public int hamming(int tag) {
    return hamming[checked(tag)];
  }

  /** A tag's decision margin: higher is clearer. */
  public float decisionMargin(int tag) {
    return margins[checked(tag)];
  }

  /** A corner's x, in pixels; corners 0 to 3 in OpenCV's order. */
  public float cornerX(int tag, int corner) {
    return corners[checked(tag) * CORNER_VALUES + 2 * checkedCorner(corner)];
  }

  /** A corner's y, in pixels. */
  public float cornerY(int tag, int corner) {
    return corners[checked(tag) * CORNER_VALUES + 2 * checkedCorner(corner) + 1];
  }

  /** Whether Visor solved a tag's poses. Without a lens calibration it sends corners only. */
  public boolean posesSolved(int tag) {
    return posesSolved[checked(tag)];
  }

  /** A pose's reprojection error: root-mean-square per coordinate, in pixels. */
  public float error(int tag, Candidate candidate) {
    return poses[checked(tag) * POSE_VALUES + ERRORS + candidate.ordinal()];
  }

  /**
   * How close a tag's two poses are to equally likely, as FRC vision usually measures it: the best
   * pose's error over the other's. 0 is certain, 1 a coin toss.
   */
  public double ambiguity(int tag) {
    double best = error(tag, Candidate.BEST);
    double other = error(tag, Candidate.OTHER);
    return other > 0 ? Math.min(1.0, best / other) : 1.0;
  }

  /** One component (0 to 2) of a pose's rotation as Visor sent it: OpenCV's rvec, radians. */
  public float rvec(int tag, Candidate candidate, int axis) {
    return poses[checked(tag) * POSE_VALUES + 6 * candidate.ordinal() + checkedAxis(axis)];
  }

  /** One component (0 to 2) of a pose's translation as Visor sent it: OpenCV's tvec, metres. */
  public float tvec(int tag, Candidate candidate, int axis) {
    return poses[checked(tag) * POSE_VALUES + 6 * candidate.ordinal() + 3 + checkedAxis(axis)];
  }

  /**
   * A tag's pose from the camera in WPILib's axes (camera-to-tag), into {@code into}.
   *
   * @throws IllegalStateException if the tag's poses weren't solved
   */
  public void cameraToTag(int tag, Candidate candidate, CameraToTag into) {
    if (!posesSolved(tag)) {
      throw new IllegalStateException("tag " + ids[tag] + "'s poses weren't solved");
    }
    int at = tag * POSE_VALUES + 6 * candidate.ordinal();
    into.setFromOpenCv(
        poses[at], poses[at + 1], poses[at + 2], poses[at + 3], poses[at + 4], poses[at + 5]);
  }

  /** Makes this frame a copy of another. */
  public void copyFrom(VisorFrame other) {
    versionId = other.versionId;
    sequence = other.sequence;
    width = other.width;
    height = other.height;
    captureNanos = other.captureNanos;
    camera = other.camera;
    clockSynced = other.clockSynced;
    dequeueTime = other.dequeueTime;
    tagCount = other.tagCount;
    int n = other.tagCount;
    System.arraycopy(other.ids, 0, ids, 0, n);
    System.arraycopy(other.hamming, 0, hamming, 0, n);
    System.arraycopy(other.margins, 0, margins, 0, n);
    System.arraycopy(other.posesSolved, 0, posesSolved, 0, n);
    System.arraycopy(other.corners, 0, corners, 0, n * CORNER_VALUES);
    System.arraycopy(other.poses, 0, poses, 0, n * POSE_VALUES);
  }

  /**
   * Sets everything but the tags, which it clears: for a simulation or a test building a frame to
   * send. The version ID is this client's.
   */
  public void setHeader(
      long sequence,
      int width,
      int height,
      long captureNanos,
      int camera,
      boolean clockSynced,
      boolean dequeueTime) {
    this.versionId = VisorVersion.ID;
    this.sequence = sequence;
    this.width = width;
    this.height = height;
    this.captureNanos = captureNanos;
    this.camera = camera;
    this.clockSynced = clockSynced;
    this.dequeueTime = dequeueTime;
    this.tagCount = 0;
  }

  /** Sets the version ID, which {@link #setHeader} makes this client's. */
  public void setVersionId(int versionId) {
    this.versionId = versionId;
  }

  /**
   * Adds a tag with its corners (x0, y0 … x3, y3) and no poses.
   *
   * @return its index, for {@link #setPoses}
   */
  public int addTag(int id, int hamming, float decisionMargin, float[] corners) {
    if (tagCount == MAX_TAGS) {
      throw new IllegalStateException("a frame carries at most " + MAX_TAGS + " tags");
    }
    if (corners.length != CORNER_VALUES) {
      throw new IllegalArgumentException("4 corners, x and y each: " + corners.length);
    }
    int tag = tagCount++;
    ids[tag] = id;
    this.hamming[tag] = hamming;
    margins[tag] = decisionMargin;
    System.arraycopy(corners, 0, this.corners, tag * CORNER_VALUES, CORNER_VALUES);
    posesSolved[tag] = false;
    java.util.Arrays.fill(poses, tag * POSE_VALUES, (tag + 1) * POSE_VALUES, 0f);
    return tag;
  }

  /**
   * Gives a tag its two poses, in OpenCV's axes as Visor sends them: each a rotation (rvec,
   * radians) and a translation (tvec, metres), with its reprojection error in pixels.
   */
  public void setPoses(
      int tag,
      float[] bestRvec,
      float[] bestTvec,
      float bestError,
      float[] otherRvec,
      float[] otherTvec,
      float otherError) {
    int at = checked(tag) * POSE_VALUES;
    System.arraycopy(bestRvec, 0, poses, at, 3);
    System.arraycopy(bestTvec, 0, poses, at + 3, 3);
    System.arraycopy(otherRvec, 0, poses, at + 6, 3);
    System.arraycopy(otherTvec, 0, poses, at + 9, 3);
    poses[at + ERRORS] = bestError;
    poses[at + ERRORS + 1] = otherError;
    posesSolved[tag] = true;
  }

  /**
   * Gives a tag its two poses from transforms in WPILib's axes, as a simulation knows them: each
   * camera-to-tag, with its reprojection error in pixels. They're stored as Visor sends them.
   */
  public void setPoses(
      int tag, CameraToTag best, float bestError, CameraToTag other, float otherError) {
    best.toOpenCv(bestRvec, bestTvec);
    other.toOpenCv(otherRvec, otherTvec);
    setPoses(tag, bestRvec, bestTvec, bestError, otherRvec, otherTvec, otherError);
  }

  private int checked(int tag) {
    if (tag < 0 || tag >= tagCount) {
      throw new IndexOutOfBoundsException("tag " + tag + " of " + tagCount);
    }
    return tag;
  }

  private static int checkedCorner(int corner) {
    if (corner < 0 || corner > 3) {
      throw new IndexOutOfBoundsException("corners are 0 to 3: " + corner);
    }
    return corner;
  }

  private static int checkedAxis(int axis) {
    if (axis < 0 || axis > 2) {
      throw new IndexOutOfBoundsException("axes are 0 to 2: " + axis);
    }
    return axis;
  }
}

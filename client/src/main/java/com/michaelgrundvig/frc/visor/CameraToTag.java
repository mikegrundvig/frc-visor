package com.michaelgrundvig.frc.visor;

/**
 * A tag's pose from the camera in WPILib's axes, the usual camera-to-tag transform: the camera's x
 * out of the lens, y left, z up; the tag's x out of its face, y to your right as you face it, z up.
 * Mutable and reusable, so reading poses allocates nothing.
 *
 * <p>Visor sends poses in OpenCV's axes: the camera's x right, y down, z out of the lens; the tag's
 * x right and y up as it's printed, z out of its face. This turns one into the other.
 *
 * <p>Build WPILib's transform from it with {@code new Transform3d(new Translation3d(x(), y(), z()),
 * new Rotation3d(new Quaternion(qw(), qx(), qy(), qz())))}.
 */
public final class CameraToTag {
  private double x;
  private double y;
  private double z;
  private double qw = 1;
  private double qx;
  private double qy;
  private double qz;
  // Working space, so converting allocates nothing.
  private final double[] opencv = new double[9];
  private final double[] wpilib = new double[9];

  /** The tag's centre ahead of the camera, in metres. */
  public double x() {
    return x;
  }

  /** The tag's centre to the camera's left, in metres. */
  public double y() {
    return y;
  }

  /** The tag's centre above the camera, in metres. */
  public double z() {
    return z;
  }

  /** The rotation's quaternion: its scalar part. */
  public double qw() {
    return qw;
  }

  /** The rotation's quaternion: x. */
  public double qx() {
    return qx;
  }

  /** The rotation's quaternion: y. */
  public double qy() {
    return qy;
  }

  /** The rotation's quaternion: z. */
  public double qz() {
    return qz;
  }

  /**
   * Sets this from a pose in OpenCV's axes: a rotation as an axis scaled by its angle (rvec) and a
   * translation (tvec).
   */
  void setFromOpenCv(double rx, double ry, double rz, double tx, double ty, double tz) {
    // The rotation's matrix, by Rodrigues' formula.
    double angle = Math.sqrt(rx * rx + ry * ry + rz * rz);
    double[] r = opencv;
    rotation(rx, ry, rz, angle, r);
    // Camera axes, OpenCV's to WPILib's: a vector (a, b, c) becomes (c, -a, -b). Tag axes: WPILib's
    // x, y, z are OpenCV's z, x, y, so the tag's WPILib columns are OpenCV's columns 2, 0 and 1.
    double[] m = wpilib;
    for (int column = 0; column < 3; column++) {
      int from = (column + 2) % 3;
      m[column] = r[6 + from];
      m[3 + column] = -r[from];
      m[6 + column] = -r[3 + from];
    }
    x = tz;
    y = -tx;
    z = -ty;
    setQuaternion(m);
  }

  /** Rodrigues' formula, row-major into {@code r}. */
  private static void rotation(double rx, double ry, double rz, double angle, double[] r) {
    if (angle < 1e-12) {
      // To first order: the identity plus the cross-product matrix.
      r[0] = 1;
      r[1] = -rz;
      r[2] = ry;
      r[3] = rz;
      r[4] = 1;
      r[5] = -rx;
      r[6] = -ry;
      r[7] = rx;
      r[8] = 1;
      return;
    }
    double kx = rx / angle;
    double ky = ry / angle;
    double kz = rz / angle;
    double c = Math.cos(angle);
    double s = Math.sin(angle);
    double t = 1 - c;
    r[0] = c + kx * kx * t;
    r[1] = kx * ky * t - kz * s;
    r[2] = kx * kz * t + ky * s;
    r[3] = ky * kx * t + kz * s;
    r[4] = c + ky * ky * t;
    r[5] = ky * kz * t - kx * s;
    r[6] = kz * kx * t - ky * s;
    r[7] = kz * ky * t + kx * s;
    r[8] = c + kz * kz * t;
  }

  /** The quaternion of a row-major rotation matrix (Shepperd's method: the largest term first). */
  private void setQuaternion(double[] m) {
    double trace = m[0] + m[4] + m[8];
    if (trace > 0) {
      double s = 2 * Math.sqrt(trace + 1);
      qw = s / 4;
      qx = (m[7] - m[5]) / s;
      qy = (m[2] - m[6]) / s;
      qz = (m[3] - m[1]) / s;
    } else if (m[0] > m[4] && m[0] > m[8]) {
      double s = 2 * Math.sqrt(1 + m[0] - m[4] - m[8]);
      qw = (m[7] - m[5]) / s;
      qx = s / 4;
      qy = (m[1] + m[3]) / s;
      qz = (m[2] + m[6]) / s;
    } else if (m[4] > m[8]) {
      double s = 2 * Math.sqrt(1 + m[4] - m[0] - m[8]);
      qw = (m[2] - m[6]) / s;
      qx = (m[1] + m[3]) / s;
      qy = s / 4;
      qz = (m[5] + m[7]) / s;
    } else {
      double s = 2 * Math.sqrt(1 + m[8] - m[0] - m[4]);
      qw = (m[3] - m[1]) / s;
      qx = (m[2] + m[6]) / s;
      qy = (m[5] + m[7]) / s;
      qz = s / 4;
    }
    double norm = Math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    qw /= norm;
    qx /= norm;
    qy /= norm;
    qz /= norm;
  }
}

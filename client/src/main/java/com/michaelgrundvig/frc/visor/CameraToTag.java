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
  private final double[] quaternion = new double[4];

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
   * Sets this pose in WPILib's axes: for a simulation building the frames Visor would send. The
   * quaternion needn't be normalized; it is here.
   */
  public void set(double x, double y, double z, double qw, double qx, double qy, double qz) {
    this.x = x;
    this.y = y;
    this.z = z;
    double norm = Math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    this.qw = qw / norm;
    this.qx = qx / norm;
    this.qy = qy / norm;
    this.qz = qz / norm;
  }

  /**
   * This pose in OpenCV's axes, as Visor sends it: the rotation as an axis scaled by its angle into
   * {@code rvec}, the translation into {@code tvec}. The opposite of {@link #setFromOpenCv}.
   */
  void toOpenCv(float[] rvec, float[] tvec) {
    // The translation: WPILib's (x, y, z) is OpenCV's (z, -x, -y).
    tvec[0] = (float) -y;
    tvec[1] = (float) -z;
    tvec[2] = (float) x;
    // The rotation's matrix in WPILib's axes, from the quaternion.
    double[] m = wpilib;
    m[0] = 1 - 2 * (qy * qy + qz * qz);
    m[1] = 2 * (qx * qy - qz * qw);
    m[2] = 2 * (qx * qz + qy * qw);
    m[3] = 2 * (qx * qy + qz * qw);
    m[4] = 1 - 2 * (qx * qx + qz * qz);
    m[5] = 2 * (qy * qz - qx * qw);
    m[6] = 2 * (qx * qz - qy * qw);
    m[7] = 2 * (qy * qz + qx * qw);
    m[8] = 1 - 2 * (qx * qx + qy * qy);
    // Back to OpenCV's axes, undoing setFromOpenCv: OpenCV's column k is WPILib's column (k + 1) %
    // 3,
    // each vector (p, q, s) turned back into (-q, -s, p).
    double[] r = opencv;
    for (int column = 0; column < 3; column++) {
      int from = (column + 1) % 3;
      r[column] = -m[3 + from];
      r[3 + column] = -m[6 + from];
      r[6 + column] = m[from];
    }
    quaternionOf(r, quaternion);
    double w = quaternion[0];
    double vx = quaternion[1];
    double vy = quaternion[2];
    double vz = quaternion[3];
    if (w < 0) {
      w = -w;
      vx = -vx;
      vy = -vy;
      vz = -vz;
    }
    double length = Math.sqrt(vx * vx + vy * vy + vz * vz);
    // The angle, from atan2 so it stays exact near zero and near a half turn.
    double scale = length < 1e-12 ? 2 : 2 * Math.atan2(length, w) / length;
    rvec[0] = (float) (vx * scale);
    rvec[1] = (float) (vy * scale);
    rvec[2] = (float) (vz * scale);
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

  /** The quaternion of a row-major rotation matrix, as this pose's rotation. */
  private void setQuaternion(double[] m) {
    quaternionOf(m, quaternion);
    qw = quaternion[0];
    qx = quaternion[1];
    qy = quaternion[2];
    qz = quaternion[3];
  }

  /**
   * The quaternion (w, x, y, z) of a row-major rotation matrix, normalized, into {@code q}
   * (Shepperd's method: the largest term first).
   */
  private static void quaternionOf(double[] m, double[] q) {
    double trace = m[0] + m[4] + m[8];
    if (trace > 0) {
      double s = 2 * Math.sqrt(trace + 1);
      q[0] = s / 4;
      q[1] = (m[7] - m[5]) / s;
      q[2] = (m[2] - m[6]) / s;
      q[3] = (m[3] - m[1]) / s;
    } else if (m[0] > m[4] && m[0] > m[8]) {
      double s = 2 * Math.sqrt(1 + m[0] - m[4] - m[8]);
      q[0] = (m[7] - m[5]) / s;
      q[1] = s / 4;
      q[2] = (m[1] + m[3]) / s;
      q[3] = (m[2] + m[6]) / s;
    } else if (m[4] > m[8]) {
      double s = 2 * Math.sqrt(1 + m[4] - m[0] - m[8]);
      q[0] = (m[2] - m[6]) / s;
      q[1] = (m[1] + m[3]) / s;
      q[2] = s / 4;
      q[3] = (m[5] + m[7]) / s;
    } else {
      double s = 2 * Math.sqrt(1 + m[8] - m[0] - m[4]);
      q[0] = (m[3] - m[1]) / s;
      q[1] = (m[2] + m[6]) / s;
      q[2] = (m[5] + m[7]) / s;
      q[3] = s / 4;
    }
    double norm = Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (int i = 0; i < 4; i++) {
      q[i] /= norm;
    }
  }
}

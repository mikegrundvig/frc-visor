package com.michaelgrundvig.frc.visor;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.Random;
import org.junit.jupiter.api.Test;

class CameraToTagTest {
  private static final double CLOSE = 1e-6;

  /** Converts one OpenCV pose. */
  private static CameraToTag convert(double[] rvec, double[] tvec) {
    VisorFrame frame = new VisorFrame();
    frame.setHeader(0, 1600, 1304, 0, 0, true, false);
    int tag = frame.addTag(1, 0, 50, new float[8]);
    float[] r = {(float) rvec[0], (float) rvec[1], (float) rvec[2]};
    float[] t = {(float) tvec[0], (float) tvec[1], (float) tvec[2]};
    frame.setPoses(tag, r, t, 0.1f, r, t, 0.2f);
    CameraToTag pose = new CameraToTag();
    frame.cameraToTag(tag, Candidate.BEST, pose);
    return pose;
  }

  @Test
  void aTagSquareInFrontFacesBackAtTheCamera() {
    // OpenCV's view of a tag 3 m ahead, upright and square to the camera: its x right (the
    // camera's x), its y up (the camera's -y), its z out of its face (the camera's -z). That's a
    // half turn about x.
    CameraToTag pose = convert(new double[] {Math.PI, 0, 0}, new double[] {0, 0, 3});
    // WPILib's, as PhotonVision gives it: 3 m ahead, and turned half round about z, its x pointing
    // back at the camera.
    assertThat(pose.x()).isCloseTo(3, within(CLOSE));
    assertThat(pose.y()).isCloseTo(0, within(CLOSE));
    assertThat(pose.z()).isCloseTo(0, within(CLOSE));
    assertThat(Math.abs(pose.qz())).isCloseTo(1, within(CLOSE));
  }

  @Test
  void theTranslationMovesToWpilibsAxes() {
    // OpenCV's x right, y down, z ahead; WPILib's x ahead, y left, z up.
    record Case(double[] opencv, double[] wpilib) {}
    Case[] cases = {
      new Case(new double[] {1, 0, 2}, new double[] {2, -1, 0}),
      new Case(new double[] {0, 1, 2}, new double[] {2, 0, -1}),
      new Case(new double[] {-0.5, -0.25, 6}, new double[] {6, 0.5, 0.25}),
    };
    for (Case c : cases) {
      CameraToTag pose = convert(new double[] {Math.PI, 0, 0}, c.opencv());
      assertThat(new double[] {pose.x(), pose.y(), pose.z()})
          .containsExactly(c.wpilib(), within(CLOSE));
    }
  }

  @Test
  void aTagTurnedOnItsVerticalAxisIsTurnedTheSameWayInWpilib() {
    // The tag square in front, then turned 30 degrees about the camera's vertical (OpenCV's y is
    // down, so a turn to the left seen from above is about -y). In WPILib that's z, the same way:
    // half a turn plus 30 degrees.
    double turn = Math.toRadians(30);
    double[] r =
        multiply(
            rotationAbout(new double[] {0, -1, 0}, turn),
            rotationAbout(new double[] {1, 0, 0}, Math.PI));
    CameraToTag pose = convert(rvecOf(r), new double[] {0, 0, 3});
    double yaw = 2 * Math.atan2(pose.qz(), pose.qw());
    assertThat(Math.IEEEremainder(yaw - (Math.PI + turn), 2 * Math.PI)).isCloseTo(0, within(1e-6));
    assertThat(pose.qx()).isCloseTo(0, within(CLOSE));
    assertThat(pose.qy()).isCloseTo(0, within(CLOSE));
  }

  @Test
  void anyPoseMovesPointsTheSameWayInBothAxes() {
    // For random poses: a point on the tag, placed in the camera's frame by the OpenCV pose and by
    // the converted WPILib pose, lands in the same place, each in its own axes.
    Random random = new Random(7);
    for (int n = 0; n < 200; n++) {
      double[] rvec = {random.nextGaussian(), random.nextGaussian(), random.nextGaussian()};
      double[] tvec = {random.nextGaussian(), random.nextGaussian(), 1 + 5 * random.nextDouble()};
      CameraToTag pose = convert(rvec, tvec);
      // The float the wire carries is what both sides use.
      double[] r = rotationAbout(normalized(floats(rvec)), norm(floats(rvec)));
      double[] t = floats(tvec);
      for (double[] onTagWpilib :
          new double[][] {{0, 0.1, 0.05}, {0, -0.08, 0}, {0.02, 0, -0.08}}) {
        // WPILib's tag axes to OpenCV's: x out of the face is OpenCV's z, y is OpenCV's x, z is y.
        double[] onTagOpencv = {onTagWpilib[1], onTagWpilib[2], onTagWpilib[0]};
        double[] cameraOpencv = add(apply(r, onTagOpencv), t);
        double[] expected = {cameraOpencv[2], -cameraOpencv[0], -cameraOpencv[1]};
        double[] actual =
            add(rotate(pose, onTagWpilib), new double[] {pose.x(), pose.y(), pose.z()});
        assertThat(actual).containsExactly(expected, within(1e-5));
      }
    }
  }

  private static double[] floats(double[] v) {
    return new double[] {(float) v[0], (float) v[1], (float) v[2]};
  }

  private static double norm(double[] v) {
    return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  }

  private static double[] normalized(double[] v) {
    double n = norm(v);
    return new double[] {v[0] / n, v[1] / n, v[2] / n};
  }

  private static double[] add(double[] a, double[] b) {
    return new double[] {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
  }

  /** A row-major rotation about a unit axis. */
  private static double[] rotationAbout(double[] k, double angle) {
    double c = Math.cos(angle);
    double s = Math.sin(angle);
    double t = 1 - c;
    return new double[] {
      c + k[0] * k[0] * t,
      k[0] * k[1] * t - k[2] * s,
      k[0] * k[2] * t + k[1] * s,
      k[1] * k[0] * t + k[2] * s,
      c + k[1] * k[1] * t,
      k[1] * k[2] * t - k[0] * s,
      k[2] * k[0] * t - k[1] * s,
      k[2] * k[1] * t + k[0] * s,
      c + k[2] * k[2] * t
    };
  }

  private static double[] multiply(double[] a, double[] b) {
    double[] m = new double[9];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
          m[3 * i + j] += a[3 * i + k] * b[3 * k + j];
        }
      }
    }
    return m;
  }

  private static double[] apply(double[] m, double[] v) {
    return new double[] {
      m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
      m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
      m[6] * v[0] + m[7] * v[1] + m[8] * v[2]
    };
  }

  /** The axis-angle vector of a rotation, by way of its quaternion, so a half turn works too. */
  private static double[] rvecOf(double[] m) {
    double w = Math.sqrt(Math.max(0, 1 + m[0] + m[4] + m[8])) / 2;
    double x = Math.copySign(Math.sqrt(Math.max(0, 1 + m[0] - m[4] - m[8])) / 2, m[7] - m[5]);
    double y = Math.copySign(Math.sqrt(Math.max(0, 1 - m[0] + m[4] - m[8])) / 2, m[2] - m[6]);
    double z = Math.copySign(Math.sqrt(Math.max(0, 1 - m[0] - m[4] + m[8])) / 2, m[3] - m[1]);
    if (w < 1e-9) {
      // A half turn: the off-diagonal sums give the axis's signs relative to its largest part.
      x = Math.sqrt(Math.max(0, (m[0] + 1) / 2));
      y = Math.copySign(Math.sqrt(Math.max(0, (m[4] + 1) / 2)), m[1] + m[3]);
      z = Math.copySign(Math.sqrt(Math.max(0, (m[8] + 1) / 2)), m[2] + m[6]);
      return new double[] {Math.PI * x, Math.PI * y, Math.PI * z};
    }
    double length = Math.sqrt(x * x + y * y + z * z);
    double angle = 2 * Math.atan2(length, w);
    return length < 1e-12
        ? new double[] {0, 0, 0}
        : new double[] {angle * x / length, angle * y / length, angle * z / length};
  }

  /** A vector turned by the pose's quaternion. */
  private static double[] rotate(CameraToTag pose, double[] v) {
    double w = pose.qw();
    double x = pose.qx();
    double y = pose.qy();
    double z = pose.qz();
    double[] m = {
      1 - 2 * (y * y + z * z),
      2 * (x * y - z * w),
      2 * (x * z + y * w),
      2 * (x * y + z * w),
      1 - 2 * (x * x + z * z),
      2 * (y * z - x * w),
      2 * (x * z - y * w),
      2 * (y * z + x * w),
      1 - 2 * (x * x + y * y)
    };
    return apply(m, v);
  }
}

package com.michaelgrundvig.frc.visor;

import static org.assertj.core.api.Assertions.assertThat;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

/**
 * Visor against PhotonVision on the same corners: what the robot gets from Visor is what it would
 * have got from PhotonVision.
 *
 * <p>testdata/photonvision-cases.json holds 600 tags PhotonVision's simulation saw across 2026's
 * field (the robot template's {@code CameraImageSim}): their corners, in OpenCV's order, and
 * PhotonVision's own solve of each one's two poses, its camera-to-target transforms in WPILib's
 * axes. testdata/photonvision-visor.json holds Visor's solve of the same corners, as its wire
 * carries it (Visor's own test keeps it current). Each goes through the wire and this client, and
 * must come out as PhotonVision's: the same two transforms, the same errors, the same ambiguity.
 * That covers the corner order, the tag's frame, both axis conversions and the solve itself.
 */
class PhotonVisionAgreementTest {
  /**
   * PhotonVision solves in 32-bit floats and the wire carries them, so the two land a few float
   * steps apart: at 6 m that's about a hundredth of a millimetre. A tenth of a millimetre, and of a
   * milliradian.
   */
  private static final double METRES = 1e-4;

  private static final double RADIANS = 1e-4;

  private static JsonNode read(String name) throws IOException {
    return new ObjectMapper()
        .readTree(Path.of(System.getProperty("visor.testdata"), name).toFile());
  }

  private static float[] floats(JsonNode array) {
    float[] values = new float[array.size()];
    for (int i = 0; i < values.length; i++) {
      values[i] = (float) array.get(i).asDouble();
    }
    return values;
  }

  /** How far apart two transforms are: metres, and radians between their rotations. */
  private static double[] apart(CameraToTag visor, JsonNode photonVision) {
    double dx = visor.x() - photonVision.get(0).asDouble();
    double dy = visor.y() - photonVision.get(1).asDouble();
    double dz = visor.z() - photonVision.get(2).asDouble();
    double dot =
        visor.qw() * photonVision.get(3).asDouble()
            + visor.qx() * photonVision.get(4).asDouble()
            + visor.qy() * photonVision.get(5).asDouble()
            + visor.qz() * photonVision.get(6).asDouble();
    return new double[] {
      Math.sqrt(dx * dx + dy * dy + dz * dz), 2 * Math.acos(Math.min(1, Math.abs(dot)))
    };
  }

  @Test
  void everyTagComesOutAsPhotonVisionSolvedIt() throws IOException {
    JsonNode cases = read("photonvision-cases.json");
    JsonNode answers = read("photonvision-visor.json");
    assertThat(answers.size()).isEqualTo(cases.get("cases").size());
    VisorFrame sent = new VisorFrame();
    VisorFrame received = new VisorFrame();
    ByteBuffer wire = ByteBuffer.allocate(VisorWire.MAX_BYTES);
    CameraToTag pose = new CameraToTag();
    double worstMetres = 0;
    double worstRadians = 0;
    double worstAmbiguity = 0;
    int closeCalls = 0;
    for (int n = 0; n < answers.size(); n++) {
      JsonNode photonVision = cases.get("cases").get(n);
      JsonNode visor = answers.get(n);
      sent.setHeader(n, cases.get("width").asInt(), cases.get("height").asInt(), 0, 0, true, false);
      int tag =
          sent.addTag(photonVision.get("tag").asInt(), 0, 50, floats(photonVision.get("corners")));
      JsonNode best = visor.get(0);
      JsonNode other = visor.get(1);
      sent.setPoses(
          tag,
          floats(best.get("rvec")),
          floats(best.get("tvec")),
          (float) best.get("error").asDouble(),
          floats(other.get("rvec")),
          floats(other.get("tvec")),
          (float) other.get("error").asDouble());
      wire.clear();
      VisorWire.encode(sent, wire);
      wire.flip();
      assertThat(VisorWire.decode(wire, received)).isEqualTo(VisorWire.Decoded.FRAME);

      // When the two poses fit almost equally well, the two solvers may rank them either way.
      double photonBest = photonVision.get("best_error").asDouble();
      double photonAlt = photonVision.get("alt_error").asDouble();
      boolean closeCall = Math.abs(photonBest - photonAlt) < 1e-3 * Math.max(photonAlt, 1e-9);
      received.cameraToTag(0, Candidate.BEST, pose);
      double[] asBest = apart(pose, photonVision.get("best"));
      boolean swapped = false;
      if (closeCall) {
        double[] asAlt = apart(pose, photonVision.get("alt"));
        swapped = asAlt[1] < asBest[1];
        closeCalls++;
      }
      JsonNode photonFirst = photonVision.get(swapped ? "alt" : "best");
      JsonNode photonSecond = photonVision.get(swapped ? "best" : "alt");
      for (Candidate candidate : Candidate.values()) {
        received.cameraToTag(0, candidate, pose);
        double[] off = apart(pose, candidate == Candidate.BEST ? photonFirst : photonSecond);
        worstMetres = Math.max(worstMetres, off[0]);
        worstRadians = Math.max(worstRadians, off[1]);
        assertThat(off[0]).as("case %d, %s pose: metres off", n, candidate).isLessThan(METRES);
        assertThat(off[1]).as("case %d, %s pose: radians off", n, candidate).isLessThan(RADIANS);
      }
      assertThat((double) received.error(0, Candidate.BEST))
          .as("case %d: best error", n)
          .isCloseTo(swapped ? photonAlt : photonBest, org.assertj.core.data.Offset.offset(1e-4));
      if (!closeCall) {
        double ambiguityOff =
            Math.abs(received.ambiguity(0) - photonVision.get("ambiguity").asDouble());
        worstAmbiguity = Math.max(worstAmbiguity, ambiguityOff);
        // A ratio of two small errors: a hundred-thousandth of a pixel in either moves it by about
        // a ten-thousandth when the other pose's error is a tenth of a pixel.
        assertThat(ambiguityOff).as("case %d: ambiguity", n).isLessThan(1e-3);
      }
    }
    System.out.printf(
        "Visor against PhotonVision, %d tags (%d close calls): worst %.2e m, %.2e rad, ambiguity"
            + " %.2e%n",
        answers.size(), closeCalls, worstMetres, worstRadians, worstAmbiguity);
  }
}

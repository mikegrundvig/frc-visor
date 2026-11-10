package com.michaelgrundvig.frc.visor;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * The configuration the robot's deploy writes, against testdata/written-config.toml: the file
 * Visor's own tests read back, setting by setting (tests/written_config.rs); and every setting at
 * the least and the most Visor takes (least-config.toml, most-config.toml), which Visor's tests run
 * through {@code visor check}. Run with VISOR_UPDATE_GOLDEN=1 to write the files again after a
 * deliberate change.
 */
class VisorConfigFileTest {
  /** Every setting, an awkward name and a value only an exponent writes. */
  static VisorConfigFile written() {
    VisorConfigFile file = VisorConfigFile.forTeam(1234);
    file
        .camera("front \"left\" \\ 1", 0, "/dev/v4l/by-id/usb-cam-video-index0", 1600, 1304)
        .fps(60)
        .minTagPixels(18)
        .timeOffsetMicros(-1500)
        .exposureMicros(1000)
        .gain(40)
        .stream(0.25, 15, 80)
        .control("power_line_frequency", 2)
        .control("brightness", -10)
        .lens(1600, 1304, 1229.75, 1230.125, 801.5, 650.25, -0.3125, 0.125, 1.0e-4, -2.5e-4, -0.03);
    file.camera("back", 3, "/dev/video2", 1280, 800);
    return file;
  }

  /**
   * Every setting at the least Visor takes: Visor's tests check that {@code visor check} passes it
   * (tests/written_config.rs).
   */
  static VisorConfigFile least() {
    VisorConfigFile file = VisorConfigFile.forTeam(1).port(1);
    file
        .camera("a", 0, "/dev/video0", 1, 1)
        .fps(1)
        .minTagPixels(8)
        .timeOffsetMicros(-VisorConfigFile.Camera.MOST_OFFSET_MICROS)
        .exposureMicros(1)
        .gain(0)
        .stream(0.05, 1, 1)
        .lens(1, 1, Double.MIN_VALUE, Double.MIN_VALUE, -1.0e300, -1.0e300);
    return file;
  }

  /** Every setting at the most Visor takes. */
  static VisorConfigFile most() {
    VisorConfigFile file = VisorConfigFile.forTeam(VisorConfigFile.MOST_TEAM).port(65_535);
    file
        .camera("z", 255, "/dev/video9", 65_535, 65_535)
        .fps(1000)
        .minTagPixels(30_000)
        .timeOffsetMicros(VisorConfigFile.Camera.MOST_OFFSET_MICROS)
        .exposureMicros(10_000_000)
        .gain(Integer.MAX_VALUE)
        .stream(1, 1000, 100)
        .control("brightness", Long.MAX_VALUE)
        .control("contrast", Long.MIN_VALUE)
        .lens(65_535, 65_535, 1.0e300, 1.0e300, 1.0e300, 1.0e300, 1, 2, 3, 4, 5, 6, 7, 8);
    return file;
  }

  @Test
  void theWrittenFilesAreTheOnesVisorReads() throws IOException {
    record Case(String golden, VisorConfigFile file) {}
    List<Case> cases =
        List.of(
            new Case("written-config.toml", written()),
            new Case("least-config.toml", least()),
            new Case("most-config.toml", most()));
    for (Case c : cases) {
      Path golden = Path.of(System.getProperty("visor.testdata"), c.golden());
      String text = c.file().toToml();
      if ("1".equals(System.getenv("VISOR_UPDATE_GOLDEN"))) {
        Files.writeString(golden, text);
      }
      assertThat(text).as(c.golden()).isEqualTo(Files.readString(golden));
    }
  }

  @Test
  void textIsQuotedAndNumbersWrittenExactly() {
    assertThat(VisorConfigFile.quoted("a\"b\\c\nd\u0001")).isEqualTo("\"a\\\"b\\\\c\\nd\\u0001\"");
    for (double value : new double[] {1229.7, 1.0e-4, -2.5e-12, 1600, 0.1 + 0.2}) {
      assertThat(Double.parseDouble(VisorConfigFile.number(value))).isEqualTo(value);
    }
  }

  @Test
  void whatVisorWouldRefuseIsRefusedHere() {
    VisorConfigFile file = VisorConfigFile.forTeam(1);
    VisorConfigFile.Camera camera = file.camera("c", 0, "/dev/video0", 4, 4).fps(30);
    record Case(String what, Runnable written, String why) {}
    List<Case> cases =
        List.of(
            new Case("team 0", () -> VisorConfigFile.forTeam(0), "from 1 to 25599, not 0"),
            new Case("team 25600", () -> VisorConfigFile.forTeam(25_600), "not 25600"),
            new Case("port 0", () -> file.port(0), "from 1 to 65535, not 0"),
            new Case("port 65536", () -> file.port(65_536), "not 65536"),
            new Case("no name", () -> file.camera(" ", 1, "/dev/video1", 4, 4), "needs a name"),
            new Case("index 256", () -> file.camera("d", 256, "/dev/video1", 4, 4), "not 256"),
            new Case("index -1", () -> file.camera("d", -1, "/dev/video1", 4, 4), "not -1"),
            new Case("width 0", () -> file.camera("d", 1, "/dev/video1", 0, 4), "not 0"),
            new Case("height 65536", () -> file.camera("d", 1, "/dev/video1", 4, 65_536), "65536"),
            new Case("a name twice", () -> file.camera("c", 1, "/dev/video1", 4, 4), "named \"c\""),
            new Case("an index twice", () -> file.camera("d", 0, "/dev/video1", 4, 4), "index 0"),
            new Case("fps 0", () -> camera.fps(0), "from 1 to 1000, not 0"),
            new Case("fps 1001", () -> camera.fps(1001), "not 1001"),
            new Case("smallest tag 7", () -> camera.minTagPixels(7), "from 8 to 30000"),
            new Case("smallest tag 30001", () -> camera.minTagPixels(30_001), "not 30001"),
            new Case(
                "time offset past 64-bit nanoseconds",
                () -> camera.timeOffsetMicros(VisorConfigFile.Camera.MOST_OFFSET_MICROS + 1),
                "not 9223372036854776"),
            new Case(
                "time offset past 64-bit nanoseconds, back",
                () -> camera.timeOffsetMicros(-VisorConfigFile.Camera.MOST_OFFSET_MICROS - 1),
                "not -9223372036854776"),
            new Case("exposure 0", () -> camera.exposureMicros(0), "from 1 to 10000000, not 0"),
            new Case("exposure over 10 s", () -> camera.exposureMicros(10_000_001), "not 10000001"),
            new Case("gain -1", () -> camera.gain(-1), "from 0 to 2147483647, not -1"),
            new Case("gain over 32 bits", () -> camera.gain(1L << 31), "not 2147483648"),
            new Case("view scale 0.04", () -> camera.stream(0.04, 10, 60), "0.05 to 1, not 0.04"),
            new Case("view scale 1.5", () -> camera.stream(1.5, 10, 60), "not 1.5"),
            new Case("view rate 0", () -> camera.stream(0.5, 0, 60), "from 1 to 30, not 0"),
            new Case("view faster than the camera", () -> camera.stream(0.5, 31, 60), "not 31"),
            new Case("quality 0", () -> camera.stream(0.5, 10, 0), "from 1 to 100, not 0"),
            new Case("quality 101", () -> camera.stream(0.5, 10, 101), "not 101"),
            new Case("a control with no name", () -> camera.control("", 1), "needs a name"),
            new Case("a lens value NaN", () -> camera.lens(4, 4, 1, 1, 2, 2, Double.NaN), "NaN"),
            new Case("a lens at another size", () -> camera.lens(8, 4, 1, 1, 2, 2), "8x4"),
            new Case("a focal length of 0", () -> camera.lens(4, 4, 0, 1, 2, 2), "positive"),
            new Case("three coefficients", () -> camera.lens(4, 4, 1, 1, 2, 2, 1, 2, 3), "not 3"));
    for (Case c : cases) {
      assertThatThrownBy(c.written()::run)
          .as(c.what())
          .isInstanceOf(IllegalArgumentException.class)
          .hasMessageContaining(c.why());
    }
    // A camera slowed below its live view's rate is refused too, whichever is set first.
    VisorConfigFile.Camera viewFirst = file.camera("e", 2, "/dev/video2", 4, 4).stream(0.5, 20, 60);
    assertThatThrownBy(() -> viewFirst.fps(10)).hasMessageContaining("its live view's, 20");
  }
}

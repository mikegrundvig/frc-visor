package com.michaelgrundvig.frc.visor;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;

/**
 * The configuration the robot's deploy writes, against testdata/written-config.toml: the file
 * Visor's own tests read back, setting by setting (tests/written_config.rs). Run with
 * VISOR_UPDATE_GOLDEN=1 to write the file again after a deliberate change.
 */
class VisorConfigFileTest {
  /** Every setting, an awkward name and a value only an exponent writes. */
  static VisorConfigFile written() {
    VisorConfigFile file = VisorConfigFile.forTeam(1234);
    file.camera("front \"left\" \\ 1", 0, "/dev/v4l/by-id/usb-cam-video-index0", 1600, 1304)
        .fps(60)
        .minTagPixels(18)
        .timeOffsetMicros(-1500)
        .exposureMicros(1000)
        .gain(40)
        .control("power_line_frequency", 2)
        .control("brightness", -10)
        .lens(1600, 1304, 1229.75, 1230.125, 801.5, 650.25, -0.3125, 0.125, 1.0e-4, -2.5e-4, -0.03);
    file.camera("back", 3, "/dev/video2", 1280, 800);
    return file;
  }

  @Test
  void theWrittenFileIsTheOneVisorReads() throws IOException {
    Path golden = Path.of(System.getProperty("visor.testdata"), "written-config.toml");
    String text = written().toToml();
    if ("1".equals(System.getenv("VISOR_UPDATE_GOLDEN"))) {
      Files.writeString(golden, text);
    }
    assertThat(text).isEqualTo(Files.readString(golden));
  }

  @Test
  void textIsQuotedAndNumbersWrittenExactly() {
    assertThat(VisorConfigFile.quoted("a\"b\\c\nd\u0001")).isEqualTo("\"a\\\"b\\\\c\\nd\\u0001\"");
    for (double value : new double[] {1229.7, 1.0e-4, -2.5e-12, 1600, 0.1 + 0.2}) {
      assertThat(Double.parseDouble(VisorConfigFile.number(value))).isEqualTo(value);
    }
  }

  @Test
  void whatCantBeWrittenIsRefused() {
    assertThatThrownBy(() -> VisorConfigFile.forTeam(0)).hasMessageContaining("positive");
    VisorConfigFile.Camera camera = VisorConfigFile.forTeam(1).camera("c", 0, "/dev/video0", 4, 4);
    assertThatThrownBy(() -> camera.lens(4, 4, 1, 1, 2, 2, Double.NaN)).hasMessageContaining("NaN");
    assertThatThrownBy(() -> camera.exposureMicros(0)).hasMessageContaining("positive time");
  }
}

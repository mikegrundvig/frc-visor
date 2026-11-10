package com.michaelgrundvig.frc.visor;

import static org.assertj.core.api.Assertions.assertThat;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;

/** A camera's live view's address, its name escaped as Visor's server reads it back. */
class VisorStreamTest {
  private record Case(String host, String camera, String url) {}

  @Test
  void aCamerasStreamIsOnItsCoprocessorByName() {
    List<Case> cases =
        List.of(
            new Case("10.12.34.11", "front-left", "http://10.12.34.11:5809/stream/front-left"),
            new Case("10.12.34.11", "Front Left", "http://10.12.34.11:5809/stream/Front%20Left"),
            new Case("127.0.0.1", "a/b?c#d", "http://127.0.0.1:5809/stream/a%2Fb%3Fc%23d"),
            new Case(
                "10.12.34.12",
                "front \"left\" \\ 1",
                "http://10.12.34.12:5809/stream/front%20%22left%22%20%5C%201"),
            new Case("10.12.34.12", "caméra", "http://10.12.34.12:5809/stream/cam%C3%A9ra"),
            new Case("fe80::1", "back", "http://[fe80::1]:5809/stream/back"));
    for (Case each : cases) {
      assertThat(VisorStream.url(each.host(), each.camera()))
          .as(each.camera())
          .isEqualTo(each.url());
    }
  }

  /**
   * The ports and paths Visor's own tests check too (testdata/addresses.json, tests/addresses.rs):
   * the port frames go to, the live view's port, and each camera's stream's path.
   */
  @Test
  void thePortsAndPathsAreTheOnesVisorUses() throws IOException {
    JsonNode shared =
        new ObjectMapper()
            .readTree(Path.of(System.getProperty("visor.testdata"), "addresses.json").toFile());
    assertThat(VisorReceiver.DEFAULT_PORT).isEqualTo(shared.get("frames_port").asInt());
    assertThat(VisorStream.PORT).isEqualTo(shared.get("live_view_port").asInt());
    for (JsonNode row : shared.get("streams")) {
      String camera = row.get("camera").asText();
      assertThat(VisorStream.url("10.12.34.11", camera))
          .as(camera)
          .isEqualTo(
              "http://10.12.34.11:"
                  + shared.get("live_view_port").asInt()
                  + row.get("path").asText());
    }
  }
}

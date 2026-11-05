package com.michaelgrundvig.frc.visor;

import static org.assertj.core.api.Assertions.assertThat;

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
}

package com.michaelgrundvig.frc.visor;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import org.junit.jupiter.api.Test;

/** A socket's thread waits through errors that keep coming, longer each time, to a second. */
class RepeatedErrorsTest {
  @Test
  void errorsInARowWaitLongerOnceTheyKeepComing() {
    record Case(int inARow, long pauseMillis) {}
    List<Case> cases =
        List.of(
            new Case(1, 0),
            new Case(RepeatedErrors.FREE, 0),
            new Case(RepeatedErrors.FREE + 1, 10),
            new Case(RepeatedErrors.FREE + 2, 20),
            new Case(RepeatedErrors.FREE + 7, 640),
            new Case(RepeatedErrors.FREE + 8, 1000),
            new Case(Integer.MAX_VALUE, 1000));
    for (Case c : cases) {
      assertThat(RepeatedErrors.pauseMillis(c.inARow()))
          .as("%d in a row", c.inARow())
          .isEqualTo(c.pauseMillis());
    }
  }

  @Test
  void aSuccessStartsTheCountAgain() {
    RepeatedErrors errors = new RepeatedErrors();
    for (int i = 0; i < RepeatedErrors.FREE; i++) {
      errors.failed();
    }
    errors.succeeded();
    long started = System.nanoTime();
    errors.failed();
    assertThat(System.nanoTime() - started)
        .as("the first error after a success")
        .isLessThan(5_000_000L);
  }
}

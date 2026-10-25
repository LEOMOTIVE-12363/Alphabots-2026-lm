package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import org.junit.jupiter.api.Test;

class LEDFourthAcceptanceTest {
  static class Bus implements LEDIOCandle.Transport {
    int configs;
    boolean failConfig, reset;

    public StatusCode configureStrip(StripTypeValue type) {
      configs++;
      return failConfig ? StatusCode.NoDevicesOnBus : StatusCode.OK;
    }

    public StatusCode clearAnimations() {
      return StatusCode.OK;
    }

    public StatusCode paint(int a, int b, int r, int g, int blue) {
      return StatusCode.OK;
    }

    public boolean hasResetOccurred() {
      boolean result = reset;
      reset = false;
      return result;
    }
  }

  @Test
  void chaseThumbnailEndpointMatchesExternalEndpoint() {
    var strip = new AddressableLEDBuffer(LEDConstants.LENGTH_PIXELS);
    var board = new AddressableLEDBuffer(8);
    double now = (strip.getLength() - 1) / LEDConstants.CHASE_SPEED_PIXELS_PER_SEC;
    LED.renderLegacyInto(strip, LEDPattern.CHASE, now, 100);
    LED.renderLegacyInto(board, LEDPattern.CHASE, now, 100);
    assertEquals(
        strip.getLED(strip.getLength() - 1),
        board.getLED(7),
        "u=1 must represent the actual final external pixel at t=" + now);
  }

  @Test
  void chaseThumbnailMustBeDarkWhenExternalTailHasExited() {
    var strip = new AddressableLEDBuffer(LEDConstants.LENGTH_PIXELS);
    var board = new AddressableLEDBuffer(8);
    double now =
        (strip.getLength() - 1 + LEDConstants.CHASE_TAIL_PIXELS)
            / LEDConstants.CHASE_SPEED_PIXELS_PER_SEC;
    LED.renderLegacyInto(strip, LEDPattern.CHASE, now, 100);
    LED.renderLegacyInto(board, LEDPattern.CHASE, now, 100);
    for (int i = 0; i < strip.getLength(); i++)
      assertEquals(0, strip.getRed(i) + strip.getGreen(i) + strip.getBlue(i));
    assertEquals(
        0,
        board.getRed(7) + board.getGreen(7) + board.getBlue(7),
        "Thumbnail tail must also exit at t=" + now);
  }

  @Test
  void failedKeepaliveMustRespectRetryInterval() {
    checkRetry(false);
  }

  @Test
  void failedResetInitMustRespectRetryInterval() {
    checkRetry(true);
  }

  void checkRetry(boolean reset) {
    var bus = new Bus();
    double[] t = {0};
    try (var io = new LEDIOCandle(bus, 200, false, .05, 24, () -> t[0])) {
      var frame = new AddressableLEDBuffer(200);
      io.writeBuffer(frame);
      bus.failConfig = true;
      bus.reset = reset;
      t[0] = 10;
      io.writeBuffer(frame);
      io.awaitConfigWork();
      assertEquals(2, bus.configs, "construction plus failed maintenance attempt");
      t[0] = 10.02;
      io.writeBuffer(frame);
      io.awaitConfigWork();
      assertEquals(
          2,
          bus.configs,
          "Only 20ms after failed config: retry must wait its configured 2s interval");
    }
  }
}

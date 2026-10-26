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

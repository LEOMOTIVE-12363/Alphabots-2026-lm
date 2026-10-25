package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.util.Arrays;
import org.junit.jupiter.api.Test;

class LEDIndependentAcceptanceTest {
  static class Bus implements LEDIOCandle.Transport {
    int[] displayed = new int[400];
    int paints;
    boolean fail;

    public StatusCode configureStrip(StripTypeValue v) {
      return StatusCode.OK;
    }

    public StatusCode clearAnimations() {
      return StatusCode.OK;
    }

    public StatusCode paint(int start, int end, int r, int g, int b) {
      paints++;
      if (fail) return StatusCode.NoDevicesOnBus;
      Arrays.fill(displayed, start, end + 1, (r << 16) | (g << 8) | b);
      return StatusCode.OK;
    }
  }

  static AddressableLEDBuffer gradient() {
    var b = new AddressableLEDBuffer(60);
    for (int i = 0; i < 60; i++) b.setRGB(i, i + 1, 30, 50);
    return b;
  }

  @Test
  void offAfterPartialGradientMustClearAllPreviouslyPaintedPixels() {
    Bus bus = new Bus();
    double[] time = {0};
    var io = new LEDIOCandle(bus, 60, false, .05, 24, () -> time[0]);
    var off = new AddressableLEDBuffer(60);
    io.writeBuffer(off);
    time[0] = .06;
    io.writeBuffer(gradient());
    assertNotEquals(0, bus.displayed[8]);
    for (int i = 0; i < 10; i++) {
      time[0] += .06;
      io.writeBuffer(off);
    }
    assertTrue(
        Arrays.stream(bus.displayed, 8, 68).allMatch(v -> v == 0),
        "Off was skipped because cached full frame still said black");
  }

  @Test
  void failedOffMustRetryOnceTransportRecovers() {
    Bus bus = new Bus();
    double[] time = {0};
    var io = new LEDIOCandle(bus, 60, false, .05, 24, () -> time[0]);
    var solid = new AddressableLEDBuffer(60);
    for (int i = 0; i < 60; i++) solid.setRGB(i, 200, 0, 0);
    io.writeBuffer(solid);
    assertNotEquals(0, bus.displayed[8]);
    bus.fail = true;
    time[0] = .06;
    var off = new AddressableLEDBuffer(60);
    io.writeBuffer(off);
    bus.fail = false;
    for (int i = 0; i < 10; i++) {
      time[0] += .06;
      io.writeBuffer(off);
    }
    assertEquals(0, bus.displayed[8], "Failed Off was cached as successful and never retried");
  }

  @Test
  void unchangedStaticGradientMustSettleAfterBoundedFullRefresh() {
    Bus bus = new Bus();
    double[] time = {0};
    var io = new LEDIOCandle(bus, 60, false, .05, 24, () -> time[0]);
    var b = gradient();
    for (int i = 0; i < 10; i++) {
      time[0] += .06;
      io.writeBuffer(b);
    }
    for (int i = 0; i < 60; i++)
      assertEquals(((i + 1) << 16) | (30 << 8) | 50, bus.displayed[i + 8]);
    int settled = bus.paints;
    for (int i = 0; i < 10; i++) {
      time[0] += .06;
      io.writeBuffer(b);
    }
    assertEquals(
        settled, bus.paints, "Static gradient must not keep using full update budget forever");
  }
}

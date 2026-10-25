package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import org.junit.jupiter.api.Test;

/**
 * Codex round-2 independent acceptance (2026-10-24 evening), ported into the suite. Adapted
 * 2026-10-25 for the third-round P1 fix: blocking device configuration now runs on a background
 * worker instead of inside {@code writeBuffer}, so these tests await the worker ({@link
 * LEDIOCandle#awaitConfigWork()}) and then drive a few more ticks — the asserted recovery behaviors
 * and their strength are unchanged.
 */
class LEDRestartAcceptanceTest {
  static class Bus implements LEDIOCandle.Transport {
    boolean ready;
    boolean configured;
    int[] shown = new int[400];

    public StatusCode configureStrip(StripTypeValue s) {
      configured = ready;
      return ready ? StatusCode.OK : StatusCode.NoDevicesOnBus;
    }

    public StatusCode clearAnimations() {
      return ready ? StatusCode.OK : StatusCode.NoDevicesOnBus;
    }

    public StatusCode paint(int a, int b, int r, int g, int bl) {
      if (!ready) return StatusCode.NoDevicesOnBus;
      java.util.Arrays.fill(shown, a, b + 1, (r << 16) | (g << 8) | bl);
      return StatusCode.OK;
    }
  }

  @Test
  void configurationRecoversWhenDeviceBecomesAvailableAfterStartup() {
    var bus = new Bus();
    double[] t = {0};
    var io = new LEDIOCandle(bus, 60, false, .05, 24, () -> t[0]);
    bus.ready = true;
    var frame = new AddressableLEDBuffer(60);
    for (int i = 0; i < 60; i++) frame.setRGB(i, 100, 0, 0);
    for (int i = 0; i < 200; i++) {
      t[0] += .06;
      io.writeBuffer(frame);
    }
    io.awaitConfigWork();
    assertTrue(
        bus.configured,
        "Device startup failure must not leave essential power/color config permanently unapplied");
  }

  @Test
  void staticFrameRestoredAfterDeviceReset() {
    var bus = new Bus();
    bus.ready = true;
    double[] t = {0};
    var io = new LEDIOCandle(bus, 60, false, .05, 24, () -> t[0]);
    var frame = new AddressableLEDBuffer(60);
    for (int i = 0; i < 60; i++) frame.setRGB(i, 100, 0, 0);
    io.writeBuffer(frame);
    assertEquals(100 << 16, bus.shown[8]);
    java.util.Arrays.fill(bus.shown, 0);
    for (int i = 0; i < 200; i++) {
      t[0] += .06;
      io.writeBuffer(frame);
    }
    io.awaitConfigWork();
    for (int i = 0; i < 3; i++) {
      t[0] += .06;
      io.writeBuffer(frame);
    }
    assertEquals(
        100 << 16,
        bus.shown[8],
        "Device reset must invalidate cached successful colors, even with static target");
  }
}

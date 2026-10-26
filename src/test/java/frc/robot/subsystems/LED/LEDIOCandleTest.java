package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.Test;

/**
 * Behavior tests for the CANdle driver against a recording fake transport (this checks the requests
 * the driver issues, NOT real CAN communication). They pin the onboard/external index split, the
 * onboard logical thumbnail (driver displays exactly the render layer's colors — never a re-sample
 * of the physical strip), the per-pixel confirmation bounded-send strategy, the bounded recovery
 * model (init retry, reset re-config, keepalive refresh — executed on the background config worker
 * since the 2026-10-25 third acceptance round, hence the {@code awaitConfigWork()} barriers), the
 * strip-type color-order handling, and the never-throw fault isolation.
 *
 * <p>The acceptance-prefixed tests are ported from Codex's independent acceptance rounds of
 * 2026-10-24 (three cache defects, then two recovery defects) with their intent and assertion
 * strength kept; recovery assertions wait for the background worker and then drive a few more
 * ticks, which is the async-preserving form of the original synchronous assertions.
 */
class LEDIOCandleTest {
  private static final int PIXELS = 10;
  private static final double STEP_SEC = 0.06; // just past the 50 ms throttle interval
  private static final double REFRESH_SEC = LEDConstants.CANDLE_REFRESH_INTERVAL_SEC;
  private static final double INIT_RETRY_SEC = LEDConstants.CANDLE_INIT_RETRY_INTERVAL_SEC;

  /**
   * Records every device operation and models the device display: a successful paint fills its
   * range, a failed one changes nothing. {@code ready} gates every device call, {@code responsive}
   * models the cached status signal, and {@link #resetDevice()} wipes the display and raises the
   * client-side reset flag (consumed when read, like Phoenix 6 hasResetOccurred).
   */
  private static final class FakeTransport implements LEDIOCandle.Transport {
    record Paint(int start, int end, int red, int green, int blue) {}

    final List<Paint> paints = new ArrayList<>();
    final int[] displayed = new int[400];
    StripTypeValue configuredStripType;
    boolean clearedAnimations;
    boolean ready = true;
    boolean responsive = true;
    boolean resetPending;
    int configCalls;
    int configSuccesses;
    StatusCode statusToReturn = StatusCode.OK;
    int failFromDeviceIndex = Integer.MAX_VALUE;
    int failUntilDeviceIndex = Integer.MAX_VALUE;

    /** When set, configureStrip signals {@link #configureStarted} and parks until released. */
    CountDownLatch configureGate;

    CountDownLatch configureStarted;

    @Override
    public StatusCode configureStrip(StripTypeValue stripType) {
      if (configureStarted != null) {
        configureStarted.countDown();
      }
      if (configureGate != null) {
        try {
          configureGate.await();
        } catch (InterruptedException e) {
          Thread.currentThread().interrupt();
        }
      }
      configCalls++;
      if (!ready || statusToReturn != StatusCode.OK) {
        return StatusCode.NoDevicesOnBus;
      }
      configuredStripType = stripType;
      configSuccesses++;
      return StatusCode.OK;
    }

    @Override
    public StatusCode clearAnimations() {
      if (ready && statusToReturn == StatusCode.OK) {
        clearedAnimations = true;
        return StatusCode.OK;
      }
      return StatusCode.NoDevicesOnBus;
    }

    @Override
    public StatusCode paint(int start, int end, int red, int green, int blue) {
      paints.add(new Paint(start, end, red, green, blue));
      if (!ready
          || statusToReturn != StatusCode.OK
          || (start >= failFromDeviceIndex && start <= failUntilDeviceIndex)) {
        return StatusCode.NoDevicesOnBus;
      }
      Arrays.fill(displayed, start, end + 1, (red << 16) | (green << 8) | blue);
      return StatusCode.OK;
    }

    @Override
    public boolean isDeviceResponsive() {
      return responsive;
    }

    @Override
    public boolean hasResetOccurred() {
      boolean occurred = resetPending;
      resetPending = false; // consumed on read, like the Phoenix 6 client-side flag
      return occurred;
    }

    /** Device power-cycle: display cleared, config reverted, reset flag raised. */
    void resetDevice() {
      Arrays.fill(displayed, 0);
      configuredStripType = null;
      resetPending = true;
    }

    /** Paints issued after the given index (i.e. not the init-time onboard dimming). */
    List<Paint> paintsFrom(int index) {
      return paints.subList(index, paints.size());
    }
  }

  /** Mutable test clock; the driver never reads time anywhere else. */
  private static final class Clock implements DoubleSupplier {
    double now;

    @Override
    public double getAsDouble() {
      return now;
    }
  }

  private static LEDIOCandle driver(
      FakeTransport transport, Clock clock, int pixels, int maxFramesPerUpdate) {
    return new LEDIOCandle(transport, pixels, false, 0.050, maxFramesPerUpdate, clock);
  }

  private static AddressableLEDBuffer bufferWithColor(int pixels, int red, int green, int blue) {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(pixels);
    for (int i = 0; i < pixels; i++) {
      buffer.setRGB(i, red, green, blue);
    }
    return buffer;
  }

  private static AddressableLEDBuffer gradientBuffer(int pixels) {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(pixels);
    for (int i = 0; i < pixels; i++) {
      buffer.setRGB(i, i * 2 + 1, 255 - i * 2, (i * 7) % 256);
    }
    return buffer;
  }

  /** Eight distinct onboard thumbnail colors, reversed order (i grows, brightness constant). */
  private static AddressableLEDBuffer thumbnailBuffer() {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(LEDConstants.ONBOARD_THUMBNAIL_PIXELS);
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, 200 - i * 10, i * 12, 60 + i * 5);
    }
    return buffer;
  }

  private static int packed(AddressableLEDBuffer buffer, int i) {
    return (buffer.getRed(i) << 16) | (buffer.getGreen(i) << 8) | buffer.getBlue(i);
  }

  @Test
  void initConfiguresRgbStripClearsAnimationsAndDimsOnboardLeds() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();

    driver(transport, clock, PIXELS, 64);

    assertEquals(StripTypeValue.RGB, transport.configuredStripType);
    assertTrue(transport.clearedAnimations, "animation slots must be cleared once at init");
    assertEquals(1, transport.paints.size(), "init must issue exactly the onboard dimming paint");
    FakeTransport.Paint onboard = transport.paints.get(0);
    assertEquals(0, onboard.start());
    assertEquals(LEDIOCandle.ONBOARD_LED_COUNT - 1, onboard.end());
    assertEquals(0, onboard.red());
    assertEquals(0, onboard.green());
    assertEquals(0, onboard.blue());
  }

  @Test
  void redGreenSwapFlagSelectsGrbStripTypeInsteadOfSoftwareSwap() {
    FakeTransport transport = new FakeTransport();
    LEDIOCandle swapped = new LEDIOCandle(transport, PIXELS, true, 0.050, 64, new Clock());

    assertEquals(StripTypeValue.GRB, transport.configuredStripType);

    // The paint payloads stay logical RGB (onboard and external alike) — the external channel
    // order is the hardware config's job, and the firmware compensates the onboard LEDs itself.
    swapped.writeOnboardBuffer(bufferWithColor(LEDConstants.ONBOARD_THUMBNAIL_PIXELS, 10, 20, 30));
    swapped.writeBuffer(bufferWithColor(PIXELS, 10, 20, 30));
    for (FakeTransport.Paint paint : transport.paintsFrom(1)) {
      assertEquals(10, paint.red());
      assertEquals(20, paint.green());
      assertEquals(30, paint.blue());
    }
    assertEquals((10 << 16) | (20 << 8) | 30, transport.displayed[0], "onboard logical color");
  }

  @Test
  void stripConfigPinsBrightnessAndPowersOnboardAndVbatStrip() {
    // Brightness is applied once in the render layer, so the device-side scalar must be pinned to
    // 1.0; the 12 V strip must be powered from VBat; the 5 V rail must be ENABLED because it feeds
    // the onboard LEDs (a disabled rail turns them off; nothing external hangs on it).
    var config = LEDIOCandle.buildStripConfig(StripTypeValue.RGB);
    assertEquals(1.0, config.LED.BrightnessScalar);
    assertEquals(VBatOutputModeValue.On, config.CANdleFeatures.VBatOutputMode);
    assertEquals(Enable5VRailValue.Enabled, config.CANdleFeatures.Enable5VRail);
    assertEquals(StripTypeValue.RGB, config.LED.StripType);
  }

  @Test
  void externalPixelsAreOffsetPastTheEightOnboardLeds() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, PIXELS, 64);

    io.writeOnboardBuffer(thumbnailBuffer());
    io.writeBuffer(gradientBuffer(PIXELS));

    List<FakeTransport.Paint> onboardPaints = new ArrayList<>();
    List<FakeTransport.Paint> externalPaints = new ArrayList<>();
    for (FakeTransport.Paint paint : transport.paintsFrom(1)) {
      if (paint.end() < LEDIOCandle.ONBOARD_LED_COUNT) {
        onboardPaints.add(paint);
      } else {
        assertTrue(
            paint.start() >= LEDIOCandle.ONBOARD_LED_COUNT,
            "external paint must start at device index 8+");
        externalPaints.add(paint);
      }
    }
    assertEquals(PIXELS, externalPaints.size(), "distinct colors => one paint per pixel");
    for (int i = 0; i < PIXELS; i++) {
      FakeTransport.Paint paint = externalPaints.get(i);
      assertEquals(LEDIOCandle.ONBOARD_LED_COUNT + i, paint.start(), "start of pixel " + i);
      assertEquals(LEDIOCandle.ONBOARD_LED_COUNT + i, paint.end(), "end of pixel " + i);
    }
    assertEquals(
        LEDConstants.ONBOARD_THUMBNAIL_PIXELS,
        onboardPaints.size(),
        "eight onboard thumbnail pixels, each its own run (distinct logical colors)");
  }

  @Test
  void adjacentSameColorPixelsCoalesceIntoOneFramePerSegment() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, PIXELS, 64);

    io.writeOnboardBuffer(bufferWithColor(LEDConstants.ONBOARD_THUMBNAIL_PIXELS, 1, 2, 3));
    io.writeBuffer(bufferWithColor(PIXELS, 1, 2, 3));

    List<FakeTransport.Paint> paints = transport.paintsFrom(1);
    assertEquals(2, paints.size(), "one onboard run + one external run");
    assertEquals(0, paints.get(0).start());
    assertEquals(LEDIOCandle.ONBOARD_LED_COUNT - 1, paints.get(0).end());
    assertEquals(LEDIOCandle.ONBOARD_LED_COUNT, paints.get(1).start());
    assertEquals(LEDIOCandle.ONBOARD_LED_COUNT + PIXELS - 1, paints.get(1).end());
    assertEquals(1, paints.get(1).red());
    assertEquals(2, paints.get(1).green());
    assertEquals(3, paints.get(1).blue());
  }

  @Test
  void offClearsBothSegmentsInOneTickAndThenSendsNothing() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, PIXELS, 64);

    io.writeOnboardBuffer(thumbnailBuffer());
    io.writeBuffer(gradientBuffer(PIXELS));
    int afterGradient = transport.paints.size();

    clock.now += STEP_SEC;
    io.writeOnboardBuffer(new AddressableLEDBuffer(LEDConstants.ONBOARD_THUMBNAIL_PIXELS));
    io.writeBuffer(bufferWithColor(PIXELS, 0, 0, 0));
    int afterOff = transport.paints.size();

    List<FakeTransport.Paint> offPaints = transport.paints.subList(afterGradient, afterOff);
    assertEquals(2, offPaints.size(), "Off must clear onboard and external in one tick");
    for (FakeTransport.Paint paint : offPaints) {
      assertEquals(0, paint.red());
      assertEquals(0, paint.green());
      assertEquals(0, paint.blue());
    }
    for (int device = 0; device < LEDIOCandle.ONBOARD_LED_COUNT + PIXELS; device++) {
      assertEquals(0, transport.displayed[device], "device index " + device + " must be dark");
    }

    clock.now += STEP_SEC;
    io.writeOnboardBuffer(new AddressableLEDBuffer(LEDConstants.ONBOARD_THUMBNAIL_PIXELS));
    io.writeBuffer(bufferWithColor(PIXELS, 0, 0, 0));
    assertEquals(afterOff, transport.paints.size(), "a settled display must cost zero traffic");
  }

  @Test
  void framesPerUpdateAreBoundedAndEveryPixelGetsRefreshed() {
    int pixels = 60; // the placeholder strip length, all pixels distinct
    int maxFrames = 10;
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, pixels, maxFrames);
    AddressableLEDBuffer gradient = gradientBuffer(pixels);
    AddressableLEDBuffer thumbnail = thumbnailBuffer();

    io.writeOnboardBuffer(thumbnail);
    io.writeBuffer(gradient);
    int mark = transport.paints.size();
    int maxSeenPerTick = mark - 1; // minus the init onboard paint
    boolean[] covered = new boolean[LEDIOCandle.ONBOARD_LED_COUNT + pixels];
    // The initial write's paints count toward coverage too — those pixels are confirmed and will
    // not be re-sent by later ticks.
    for (FakeTransport.Paint paint : transport.paintsFrom(1)) {
      for (int device = paint.start(); device <= paint.end(); device++) {
        covered[device] = true;
      }
    }
    for (int tick = 0; tick < 12 && !allCovered(covered); tick++) {
      clock.now += STEP_SEC;
      io.writeOnboardBuffer(thumbnail);
      io.writeBuffer(gradient);
      int sentThisTick = transport.paints.size() - mark;
      maxSeenPerTick = Math.max(maxSeenPerTick, sentThisTick);
      mark = transport.paints.size();
      for (FakeTransport.Paint paint : transport.paintsFrom(mark - sentThisTick)) {
        for (int device = paint.start(); device <= paint.end(); device++) {
          covered[device] = true;
        }
      }
    }

    assertTrue(maxSeenPerTick <= maxFrames, "no update tick may exceed the frame budget");
    assertTrue(allCovered(covered), "a bounded sequence of ticks must reach every pixel");
  }

  @Test
  void updatesInsideTheIntervalAreSkipped() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, PIXELS, 64);

    io.writeOnboardBuffer(bufferWithColor(LEDConstants.ONBOARD_THUMBNAIL_PIXELS, 9, 9, 9));
    io.writeBuffer(bufferWithColor(PIXELS, 9, 9, 9));
    int afterFirst = transport.paints.size();
    assertNotEquals(1, afterFirst); // sanity: the first write actually sent

    clock.now += 0.010; // inside the 50 ms interval
    io.writeOnboardBuffer(bufferWithColor(LEDConstants.ONBOARD_THUMBNAIL_PIXELS, 8, 8, 8));
    io.writeBuffer(bufferWithColor(PIXELS, 8, 8, 8));
    assertEquals(afterFirst, transport.paints.size(), "throttled tick must not send");

    clock.now += 0.050; // now eligible
    io.writeOnboardBuffer(bufferWithColor(LEDConstants.ONBOARD_THUMBNAIL_PIXELS, 8, 8, 8));
    io.writeBuffer(bufferWithColor(PIXELS, 8, 8, 8));
    assertEquals(afterFirst + 2, transport.paints.size(), "eligible tick must send the new colors");
  }

  @Test
  void deviceFailuresNeverThrowAndRequestsStayBounded() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, PIXELS, 4);
    transport.statusToReturn = StatusCode.NoDevicesOnBus;

    io.writeBuffer(gradientBuffer(PIXELS));
    clock.now += STEP_SEC;
    io.writeBuffer(gradientBuffer(PIXELS));
    clock.now += STEP_SEC;
    io.writeBuffer(gradientBuffer(PIXELS));

    // A dead device must neither crash the loop nor flood the bus: init paint + at most the frame
    // budget per eligible tick across all writes above.
    assertTrue(transport.paints.size() <= 1 + 4 * 3);
    assertNotNull(transport.configuredStripType, "init ran to completion despite failures");
  }

  @Test
  void pixelsBeyondTheCandleExternalMaximumAreClamped() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    int overlong = LEDIOCandle.MAX_EXTERNAL_PIXELS + 20;
    LEDIOCandle io = driver(transport, clock, overlong, 1000);

    io.writeBuffer(gradientBuffer(overlong));

    for (FakeTransport.Paint paint : transport.paintsFrom(1)) {
      assertTrue(
          paint.end() <= 7 + LEDIOCandle.MAX_EXTERNAL_PIXELS,
          "paint end " + paint.end() + " exceeds device index 399");
    }
  }

  @Test
  void simFactoryRunsOfflineAgainstTheNoneStub() {
    LEDIOCandle io = LEDIOCandle.forSim(false);
    assertNotNull(io);
    // CANdle.none() performs no device I/O; writing must complete cleanly without any CAN device.
    io.writeBuffer(gradientBuffer(LEDConstants.LENGTH_PIXELS));
  }

  @Test
  void configuredSevenMeterStripPaintsAllOneHundredFortyPixels() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io =
        driver(
            transport,
            clock,
            LEDConstants.LENGTH_PIXELS,
            LEDConstants.CANDLE_MAX_FRAMES_PER_UPDATE);
    AddressableLEDBuffer frame = new AddressableLEDBuffer(140);
    AddressableLEDBuffer thumbnail = thumbnailBuffer();
    for (int i = 0; i < 140; i++) {
      // Every channel is in 0-255, and red = i makes all 140 colors distinct.
      frame.setRGB(i, i, 255 - i, (i * 73 + 19) % 256);
    }

    // 8 onboard + 140 external pixels = 148 distinct runs. The hand-derived worst-case bound is
    // ceil(148 / 24) = 7 ticks, with no tick allowed to exceed 24 frames.
    for (int tick = 0; tick < 7; tick++) {
      clock.now += STEP_SEC;
      int before = transport.paints.size();
      io.writeOnboardBuffer(thumbnail);
      io.writeBuffer(frame);
      assertTrue(
          transport.paints.size() - before <= 24,
          "refresh tick " + (tick + 1) + " must stay within the 24-frame budget");
    }

    for (int i = 0; i < 140; i++) {
      int expected = (i << 16) | ((255 - i) << 8) | ((i * 73 + 19) % 256);
      assertEquals(
          expected,
          transport.displayed[i + 8],
          "7 m strip external pixel " + i + " (device index " + (i + 8) + ")");
    }
    assertEquals(7.0, LEDConstants.LENGTH_METERS);
    assertEquals(20.0, LEDConstants.PIXELS_PER_METER);
    assertEquals(140, LEDConstants.LENGTH_PIXELS);
  }

  // ---- Round-1 acceptance ports (Codex independent acceptance, 2026-10-24 morning) ------------

  @Test
  void acceptanceOffAfterPartialGradientMustClearAllPreviouslyPaintedPixels() {
    FakeTransport transport = new FakeTransport();
    double[] time = {0};
    LEDIOCandle io = new LEDIOCandle(transport, 60, false, 0.05, 24, () -> time[0]);
    AddressableLEDBuffer off = new AddressableLEDBuffer(60);
    io.writeBuffer(off);
    time[0] = 0.06;
    io.writeBuffer(gradientBuffer(60));
    assertNotEquals(0, transport.displayed[8]);
    for (int i = 0; i < 10; i++) {
      time[0] += 0.06;
      io.writeBuffer(off);
    }
    for (int device = 0; device < 8 + 60; device++) {
      assertEquals(
          0,
          transport.displayed[device],
          "Off was skipped because cached state still said dark (device " + device + ")");
    }
  }

  @Test
  void acceptanceFailedOffMustRetryOnceTransportRecovers() {
    FakeTransport transport = new FakeTransport();
    double[] time = {0};
    LEDIOCandle io = new LEDIOCandle(transport, 60, false, 0.05, 24, () -> time[0]);
    io.writeBuffer(bufferWithColor(60, 200, 0, 0));
    assertNotEquals(0, transport.displayed[8]);
    transport.statusToReturn = StatusCode.NoDevicesOnBus;
    time[0] = 0.06;
    io.writeBuffer(new AddressableLEDBuffer(60));
    transport.statusToReturn = StatusCode.OK;
    for (int i = 0; i < 10; i++) {
      time[0] += 0.06;
      io.writeBuffer(new AddressableLEDBuffer(60));
    }
    assertEquals(
        0, transport.displayed[8], "Failed Off was cached as successful and never retried");
  }

  @Test
  void acceptanceUnchangedStaticGradientMustSettleAfterBoundedFullRefresh() {
    FakeTransport transport = new FakeTransport();
    double[] time = {0};
    LEDIOCandle io = new LEDIOCandle(transport, 60, false, 0.05, 24, () -> time[0]);
    AddressableLEDBuffer gradient = gradientBuffer(60);
    for (int i = 0; i < 10; i++) {
      time[0] += 0.06;
      io.writeBuffer(gradient);
    }
    for (int i = 0; i < 60; i++) {
      assertEquals(packed(gradient, i), transport.displayed[i + 8], "external pixel " + i);
    }
    int settled = transport.paints.size();
    for (int i = 0; i < 10; i++) {
      time[0] += 0.06;
      io.writeBuffer(gradient);
    }
    assertEquals(
        settled,
        transport.paints.size(),
        "Static gradient must not keep using full budget forever");
  }

  // ---- Round-2 acceptance ports (recovery round, 2026-10-24 evening) --------------------------

  @Test
  void acceptanceConfigurationRecoversWhenDeviceBecomesAvailableAfterStartup() {
    FakeTransport transport = new FakeTransport();
    transport.ready = false; // device powers up after the robot program
    double[] time = {0};
    LEDIOCandle io = new LEDIOCandle(transport, 60, false, 0.05, 24, () -> time[0]);
    assertEquals(0, transport.configSuccesses, "sanity: startup config failed");
    transport.ready = true;
    AddressableLEDBuffer frame = bufferWithColor(60, 100, 0, 0);
    for (int i = 0; i < 200; i++) {
      time[0] += 0.06;
      io.writeBuffer(frame);
    }
    io.awaitConfigWork();
    assertTrue(
        transport.configSuccesses >= 1,
        "Device startup failure must not leave essential power/color config permanently unapplied");
    assertEquals(StripTypeValue.RGB, transport.configuredStripType);
  }

  @Test
  void acceptanceStaticFrameRestoredAfterDeviceReset() {
    FakeTransport transport = new FakeTransport();
    double[] time = {0};
    LEDIOCandle io = new LEDIOCandle(transport, 60, false, 0.05, 24, () -> time[0]);
    AddressableLEDBuffer frame = bufferWithColor(60, 100, 0, 0);
    io.writeBuffer(frame);
    assertEquals(100 << 16, transport.displayed[8]);
    transport.resetDevice(); // device power-cycle behind the driver's back: display wiped
    for (int i = 0; i < 200; i++) {
      time[0] += 0.06;
      io.writeBuffer(frame);
    }
    io.awaitConfigWork();
    for (int i = 0; i < 3; i++) {
      time[0] += 0.06;
      io.writeBuffer(frame);
    }
    assertEquals(
        100 << 16,
        transport.displayed[8],
        "Device reset must invalidate cached successful colors, even with static target");
  }

  @Test
  void failedResetReinitializationRetriesAndRestoresConfiguration() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, 60, 24);
    AddressableLEDBuffer frame = bufferWithColor(60, 100, 0, 0);
    io.writeBuffer(frame);
    transport.resetDevice();
    transport.statusToReturn = StatusCode.NoDevicesOnBus;

    clock.now += REFRESH_SEC + STEP_SEC;
    io.writeBuffer(frame);
    io.awaitConfigWork(); // the reset reinitialization runs on the background config worker
    assertEquals(1, transport.configSuccesses, "sanity: reset reinitialization failed");

    transport.statusToReturn = StatusCode.OK;
    for (int i = 0; i < 200; i++) {
      clock.now += STEP_SEC;
      io.writeBuffer(frame);
    }
    io.awaitConfigWork();

    assertTrue(
        transport.configSuccesses >= 2,
        "a failed reset reinitialization must enter the bounded retry path");
    assertEquals(StripTypeValue.RGB, transport.configuredStripType);
    assertEquals(100 << 16, transport.displayed[8]);
  }

  @Test
  void undetectedResetRestoresConfigurationAndLatestFrame() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, 60, 24);
    AddressableLEDBuffer frame = bufferWithColor(60, 100, 0, 0);
    io.writeBuffer(frame);
    transport.resetDevice();
    transport.resetPending = false; // no reset signal reaches the driver

    for (int i = 0; i < 200; i++) {
      clock.now += STEP_SEC;
      io.writeBuffer(frame);
    }
    io.awaitConfigWork();
    for (int i = 0; i < 3; i++) {
      clock.now += STEP_SEC;
      io.writeBuffer(frame);
    }

    assertEquals(
        StripTypeValue.RGB,
        transport.configuredStripType,
        "an undetected reset must not leave the device at flash-default configuration");
    assertEquals(100 << 16, transport.displayed[8]);
  }

  @Test
  void mixedPaintResultsRateLimitFaultsWithoutFalseRecovery() {
    java.io.ByteArrayOutputStream capture = new java.io.ByteArrayOutputStream();
    java.io.PrintStream originalErr = System.err;
    System.setErr(new java.io.PrintStream(capture));
    try {
      FakeTransport transport = new FakeTransport();
      transport.failFromDeviceIndex = 40;
      transport.failUntilDeviceIndex = 47;
      Clock clock = new Clock();
      LEDIOCandle io = driver(transport, clock, 60, 64);

      io.writeBuffer(gradientBuffer(60));
      String firstTick = capture.toString();
      org.junit.jupiter.api.Assertions.assertFalse(
          firstTick.contains("CAN communication recovered"),
          "mixed success/failure must not be reported as recovery");
      int firstFaultLogs = firstTick.split("CANdle update failed", -1).length - 1;
      assertEquals(1, firstFaultLogs, "one mixed update must produce one fault report");

      clock.now += STEP_SEC;
      io.writeBuffer(gradientBuffer(60));
      String secondTick = capture.toString();
      int secondFaultLogs = secondTick.split("CANdle update failed", -1).length - 1;
      assertEquals(firstFaultLogs, secondFaultLogs, "fault reports must stay rate-limited");
      org.junit.jupiter.api.Assertions.assertFalse(
          secondTick.contains("CAN communication recovered"),
          "output is still degraded while any paint fails");
    } finally {
      System.setErr(originalErr);
    }
  }

  // ---- Recovery-model behavior ---------------------------------------------------------------

  @Test
  void unresponsiveDeviceNeverRetriesConfigFromTheLoop() {
    FakeTransport transport = new FakeTransport();
    transport.ready = false;
    transport.responsive = false; // cached status signal says: no device answering
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, 60, 24);

    int configCallsAfterBoot = transport.configCalls; // the single construction-time attempt
    for (int i = 0; i < 200; i++) {
      clock.now += STEP_SEC;
      io.writeOnboardBuffer(thumbnailBuffer());
      io.writeBuffer(gradientBuffer(60));
    }

    assertEquals(
        configCallsAfterBoot,
        transport.configCalls,
        "an unresponsive device must not be re-configured from the loop (no blocking retries)");
    assertTrue(transport.paints.size() <= 1 + 24 * 200, "sanity: writes still bounded");
  }

  @Test
  void detectedResetReappliesConfigAndRepaintsLatestTarget() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, 60, 24);
    AddressableLEDBuffer first = bufferWithColor(60, 100, 0, 0);
    io.writeOnboardBuffer(thumbnailBuffer());
    io.writeBuffer(first);
    clock.now += STEP_SEC;
    io.writeBuffer(first);
    int configCallsBefore = transport.configCalls;
    int paintsBefore = transport.paints.size();

    transport.resetDevice(); // wipes display + reverts config + raises the reset flag
    AddressableLEDBuffer latest = bufferWithColor(60, 0, 0, 200);
    for (int i = 0; i < 100; i++) {
      clock.now += STEP_SEC;
      io.writeBuffer(latest);
    }
    io.awaitConfigWork();
    for (int i = 0; i < 3; i++) {
      clock.now += STEP_SEC;
      io.writeBuffer(latest);
    }

    assertTrue(
        transport.configCalls > configCallsBefore,
        "a detected reset must re-apply the device config (it reverts on reset)");
    for (int i = 0; i < 60; i++) {
      assertEquals(packed(latest, i), transport.displayed[i + 8], "external pixel " + i);
    }
    assertTrue(transport.paints.size() > paintsBefore, "the latest target must be repainted");
  }

  @Test
  void resetDuringPartialRefreshStillConvergesToTheLatestFrame() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, 60, 10); // partial refreshes: 68 runs / 10 per tick
    AddressableLEDBuffer gradient = gradientBuffer(60);
    io.writeOnboardBuffer(thumbnailBuffer());
    io.writeBuffer(gradient);
    clock.now += STEP_SEC;
    io.writeBuffer(gradient); // two partial ticks in

    transport.resetDevice(); // reset mid-partial-refresh
    for (int i = 0; i < 100; i++) {
      clock.now += STEP_SEC;
      io.writeOnboardBuffer(thumbnailBuffer());
      io.writeBuffer(gradient);
    }
    io.awaitConfigWork();
    for (int i = 0; i < 10; i++) { // ceil(68 runs / 10 frames) ticks to fully repaint
      clock.now += STEP_SEC;
      io.writeOnboardBuffer(thumbnailBuffer());
      io.writeBuffer(gradient);
    }
    for (int i = 0; i < 60; i++) {
      assertEquals(packed(gradient, i), transport.displayed[i + 8], "external pixel " + i);
    }
    for (int s = 0; s < LEDConstants.ONBOARD_THUMBNAIL_PIXELS; s++) {
      assertEquals(packed(thumbnailBuffer(), s), transport.displayed[s], "onboard LED " + s);
    }
  }

  @Test
  void healthyDeviceGetsBoundedConfigAndPaintKeepalive() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, 60, 24);
    AddressableLEDBuffer frame = bufferWithColor(60, 100, 0, 0);
    io.writeBuffer(frame);
    int configCallsAfterBoot = transport.configCalls;
    int settled = transport.paints.size();

    // ~12 s of a static frame: config and paint keepalive must both exist and stay bounded.
    for (int i = 0; i < 200; i++) {
      clock.now += STEP_SEC;
      io.writeBuffer(frame);
    }
    io.awaitConfigWork();
    for (int i = 0; i < 3; i++) {
      clock.now += STEP_SEC;
      io.writeBuffer(frame); // the keepalive's invalidation repaints the static frame
    }

    assertTrue(
        transport.configCalls > configCallsAfterBoot,
        "periodic config refresh must restore configuration after an undetected reset");
    assertTrue(
        transport.configCalls <= configCallsAfterBoot + Math.ceil(12.0 / REFRESH_SEC) + 1,
        "config refresh must stay bounded");
    int keepalivePaints = transport.paints.size() - settled;
    assertTrue(keepalivePaints > 0, "a bounded keepalive refresh must exist for static frames");
    assertTrue(
        keepalivePaints <= 2 * Math.ceil(12.0 / REFRESH_SEC) + 2,
        "keepalive traffic must stay bounded (a couple of frames per refresh window), got "
            + keepalivePaints);
  }

  // ---- Non-blocking configuration (third acceptance round, 2026-10-25) ------------------------

  /**
   * In-suite twin of the independent timing test: a slow config apply must never stall the loop.
   */
  @Test
  void configLatencyNeverStallsThePeriodicWrite() throws InterruptedException {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, PIXELS, 64);
    io.writeBuffer(bufferWithColor(PIXELS, 9, 9, 9)); // settle, so the measured tick only decides

    // The next keepalive's configureStrip parks on the worker until released (a busy CAN bus).
    transport.configureGate = new CountDownLatch(1);
    transport.configureStarted = new CountDownLatch(1);
    clock.now += REFRESH_SEC + STEP_SEC; // keepalive config becomes due
    long start = System.nanoTime();
    io.writeBuffer(bufferWithColor(PIXELS, 9, 9, 9));
    double elapsedMs = (System.nanoTime() - start) / 1e6;
    assertTrue(
        elapsedMs < 20.0,
        "CAN configuration latency must not stall the 20 ms robot loop: " + elapsedMs + " ms");

    transport.configureGate.countDown();
    io.awaitConfigWork();
  }

  @Test
  void sendsAreDeferredWhileAConfigTaskOwnsTheDevice() throws Exception {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, PIXELS, 64);
    io.writeBuffer(bufferWithColor(PIXELS, 10, 0, 0));
    transport.configureGate = new CountDownLatch(1);
    transport.configureStarted = new CountDownLatch(1);

    clock.now += REFRESH_SEC + STEP_SEC; // keepalive due: config task handed to the worker
    io.writeBuffer(bufferWithColor(PIXELS, 10, 0, 0));
    assertTrue(
        transport.configureStarted.await(5, TimeUnit.SECONDS),
        "the config task must actually reach the device");

    // The worker now owns the device (parked inside configureStrip under the hardware lock): a
    // dirty frame must neither block the loop nor race its paints against the config.
    clock.now += STEP_SEC;
    int paintsBefore = transport.paints.size();
    long start = System.nanoTime();
    io.writeBuffer(bufferWithColor(PIXELS, 0, 0, 200));
    double elapsedMs = (System.nanoTime() - start) / 1e6;
    assertTrue(elapsedMs < 20.0, "the loop must not wait for an in-flight config task");
    assertEquals(
        paintsBefore, transport.paints.size(), "no paints may run concurrently with the config");

    transport.configureGate.countDown();
    io.awaitConfigWork();
    for (int i = 0; i < 3; i++) {
      clock.now += STEP_SEC;
      io.writeBuffer(bufferWithColor(PIXELS, 0, 0, 200));
    }
    for (int i = 0; i < PIXELS; i++) {
      assertEquals(
          200,
          transport.displayed[i + 8],
          "external pixel " + i + " must catch up after the config");
    }
  }

  @Test
  void failedKeepaliveConfigRetriesAndRestoresTheLatestFrame() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, PIXELS, 64);
    AddressableLEDBuffer frame = bufferWithColor(PIXELS, 100, 0, 0);
    io.writeBuffer(frame);
    int successesAfterBoot = transport.configSuccesses; // the construction-time config

    clock.now += REFRESH_SEC + STEP_SEC; // keepalive due
    transport.statusToReturn = StatusCode.NoDevicesOnBus; // ... and the config fails
    io.writeBuffer(frame);
    io.awaitConfigWork();
    // The failed keepalive must not be cached as success: the driver becomes uninitialized.
    assertEquals(successesAfterBoot, transport.configSuccesses);

    transport.statusToReturn = StatusCode.OK;
    for (int i = 0; i < 200; i++) { // the bounded init-retry path takes over (device responsive)
      clock.now += STEP_SEC;
      io.writeBuffer(frame);
    }
    io.awaitConfigWork();
    for (int i = 0; i < 3; i++) {
      clock.now += STEP_SEC;
      io.writeBuffer(frame);
    }
    assertTrue(transport.configSuccesses > successesAfterBoot, "the failed config must be retried");
    assertEquals(StripTypeValue.RGB, transport.configuredStripType);
    assertEquals(100 << 16, transport.displayed[8]);
  }

  /**
   * Fourth acceptance round (2026-10-26): every ACCEPTED config task stamps the shared attempt
   * clock, so a failed keepalive/reset config is retried only after the full 2 s interval — and
   * once the interval expires and the device recovers, the retry lands and the latest frame shows.
   */
  @Test
  void failedConfigRetriesOnlyAfterTheIntervalAndThenRestoresTheLatestFrame() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, PIXELS, 64);
    AddressableLEDBuffer red = bufferWithColor(PIXELS, 100, 0, 0);
    io.writeBuffer(red);
    transport.statusToReturn = StatusCode.NoDevicesOnBus;

    clock.now += 3 * REFRESH_SEC; // keepalive due; the config fails
    io.writeBuffer(red);
    io.awaitConfigWork();
    int attemptsAfterFailure = transport.configCalls;

    clock.now += 0.02; // only 20 ms later — inside the 2 s retry interval
    io.writeBuffer(red);
    io.awaitConfigWork();
    assertEquals(
        attemptsAfterFailure,
        transport.configCalls,
        "a retry inside the configured interval must not configure again");

    transport.statusToReturn = StatusCode.OK;
    AddressableLEDBuffer blue = bufferWithColor(PIXELS, 0, 0, 200);
    clock.now += LEDConstants.CANDLE_INIT_RETRY_INTERVAL_SEC + STEP_SEC; // interval expired
    io.writeBuffer(blue);
    io.awaitConfigWork();
    assertTrue(
        transport.configCalls > attemptsAfterFailure, "the retry must fire once the interval ends");
    for (int i = 0; i < 3; i++) {
      clock.now += STEP_SEC;
      io.writeBuffer(blue);
    }
    for (int i = 0; i < PIXELS; i++) {
      assertEquals(200, transport.displayed[i + 8], "latest frame after recovery, pixel " + i);
    }
  }

  @Test
  void closeStopsNewConfigTasksAndDrainsTheWorker() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, PIXELS, 64);
    io.writeBuffer(bufferWithColor(PIXELS, 1, 2, 3));
    io.close();
    int configCallsAfterClose = transport.configCalls;

    clock.now += 3 * REFRESH_SEC; // keepalives would be due, but the driver is closed
    for (int i = 0; i < 10; i++) {
      clock.now += STEP_SEC;
      io.writeBuffer(bufferWithColor(PIXELS, 4, 5, 6));
    }
    io.awaitConfigWork();
    assertEquals(
        configCallsAfterClose,
        transport.configCalls,
        "a closed driver must not submit new config work");
  }

  // ---- Onboard logical thumbnail (driver side) ------------------------------------------------

  @Test
  void onboardShowsRenderSuppliedLogicalColorsNotExternalSampling() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    LEDIOCandle io = driver(transport, clock, 60, 64);
    AddressableLEDBuffer thumbnail = thumbnailBuffer();
    AddressableLEDBuffer external = gradientBuffer(60);

    io.writeOnboardBuffer(thumbnail);
    io.writeBuffer(external);
    clock.now += STEP_SEC;
    io.writeOnboardBuffer(thumbnail);
    io.writeBuffer(external); // settle

    // Device 0-7 must show exactly the render layer's logical colors — not samples of the external
    // strip, whatever its wiring order, reversed flags or decoration sections are.
    for (int s = 0; s < LEDConstants.ONBOARD_THUMBNAIL_PIXELS; s++) {
      assertEquals(packed(thumbnail, s), transport.displayed[s], "onboard LED " + s);
    }
  }

  @Test
  void onboardAndExternalShareOneFrameBudget() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    int maxFrames = 24;
    LEDIOCandle io = driver(transport, clock, 60, maxFrames);

    io.writeOnboardBuffer(thumbnailBuffer()); // 8 distinct logical colors
    io.writeBuffer(gradientBuffer(60)); // 60 distinct external colors

    List<FakeTransport.Paint> paints = transport.paintsFrom(1);
    assertEquals(maxFrames, paints.size(), "the tick must spend exactly the shared budget");
    boolean sawOnboard = false;
    boolean sawExternal = false;
    for (FakeTransport.Paint paint : paints) {
      sawOnboard |= paint.end() < LEDIOCandle.ONBOARD_LED_COUNT;
      sawExternal |= paint.start() >= LEDIOCandle.ONBOARD_LED_COUNT;
    }
    assertTrue(sawOnboard, "onboard pixels must share the same budget (not starved out)");
    assertTrue(sawExternal, "external pixels must still get part of the budget");
  }

  @Test
  void dynamicFramesRotateFairlyAndAlwaysUseTheLatestColors() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    int maxFrames = 24;
    LEDIOCandle io = driver(transport, clock, 60, maxFrames);
    AddressableLEDBuffer first = gradientBuffer(60);
    AddressableLEDBuffer firstThumbnail = thumbnailBuffer();

    io.writeOnboardBuffer(firstThumbnail);
    io.writeBuffer(first); // partial: only 24 of 68 runs leave
    AddressableLEDBuffer second = new AddressableLEDBuffer(60);
    for (int i = 0; i < 60; i++) {
      second.setRGB(i, 200 - i, (i * 3) % 256, 40);
    }
    AddressableLEDBuffer secondThumbnail =
        bufferWithColor(LEDConstants.ONBOARD_THUMBNAIL_PIXELS, 5, 6, 7);
    // ceil(68/24) ticks of the new frame must repaint everything; nothing may keep "first" colors.
    for (int tick = 0; tick < 3; tick++) {
      clock.now += STEP_SEC;
      io.writeOnboardBuffer(secondThumbnail);
      io.writeBuffer(second);
    }
    for (int i = 0; i < 60; i++) {
      assertEquals(
          packed(second, i),
          transport.displayed[i + 8],
          "external pixel " + i + " never caught up to the latest frame");
    }
    for (int s = 0; s < LEDConstants.ONBOARD_THUMBNAIL_PIXELS; s++) {
      assertEquals(
          packed(secondThumbnail, s),
          transport.displayed[s],
          "onboard LED " + s + " never caught up to the latest frame");
    }
  }

  @Test
  void mixedFailuresConvergeToTheLatestTargetWithinBudget() {
    FakeTransport transport = new FakeTransport();
    Clock clock = new Clock();
    int maxFrames = 24;
    LEDIOCandle io = driver(transport, clock, 60, maxFrames);
    AddressableLEDBuffer frameA = shiftBuffer(60, 0);
    AddressableLEDBuffer thumbnailA = thumbnailBuffer();

    io.writeOnboardBuffer(thumbnailA);
    io.writeBuffer(frameA);
    clock.now += STEP_SEC;
    io.writeOnboardBuffer(thumbnailA);
    io.writeBuffer(frameA); // frame A fully confirmed

    // Now every paint at/above device index 40 fails while lower paints still succeed: one tick
    // mixes successful and failed requests, and the frame keeps moving.
    transport.failFromDeviceIndex = 40;
    for (int tick = 0; tick < 4; tick++) {
      clock.now += STEP_SEC;
      int before = transport.paints.size();
      io.writeBuffer(shiftBuffer(60, tick + 1));
      assertTrue(
          transport.paints.size() - before <= maxFrames,
          "failure ticks must stay within the frame budget");
    }

    // Transport recovers and the target settles: within a bounded number of ticks the whole
    // device (onboard + external) must show exactly the latest target, not any failed-era frame.
    transport.failFromDeviceIndex = Integer.MAX_VALUE;
    AddressableLEDBuffer latest = shiftBuffer(60, 99);
    AddressableLEDBuffer latestThumbnail =
        bufferWithColor(LEDConstants.ONBOARD_THUMBNAIL_PIXELS, 90, 80, 70);
    for (int tick = 0; tick < 5; tick++) {
      clock.now += STEP_SEC;
      io.writeOnboardBuffer(latestThumbnail);
      io.writeBuffer(latest);
    }
    for (int i = 0; i < 60; i++) {
      assertEquals(packed(latest, i), transport.displayed[i + 8], "external pixel " + i);
    }
    for (int s = 0; s < LEDConstants.ONBOARD_THUMBNAIL_PIXELS; s++) {
      assertEquals(packed(latestThumbnail, s), transport.displayed[s], "onboard LED " + s);
    }
  }

  /** Distinct-per-tick buffer used by the mixed-failure test. */
  private static AddressableLEDBuffer shiftBuffer(int pixels, int shift) {
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(pixels);
    for (int i = 0; i < pixels; i++) {
      int v = ((i + shift * 5) % 256);
      buffer.setRGB(i, v, 255 - v, (v * 3) % 256);
    }
    return buffer;
  }

  private static boolean allCovered(boolean[] covered) {
    for (boolean pixel : covered) {
      if (!pixel) {
        return false;
      }
    }
    return true;
  }
}

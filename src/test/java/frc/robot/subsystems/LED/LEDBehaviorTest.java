package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.BallFlowTracker;
import frc.robot.util.BallFlowTracker.FlowKind;
import frc.robot.util.DashboardControls;
import frc.robot.util.DashboardControlsTestUtil;
import java.lang.reflect.Field;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

/** The dashboard style is the only input allowed to change the rendered LED pixels. */
class LEDBehaviorTest {
  @TempDir static Path tempDir;

  private static final String LEGACY_FLOW_KEY = "/SmartDashboard/LED Follow Ball Flow";
  private static final String BRIGHTNESS_KEY = "/Tuning/LED Brightness (%)";
  private static final double[] trackerNow = {0.0};

  private static final int SAMPLES_PER_PERIOD = 12;
  private static final StyleCase STATIC_STYLE =
      new StyleCase(
          "Candle Memory Static",
          LEDPattern.CANDLE_MEMORY,
          IdleEffect.STATIC,
          1.0,
          false,
          0x7020D0);
  private static final List<StyleCase> ANIMATED_STYLES =
      List.of(
          new StyleCase(
              "theme Slow Flow",
              LEDPattern.MARIGOLD_BRIDGE,
              IdleEffect.SLOW_FLOW,
              LEDConstants.IDLE_SLOW_FLOW_PERIOD_SEC,
              true,
              null),
          new StyleCase(
              "theme Chase",
              LEDPattern.MARIGOLD_BRIDGE,
              IdleEffect.CHASE,
              LEDConstants.IDLE_CHASE_PERIOD_SEC,
              true,
              null),
          new StyleCase(
              "theme Breathe",
              LEDPattern.MARIGOLD_BRIDGE,
              IdleEffect.BREATHE,
              LEDConstants.IDLE_BREATHE_PERIOD_SEC,
              true,
              null),
          new StyleCase(
              "legacy Rainbow",
              LEDPattern.RAINBOW,
              IdleEffect.STATIC,
              LEDConstants.RAINBOW_FULL_CYCLE_SEC,
              true,
              null));

  private static String savedPatternSelected;
  private static String savedEffectSelected;
  private static boolean savedPatternSelectedExists;
  private static boolean savedEffectSelectedExists;
  private static String savedPatternChooserName;
  private static String savedEffectChooserName;
  private static Object savedSettingsStore;
  private static Object temporarySettingsStore;
  private static boolean savedLegacyFollowExists;
  private static boolean savedLegacyFollow;
  private static double savedBrightness;
  private static boolean savedEnabled;
  private static boolean savedAttached;
  private static boolean savedTimingPaused;

  private static double[] savedTrackerReports;
  private static DoubleSupplier savedTrackerClock;

  private static LEDPattern savedLastPattern;
  private static int savedLoopsSinceBoot;
  private static LEDTheme savedActiveTheme;
  private static LEDTheme savedPreviousTheme;
  private static double savedThemeTransitionStartSec;
  private static Double savedLastTimestampSec;
  private static Object savedFlowLayer;
  private static Double savedFlowPhase;
  private static Double savedFlowBlend;
  private static Double savedFlowDirection;
  private static int[] savedExternal;
  private static int[] savedOnboard;

  @BeforeAll
  static void setUp() throws Exception {
    HAL.initialize(500, 0);
    savedTimingPaused = measureTimingPaused();
    SimHooks.pauseTiming();
    savedEnabled = DriverStation.isEnabled();
    savedAttached = DriverStation.isDSAttached();

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    savedSettingsStore = settingsStoreField().get(null);
    savedPatternSelectedExists = nt.getEntry(LED.PATTERN_SETTING_KEY + "/selected").exists();
    savedEffectSelectedExists = nt.getEntry(LED.EFFECT_SETTING_KEY + "/selected").exists();
    savedPatternSelected = nt.getEntry(LED.PATTERN_SETTING_KEY + "/selected").getString("");
    savedEffectSelected = nt.getEntry(LED.EFFECT_SETTING_KEY + "/selected").getString("");
    savedLegacyFollowExists = nt.getEntry(LEGACY_FLOW_KEY).exists();
    savedLegacyFollow = nt.getEntry(LEGACY_FLOW_KEY).getBoolean(true);

    assertNotNull(LED.mInstance, "the real LED subsystem must be constructed");
    refreshLedInputs();
    savedPatternChooserName = dashboardName(chooserValue("patternChooser"));
    savedEffectChooserName = dashboardName(chooserValue("effectChooser"));
    temporarySettingsStore = newSettingsStore(tempDir.resolve("behavior-settings.txt"));
    settingsStoreField().set(null, temporarySettingsStore);

    snapshotTracker();
    snapshotLed();
    DashboardControlsTestUtil.refresh();
    savedBrightness = DashboardControls.getLedBrightnessPercent();
    setBrightness(100.0);

    BallFlowTracker.resetForTest();
    BallFlowTracker.setClockForTest(() -> trackerNow[0]);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  @AfterAll
  static void tearDown() throws Exception {
    restoreSelected(
        LED.PATTERN_SETTING_KEY,
        savedPatternSelectedExists,
        savedPatternSelected,
        savedPatternChooserName,
        LEDPattern.MARIGOLD_BRIDGE.getDashboardName());
    restoreSelected(
        LED.EFFECT_SETTING_KEY,
        savedEffectSelectedExists,
        savedEffectSelected,
        savedEffectChooserName,
        IdleEffect.SLOW_FLOW.getDashboardName());
    if (savedLegacyFollowExists) {
      NetworkTableInstance.getDefault().getEntry(LEGACY_FLOW_KEY).setBoolean(savedLegacyFollow);
    } else {
      NetworkTableInstance.getDefault().getEntry(LEGACY_FLOW_KEY).unpublish();
    }
    setBrightness(savedBrightness);
    restoreSettingsStore();
    restoreTracker();
    restoreLed();

    DriverStationSim.setEnabled(savedEnabled);
    DriverStationSim.setDsAttached(savedAttached);
    DriverStationSim.notifyNewData();
    // SimHooks has no paused-state getter; restore the state measured at setup, not an assumed one.
    // FPGA time cannot be rewound safely; all assertions use relative steps, so restoring the mode
    // is sufficient and avoids imposing a new clock origin on other test classes.
    if (savedTimingPaused) {
      SimHooks.pauseTiming();
    } else {
      SimHooks.resumeTiming();
    }
  }

  @Test
  void periodicUsesOnlyDashboardStyleForEveryRobotState() throws Exception {
    assertStyleBehavior(STATIC_STYLE);
  }

  @Test
  void periodicAnimatedStylesRemainTimeVaryingAndStateIndependent() throws Exception {
    for (StyleCase style : ANIMATED_STYLES) {
      assertStyleBehavior(style);
    }
  }

  /** Runs the real periodic path for one style, including all robot-state combinations. */
  private static void assertStyleBehavior(StyleCase style) throws Exception {
    settleStyle(style);
    assertStyleSelected(style);

    // Prime the old action layer so the negative control cannot pass with a zero blend.
    selectStyle(style, true);
    for (int i = 0; i < 25; i++) {
      BallFlowTracker.resetForTest();
      BallFlowTracker.report(FlowKind.FEED);
      runPeriodic();
      SimHooks.stepTiming(0.02);
    }

    // Take the reference at the exact timestamp used by the state matrix. This matters for animated
    // styles: their legitimate time motion must not be mistaken for a robot-state change.
    selectStyle(style, false);
    BallFlowTracker.resetForTest();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    runPeriodic();
    assertStyleSelected(style);
    int[] expectedExternal = pixels("buffer");
    int[] expectedOnboard = pixels("onboardBuffer");
    assertHasNonBlack(expectedExternal, style.name() + " external reference is entirely black");
    assertHasNonBlack(expectedOnboard, style.name() + " onboard reference is entirely black");
    if (style.expectedFirstPixel() != null) {
      // Independent design-stop literal: every fusion theme starts at RGB 7020D0 (purple), not a
      // render-derived expectation. This catches a test that accidentally selects the wrong theme
      // or black output.
      assertEquals(
          style.expectedFirstPixel().intValue(),
          expectedExternal[0],
          style.name() + " external first pixel must be the theme's first design stop");
      assertEquals(
          style.expectedFirstPixel().intValue(),
          expectedOnboard[0],
          style.name() + " onboard first pixel must be the theme's first design stop");
    }

    // Re-enable the old overlay without advancing time, then compare every state against the frame
    // above. Current production ignores this legacy input.
    selectStyle(style, true);
    for (StateFrame frame : renderAllStates()) {
      assertArrayEquals(
          expectedExternal,
          frame.external(),
          style.name() + ": " + frame.name() + " changed external pixels");
      assertArrayEquals(
          expectedOnboard,
          frame.onboard(),
          style.name() + ": " + frame.name() + " changed onboard pixels");
    }

    // Remove the old overlay for the time-domain check. Current code has no such input; this is
    // simply a no-op there and keeps the old-baseline negative control focused on its render path.
    selectStyle(style, false);
    BallFlowTracker.resetForTest();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    runPeriodic();
    assertStyleSelected(style);
    int[] beforeExternal = pixels("buffer");
    int[] beforeOnboard = pixels("onboardBuffer");

    assertTimeSamples(style, beforeExternal, beforeOnboard);
  }

  /** Samples a complete period so quantization at one starting phase cannot hide real animation. */
  private static void assertTimeSamples(
      StyleCase style, int[] initialExternal, int[] initialOnboard) throws Exception {
    boolean externalChanged = false;
    boolean onboardChanged = false;
    for (int sample = 1; sample <= SAMPLES_PER_PERIOD; sample++) {
      SimHooks.stepTiming(style.periodSec() / SAMPLES_PER_PERIOD);
      runPeriodic();
      int[] currentExternal = pixels("buffer");
      int[] currentOnboard = pixels("onboardBuffer");
      if (style.changesWithTime()) {
        externalChanged |= !Arrays.equals(initialExternal, currentExternal);
        onboardChanged |= !Arrays.equals(initialOnboard, currentOnboard);
      } else {
        assertArrayEquals(
            initialExternal,
            currentExternal,
            style.name() + " external STATIC pixels changed at sample " + sample);
        assertArrayEquals(
            initialOnboard,
            currentOnboard,
            style.name() + " onboard STATIC pixels changed at sample " + sample);
      }
    }
    if (style.changesWithTime()) {
      assertTrue(
          externalChanged,
          style.name()
              + " external pixels never changed across a full "
              + style.periodSec()
              + " s period");
      assertTrue(
          onboardChanged,
          style.name()
              + " onboard pixels never changed across a full "
              + style.periodSec()
              + " s period");
    }
  }

  private static void settleStyle(StyleCase style) throws Exception {
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    BallFlowTracker.resetForTest();
    selectStyle(style, false);
    // Let a pattern-change recommendation finish before selecting the requested effect again.
    for (int i = 0; i < 8; i++) {
      runPeriodic();
      SimHooks.stepTiming(0.02);
    }
    selectStyle(style, false);
    // Also settles theme crossfade (0.6 s) and fades any old action layer out (0.35 s).
    for (int i = 0; i < 40; i++) {
      runPeriodic();
      SimHooks.stepTiming(0.02);
    }
  }

  private static List<StateFrame> renderAllStates() throws Exception {
    List<StateFrame> frames = new ArrayList<>();
    frames.add(renderState("enabled idle", null, true));
    frames.add(renderState("enabled forward/intake", FlowKind.INTAKE, true));
    frames.add(renderState("enabled forward/shoot", FlowKind.FEED, true));
    frames.add(renderState("enabled reverse/eject", FlowKind.FULL_REVERSE, true));
    frames.add(renderState("disabled idle", null, false));
    frames.add(renderState("disabled intake", FlowKind.INTAKE, false));
    frames.add(renderState("disabled forward", FlowKind.FEED, false));
    frames.add(renderState("disabled reverse", FlowKind.FULL_REVERSE, false));
    return frames;
  }

  private static StateFrame renderState(String name, FlowKind kind, boolean enabled)
      throws Exception {
    BallFlowTracker.resetForTest();
    if (kind != null) {
      BallFlowTracker.report(kind);
    }
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.notifyNewData();
    runPeriodic();
    return new StateFrame(name, pixels("buffer"), pixels("onboardBuffer"));
  }

  private static void selectStyle(StyleCase style, boolean legacyFollow) throws Exception {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getEntry(LED.PATTERN_SETTING_KEY + "/selected")
        .setString(style.pattern().getDashboardName());
    nt.getEntry(LED.EFFECT_SETTING_KEY + "/selected").setString(style.effect().getDashboardName());
    // Only a negative-control input for the old implementation; current production has no consumer.
    nt.getEntry(LEGACY_FLOW_KEY).setBoolean(legacyFollow);
    refreshLedInputs();
  }

  private static void assertStyleSelected(StyleCase style) throws Exception {
    assertEquals(
        style.pattern(),
        chooserValue("patternChooser"),
        style.name() + " pattern chooser did not apply the requested value");
    assertEquals(
        style.effect(),
        chooserValue("effectChooser"),
        style.name() + " idle-effect chooser did not apply the requested value");
  }

  private static Object chooserValue(String fieldName) throws Exception {
    Field field = LED.class.getDeclaredField(fieldName);
    field.setAccessible(true);
    return ((LoggedDashboardChooser<?>) field.get(LED.mInstance)).get();
  }

  private static void runPeriodic() throws Exception {
    refreshLedInputs();
    LED.mInstance.periodic();
  }

  /** Logger normally refreshes these inputs before Robot.robotPeriodic; do that explicitly here. */
  private static void refreshLedInputs() throws Exception {
    SmartDashboard.updateValues();
    for (Field field : LED.class.getDeclaredFields()) {
      if (!LoggedNetworkInput.class.isAssignableFrom(field.getType())) {
        continue;
      }
      field.setAccessible(true);
      ((LoggedNetworkInput) field.get(LED.mInstance)).periodic();
    }
  }

  private static int[] pixels(String fieldName) throws Exception {
    Field field = LED.class.getDeclaredField(fieldName);
    field.setAccessible(true);
    AddressableLEDBuffer buffer = (AddressableLEDBuffer) field.get(LED.mInstance);
    int[] packed = new int[buffer.getLength()];
    for (int i = 0; i < packed.length; i++) {
      packed[i] = (buffer.getRed(i) << 16) | (buffer.getGreen(i) << 8) | buffer.getBlue(i);
    }
    return packed;
  }

  private static void writePixels(String fieldName, int[] packed) throws Exception {
    Field field = LED.class.getDeclaredField(fieldName);
    field.setAccessible(true);
    AddressableLEDBuffer buffer = (AddressableLEDBuffer) field.get(LED.mInstance);
    for (int i = 0; i < packed.length; i++) {
      buffer.setRGB(i, (packed[i] >> 16) & 0xFF, (packed[i] >> 8) & 0xFF, packed[i] & 0xFF);
    }
  }

  private static void assertHasNonBlack(int[] pixels, String message) {
    assertTrue(Arrays.stream(pixels).anyMatch(pixel -> pixel != 0), message);
  }

  private static String dashboardName(Object value) {
    if (value instanceof LEDPattern pattern) {
      return pattern.getDashboardName();
    }
    if (value instanceof IdleEffect effect) {
      return effect.getDashboardName();
    }
    return null;
  }

  private static void restoreSelected(
      String settingKey, boolean existed, String rawSelected, String chooserName, String fallback)
      throws Exception {
    if (!existed) {
      // unpublish() removes the value instead of inventing a default entry for an originally absent
      // selected key. The chooser's own default remains effective when periodic() next reads it.
      var selectedEntry = NetworkTableInstance.getDefault().getEntry(settingKey + "/selected");
      selectedEntry.unpublish();
      refreshLedInputs();
      assertFalse(selectedEntry.exists(), settingKey + " selected was republished after unpublish");
      return;
    }
    String selected = rawSelected == null || rawSelected.isEmpty() ? chooserName : rawSelected;
    if (selected == null || selected.isEmpty()) {
      selected = fallback;
    }
    NetworkTableInstance.getDefault().getEntry(settingKey + "/selected").setString(selected);
    refreshLedInputs();
  }

  private static Field settingsStoreField() throws NoSuchFieldException {
    Field field = DashboardControls.class.getDeclaredField("settingsFile");
    field.setAccessible(true);
    return field;
  }

  private static Object newSettingsStore(Path path) throws Exception {
    Class<?> storeClass = Class.forName("frc.robot.util.DashboardSettingsFile");
    var constructor = storeClass.getDeclaredConstructor(Path.class);
    constructor.setAccessible(true);
    return constructor.newInstance(path);
  }

  private static void restoreSettingsStore() throws Exception {
    try {
      if (temporarySettingsStore != null) {
        // Flush only the disposable store; never write test values into the previous store.
        var saveNow = temporarySettingsStore.getClass().getDeclaredMethod("saveNow");
        saveNow.setAccessible(true);
        saveNow.invoke(temporarySettingsStore);
      }
    } finally {
      // This may be null (the lazy default) or another test class's store; restore it verbatim.
      settingsStoreField().set(null, savedSettingsStore);
      assertSame(
          savedSettingsStore, settingsStoreField().get(null), "settings store was not restored");
    }
  }

  private static void setBrightness(double value) {
    DashboardControlsTestUtil.setFullKey(BRIGHTNESS_KEY, value);
  }

  private static boolean measureTimingPaused() throws InterruptedException {
    long before = edu.wpi.first.wpilibj.RobotController.getFPGATime();
    Thread.sleep(5);
    long after = edu.wpi.first.wpilibj.RobotController.getFPGATime();
    return before == after;
  }

  private static void snapshotTracker() throws Exception {
    Field reports = BallFlowTracker.class.getDeclaredField("lastReportSec");
    reports.setAccessible(true);
    savedTrackerReports = ((double[]) reports.get(null)).clone();
    Field clock = BallFlowTracker.class.getDeclaredField("clockForTest");
    clock.setAccessible(true);
    savedTrackerClock = (DoubleSupplier) clock.get(null);
  }

  private static void restoreTracker() throws Exception {
    Field clock = BallFlowTracker.class.getDeclaredField("clockForTest");
    clock.setAccessible(true);
    clock.set(null, savedTrackerClock);
    Field reports = BallFlowTracker.class.getDeclaredField("lastReportSec");
    reports.setAccessible(true);
    double[] target = (double[]) reports.get(null);
    System.arraycopy(savedTrackerReports, 0, target, 0, target.length);
  }

  private static void snapshotLed() throws Exception {
    savedLastPattern = (LEDPattern) readLedField("lastPattern");
    savedLoopsSinceBoot = (Integer) readLedField("loopsSinceBoot");
    savedActiveTheme = (LEDTheme) readLedField("activeTheme");
    savedPreviousTheme = (LEDTheme) readLedField("previousTheme");
    savedThemeTransitionStartSec = (Double) readLedField("themeTransitionStartSec");
    Field lastTimestamp = optionalLedField("lastTimestampSec");
    savedLastTimestampSec = lastTimestamp == null ? null : lastTimestamp.getDouble(LED.mInstance);
    Field flow = optionalLedField("flowLayer");
    savedFlowLayer = flow == null ? null : flow.get(LED.mInstance);
    if (savedFlowLayer != null) {
      savedFlowPhase = readNestedDouble(savedFlowLayer, "phase");
      savedFlowBlend = readNestedDouble(savedFlowLayer, "blend");
      savedFlowDirection = readNestedDouble(savedFlowLayer, "directionSign");
    }
    savedExternal = pixels("buffer");
    savedOnboard = pixels("onboardBuffer");
  }

  private static void restoreLed() throws Exception {
    writeLedField("lastPattern", savedLastPattern);
    writeLedField("loopsSinceBoot", savedLoopsSinceBoot);
    writeLedField("activeTheme", savedActiveTheme);
    writeLedField("previousTheme", savedPreviousTheme);
    writeLedField("themeTransitionStartSec", savedThemeTransitionStartSec);
    Field lastTimestamp = optionalLedField("lastTimestampSec");
    if (lastTimestamp != null && savedLastTimestampSec != null) {
      lastTimestamp.setDouble(LED.mInstance, savedLastTimestampSec);
    }
    Field flow = optionalLedField("flowLayer");
    if (flow != null && savedFlowLayer != null) {
      flow.set(LED.mInstance, savedFlowLayer);
      writeNestedDouble(savedFlowLayer, "phase", savedFlowPhase);
      writeNestedDouble(savedFlowLayer, "blend", savedFlowBlend);
      writeNestedDouble(savedFlowLayer, "directionSign", savedFlowDirection);
    }
    writePixels("buffer", savedExternal);
    writePixels("onboardBuffer", savedOnboard);
  }

  private static Object readLedField(String name) throws Exception {
    Field field = LED.class.getDeclaredField(name);
    field.setAccessible(true);
    return field.get(LED.mInstance);
  }

  private static void writeLedField(String name, Object value) throws Exception {
    Field field = LED.class.getDeclaredField(name);
    field.setAccessible(true);
    field.set(LED.mInstance, value);
  }

  private static Field optionalLedField(String name) {
    try {
      Field field = LED.class.getDeclaredField(name);
      field.setAccessible(true);
      return field;
    } catch (NoSuchFieldException e) {
      return null;
    }
  }

  private static Double readNestedDouble(Object owner, String name) throws Exception {
    Field field = owner.getClass().getDeclaredField(name);
    field.setAccessible(true);
    return field.getDouble(owner);
  }

  private static void writeNestedDouble(Object owner, String name, Double value) throws Exception {
    if (value == null) {
      return;
    }
    Field field = owner.getClass().getDeclaredField(name);
    field.setAccessible(true);
    field.setDouble(owner, value);
  }

  private record StyleCase(
      String name,
      LEDPattern pattern,
      IdleEffect effect,
      double periodSec,
      boolean changesWithTime,
      Integer expectedFirstPixel) {}

  private record StateFrame(String name, int[] external, int[] onboard) {}
}

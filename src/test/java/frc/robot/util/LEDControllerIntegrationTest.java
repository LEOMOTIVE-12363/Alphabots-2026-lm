package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.simulation.MapleSimArena;
import frc.robot.subsystems.LED.IdleEffect;
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.LED.LEDConstants;
import frc.robot.subsystems.LED.LEDPattern;
import frc.robot.subsystems.LED.LEDTheme;
import frc.robot.subsystems.drive.Drive;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Real-button integration for the demo pad's LED theme and idle-effect cycling. */
class LEDControllerIntegrationTest {
  private static final int BUTTON_BACK = 7;
  private static final int BUTTON_START = 8;
  private static final double STEP_SEC = 0.02;
  private static final double EPS = 1e-3;

  private static final NetworkTableInstance NT = NetworkTableInstance.getDefault();
  private static final CommandScheduler SCHEDULER = CommandScheduler.getInstance();

  @TempDir static Path tempDir;

  private static EventLoop previousButtonLoop;
  private static RobotContainer container;
  private static LoggedDashboardChooser<LEDPattern> patternChooser;
  private static LoggedDashboardChooser<IdleEffect> effectChooser;
  private static DashboardSettingsFile settings;
  private static Path settingsPath;
  private static int settingsCounter;

  @BeforeAll
  static void setUpContainer() throws Exception {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    previousButtonLoop = SCHEDULER.getActiveButtonLoop();
    SCHEDULER.cancelAll();
    SCHEDULER.setActiveButtonLoop(SCHEDULER.getDefaultButtonLoop());
    SCHEDULER.getDefaultButtonLoop().clear();

    DriverStationSim.resetData();
    DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
    DriverStationSim.notifyNewData();
    settingsPath = tempDir.resolve("boot-settings.txt");
    settings = new DashboardSettingsFile(settingsPath);
    DashboardControls.setSettingsFileForTest(settings);
    container = new RobotContainer();
    patternChooser = chooserField("patternChooser");
    effectChooser = chooserField("effectChooser");
  }

  @BeforeEach
  void resetState() {
    SCHEDULER.cancelAll();
    DriverStationSim.resetData();
    DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DashboardControlsTestUtil.resetDefaults();
    BallFlowTracker.resetForTest();
    BallFlowTracker.setClockForTest(null);

    settingsPath = tempDir.resolve("settings-" + settingsCounter++ + ".txt");
    settings = new DashboardSettingsFile(settingsPath);
    DashboardControls.setSettingsFileForTest(settings);
    selectFromElastic(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.SLOW_FLOW);
    settle(4);
    settings.saveNow();
  }

  @AfterEach
  void cleanUpInputs() {
    SCHEDULER.cancelAll();
    resetJoystick(DriverControls.FULL_PORT);
    resetJoystick(DriverControls.DEMO_PORT);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    settings.saveNow();
  }

  @AfterAll
  static void tearDownClass() {
    writeSelected(LED.PATTERN_SETTING_KEY, LEDPattern.MARIGOLD_BRIDGE.getDashboardName());
    writeSelected(LED.EFFECT_SETTING_KEY, IdleEffect.SLOW_FLOW.getDashboardName());
    settle(4);
    settings.saveNow();
    SCHEDULER.cancelAll();
    SCHEDULER.getDefaultButtonLoop().clear();
    if (Drive.mInstance != null) {
      SCHEDULER.unregisterSubsystem(Drive.mInstance);
      Drive.mInstance = null;
    }
    SCHEDULER.setActiveButtonLoop(previousButtonLoop);
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
    DriverStationSim.notifyNewData();
    DashboardControlsTestUtil.resetDefaults();
    DashboardControls.setSettingsFileForTest(null);
    BallFlowTracker.resetForTest();
    BallFlowTracker.setClockForTest(null);
  }

  @Test
  void demoBackCyclesThemesInOrderAndWraps() {
    connect(DriverControls.DEMO_PORT);
    LEDPattern[] expected = {
      LEDPattern.FESTIVAL_NIGHT, LEDPattern.ALEBRIJE_GLOW, LEDPattern.MARIGOLD_BRIDGE
    };
    for (LEDPattern pattern : expected) {
      click(DriverControls.DEMO_PORT, BUTTON_BACK);
      assertSelection(pattern, pattern.theme().orElseThrow().getRecommendedEffect());
    }
  }

  @Test
  void demoStartCyclesIdleEffectsInOrderAndWraps() {
    connect(DriverControls.DEMO_PORT);
    selectFromElastic(LEDPattern.FESTIVAL_NIGHT, IdleEffect.STATIC);
    IdleEffect[] expected = {
      IdleEffect.BREATHE,
      IdleEffect.SLOW_FLOW,
      IdleEffect.CHASE,
      IdleEffect.COLOR_STEP,
      IdleEffect.STATIC
    };
    for (IdleEffect effect : expected) {
      click(DriverControls.DEMO_PORT, BUTTON_START);
      assertSelection(LEDPattern.FESTIVAL_NIGHT, effect);
    }
  }

  @Test
  void backHoldIsOneEdgeAndOnlyRepressAdvancesAgain() {
    connect(DriverControls.DEMO_PORT);
    hold(DriverControls.DEMO_PORT, BUTTON_BACK);
    settle(10);
    assertSelection(LEDPattern.FESTIVAL_NIGHT, IdleEffect.BREATHE);

    release(DriverControls.DEMO_PORT, BUTTON_BACK);
    hold(DriverControls.DEMO_PORT, BUTTON_BACK);
    settle(4);
    assertSelection(LEDPattern.ALEBRIJE_GLOW, IdleEffect.SLOW_FLOW);
  }

  @Test
  void startHoldIsOneEdgeAndOnlyRepressAdvancesAgain() {
    connect(DriverControls.DEMO_PORT);
    selectFromElastic(LEDPattern.FESTIVAL_NIGHT, IdleEffect.STATIC);
    hold(DriverControls.DEMO_PORT, BUTTON_START);
    settle(10);
    assertSelection(LEDPattern.FESTIVAL_NIGHT, IdleEffect.BREATHE);

    release(DriverControls.DEMO_PORT, BUTTON_START);
    hold(DriverControls.DEMO_PORT, BUTTON_START);
    settle(4);
    assertSelection(LEDPattern.FESTIVAL_NIGHT, IdleEffect.SLOW_FLOW);
  }

  @Test
  void elasticSelectionsAreTheNextCycleStartingPoint() {
    connect(DriverControls.DEMO_PORT);
    setElasticPattern(LEDPattern.ALEBRIJE_GLOW);
    click(DriverControls.DEMO_PORT, BUTTON_BACK);
    assertSelection(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.SLOW_FLOW);

    setElasticEffect(IdleEffect.BREATHE);
    click(DriverControls.DEMO_PORT, BUTTON_START);
    assertSelection(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.SLOW_FLOW);
  }

  @Test
  void offBacksToMarigoldAndStartIsACompleteNoOp() {
    connect(DriverControls.DEMO_PORT);
    selectFromElastic(LEDPattern.OFF, IdleEffect.COLOR_STEP);
    click(DriverControls.DEMO_PORT, BUTTON_BACK);
    assertSelection(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.SLOW_FLOW);

    selectFromElastic(LEDPattern.OFF, IdleEffect.CHASE);
    click(DriverControls.DEMO_PORT, BUTTON_START);
    assertSelection(LEDPattern.OFF, IdleEffect.CHASE);
  }

  @Test
  void themeRecommendationCanBeOverriddenByStart() {
    connect(DriverControls.DEMO_PORT);
    selectFromElastic(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.STATIC);
    click(DriverControls.DEMO_PORT, BUTTON_BACK);
    assertSelection(LEDPattern.FESTIVAL_NIGHT, IdleEffect.BREATHE);

    click(DriverControls.DEMO_PORT, BUTTON_START);
    settle(10);
    assertSelection(LEDPattern.FESTIVAL_NIGHT, IdleEffect.SLOW_FLOW);
  }

  @Test
  void realPeriodicSynchronizesDashboardRenderingAndSavedFile() throws Exception {
    connect(DriverControls.DEMO_PORT);
    setElasticPattern(LEDPattern.FESTIVAL_NIGHT);
    setElasticEffect(IdleEffect.COLOR_STEP);
    click(DriverControls.DEMO_PORT, BUTTON_START);
    settle(40);

    assertSelection(LEDPattern.FESTIVAL_NIGHT, IdleEffect.STATIC);
    settings.saveNow();
    assertTrue(Files.isRegularFile(settingsPath), "the real settings file must exist");
    DashboardSettingsFile reopened = new DashboardSettingsFile(settingsPath);
    assertEquals(
        LEDPattern.FESTIVAL_NIGHT.getDashboardName(),
        reopened.getString(LED.PATTERN_SETTING_KEY, "missing"));
    assertEquals(
        IdleEffect.STATIC.getDashboardName(),
        reopened.getString(LED.EFFECT_SETTING_KEY, "missing"));

    AddressableLEDBuffer rendered = ledBuffer();
    int last = LEDConstants.LENGTH_PIXELS - 1;
    assertStaticFestivalPixel(rendered, 0);
    assertStaticFestivalPixel(rendered, last / 2);
    assertStaticFestivalPixel(rendered, last);
  }

  @Test
  void fullBackIsIgnoredAndFullStartKeepsSimulationResetWithoutCyclingEffect() {
    connect(DriverControls.FULL_PORT);
    selectFromElastic(LEDPattern.FESTIVAL_NIGHT, IdleEffect.COLOR_STEP);
    Drive.mInstance.setPose(new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(137)));

    click(DriverControls.FULL_PORT, BUTTON_BACK);
    assertSelection(LEDPattern.FESTIVAL_NIGHT, IdleEffect.COLOR_STEP);
    assertDrivePose(3.0, 2.0, 137.0);

    connect(DriverControls.DEMO_PORT);
    click(DriverControls.FULL_PORT, BUTTON_START);
    assertSelection(LEDPattern.FESTIVAL_NIGHT, IdleEffect.COLOR_STEP);
    assertDrivePose(3.0, 2.0, 137.0);

    disconnect(DriverControls.DEMO_PORT);
    click(DriverControls.FULL_PORT, BUTTON_START);
    assertSelection(LEDPattern.FESTIVAL_NIGHT, IdleEffect.COLOR_STEP);
    assertDrivePose(1.5, 4.0, 0.0);
    assertEquals(1.5, MapleSimArena.getInstance().getRobotPose().getX(), EPS);
    assertEquals(4.0, MapleSimArena.getInstance().getRobotPose().getY(), EPS);
  }

  @Test
  void demoOwnsLedButtonsAcrossPriorityChangesAndDisconnect() {
    connect(DriverControls.FULL_PORT);
    connect(DriverControls.DEMO_PORT);
    click(DriverControls.FULL_PORT, BUTTON_BACK);
    assertSelection(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.SLOW_FLOW);

    click(DriverControls.DEMO_PORT, BUTTON_BACK);
    assertSelection(LEDPattern.FESTIVAL_NIGHT, IdleEffect.BREATHE);
    hold(DriverControls.DEMO_PORT, BUTTON_BACK);
    settle(4);
    assertSelection(LEDPattern.ALEBRIJE_GLOW, IdleEffect.SLOW_FLOW);

    disconnect(DriverControls.DEMO_PORT);
    assertSelection(LEDPattern.ALEBRIJE_GLOW, IdleEffect.SLOW_FLOW);
    click(DriverControls.FULL_PORT, BUTTON_BACK);
    assertSelection(LEDPattern.ALEBRIJE_GLOW, IdleEffect.SLOW_FLOW);

    connect(DriverControls.DEMO_PORT);
    click(DriverControls.DEMO_PORT, BUTTON_BACK);
    assertSelection(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.SLOW_FLOW);
  }

  @Test
  void disabledEdgesDoNotFireOnReenableButFreshPressesWork() {
    connect(DriverControls.DEMO_PORT);
    DriverStationSim.setEnabled(false);
    settle(2);
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_BACK, true);
    settle(3);
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_START, true);
    settle(3);
    assertSelection(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.SLOW_FLOW);

    DriverStationSim.setEnabled(true);
    settle(8);
    assertSelection(LEDPattern.MARIGOLD_BRIDGE, IdleEffect.SLOW_FLOW);

    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_BACK, false);
    DriverStationSim.setJoystickButton(DriverControls.DEMO_PORT, BUTTON_START, false);
    settle(2);
    click(DriverControls.DEMO_PORT, BUTTON_BACK);
    assertSelection(LEDPattern.FESTIVAL_NIGHT, IdleEffect.BREATHE);
    click(DriverControls.DEMO_PORT, BUTTON_START);
    assertSelection(LEDPattern.FESTIVAL_NIGHT, IdleEffect.SLOW_FLOW);
  }

  @Test
  void chooserOptionsRemainComplete() {
    Set<String> patterns =
        new HashSet<>(
            Arrays.asList(entry(LED.PATTERN_SETTING_KEY, "options").getStringArray(new String[0])));
    Set<String> effects =
        new HashSet<>(
            Arrays.asList(entry(LED.EFFECT_SETTING_KEY, "options").getStringArray(new String[0])));
    Set<String> expectedPatterns = new HashSet<>();
    for (LEDPattern pattern : LEDPattern.values()) {
      expectedPatterns.add(pattern.getDashboardName());
    }
    Set<String> expectedEffects = new HashSet<>();
    for (IdleEffect effect : IdleEffect.values()) {
      expectedEffects.add(effect.getDashboardName());
    }
    assertEquals(expectedPatterns, patterns);
    assertEquals(expectedEffects, effects);
    assertEquals(LEDPattern.values().length, patterns.size());
    assertEquals(IdleEffect.values().length, effects.size());
  }

  /** One robot-loop ordering, including the real chooser inputs and persistence boundary. */
  private static void stepRobot() {
    DriverStationSim.notifyNewData();
    patternChooser.periodic();
    effectChooser.periodic();
    DashboardControlsTestUtil.refresh();
    SCHEDULER.run();
    DashboardControls.persistIfChanged();
    SmartDashboard.updateValues();
    NT.flush();
    SimHooks.stepTiming(STEP_SEC);
  }

  private static void settle(int loops) {
    for (int i = 0; i < loops; i++) {
      stepRobot();
    }
  }

  private static void click(int port, int button) {
    hold(port, button);
    settle(4);
    release(port, button);
  }

  private static void hold(int port, int button) {
    DriverStationSim.setJoystickButton(port, button, true);
    settle(4);
  }

  private static void release(int port, int button) {
    DriverStationSim.setJoystickButton(port, button, false);
    settle(2);
  }

  private static void connect(int port) {
    DriverStationSim.setJoystickIsXbox(port, true);
    DriverStationSim.setJoystickAxisCount(port, 6);
    DriverStationSim.setJoystickButtonCount(port, 10);
    DriverStationSim.setJoystickPOVCount(port, 1);
    neutralizeJoystick(port);
    settle(2);
  }

  private static void disconnect(int port) {
    resetJoystick(port);
    settle(2);
  }

  private static void resetJoystick(int port) {
    neutralizeJoystick(port);
    DriverStationSim.setJoystickIsXbox(port, false);
    DriverStationSim.setJoystickAxisCount(port, 0);
    DriverStationSim.setJoystickButtonCount(port, 0);
    DriverStationSim.setJoystickPOVCount(port, 0);
  }

  private static void neutralizeJoystick(int port) {
    for (int axis = 0; axis < 6; axis++) {
      DriverStationSim.setJoystickAxis(port, axis, 0.0);
    }
    for (int button = 1; button <= 10; button++) {
      DriverStationSim.setJoystickButton(port, button, false);
    }
    DriverStationSim.setJoystickPOV(port, 0, -1);
  }

  private static void selectFromElastic(LEDPattern pattern, IdleEffect effect) {
    writeSelected(LED.PATTERN_SETTING_KEY, pattern.getDashboardName());
    settle(4);
    writeSelected(LED.EFFECT_SETTING_KEY, effect.getDashboardName());
    settle(4);
    assertSelection(pattern, effect);
  }

  private static void setElasticPattern(LEDPattern pattern) {
    writeSelected(LED.PATTERN_SETTING_KEY, pattern.getDashboardName());
    settle(4);
    assertPattern(pattern);
  }

  private static void setElasticEffect(IdleEffect effect) {
    writeSelected(LED.EFFECT_SETTING_KEY, effect.getDashboardName());
    settle(4);
    assertEffect(effect);
  }

  private static void writeSelected(String settingKey, String dashboardName) {
    NT.getEntry(settingKey + "/selected").setString(dashboardName);
  }

  private static edu.wpi.first.networktables.NetworkTableEntry entry(
      String settingKey, String child) {
    return NT.getEntry(settingKey + "/" + child);
  }

  private static void assertSelection(LEDPattern pattern, IdleEffect effect) {
    assertAll(() -> assertPattern(pattern), () -> assertEffect(effect));
  }

  private static void assertPattern(LEDPattern expected) {
    assertAll(
        () ->
            assertEquals(
                expected.getDashboardName(),
                entry(LED.PATTERN_SETTING_KEY, "selected").getString("missing"),
                "pattern selected"),
        () ->
            assertEquals(
                expected.getDashboardName(),
                entry(LED.PATTERN_SETTING_KEY, "active").getString("missing"),
                "pattern active"),
        () -> assertEquals(expected, patternChooser.get(), "pattern used by LED.periodic"));
  }

  private static void assertEffect(IdleEffect expected) {
    assertAll(
        () ->
            assertEquals(
                expected.getDashboardName(),
                entry(LED.EFFECT_SETTING_KEY, "selected").getString("missing"),
                "effect selected"),
        () ->
            assertEquals(
                expected.getDashboardName(),
                entry(LED.EFFECT_SETTING_KEY, "active").getString("missing"),
                "effect active"),
        () -> assertEquals(expected, effectChooser.get(), "effect used by LED.periodic"));
  }

  private static void assertDrivePose(double x, double y, double headingDeg) {
    Pose2d pose = Drive.mInstance.getPose();
    assertEquals(x, pose.getX(), EPS, "drive x");
    assertEquals(y, pose.getY(), EPS, "drive y");
    assertEquals(
        headingDeg,
        Rotation2d.fromRadians(pose.getRotation().getRadians()).getDegrees(),
        EPS,
        "drive heading");
  }

  private static void assertStaticFestivalPixel(AddressableLEDBuffer buffer, int index) {
    double u = LEDConstants.SECTIONS[0].uForPixel(index);
    Color expected = LEDTheme.FESTIVAL_NIGHT.colorAt(u);
    assertEquals(to255(expected.red), buffer.getRed(index), "red at pixel " + index);
    assertEquals(to255(expected.green), buffer.getGreen(index), "green at pixel " + index);
    assertEquals(to255(expected.blue), buffer.getBlue(index), "blue at pixel " + index);
  }

  private static int to255(double channel) {
    return (int) Math.round(channel * 255.0);
  }

  private static AddressableLEDBuffer ledBuffer() {
    try {
      Field field = LED.class.getDeclaredField("buffer");
      field.setAccessible(true);
      return (AddressableLEDBuffer) field.get(LED.mInstance);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException(e);
    }
  }

  @SuppressWarnings("unchecked")
  private static <T> LoggedDashboardChooser<T> chooserField(String name) {
    try {
      Field field = LED.class.getDeclaredField(name);
      field.setAccessible(true);
      return (LoggedDashboardChooser<T>) field.get(LED.mInstance);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException(e);
    }
  }
}

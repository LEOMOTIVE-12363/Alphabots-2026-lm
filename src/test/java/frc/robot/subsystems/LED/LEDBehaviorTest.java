package frc.robot.subsystems.LED;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.BallFlowTracker;
import frc.robot.util.BallFlowTracker.FlowKind;
import frc.robot.util.DashboardControlsTestUtil;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

/** The dashboard style is the only input allowed to change the rendered LED pixels. */
class LEDBehaviorTest {
  private static final String LEGACY_FLOW_KEY = "/SmartDashboard/LED Follow Ball Flow";
  private static final double[] trackerNow = {0.0};

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    BallFlowTracker.resetForTest();
    BallFlowTracker.setClockForTest(() -> trackerNow[0]);
    DashboardControlsTestUtil.resetDefaults();
    assertNotNull(LED.mInstance, "the real LED subsystem must be constructed");
  }

  @AfterAll
  static void tearDown() {
    BallFlowTracker.resetForTest();
    BallFlowTracker.setClockForTest(null);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setDsAttached(false);
    DriverStationSim.notifyNewData();
    DashboardControlsTestUtil.resetDefaults();
  }

  @Test
  void periodicUsesOnlyDashboardStyleForEveryRobotState() throws Exception {
    selectStyle(LEDPattern.CANDLE_MEMORY, IdleEffect.STATIC, false);

    // Let theme restoration/transition settle. With the legacy input off, this is an independent
    // static reference even on the old implementation; the new implementation has no such input.
    for (int i = 0; i < 40; i++) {
      runPeriodic();
      SimHooks.stepTiming(0.02);
    }
    int[] expectedExternal = pixels("buffer");
    int[] expectedOnboard = pixels("onboardBuffer");

    // Prime the old action layer so the RED assertion cannot accidentally pass with a zero blend.
    selectStyle(LEDPattern.CANDLE_MEMORY, IdleEffect.STATIC, true);
    for (int i = 0; i < 25; i++) {
      BallFlowTracker.resetForTest();
      BallFlowTracker.report(FlowKind.FEED);
      runPeriodic();
      SimHooks.stepTiming(0.02);
    }

    List<StateFrame> frames = new ArrayList<>();
    frames.add(renderState("enabled idle", null, true));
    frames.add(renderState("enabled forward/intake", FlowKind.INTAKE, true));
    frames.add(renderState("enabled forward/shoot", FlowKind.FEED, true));
    frames.add(renderState("enabled reverse/eject", FlowKind.FULL_REVERSE, true));
    frames.add(renderState("disabled idle", null, false));
    frames.add(renderState("disabled intake", FlowKind.INTAKE, false));
    frames.add(renderState("disabled forward", FlowKind.FEED, false));
    frames.add(renderState("disabled reverse", FlowKind.FULL_REVERSE, false));

    for (StateFrame frame : frames) {
      assertArrayEquals(expectedExternal, frame.external, frame.name + " changed external pixels");
      assertArrayEquals(expectedOnboard, frame.onboard, frame.name + " changed onboard pixels");
    }
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

  private static void selectStyle(LEDPattern pattern, IdleEffect effect, boolean legacyFollow)
      throws Exception {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getEntry(LED.PATTERN_SETTING_KEY + "/selected").setString(pattern.getDashboardName());
    nt.getEntry(LED.EFFECT_SETTING_KEY + "/selected").setString(effect.getDashboardName());
    // This legacy key is intentionally used only to obtain an old-code static reference. It has no
    // consumer after the dashboard-only change, so the same test remains valid after removal.
    nt.getEntry(LEGACY_FLOW_KEY).setBoolean(legacyFollow);
    refreshLedInputs();
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

  private record StateFrame(String name, int[] external, int[] onboard) {}
}

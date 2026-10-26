package frc.robot.commands.ManualCommand;

import static frc.robot.util.DashboardControlsTestUtil.FLYWHEEL;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.lib.bases.MotorSubsystem;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.util.DashboardControlsTestUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.OperatorStatus;
import frc.robot.util.OperatorStatus.Action;
import frc.robot.util.ShotReadiness;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Runs Manual against the sim mechanism IO, with known mechanism state before every assertion. */
class ManualTest {
  private static final double EPS = 1e-6;

  @BeforeAll
  static void setUpHal() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
  }

  @AfterEach
  void resetDashboard() throws Exception {
    DashboardControlsTestUtil.resetDefaults();
    overrideTunable(ManualConstants.HoodDegTunable, "Manual/HoodDegree", 0.0);
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(null);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setDsAttached(false);
    DriverStationSim.notifyNewData();
    Drum.mInstance.stop();
    Hood.mInstance.stop();
    Feeder.mInstance.stop();
    Indexer.mInstance.stop();
    IntakeRoller.mInstance.stop();
    runPeriodics();
  }

  /** One robot loop: run the command, advance time, then update mechanism inputs. */
  private static void step(Manual manual) {
    manual.execute();
    SimHooks.stepTiming(0.02);
    runPeriodics();
  }

  /** Updates mechanism inputs without running the command (for assertions after end()). */
  private static void runPeriodics() {
    Drum.mInstance.periodic();
    Hood.mInstance.periodic();
    Feeder.mInstance.periodic();
    Indexer.mInstance.periodic();
    IntakeDeploy.mInstance.periodic();
    IntakeRoller.mInstance.periodic();
  }

  /** Steps until the feeder is commanded to feed; fails after maxLoops. */
  private static void stepUntilFeeding(Manual manual, int maxLoops) {
    for (int i = 0; i < maxLoops; i++) {
      step(manual);
      if (Feeder.mInstance.getVelocitySetpointRotPerSec() > 0.0) {
        return;
      }
    }
    fail("Manual never fed within " + maxLoops + " loops");
  }

  private static void assertManualFeedPath() {
    assertEquals(
        ManualConstants.FeederRotpsTunable.getAsDouble(),
        Feeder.mInstance.getVelocitySetpointRotPerSec(),
        EPS);
    assertEquals(
        ManualConstants.IndexerRotpsTunable.getAsDouble(),
        Indexer.mInstance.getVelocitySetpointRotPerSec(),
        EPS);
    assertEquals(
        ManualConstants.IntakeRollerRotpsTunable.getAsDouble(),
        IntakeRoller.mInstance.getVelocitySetpointRotPerSec(),
        EPS);
  }

  private static void overrideInput(MotorSubsystem subsystem, String field, double value)
      throws Exception {
    Field inputsField = MotorSubsystem.class.getDeclaredField("inputs");
    inputsField.setAccessible(true);
    Object inputs = inputsField.get(subsystem);
    for (Class<?> c = inputs.getClass(); c != null; c = c.getSuperclass()) {
      try {
        Field f = c.getDeclaredField(field);
        f.setAccessible(true);
        f.setDouble(inputs, value);
        return;
      } catch (NoSuchFieldException e) {
        // Walk up to the declaring class.
      }
    }
    throw new NoSuchFieldException(field);
  }

  private static void injectDrumVelocity(double rotPerSec) throws Exception {
    overrideInput(Drum.mInstance, "velocityRadPerSec", rotPerSec * 2.0 * Math.PI);
  }

  private static void injectHoodPosition(double rot) throws Exception {
    overrideInput(Hood.mInstance, "positionRad", rot * 2.0 * Math.PI);
  }

  private static void overrideTunable(LoggedTunableNumber tunable, String key, double value)
      throws Exception {
    edu.wpi.first.networktables.NetworkTableInstance.getDefault()
        .getEntry("/Tuning/" + key)
        .setDouble(value);
    Field field = LoggedTunableNumber.class.getDeclaredField("dashboardNumber");
    field.setAccessible(true);
    ((LoggedNetworkNumber) field.get(tunable)).periodic();
  }

  @Test
  void waitsForStableSpeedBeforeFeeding() {
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    Drum.mInstance.setVelocityRotPerSec(0.0);
    Hood.mInstance.setPositionRot(0.25);
    runPeriodics();

    Manual manual = new Manual();
    manual.initialize();
    // First execute drives the Manual targets but must not feed: the drum is still at 0 against
    // its 50 rot/s target, and the hood being off-target must not matter (it never gates).
    step(manual);
    assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(0.0, Indexer.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(
        ManualConstants.ShooterRotpsTunable.getAsDouble(),
        Drum.mInstance.getVelocitySetpointRotPerSec(),
        EPS);
    assertEquals(
        ManualConstants.HoodDegTunable.getAsDouble() / 360.0,
        Hood.mInstance.getPositionSetpointRot(),
        EPS);

    stepUntilFeeding(manual, 30);
    assertManualFeedPath();
    manual.end(true);
  }

  @Test
  void zeroFlywheelStopsFeedAndPositiveLimitResumesAfterWindow() {
    Manual manual = new Manual();
    manual.initialize();
    stepUntilFeeding(manual, 50);

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    step(manual);
    assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(0.0, Indexer.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(0.0, IntakeRoller.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(0.0, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);

    for (int i = 0; i < 10; i++) {
      step(manual);
    }
    assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS);

    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    // The drum must spin back up and re-earn the stable window before feeding resumes.
    for (int i = 0; i < 5; i++) {
      step(manual);
      assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS, "recovery loop " + i);
    }
    stepUntilFeeding(manual, 25);
    assertManualFeedPath();
    manual.end(true);
  }

  @Test
  void measuredDrumSagPausesSameLoopAndAutoResumes() throws Exception {
    Manual manual = new Manual();
    manual.initialize();
    stepUntilFeeding(manual, 50);
    double drumTarget = Drum.mInstance.getVelocitySetpointRotPerSec();

    injectDrumVelocity(0.0);
    step(manual);
    assertEquals(
        0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS, "sag stops the feed same-loop");
    assertEquals(0.0, Indexer.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(drumTarget, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);

    for (int i = 0; i < 5; i++) {
      injectDrumVelocity(0.0);
      step(manual);
      assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS, "sag loop " + i);
    }

    // Recovery (no injection re-reads the on-target sim value) auto-resumes after the window.
    for (int i = 0; i < 5; i++) {
      step(manual);
      assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS, "window loop " + i);
    }
    stepUntilFeeding(manual, 15);
    assertManualFeedPath();
    manual.end(true);
  }

  @Test
  void measuredHoodDriftDoesNotPauseFeeding() throws Exception {
    Manual manual = new Manual();
    manual.initialize();
    stepUntilFeeding(manual, 50);
    double hoodTarget = Hood.mInstance.getPositionSetpointRot();

    for (int i = 0; i < 3; i++) {
      injectHoodPosition(hoodTarget + 0.1);
      step(manual);
      assertManualFeedPath();
      assertEquals(hoodTarget, Hood.mInstance.getPositionSetpointRot(), EPS);
    }
    manual.end(true);
  }

  @Test
  void drumTargetChangePausesBrieflyThenResumes() {
    Manual manual = new Manual();
    manual.initialize();
    stepUntilFeeding(manual, 50);

    // New drum target 25 rot/s vs the still-measured 50: the edit loop pauses, and the feed
    // resumes at the new target once the stable window is re-earned.
    DashboardControlsTestUtil.set(FLYWHEEL, 50.0);
    step(manual);
    assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(
        0.5 * ManualConstants.ShooterRotpsTunable.getAsDouble(),
        Drum.mInstance.getVelocitySetpointRotPerSec(),
        EPS);

    stepUntilFeeding(manual, 25);
    assertManualFeedPath();
    assertEquals(
        0.5 * ManualConstants.ShooterRotpsTunable.getAsDouble(),
        Drum.mInstance.getVelocitySetpointRotPerSec(),
        EPS);
    manual.end(true);
  }

  @Test
  void hoodTargetChangeDoesNotPauseFeeding() throws Exception {
    Manual manual = new Manual();
    manual.initialize();
    stepUntilFeeding(manual, 50);

    overrideTunable(ManualConstants.HoodDegTunable, "Manual/HoodDegree", 12.0);
    step(manual);

    assertManualFeedPath();
    assertEquals(12.0 / 360.0, Hood.mInstance.getPositionSetpointRot(), EPS);
    manual.end(true);
  }

  @Test
  void operatorPanelMatchesGatedManualDecision() {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(Timer::getFPGATimestamp);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Drum.mInstance.setVelocityRotPerSec(0.0);
    Hood.mInstance.setPositionRot(0.25);
    runPeriodics();

    Manual manual = new Manual();
    manual.initialize();
    step(manual);

    assertEquals(Action.SPINNING_UP, OperatorStatus.currentAction());
    assertFalse(OperatorStatus.feedAllowed());
    assertNotEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());

    stepUntilFeeding(manual, 30);
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());
    assertTrue(OperatorStatus.feedAllowed());
    assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());
    manual.end(true);
  }

  @Test
  void endStopsEverything() {
    Manual manual = new Manual();
    manual.initialize();
    stepUntilFeeding(manual, 50);

    manual.end(true);
    runPeriodics();

    assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(0.0, Indexer.mInstance.getVelocitySetpointRotPerSec(), EPS);
  }

  @Test
  void forceFeedTimeoutReleasesAPermanentlyBelowSpeedDrum() throws Exception {
    Manual manual = new Manual();
    manual.initialize();
    stepUntilFeeding(manual, 50);

    // Sustained below-speed measurement: the gate holds for the 3 s timeout (150 loops of
    // 0.02 s — Manual accumulates from the first waiting loop), then force-feeds.
    for (int i = 0; i < 156; i++) {
      injectDrumVelocity(0.0);
      step(manual);
      if (i < 150) {
        assertEquals(
            0.0,
            Feeder.mInstance.getVelocitySetpointRotPerSec(),
            EPS,
            "gate holds before the timeout, loop " + i);
      }
    }
    assertTrue(Feeder.mInstance.getVelocitySetpointRotPerSec() > 0, "force-feed after the timeout");

    // 0% still stops a latched force-feed, and the latch survives the 0% hold.
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    step(manual);
    assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS);
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    injectDrumVelocity(0.0);
    step(manual);
    assertTrue(
        Feeder.mInstance.getVelocitySetpointRotPerSec() > 0, "latch persists across the 0% hold");

    // A fresh press re-arms the gate.
    manual.end(true);
    manual.initialize();
    injectDrumVelocity(0.0);
    step(manual);
    assertEquals(0.0, Feeder.mInstance.getVelocitySetpointRotPerSec(), EPS, "fresh press re-arms");
    manual.end(true);
  }

  @Test
  void acceptanceManualStopsOnFirstZeroFlywheelLoop() {
    Manual manual = new Manual();
    manual.initialize();
    stepUntilFeeding(manual, 50);

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    step(manual);
    assertEquals(
        0.0,
        Feeder.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "feeder must stop on the detection loop");
    assertEquals(
        0.0,
        Indexer.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "indexer must stop on the detection loop");
    assertEquals(
        0.0,
        IntakeRoller.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "intake roller must stop on the detection loop");
    manual.end(true);
  }
}

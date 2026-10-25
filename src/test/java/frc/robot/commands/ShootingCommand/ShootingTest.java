package frc.robot.commands.ShootingCommand;

import static frc.robot.util.DashboardControlsTestUtil.FLYWHEEL;
import static frc.robot.util.DashboardControlsTestUtil.HOOD;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.lib.io.MotorIO;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Drum.DrumConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodConstants;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.DashboardControlsTestUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.OperatorStatus;
import frc.robot.util.OperatorStatus.Action;
import frc.robot.util.ShotReadiness;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Runs Shooting against the sim mechanism IO with no Drive, pose, or vision ever created: if the
 * command touched any of them it would throw on the null Drive.mInstance.
 *
 * <p>The instant-setpoint sim snaps Drum/Hood to their setpoints on the next periodic, so a
 * dashboard edit leaves exactly one loop where the new target and old measurement disagree. Feeding
 * must continue through that mismatch; only a 0% flywheel limit pauses the feed path.
 */
class ShootingTest {
  private static final double EPS = 1e-6;

  @BeforeAll
  static void setUpHal() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
  }

  @AfterEach
  void resetDashboard() {
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(null);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setDsAttached(false);
    DriverStationSim.notifyNewData();
    DashboardControlsTestUtil.resetDefaults();
  }

  /** One robot loop: run the command, advance time, then update mechanism inputs. */
  private static void step(Shooting shooting) {
    shooting.execute();
    SimHooks.stepTiming(0.02);
    Drum.mInstance.periodic();
    Hood.mInstance.periodic();
    Feeder.mInstance.periodic();
    Indexer.mInstance.periodic();
    IntakeDeploy.mInstance.periodic();
    IntakeRoller.mInstance.periodic();
  }

  /** Same loop, plus the operator-panel periodic the robot would run. */
  private static void stepWithPanel(Shooting shooting) {
    shooting.execute();
    OperatorStatus.update();
    SimHooks.stepTiming(0.02);
    Drum.mInstance.periodic();
    Hood.mInstance.periodic();
    Feeder.mInstance.periodic();
    Indexer.mInstance.periodic();
    IntakeDeploy.mInstance.periodic();
    IntakeRoller.mInstance.periodic();
  }

  /** The same panel loop for the Manual command. */
  private static void stepWithPanel(frc.robot.commands.ManualCommand.Manual manual) {
    manual.execute();
    OperatorStatus.update();
    SimHooks.stepTiming(0.02);
    Drum.mInstance.periodic();
    Hood.mInstance.periodic();
    Feeder.mInstance.periodic();
    Indexer.mInstance.periodic();
    IntakeDeploy.mInstance.periodic();
    IntakeRoller.mInstance.periodic();
  }

  private static double feederSetpoint() {
    return Feeder.mInstance.getVelocitySetpointRotPerSec();
  }

  private static double indexerSetpoint() {
    return Indexer.mInstance.getVelocitySetpointRotPerSec();
  }

  private static double feedSpeed() {
    return ShootingConstants.FeederRotpsTunable.getAsDouble();
  }

  private static boolean isFeeding() {
    return Math.abs(feederSetpoint() - feedSpeed()) < EPS;
  }

  /** Steps until the feeder runs at the tuned feed speed; false when the loop budget ran out. */
  private static boolean stepUntilFeeding(Shooting shooting, int maxLoops) {
    for (int i = 0; i < maxLoops; i++) {
      step(shooting);
      if (isFeeding()) {
        return true;
      }
    }
    return false;
  }

  @Test
  void velocityModeFeedsOnFirstExecuteWithoutPoseOrVision() {
    assertNull(Drive.mInstance, "test must run with no drivetrain / pose source");
    DashboardControlsTestUtil.set(FLYWHEEL, 50.0);
    DashboardControlsTestUtil.set(HOOD, 100.0);
    Drum.mInstance.setVelocityRotPerSec(0.0);
    Hood.mInstance.setPositionRot(0.0);
    Indexer.mInstance.stop();
    Feeder.mInstance.stop();
    Drum.mInstance.periodic();
    Hood.mInstance.periodic();
    Indexer.mInstance.periodic();
    Feeder.mInstance.periodic();
    assertEquals(0.0, Drum.mInstance.getVelocityRotPerSec(), EPS, "drum starts stopped");
    assertEquals(0.0, Hood.mInstance.getPositionRot(), EPS, "hood starts off its 100% target");

    Shooting shooting = new Shooting(false);
    assertFalse(shooting.getRequirements().contains(Drive.mInstance));
    shooting.initialize();
    step(shooting);

    assertEquals(feedSpeed(), feederSetpoint(), EPS, "feeder runs on the first execute");
    assertEquals(
        ShootingConstants.IndexerRotpsTunable.getAsDouble(),
        indexerSetpoint(),
        EPS,
        "indexer runs on the first execute");
    assertEquals(
        0.5 * DrumConstants.MaxShotVelocity, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(HoodConstants.MaxAngleDeg / 360.0, Hood.mInstance.getPositionSetpointRot(), EPS);
    shooting.end(true);
  }

  @Test
  void zeroFlywheelNeverFeeds() {
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);

    Shooting shooting = new Shooting(false);
    shooting.initialize();
    for (int i = 0; i < 50; i++) {
      step(shooting);
    }

    assertEquals(0.0, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(0.0, feederSetpoint(), EPS);
    assertEquals(0.0, indexerSetpoint(), EPS);
    shooting.end(true);
  }

  @Test
  void startupFeedsWithoutWaitingForReadinessDebounce() {
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    step(shooting);

    assertTrue(isFeeding(), "the first execute feeds without a 0.15 s stable window");
    assertEquals(ShootingConstants.IndexerRotpsTunable.getAsDouble(), indexerSetpoint(), EPS);
    shooting.end(true);
  }

  @Test
  void pausesOnZeroFlywheelMidVolleyAndAutoResumes() {
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    assertTrue(stepUntilFeeding(shooting, 30), "reached SHOOT");

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    step(shooting);
    assertEquals(0.0, feederSetpoint(), EPS, "feed path stopped on the detection loop");
    assertEquals(0.0, indexerSetpoint(), EPS, "indexer stopped on the detection loop");
    assertEquals(
        0.0,
        Drum.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "drum still commanded, held at zero by the limit");
    for (int i = 0; i < 18; i++) {
      step(shooting);
      assertEquals(0.0, feederSetpoint(), EPS, "still paused at hold loop " + i);
    }

    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    step(shooting);
    assertTrue(isFeeding(), "positive limit resumes on that loop, without re-initialization");
    assertEquals(ShootingConstants.IndexerRotpsTunable.getAsDouble(), indexerSetpoint(), EPS);
    shooting.end(true);
  }

  @Test
  void pauseFreezesRetractTimer() {
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    assertTrue(stepUntilFeeding(shooting, 30), "reached SHOOT");

    // Pause almost immediately after feeding starts, long before the 0.2 s retract delay
    // accumulates, and hold the pause for twice that delay: without the freeze, RETRACT would
    // fire on the very first loop after resume.
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    for (int i = 0; i < 20; i++) {
      step(shooting);
    }
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    step(shooting);
    assertTrue(isFeeding(), "resumed feeding on the positive-limit loop");

    double retractCm = ShootingConstants.INTAKE_RETRACT_POSITION_CM.getAsDouble();
    for (int i = 0; i < 3; i++) {
      assertTrue(
          Math.abs(IntakeDeploy.mInstance.getPositionSetpointRot() - retractCm) > EPS,
          "retract timer resumed mid-count, no instant RETRACT at loop " + i);
      step(shooting);
    }
    boolean retracted = false;
    for (int i = 0; i < 15 && !retracted; i++) {
      step(shooting);
      retracted = Math.abs(IntakeDeploy.mInstance.getPositionSetpointRot() - retractCm) < EPS;
    }
    assertTrue(retracted, "RETRACT fired after the remaining feed time");
    shooting.end(true);
  }

  @Test
  void interruptDuringPauseDoesNotAutoResume() {
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    assertTrue(stepUntilFeeding(shooting, 30), "reached SHOOT");

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    step(shooting);
    assertEquals(0.0, feederSetpoint(), EPS, "paused on the detection loop");

    shooting.end(true);
    assertEquals(0.0, feederSetpoint(), EPS, "end() stopped the feeder");
    assertEquals(0.0, indexerSetpoint(), EPS, "end() stopped the indexer");

    // Releasing and re-pressing the button re-initializes the same instance; the fresh volley must
    // remain stopped while the limit is still 0 rather than resuming the old volley.
    shooting.initialize();
    for (int i = 0; i < 30; i++) {
      step(shooting);
      assertEquals(0.0, feederSetpoint(), EPS, "no auto-resume of the old volley at loop " + i);
    }
    shooting.end(true);
  }

  @Test
  void midVolleyTargetChangeDoesNotInterruptFeedingOrRetractTiming() throws Exception {
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    assertTrue(stepUntilFeeding(shooting, 30), "reached SHOOT at 100%");

    double retractElapsedBeforeEdit = retractElapsed(shooting);
    DashboardControlsTestUtil.set(FLYWHEEL, 50.0);
    step(shooting);
    assertTrue(isFeeding(), "the target edit does not pause on its stale-measurement loop");
    assertEquals(ShootingConstants.IndexerRotpsTunable.getAsDouble(), indexerSetpoint(), EPS);
    assertTrue(
        retractElapsed(shooting) > retractElapsedBeforeEdit,
        "a target edit must not freeze the retract timer");
    assertEquals(
        0.5 * DrumConstants.MaxShotVelocity,
        Drum.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "drum now commanded at 50%");
    shooting.end(true);
  }

  @Test
  void unjamResumesImmediatelyWithoutReadinessGate() throws Exception {
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    assertTrue(stepUntilFeeding(shooting, 30), "reached SHOOT");

    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    boolean unjamming = false;
    for (int i = 0; i < 20 && !unjamming; i++) {
      step(shooting);
      unjamming = indexerSetpoint() < -EPS;
    }
    assertTrue(unjamming, "UNJAM active: indexer backing off");
    assertEquals(-ShootingConstants.UNJAM_INDEXER_Current.getAsDouble(), indexerSetpoint(), EPS);
    assertEquals(
        -ShootingConstants.UNJAM_Feeder_Current.getAsDouble(),
        Feeder.mInstance.getVoltageSetpoint(),
        EPS);
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);

    for (int i = 0; i < 40 && stateOf(shooting).equals("UNJAM"); i++) {
      step(shooting);
      if (stateOf(shooting).equals("UNJAM")) {
        assertTrue(indexerSetpoint() < -EPS, "indexer keeps reversing until the back-off ends");
      }
    }
    assertNotEquals("UNJAM", stateOf(shooting), "back-off ran to completion");

    // Change both targets after the back-off so the next execute sees stale, off-target
    // measurements. It must nevertheless feed on that first post-UNJAM loop, with no stable window.
    DashboardControlsTestUtil.set(FLYWHEEL, 50.0);
    DashboardControlsTestUtil.set(HOOD, 100.0);
    step(shooting);
    assertTrue(isFeeding(), "forward feed resumes on the first loop after UNJAM");
    assertEquals(ShootingConstants.IndexerRotpsTunable.getAsDouble(), indexerSetpoint(), EPS);
    assertEquals(
        0.5 * DrumConstants.MaxShotVelocity, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(HoodConstants.MaxAngleDeg / 360.0, Hood.mInstance.getPositionSetpointRot(), EPS);
    shooting.end(true);
  }

  @Test
  void unjamIsNotSuppressedByZeroFlywheel() throws Exception {
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    assertTrue(stepUntilFeeding(shooting, 30), "reached SHOOT");

    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    boolean unjamming = false;
    for (int i = 0; i < 20 && !unjamming; i++) {
      step(shooting);
      unjamming = indexerSetpoint() < -EPS;
    }
    assertTrue(unjamming, "UNJAM active: indexer backing off");
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    for (int i = 0; i < 10; i++) {
      step(shooting);
      assertTrue(indexerSetpoint() < -EPS, "back-off continues despite 0% flywheel, loop " + i);
    }
    for (int i = 0; i < 20 && stateOf(shooting).equals("UNJAM"); i++) {
      step(shooting);
    }
    assertNotEquals("UNJAM", stateOf(shooting), "back-off ran to completion at 0%");

    for (int i = 0; i < 30; i++) {
      step(shooting);
      assertEquals(0.0, feederSetpoint(), EPS, "paused while the flywheel limit is 0, loop " + i);
    }

    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    step(shooting);
    assertTrue(isFeeding(), "positive limit restores feed immediately after a 0% hold");
    shooting.end(true);
  }

  @Test
  void operatorPanelMatchesFeedingDecision() {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(Timer::getFPGATimestamp);
    // DriverStation.isEnabled() is enabled && dsAttached; notifyNewData() refreshes the cache.
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    Shooting shooting = new Shooting(false);
    shooting.initialize();
    boolean feeding = false;
    for (int i = 0; i < 30 && !feeding; i++) {
      stepWithPanel(shooting);
      feeding = isFeeding();
    }
    assertTrue(feeding, "reached SHOOT");
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());
    assertTrue(OperatorStatus.feedAllowed(), "ready + enabled + no jam");

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    stepWithPanel(shooting);
    assertEquals(Action.WAITING_RECOVERY, OperatorStatus.currentAction());
    assertFalse(OperatorStatus.feedAllowed());
    assertEquals(ShotReadiness.REASON_FLYWHEEL_ZERO, OperatorStatus.feedBlockedReason());

    // Key released: the command stops heartbeating; after the 0.15 s expiry the panel is Idle.
    shooting.end(true);
    for (int i = 0; i < 10; i++) {
      SimHooks.stepTiming(0.02);
      Drum.mInstance.periodic();
      Hood.mInstance.periodic();
    }
    OperatorStatus.update();
    assertEquals(Action.IDLE, OperatorStatus.currentAction());
  }

  private static String stateOf(Shooting s) throws Exception {
    var f = Shooting.class.getDeclaredField("state");
    f.setAccessible(true);
    return f.get(s).toString();
  }

  private static double retractElapsed(Shooting s) throws Exception {
    var f = Shooting.class.getDeclaredField("retractTimer");
    f.setAccessible(true);
    return ((Timer) f.get(s)).get();
  }

  /** Overrides a tunable as an Elastic edit would, refreshing the cached value by hand. */
  private static void overrideTunable(LoggedTunableNumber tunable, String key, double value)
      throws Exception {
    NetworkTableInstance.getDefault().getEntry("/Tuning/" + key).setDouble(value);
    var f = LoggedTunableNumber.class.getDeclaredField("dashboardNumber");
    f.setAccessible(true);
    ((LoggedNetworkNumber) f.get(tunable)).periodic();
  }

  @Test
  void acceptanceStopsOnFirstZeroFlywheelLoop() {
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30));
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    step(s);
    assertEquals(0.0, feederSetpoint(), EPS, "must stop on detection loop");
    assertEquals(0.0, indexerSetpoint(), EPS, "indexer must stop too");
    s.end(true);
  }

  @Test
  void acceptanceUnjamFreezesRetractTimer() throws Exception {
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30));
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    for (int i = 0; i < 30 && !stateOf(s).equals("UNJAM"); i++) step(s);
    assertEquals("UNJAM", stateOf(s));
    double before = retractElapsed(s);
    for (int i = 0; i < 5; i++) step(s);
    assertEquals(before, retractElapsed(s), EPS, "UNJAM time must not count");
    s.end(true);
  }

  @Test
  void acceptanceUnjamDoesNotRequireFreshStableWindow() throws Exception {
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30));
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    for (int i = 0; i < 30 && !stateOf(s).equals("UNJAM"); i++) step(s);
    assertEquals("UNJAM", stateOf(s));
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);
    for (int i = 0; i < 40 && stateOf(s).equals("UNJAM"); i++) step(s);
    assertNotEquals("UNJAM", stateOf(s));
    step(s);
    assertTrue(isFeeding(), "one post-unjam execute must restore forward feed");
    assertEquals(ShootingConstants.IndexerRotpsTunable.getAsDouble(), indexerSetpoint(), EPS);
    s.end(true);
  }

  @Test
  void acceptancePanelUsesManualTarget() {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(Timer::getFPGATimestamp);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    var m = new frc.robot.commands.ManualCommand.Manual();
    m.initialize();
    for (int i = 0; i < 30; i++) {
      m.execute();
      OperatorStatus.update();
      SimHooks.stepTiming(.02);
      Drum.mInstance.periodic();
      Hood.mInstance.periodic();
      Feeder.mInstance.periodic();
      Indexer.mInstance.periodic();
    }
    assertTrue(feederSetpoint() > 0, "Manual is actually feeding");
    assertTrue(OperatorStatus.feedAllowed(), "panel should agree with active Manual target");
    m.end(true);
  }

  @Test
  void acceptancePanelAllowsImmediateFeedingBeforeStableReady() {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(Timer::getFPGATimestamp);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Shooting s = new Shooting(false);
    s.initialize();
    stepWithPanel(s);

    assertTrue(isFeeding(), "feeding starts without a startup debounce");
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());
    assertTrue(OperatorStatus.feedAllowed(), "panel matches the actual first-loop feed decision");
    assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());
    s.end(true);
  }

  @Test
  void acceptanceCurrentModeFeedsImmediatelyAndStopsAtZeroFlywheel() {
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    DashboardControlsTestUtil.set(HOOD, 100.0);
    Drum.mInstance.setVelocityRotPerSec(0.0);
    Hood.mInstance.setPositionRot(0.0);
    Indexer.mInstance.stop();
    Feeder.mInstance.stop();
    Drum.mInstance.periodic();
    Hood.mInstance.periodic();
    Indexer.mInstance.periodic();
    Feeder.mInstance.periodic();
    assertEquals(0.0, Drum.mInstance.getVelocityRotPerSec(), EPS, "drum starts stopped");
    assertEquals(0.0, Hood.mInstance.getPositionRot(), EPS, "hood starts off its 100% target");

    Shooting s = new Shooting(true);
    s.initialize();
    step(s);
    assertEquals(MotorIO.ControlMode.CURRENT, Feeder.mInstance.getControlMode());
    assertEquals(50.0, Feeder.mInstance.getCurrentSetpointAmps(), EPS);
    assertEquals(MotorIO.ControlMode.CURRENT, Indexer.mInstance.getControlMode());
    assertEquals(50.0, Indexer.mInstance.getCurrentSetpointAmps(), EPS);
    assertEquals(DrumConstants.MaxShotVelocity, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(HoodConstants.MaxAngleDeg / 360.0, Hood.mInstance.getPositionSetpointRot(), EPS);

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    step(s);
    assertNotEquals(
        MotorIO.ControlMode.CURRENT,
        Feeder.mInstance.getControlMode(),
        "feeder leaves CURRENT on the detection loop");
    assertEquals(0.0, Feeder.mInstance.getCurrentSetpointAmps(), EPS, "no current request left");
    assertEquals(0.0, Feeder.mInstance.getVoltageSetpoint(), EPS);
    assertEquals(0.0, feederSetpoint(), EPS);
    assertNotEquals(MotorIO.ControlMode.CURRENT, Indexer.mInstance.getControlMode());
    assertEquals(0.0, Indexer.mInstance.getCurrentSetpointAmps(), EPS);
    assertEquals(0.0, Indexer.mInstance.getVoltageSetpoint(), EPS);
    assertEquals(0.0, indexerSetpoint(), EPS);

    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    step(s);
    assertEquals(MotorIO.ControlMode.CURRENT, Feeder.mInstance.getControlMode());
    assertEquals(50.0, Feeder.mInstance.getCurrentSetpointAmps(), EPS);
    assertEquals(MotorIO.ControlMode.CURRENT, Indexer.mInstance.getControlMode());
    assertEquals(50.0, Indexer.mInstance.getCurrentSetpointAmps(), EPS);
    s.end(true);
  }

  @Test
  void acceptanceUnjamRetractFiresAfterRemainingTime() throws Exception {
    // Stretch the retract delay so the jam + back-off land inside it: without the freeze at UNJAM
    // entry the timer would outrun the delay during the back-off and RETRACT would fire instantly
    // on resume; with it, only the pre-jam feeding time counts.
    overrideTunable(ShootingConstants.RETRACT_DELAY_SEC, "Shooting/retract_delay", 0.5);
    try {
      Shooting s = new Shooting(false);
      s.initialize();
      assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");

      Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
      for (int i = 0; i < 30 && !stateOf(s).equals("UNJAM"); i++) {
        step(s);
      }
      assertEquals("UNJAM", stateOf(s));
      double frozenAt = retractElapsed(s);
      assertTrue(frozenAt < 0.5, "retract delay not yet reached when the jam fired");
      Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);

      for (int i = 0; i < 40 && stateOf(s).equals("UNJAM"); i++) {
        step(s);
      }
      assertNotEquals("UNJAM", stateOf(s));
      assertEquals(frozenAt, retractElapsed(s), EPS, "unjam time never counted");
      step(s);
      assertTrue(isFeeding(), "feed resumes on the first post-unjam loop");

      double retractCm = ShootingConstants.INTAKE_RETRACT_POSITION_CM.getAsDouble();
      for (int i = 0; i < 5; i++) {
        assertTrue(
            Math.abs(IntakeDeploy.mInstance.getPositionSetpointRot() - retractCm) > EPS,
            "no instant retract on resume, loop " + i);
        step(s);
      }
      boolean retracted = false;
      for (int i = 0; i < 40 && !retracted; i++) {
        step(s);
        retracted = Math.abs(IntakeDeploy.mInstance.getPositionSetpointRot() - retractCm) < EPS;
      }
      assertTrue(retracted, "retract fired after the remaining feed time");
      s.end(true);
    } finally {
      overrideTunable(ShootingConstants.RETRACT_DELAY_SEC, "Shooting/retract_delay", 0.2);
    }
  }

  @Test
  void acceptancePanelTracksControlLoopByLoop() throws Exception {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(Timer::getFPGATimestamp);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    Shooting s = new Shooting(false);
    s.initialize();
    stepWithPanel(s);
    assertTrue(isFeeding(), "startup feeds on the first loop");
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());
    assertTrue(OperatorStatus.feedAllowed());
    assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    stepWithPanel(s);
    assertEquals("PAUSED", stateOf(s));
    assertEquals(0.0, feederSetpoint(), EPS, "feed stops on the detection loop");
    assertEquals(0.0, indexerSetpoint(), EPS);
    assertEquals(Action.WAITING_RECOVERY, OperatorStatus.currentAction());
    assertFalse(OperatorStatus.feedAllowed());
    assertEquals(ShotReadiness.REASON_FLYWHEEL_ZERO, OperatorStatus.feedBlockedReason());

    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    stepWithPanel(s);
    assertTrue(isFeeding(), "feed resumes on the positive-limit loop");
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());
    assertTrue(OperatorStatus.feedAllowed(), "panel allows on the same resume loop");
    assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());

    // While the jam debounce runs the volley keeps feeding and the panel agrees; in UNJAM the panel
    // shows the jam action and blocks feeding.
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    boolean unjammed = false;
    for (int i = 0; i < 30 && !unjammed; i++) {
      stepWithPanel(s);
      // The jam debounce (0.2 s) and the retract delay (0.2 s) expire on the same loop here; which
      // feeding state the jam trips from depends on a one-loop test-clock lag, so accept both.
      String st = stateOf(s);
      if (st.equals("UNJAM")) {
        unjammed = true;
      } else {
        assertTrue(
            st.equals("SHOOT") || st.equals("RETRACT"),
            "still in the volley, loop " + i + ": " + st);
        assertTrue(isFeeding(), "still feeding during the jam debounce, loop " + i);
        assertTrue(OperatorStatus.feedAllowed(), "loop " + i);
      }
    }
    assertTrue(unjammed, "UNJAM triggered");
    assertEquals(Action.CLEARING_JAM, OperatorStatus.currentAction());
    assertFalse(OperatorStatus.feedAllowed(), "jam clearing blocks feeding");
    assertEquals(ShotReadiness.REASON_CLEARING_JAM, OperatorStatus.feedBlockedReason());
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);
    for (int i = 0; i < 40 && stateOf(s).equals("UNJAM"); i++) {
      stepWithPanel(s);
      if (stateOf(s).equals("UNJAM")) {
        assertFalse(OperatorStatus.feedAllowed(), "UNJAM loop " + i);
        assertEquals(ShotReadiness.REASON_CLEARING_JAM, OperatorStatus.feedBlockedReason());
      }
    }
    assertNotEquals("UNJAM", stateOf(s), "back-off ran to completion");

    stepWithPanel(s);
    assertTrue(isFeeding(), "feed resumes on the first post-unjam loop");
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());
    assertTrue(OperatorStatus.feedAllowed(), "no stale jam overlay may block actual feeding");
    assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());

    // Key released: end() still stops the motors and clears the command decision. Once the action
    // heartbeat expires, a positive-limit fallback ignores drum/hood readiness diagnostics.
    s.end(true);
    // stop() clears the IO request immediately; the subsystem getters read refreshed inputs.
    SimHooks.stepTiming(0.02);
    Feeder.mInstance.periodic();
    Indexer.mInstance.periodic();
    assertEquals(0.0, feederSetpoint(), EPS, "end() stops the feeder");
    assertEquals(0.0, indexerSetpoint(), EPS, "end() stops the indexer");
    for (int i = 0; i < 10; i++) {
      SimHooks.stepTiming(0.02);
      Drum.mInstance.periodic();
      Hood.mInstance.periodic();
    }
    OperatorStatus.update();
    assertEquals(Action.IDLE, OperatorStatus.currentAction());
    assertTrue(OperatorStatus.feedAllowed(), "positive-limit fallback is not readiness-gated");
    assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());
  }

  @Test
  void acceptancePanelFollowsActiveCommandSwitch() {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(Timer::getFPGATimestamp);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    Shooting shooting = new Shooting(false);
    shooting.initialize();
    boolean feeding = false;
    for (int i = 0; i < 30 && !feeding; i++) {
      stepWithPanel(shooting);
      feeding = isFeeding();
    }
    assertTrue(feeding, "Shooting reached SHOOT");
    assertTrue(OperatorStatus.feedAllowed(), "panel follows Shooting");
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());

    shooting.end(true);
    // Refresh the subsystem inputs before reading the stopped request.
    SimHooks.stepTiming(0.02);
    Feeder.mInstance.periodic();
    Indexer.mInstance.periodic();
    assertEquals(0.0, feederSetpoint(), EPS, "Shooting end() stops the feeder");
    assertEquals(0.0, indexerSetpoint(), EPS, "Shooting end() stops the indexer");
    for (int i = 0; i < 10; i++) {
      SimHooks.stepTiming(0.02);
      Drum.mInstance.periodic();
      Hood.mInstance.periodic();
    }
    OperatorStatus.update();
    assertEquals(Action.IDLE, OperatorStatus.currentAction());
    assertTrue(
        OperatorStatus.feedAllowed(), "expired Shooting decision uses positive-limit fallback");
    assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());

    // Manual uses its own 50 rot/s target. While it is active, the panel follows the command's
    // actual feed decision rather than comparing that target with the dashboard fallback target.
    var manual = new frc.robot.commands.ManualCommand.Manual();
    manual.initialize();
    feeding = false;
    for (int i = 0; i < 30 && !feeding; i++) {
      stepWithPanel(manual);
      feeding = feederSetpoint() > 0;
    }
    assertTrue(feeding, "Manual reached SHOOT");
    assertTrue(OperatorStatus.feedAllowed(), "panel follows the active Manual target");
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());

    manual.end(true);
    // Refresh the subsystem inputs before reading the stopped request.
    SimHooks.stepTiming(0.02);
    Feeder.mInstance.periodic();
    Indexer.mInstance.periodic();
    assertEquals(0.0, feederSetpoint(), EPS, "Manual end() stops the feeder");
    assertEquals(0.0, indexerSetpoint(), EPS, "Manual end() stops the indexer");
    for (int i = 0; i < 10; i++) {
      SimHooks.stepTiming(0.02);
      Drum.mInstance.periodic();
      Hood.mInstance.periodic();
    }
    OperatorStatus.update();
    assertEquals(Action.IDLE, OperatorStatus.currentAction());
    assertTrue(
        OperatorStatus.feedAllowed(), "expired Manual decision uses positive-limit fallback");
    assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());
  }

  @Test
  void longPauseThenHighCurrentSpikeDoesNotUnjam() throws Exception {
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");

    // Hold a 0.4 s pause: if paused time free-ran into the jam debouncer, the 0.2 s confirmation
    // window would already be filled when feeding resumes.
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    for (int i = 0; i < 20; i++) {
      step(s);
    }
    assertEquals("PAUSED", stateOf(s));

    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    step(s);
    assertTrue(isFeeding(), "resumed feeding on the positive-limit loop");

    // Refresh the indexer inputs before the next execute: step() applies the override in the
    // periodic after execute, so without this the first post-resume jam check would read the
    // stale low current, feed a clear sample, and hide a free-running debouncer.
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    Indexer.mInstance.periodic();
    step(s);
    assertNotEquals(
        "UNJAM", stateOf(s), "a single high-current sample after a long pause must not trip UNJAM");

    // The spike falls right back: no jam may ever be confirmed.
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);
    Indexer.mInstance.periodic();
    for (int i = 0; i < 15; i++) {
      step(s);
      assertNotEquals("UNJAM", stateOf(s), "loop " + i);
    }
    s.end(true);
  }

  @Test
  void longPauseThenSustainedHighCurrentUnjamsOnlyAfterFullWindow() throws Exception {
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    for (int i = 0; i < 20; i++) {
      step(s);
    }
    assertEquals("PAUSED", stateOf(s));

    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    step(s);
    assertTrue(isFeeding(), "resumed feeding on the positive-limit loop");

    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    Indexer.mInstance.periodic();
    // The 0.2 s window is ten 0.02 s loops of sustained high current; a debouncer that free-ran
    // through the pause would trip on the very first loop instead.
    for (int i = 0; i < 8; i++) {
      step(s);
      assertNotEquals("UNJAM", stateOf(s), "the window restarts at the resume, loop " + i);
    }
    boolean unjammed = false;
    for (int i = 0; i < 20 && !unjammed; i++) {
      step(s);
      unjammed = stateOf(s).equals("UNJAM");
    }
    assertTrue(unjammed, "sustained high current still trips UNJAM after the full window");
    s.end(true);
  }

  @Test
  void postUnjamRecoveryDoesNotRetripInstantly() throws Exception {
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");

    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    for (int i = 0; i < 30 && !stateOf(s).equals("UNJAM"); i++) {
      step(s);
    }
    assertEquals("UNJAM", stateOf(s));
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);

    for (int i = 0; i < 40 && stateOf(s).equals("UNJAM"); i++) {
      step(s);
    }
    assertNotEquals("UNJAM", stateOf(s));
    step(s);
    assertTrue(isFeeding(), "feeding again on the first post-unjam loop");

    // No residual timing from the pre-unjam window may carry over: the first spike after the
    // recovery must again have to persist for the full debounce.
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    Indexer.mInstance.periodic();
    for (int i = 0; i < 8; i++) {
      step(s);
      assertNotEquals("UNJAM", stateOf(s), "post-unjam window restarts clean, loop " + i);
    }
    s.end(true);
  }

  @Test
  void midVolleyHoodTargetChangeDoesNotInterruptFeedingOrRetractTiming() throws Exception {
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");

    double retractElapsedBeforeEdit = retractElapsed(s);
    DashboardControlsTestUtil.set(HOOD, 100.0);
    step(s);
    assertTrue(isFeeding(), "hood target change must not stop the feed on the edit loop");
    assertEquals(ShootingConstants.IndexerRotpsTunable.getAsDouble(), indexerSetpoint(), EPS);
    assertTrue(
        retractElapsed(s) > retractElapsedBeforeEdit,
        "hood target change must not freeze the retract timer");
    assertEquals(HoodConstants.MaxAngleDeg / 360.0, Hood.mInstance.getPositionSetpointRot(), EPS);
    assertEquals(DrumConstants.MaxShotVelocity, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);
    s.end(true);
  }

  @Test
  void longZeroFlywheelHoldDoesNotPreFillJamWindow() throws Exception {
    // A long 0% hold is not feeding and must not pre-fill the jam window. The first high-current
    // sample after the positive-limit resume still needs the full 0.2 s confirmation.
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    Shooting s = new Shooting(false);
    s.initialize();
    for (int i = 0; i < 25; i++) {
      step(s);
      assertEquals(0.0, feederSetpoint(), EPS);
      assertEquals(0.0, indexerSetpoint(), EPS);
    }

    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    step(s);
    assertTrue(isFeeding(), "feeding resumes on the first positive-limit loop");

    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    Indexer.mInstance.periodic(); // the override must reach the input before the next execute
    for (int i = 0; i < 8; i++) {
      step(s);
      assertNotEquals("UNJAM", stateOf(s), "0% hold must not pre-fill the window, loop " + i);
    }

    boolean unjammed = false;
    for (int i = 0; i < 20 && !unjammed; i++) {
      step(s);
      unjammed = stateOf(s).equals("UNJAM");
    }
    assertTrue(unjammed, "sustained high current still trips UNJAM after the full window");
    s.end(true);
  }

  @Test
  void repeatedPauseResumeCyclesKeepJamWindowClean() throws Exception {
    // Two full pause/resume cycles, then a current spike: neither pause may pre-fill the window.
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");

    for (int cycle = 0; cycle < 2; cycle++) {
      DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
      step(s);
      assertEquals("PAUSED", stateOf(s), "paused, cycle " + cycle);
      for (int i = 0; i < 15; i++) {
        step(s);
      }
      DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
      step(s);
      assertTrue(isFeeding(), "resumed feeding on one loop, cycle " + cycle);
    }

    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    Indexer.mInstance.periodic();
    for (int i = 0; i < 8; i++) {
      step(s);
      assertNotEquals("UNJAM", stateOf(s), "no pause time pre-fills the window, loop " + i);
    }
    s.end(true);
  }

  @Test
  void confirmedJamTakesPrecedenceOverZeroFlywheelOnSameLoop() throws Exception {
    // A shadow debouncer created at the same mock-clock timestamp and fed the same samples tells
    // us exactly which loop completes the 0.2 s confirmation, without depending on test order or
    // a fixed sample count. On that loop the flywheel limit also drops to 0%; the already-earned
    // UNJAM must take precedence over the same-loop pause.
    Shooting s = new Shooting(false);
    s.initialize();
    Debouncer shadowJam =
        new Debouncer(ShootingConstants.INDEXER_JAM_DEBOUNCE_SEC, DebounceType.kRising);
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);
    Indexer.mInstance.periodic();
    assertTrue(!shadowJam.calculate(false), "shadow starts from a clear sample");
    step(s);
    assertTrue(isFeeding(), "first execute feeds and gives the jam debouncer a clear sample");

    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    boolean confirmed = false;
    for (int sample = 1; sample <= 30 && !confirmed; sample++) {
      Indexer.mInstance.periodic();
      boolean shadowConfirmed = shadowJam.calculate(true);
      if (shadowConfirmed) {
        DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
      }
      step(s);
      if (shadowConfirmed) {
        confirmed = true;
      } else {
        assertEquals(
            ShootingConstants.IndexerRotpsTunable.getAsDouble(),
            indexerSetpoint(),
            EPS,
            "the confirmation is not complete at high-current sample " + sample);
      }
    }
    assertTrue(confirmed, "found the confirmation sample");
    assertEquals(
        -ShootingConstants.UNJAM_INDEXER_Current.getAsDouble(),
        indexerSetpoint(),
        EPS,
        "confirmed jam takes precedence over the same-loop 0% pause");
    assertEquals(
        -ShootingConstants.UNJAM_Feeder_Current.getAsDouble(),
        Feeder.mInstance.getVoltageSetpoint(),
        EPS,
        "feeder reverses with the confirmed jam despite the 0% limit");
    assertEquals(
        0.0,
        Drum.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "the 0% limit still holds the drum target at zero during UNJAM");
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);

    for (int i = 0; i < 20; i++) {
      step(s);
      assertTrue(indexerSetpoint() < -EPS, "UNJAM keeps reversing at 0%, loop " + i);
    }
    boolean reversing = true;
    for (int i = 0; i < 30 && reversing; i++) {
      step(s);
      reversing = indexerSetpoint() < -EPS;
    }
    assertFalse(reversing, "the full back-off completes at 0%");
    assertEquals(0.0, indexerSetpoint(), EPS, "indexer stays stopped after UNJAM while at 0%");
    assertEquals(0.0, feederSetpoint(), EPS, "feeder stays stopped after UNJAM while at 0%");
    s.end(true);
  }

  @Test
  void secondIndependentJamStillTripsAfterFullWindow() throws Exception {
    // Two complete, independent jam cycles: after the first back-off and immediate recovery, a
    // second sustained stall must trip UNJAM again without inheriting the first window's timing.
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");

    for (int jam = 1; jam <= 2; jam++) {
      Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
      boolean unjammed = false;
      for (int i = 0; i < 30 && !unjammed; i++) {
        step(s);
        unjammed = stateOf(s).equals("UNJAM");
      }
      assertTrue(unjammed, "jam " + jam + " tripped");
      Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);
      for (int i = 0; i < 40 && stateOf(s).equals("UNJAM"); i++) {
        step(s);
      }
      assertNotEquals("UNJAM", stateOf(s), "back-off " + jam + " completed");
      step(s);
      assertTrue(isFeeding(), "feeding recovered immediately after jam " + jam);
    }
    s.end(true);
  }
}

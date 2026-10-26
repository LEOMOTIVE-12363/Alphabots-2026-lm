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
 * <p>The instant-setpoint sim snaps Drum/Hood to their setpoints on the next periodic, so after a
 * press or a dashboard edit the flywheel reaches its new target within a loop or two. Feeding is
 * gated on that measured speed plus the 0.15 s stable window: a press from a stopped drum feeds
 * only after spin-up, and a mid-volley edit or sag pauses the feed path on the detection loop and
 * auto-resumes once the shot re-earns the stable window. The hood never gates feeding.
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
  void velocityModeWaitsForFlywheelSpeedThenFeeds() {
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

    Shooting shooting = new Shooting(false);
    assertFalse(shooting.getRequirements().contains(Drive.mInstance));
    shooting.initialize();
    step(shooting);

    // The very first execute drives the shooter targets but must not feed: the drum is still at 0
    // against its 33.5 rot/s target, so a hopper-fed ball would jam against a slow flywheel.
    assertEquals(0.0, feederSetpoint(), EPS, "no feed on the first execute from a stopped drum");
    assertEquals(0.0, indexerSetpoint(), EPS);
    assertEquals(
        0.5 * DrumConstants.MaxShotVelocity,
        Drum.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "drum commanded from the first execute");
    assertEquals(HoodConstants.MaxAngleDeg / 360.0, Hood.mInstance.getPositionSetpointRot(), EPS);

    assertTrue(stepUntilFeeding(shooting, 30), "feeds once the flywheel is at speed and stable");
    assertEquals(feedSpeed(), feederSetpoint(), EPS);
    assertEquals(ShootingConstants.IndexerRotpsTunable.getAsDouble(), indexerSetpoint(), EPS);
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
  void startupWaitsForStableSpeedWindow() {
    // Defaults are ready from boot (flywheel 100%, drum snapped to target after the first loop),
    // but feeding still waits for the full 0.15 s stable window before the first ball moves.
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    for (int i = 0; i < 3; i++) {
      step(shooting);
      assertEquals(0.0, feederSetpoint(), EPS, "still spinning up at loop " + i);
    }
    for (int i = 3; i < 12; i++) {
      step(shooting);
    }
    assertEquals(feedSpeed(), feederSetpoint(), EPS, "feeding once stable-ready");
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
    assertTrue(
        stepUntilFeeding(shooting, 15),
        "auto-resumed within the spin-up and debounce window, no end/initialize in between");
    assertTrue(isFeeding());
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
    assertTrue(stepUntilFeeding(shooting, 15), "resumed feeding after the recovery window");

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
  void midVolleyTargetChangePausesBriefly() throws Exception {
    Shooting shooting = new Shooting(false);
    shooting.initialize();
    assertTrue(stepUntilFeeding(shooting, 30), "reached SHOOT at 100%");

    double retractElapsedBeforeEdit = retractElapsed(shooting);
    // New drum target 33.5 rot/s vs the still-measured 67: not ready until the sim snaps, and the
    // pause freezes the retract timer for that stretch.
    DashboardControlsTestUtil.set(FLYWHEEL, 50.0);
    step(shooting);
    assertEquals(0.0, feederSetpoint(), EPS, "feeding paused on the detection loop");
    assertEquals(
        retractElapsedBeforeEdit,
        retractElapsed(shooting),
        EPS,
        "the pause freezes the retract timer");
    assertEquals(
        0.5 * DrumConstants.MaxShotVelocity,
        Drum.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "the pause loop already drives the drum to the new target");

    assertTrue(stepUntilFeeding(shooting, 15), "auto-resumed at the new target");
    assertTrue(
        retractElapsed(shooting) > retractElapsedBeforeEdit,
        "feeding time after the resume counts toward the retract delay again");
    shooting.end(true);
  }

  @Test
  void unjamRoutesThroughReadinessGateBeforeResuming() throws Exception {
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
    // measurements. The readiness gate runs before any forward feed — the first post-unjam loop
    // must stay stopped even at a positive limit, and the volley resumes only after the drum
    // reaches the new target and re-earns the full stable window.
    DashboardControlsTestUtil.set(FLYWHEEL, 50.0);
    DashboardControlsTestUtil.set(HOOD, 100.0);
    step(shooting);
    assertEquals(0.0, feederSetpoint(), EPS, "not feeding on the first loop after unjam");
    assertEquals("PAUSED", stateOf(shooting));

    assertTrue(stepUntilFeeding(shooting, 15), "forward feed resumes after re-earning the window");
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
    assertTrue(
        stepUntilFeeding(shooting, 15), "feeding resumed once stable-ready after the 0% hold");
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
  void acceptanceUnjamRequiresFreshStableWindow() throws Exception {
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30));
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(100.0);
    for (int i = 0; i < 30 && !stateOf(s).equals("UNJAM"); i++) step(s);
    assertEquals("UNJAM", stateOf(s));
    Indexer.mInstance.setStatorCurrentAmpsOverrideForTest(null);
    for (int i = 0; i < 40 && stateOf(s).equals("UNJAM"); i++) step(s);
    assertEquals("PAUSED", stateOf(s));
    step(s);
    assertEquals("PAUSED", stateOf(s), "one sample cannot prove stable recovery after unjam");
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
  void acceptancePanelWaitsForStableSpeed() {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(Timer::getFPGATimestamp);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Shooting s = new Shooting(false);
    s.initialize();
    stepWithPanel(s);
    stepWithPanel(s);
    stepWithPanel(s);
    assertEquals(0.0, feederSetpoint(), EPS, "still inside the startup speed window");
    assertEquals(Action.SPINNING_UP, OperatorStatus.currentAction());
    assertFalse(OperatorStatus.feedAllowed(), "not yet permitted to feed");
    assertNotEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());
    s.end(true);
  }

  @Test
  void acceptanceCurrentModeWaitsForSpeedAndStopsOnFirstNotReadyLoop() {
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

    // Energy-save mode feeds with a 50 A current request, but only once the flywheel is at speed:
    // from a stopped drum the first execute requests the shooter targets and nothing else.
    Shooting s = new Shooting(true);
    s.initialize();
    step(s);
    assertNotEquals(MotorIO.ControlMode.CURRENT, Feeder.mInstance.getControlMode());
    assertEquals(0.0, Feeder.mInstance.getCurrentSetpointAmps(), EPS, "no current request yet");
    assertEquals(DrumConstants.MaxShotVelocity, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);
    assertEquals(HoodConstants.MaxAngleDeg / 360.0, Hood.mInstance.getPositionSetpointRot(), EPS);

    boolean currentFeeding = false;
    for (int i = 0; i < 30 && !currentFeeding; i++) {
      step(s);
      currentFeeding =
          Feeder.mInstance.getControlMode() == MotorIO.ControlMode.CURRENT
              && Math.abs(Feeder.mInstance.getCurrentSetpointAmps() - 50.0) < EPS;
    }
    assertTrue(currentFeeding, "energy-save mode feeds at 50 A once at speed");
    assertEquals(MotorIO.ControlMode.CURRENT, Indexer.mInstance.getControlMode());
    assertEquals(50.0, Indexer.mInstance.getCurrentSetpointAmps(), EPS);

    // The same-loop stop must drop the current request, not just a velocity setpoint.
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
    currentFeeding = false;
    for (int i = 0; i < 25 && !currentFeeding; i++) {
      step(s);
      currentFeeding =
          Feeder.mInstance.getControlMode() == MotorIO.ControlMode.CURRENT
              && Math.abs(Feeder.mInstance.getCurrentSetpointAmps() - 50.0) < EPS;
    }
    assertTrue(currentFeeding, "current feed resumes after the recovery window");
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
      assertEquals("PAUSED", stateOf(s));
      assertEquals(frozenAt, retractElapsed(s), EPS, "unjam time never counted");

      boolean resumed = false;
      for (int i = 0; i < 15 && !resumed; i++) {
        step(s);
        resumed = stateOf(s).equals("SHOOT");
      }
      assertTrue(resumed, "resumed after the post-unjam stable window");

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
    // Startup: the panel must never allow feeding before the control loop enters SHOOT; while the
    // drum is still below speed the reason is the speed gate, and once at speed but inside the
    // debounce it is Stabilizing.
    boolean sawStabilizing = false;
    boolean reachedShoot = false;
    for (int i = 0; i < 30 && !reachedShoot; i++) {
      stepWithPanel(s);
      String st = stateOf(s);
      if (st.equals("SHOOT") || st.equals("RETRACT")) {
        reachedShoot = true;
        assertTrue(
            OperatorStatus.feedAllowed(), "panel allows on the same loop control enters SHOOT");
        assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());
      } else {
        assertEquals("SPIN_UP", st);
        assertFalse(OperatorStatus.feedAllowed(), "no early allow at startup, loop " + i);
        String reason = OperatorStatus.feedBlockedReason();
        assertTrue(
            reason.equals(ShotReadiness.REASON_STABILIZING)
                || reason.equals(ShotReadiness.REASON_DRUM_NOT_AT_SPEED),
            "startup loop " + i + " reason: " + reason);
        sawStabilizing |= reason.equals(ShotReadiness.REASON_STABILIZING);
      }
    }
    assertTrue(reachedShoot, "reached SHOOT");
    assertTrue(sawStabilizing, "the startup debounce phase shows Stabilizing");

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    stepWithPanel(s);
    assertEquals("PAUSED", stateOf(s));
    assertEquals(0.0, feederSetpoint(), EPS, "feed stops on the detection loop");
    assertEquals(0.0, indexerSetpoint(), EPS);
    assertEquals(Action.WAITING_RECOVERY, OperatorStatus.currentAction());
    assertFalse(OperatorStatus.feedAllowed());
    assertEquals(ShotReadiness.REASON_FLYWHEEL_ZERO, OperatorStatus.feedBlockedReason());

    // Recovery: the panel waits out the same spin-up and debounce the control loop does.
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    boolean resumed = false;
    boolean sawRecoveryStabilizing = false;
    for (int i = 0; i < 15 && !resumed; i++) {
      stepWithPanel(s);
      String st = stateOf(s);
      if (st.equals("SHOOT") || st.equals("RETRACT")) {
        resumed = true;
        assertTrue(OperatorStatus.feedAllowed(), "panel allows on the resume loop");
        assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());
      } else {
        assertEquals("PAUSED", st);
        assertFalse(OperatorStatus.feedAllowed(), "no early allow during recovery, loop " + i);
        String reason = OperatorStatus.feedBlockedReason();
        assertTrue(
            reason.equals(ShotReadiness.REASON_STABILIZING)
                || reason.equals(ShotReadiness.REASON_DRUM_NOT_AT_SPEED),
            "recovery loop " + i + " reason: " + reason);
        sawRecoveryStabilizing |= reason.equals(ShotReadiness.REASON_STABILIZING);
      }
    }
    assertTrue(resumed, "auto-resumed");
    assertTrue(sawRecoveryStabilizing, "the recovery debounce phase shows Stabilizing");

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

    // After the back-off the volley must re-earn a fresh stable window: never allowed while
    // PAUSED, allowed again on the resume loop. The CLEARING_JAM heartbeat stays fresh for its
    // 0.15 s expiry into the pause, so the panel reason is the jam overlay first, then the
    // command's own waiting decision.
    assertEquals("PAUSED", stateOf(s));
    assertFalse(OperatorStatus.feedAllowed(), "no feeding right after the back-off");
    resumed = false;
    for (int i = 0; i < 15 && !resumed; i++) {
      stepWithPanel(s);
      // The volley resumes into whichever feeding state the jam interrupted (SHOOT or RETRACT).
      String st = stateOf(s);
      if (st.equals("SHOOT") || st.equals("RETRACT")) {
        resumed = true;
        assertTrue(OperatorStatus.feedAllowed(), "panel allows on the post-unjam resume loop");
      } else {
        assertEquals("PAUSED", st);
        assertFalse(OperatorStatus.feedAllowed(), "post-unjam debounce loop " + i);
        String reason = OperatorStatus.feedBlockedReason();
        assertTrue(
            reason.equals(ShotReadiness.REASON_CLEARING_JAM)
                || reason.equals(ShotReadiness.REASON_STABILIZING),
            "post-unjam loop " + i + " reason: " + reason);
      }
    }
    assertTrue(resumed, "resumed after the post-unjam stable window");

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
    assertTrue(stepUntilFeeding(s, 15), "resumed feeding after the recovery window");

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
    assertTrue(stepUntilFeeding(s, 15), "resumed feeding after the recovery window");

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
    assertTrue(stepUntilFeeding(s, 15), "feeding recovered after the post-unjam window");

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
    assertTrue(stepUntilFeeding(s, 25), "feeding resumes after the spin-up and stable window");

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
      assertTrue(
          stepUntilFeeding(s, 15), "resumed feeding after the recovery window, cycle " + cycle);
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
    // Spin-up loops are not feeding: the command's jam debouncer gets a clear sample each loop,
    // and the shadow must stay anchored with it or its 0.2 s window would free-run early.
    boolean feeding = false;
    for (int i = 0; i < 30 && !feeding; i++) {
      shadowJam.calculate(false);
      step(s);
      feeding = isFeeding();
    }
    assertTrue(feeding, "reached SHOOT with the shadow debouncer kept in sync");

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
      assertTrue(stepUntilFeeding(s, 15), "feeding recovered after jam " + jam);
    }
    s.end(true);
  }
}

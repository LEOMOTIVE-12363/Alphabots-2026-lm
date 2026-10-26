package frc.robot.commands.ShootingCommand;

import static frc.robot.util.DashboardControlsTestUtil.FLYWHEEL;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Drum.DrumConstants;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.DashboardControlsTestUtil;
import frc.robot.util.OperatorStatus;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Measurement faults with the target left unchanged: the drum sags below its setpoint and the hood
 * drifts off its target. ShootingTest covers target-side edits and the 0% limit; these tests prove
 * that a measured drum fault pauses feeding on the very loop it appears (the flywheel-speed gate)
 * while a measured hood fault never does, and that recovery auto-resumes the volley.
 *
 * <p>Injection semantics: the sim mechanism IO is instant-setpoint (the measured state snaps to the
 * setpoint every periodic), so an injected value only exists for the execute() it precedes and is
 * cleared by the next periodic. Sustained faults are re-injected every loop; recovery happens on
 * the first loop without injection. Each fault is checked against the readiness tolerance before
 * execute so a broken injection cannot produce a vacuous pass. The sim still cannot model real
 * flywheel spin-down/spin-up dynamics.
 */
class ShootingMeasurementFaultTest {
  private static final double EPS = 1e-6;
  /** Well outside both tolerances: drum 1.0 rot/s, hood 0.02 rot. */
  private static final double DRUM_SAG_ROTPS = DrumConstants.MaxShotVelocity - 7.0;

  private static final double HOOD_DRIFT_ROT = 0.1;

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

  private static double feederSetpoint() {
    return Feeder.mInstance.getVelocitySetpointRotPerSec();
  }

  private static double indexerSetpoint() {
    return Indexer.mInstance.getVelocitySetpointRotPerSec();
  }

  private static boolean isFeeding() {
    return Math.abs(feederSetpoint() - ShootingConstants.FeederRotpsTunable.getAsDouble()) < EPS;
  }

  private static boolean isCurrentFeeding() {
    return Feeder.mInstance.getControlMode() == MotorIO.ControlMode.CURRENT
        && Math.abs(Feeder.mInstance.getCurrentSetpointAmps() - 50.0) < EPS;
  }

  private static boolean stepUntilFeeding(Shooting shooting, int maxLoops) {
    for (int i = 0; i < maxLoops; i++) {
      step(shooting);
      if (isFeeding()) {
        return true;
      }
    }
    return false;
  }

  private static boolean stepUntilCurrentFeeding(Shooting shooting, int maxLoops) {
    for (int i = 0; i < maxLoops; i++) {
      step(shooting);
      if (isCurrentFeeding()) {
        return true;
      }
    }
    return false;
  }

  private static String stateOf(Shooting s) throws Exception {
    Field f = Shooting.class.getDeclaredField("state");
    f.setAccessible(true);
    return f.get(s).toString();
  }

  private static double retractElapsed(Shooting s) throws Exception {
    Field f = Shooting.class.getDeclaredField("retractTimer");
    f.setAccessible(true);
    return ((Timer) f.get(s)).get();
  }

  /**
   * Overwrites one field of a subsystem's logged inputs, mirroring what a real measurement fault
   * would look like at the only layer the control loop reads. Must run AFTER a periodic (which
   * refreshes inputs from the IO) and BEFORE the execute() that should see the fault.
   */
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
        // walk up to the declaring class
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

  @Test
  void measuredDrumSagPausesSameLoopAndAutoResumes() throws Exception {
    assertNull(Drive.mInstance, "test must run with no drivetrain / pose source");
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");

    double targetBefore = Drum.mInstance.getVelocitySetpointRotPerSec();
    injectDrumVelocity(DRUM_SAG_ROTPS);
    assertTrue(
        Math.abs(Drum.mInstance.getVelocityRotPerSec() - targetBefore)
            >= ShootingConstants.SHOOTER_VELOCITY_TOLERANCE_ROTPS,
        "the injected drum measurement is genuinely outside the readiness tolerance");
    step(s);
    assertEquals(0.0, feederSetpoint(), EPS, "drum sag stops the feeder on the detection loop");
    assertEquals(0.0, indexerSetpoint(), EPS, "indexer stops too");
    assertEquals(
        targetBefore,
        Drum.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "drum keeps being driven to the unchanged target");
    assertEquals("PAUSED", stateOf(s));

    // Sustained sag keeps the volley paused; the retract timer stays frozen for that stretch.
    double retractAtPause = retractElapsed(s);
    for (int i = 0; i < 10; i++) {
      injectDrumVelocity(DRUM_SAG_ROTPS);
      step(s);
      assertEquals(0.0, feederSetpoint(), EPS, "sustained sag keeps the volley paused, loop " + i);
    }
    assertEquals(retractAtPause, retractElapsed(s), EPS, "pause time never counts toward retract");

    // Recovery: the first loop without injection re-reads the on-target sim value and the volley
    // auto-resumes once the full stable window is re-earned.
    assertTrue(stepUntilFeeding(s, 15), "auto-resumed after the measurement recovered");
    assertTrue(retractElapsed(s) > retractAtPause, "feeding time counts toward retract again");
    s.end(true);
  }

  @Test
  void measuredHoodDriftDoesNotInterruptFeedOrShooterTargets() throws Exception {
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");

    double hoodTarget = Hood.mInstance.getPositionSetpointRot();
    injectHoodPosition(hoodTarget + HOOD_DRIFT_ROT);
    assertTrue(
        Math.abs(Hood.mInstance.getPositionRot() - hoodTarget)
            >= ShootingConstants.HOOD_ANGLE_TOLERANCE_ROT,
        "the injected hood measurement is genuinely outside the readiness tolerance");
    step(s);
    assertTrue(isFeeding(), "hood drift must not stop the feeder");
    assertEquals(ShootingConstants.IndexerRotpsTunable.getAsDouble(), indexerSetpoint(), EPS);
    assertEquals(
        hoodTarget,
        Hood.mInstance.getPositionSetpointRot(),
        EPS,
        "hood keeps being driven to the unchanged target");
    assertEquals(
        DrumConstants.MaxShotVelocity,
        Drum.mInstance.getVelocitySetpointRotPerSec(),
        EPS,
        "drum keeps being driven");

    for (int i = 0; i < 10; i++) {
      injectHoodPosition(hoodTarget + HOOD_DRIFT_ROT);
      step(s);
      assertTrue(isFeeding(), "sustained hood drift must not pause the volley, loop " + i);
      assertEquals(ShootingConstants.IndexerRotpsTunable.getAsDouble(), indexerSetpoint(), EPS);
    }
    assertEquals("RETRACT", stateOf(s), "hood drift must not freeze the retract timer");
    step(s);
    assertTrue(isFeeding(), "feed also continues after the measured hood recovers");
    s.end(true);
  }

  @Test
  void faultsInsideToleranceDoNotPauseTheVolley() throws Exception {
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");

    injectDrumVelocity(
        DrumConstants.MaxShotVelocity - 0.5 * ShootingConstants.SHOOTER_VELOCITY_TOLERANCE_ROTPS);
    step(s);
    assertTrue(isFeeding(), "a drum fault inside the tolerance must not pause");

    double hoodTarget = Hood.mInstance.getPositionSetpointRot();
    injectHoodPosition(hoodTarget + 0.5 * ShootingConstants.HOOD_ANGLE_TOLERANCE_ROT);
    step(s);
    assertTrue(isFeeding(), "a hood fault inside the tolerance must not pause");
    s.end(true);
  }

  @Test
  void measuredDrumSagWithdrawsCurrentFeedUntilRecovery() throws Exception {
    Shooting s = new Shooting(true);
    s.initialize();
    assertTrue(stepUntilCurrentFeeding(s, 30), "energy-save mode feeding at 50 A");
    assertEquals(MotorIO.ControlMode.CURRENT, Indexer.mInstance.getControlMode());

    double targetBefore = Drum.mInstance.getVelocitySetpointRotPerSec();
    injectDrumVelocity(DRUM_SAG_ROTPS);
    assertTrue(
        Math.abs(Drum.mInstance.getVelocityRotPerSec() - targetBefore)
            >= ShootingConstants.SHOOTER_VELOCITY_TOLERANCE_ROTPS,
        "the injected current-mode drum fault is outside the readiness tolerance");
    step(s);
    assertNotEquals(
        MotorIO.ControlMode.CURRENT,
        Feeder.mInstance.getControlMode(),
        "drum sag withdraws the feeder 50 A request on the detection loop");
    assertEquals(0.0, Feeder.mInstance.getCurrentSetpointAmps(), EPS, "no current request left");
    assertNotEquals(MotorIO.ControlMode.CURRENT, Indexer.mInstance.getControlMode());
    assertEquals(0.0, Indexer.mInstance.getCurrentSetpointAmps(), EPS);
    assertEquals(targetBefore, Drum.mInstance.getVelocitySetpointRotPerSec(), EPS);

    for (int i = 0; i < 10; i++) {
      injectDrumVelocity(DRUM_SAG_ROTPS);
      step(s);
      assertFalse(
          isCurrentFeeding(), "sustained sag keeps the current request withdrawn, loop " + i);
    }

    // Recovery restores the 50 A request once the stable window is re-earned.
    boolean resumed = false;
    for (int i = 0; i < 15 && !resumed; i++) {
      step(s);
      resumed = isCurrentFeeding();
    }
    assertTrue(resumed, "current feed restored after the measurement recovered");
    s.end(true);
  }

  @Test
  void pauseDuringRetractResumesIntoRetractAndKeepsIntakeRetracted() throws Exception {
    // A 0% limit pauses from RETRACT too: the pause keeps pulling the intake in, and a positive
    // limit returns to RETRACT once the spin-up and stable window complete.
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");
    for (int i = 0; i < 30 && !stateOf(s).equals("RETRACT"); i++) {
      step(s);
    }
    assertEquals("RETRACT", stateOf(s), "the 0.2 s retract delay elapsed");
    // The loop that flips the state to RETRACT still runs SHOOT's outputs (which never touch the
    // deploy setpoint); the retract position write lands one loop later.
    step(s);
    step(s);
    double retractCm = ShootingConstants.INTAKE_RETRACT_POSITION_CM.getAsDouble();
    assertEquals(
        retractCm, IntakeDeploy.mInstance.getPositionSetpointRot(), EPS, "intake retracting");

    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    step(s);
    assertEquals("PAUSED", stateOf(s));
    assertEquals(0.0, feederSetpoint(), EPS, "feed stopped on the detection loop");
    assertEquals(0.0, indexerSetpoint(), EPS);
    for (int i = 0; i < 10; i++) {
      step(s);
      assertEquals(
          retractCm,
          IntakeDeploy.mInstance.getPositionSetpointRot(),
          EPS,
          "paused RETRACT keeps the intake retracted, loop " + i);
    }

    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    boolean resumed = false;
    for (int i = 0; i < 15 && !resumed; i++) {
      step(s);
      resumed = isFeeding();
      if (!resumed) {
        assertEquals(
            retractCm,
            IntakeDeploy.mInstance.getPositionSetpointRot(),
            EPS,
            "still-paused RETRACT keeps the intake retracted, recovery loop " + i);
      }
    }
    assertTrue(resumed, "resumed feeding after the recovery window");
    assertEquals("RETRACT", stateOf(s), "resume returns to the remembered RETRACT, not SHOOT");
    s.end(true);
  }

  @Test
  void forceFeedTimeoutReleasesAPermanentlyBelowSpeedDrum() throws Exception {
    // The field failure: the drum can never enter the tolerance (pure-P loop standing error), so
    // the gate would hold forever. After 3 s of accumulated waiting the volley must force-feed.
    Shooting s = new Shooting(false);
    s.initialize();
    assertTrue(stepUntilFeeding(s, 30), "reached SHOOT");

    // Sustained below-speed measurement at 100%: the gate pauses and the wait accumulates
    // (~150 loops of 0.02 s); one extra loop margin on both sides of the boundary.
    for (int i = 0; i < 156; i++) {
      injectDrumVelocity(DRUM_SAG_ROTPS);
      step(s);
      if (i < 150) {
        assertEquals(0.0, feederSetpoint(), EPS, "gate holds before the timeout, loop " + i);
      }
    }
    assertTrue(isFeeding(), "force-feed after 3 s of accumulated waiting");

    // The latch stays closed: a continuing below-speed measurement no longer pauses the volley.
    for (int i = 0; i < 10; i++) {
      injectDrumVelocity(DRUM_SAG_ROTPS);
      step(s);
      assertTrue(isFeeding(), "latched force-feed ignores the speed gate, loop " + i);
    }

    // Release and re-press resets the latch: the same measurement pauses again.
    s.end(true);
    s.initialize();
    injectDrumVelocity(DRUM_SAG_ROTPS);
    step(s);
    assertEquals(0.0, feederSetpoint(), EPS, "fresh press re-arms the speed gate");
    s.end(true);
  }

  @Test
  void zeroFlywheelWaitNeverAccumulatesTowardForceFeed() throws Exception {
    // A 0% limit is the operator deliberately holding the drum stopped: no amount of 0% time may
    // count toward the force-feed timeout.
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    Shooting s = new Shooting(false);
    s.initialize();
    for (int i = 0; i < 200; i++) { // 4 s — beyond the 3 s timeout
      step(s);
      assertEquals(0.0, feederSetpoint(), EPS, "0% never force-feeds, loop " + i);
    }

    // Back at 100% with the drum held below speed, the wait starts from zero: still paused.
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    for (int i = 0; i < 10; i++) {
      injectDrumVelocity(DRUM_SAG_ROTPS);
      step(s);
      assertEquals(0.0, feederSetpoint(), EPS, "0% time did not count, loop " + i);
    }
    s.end(true);
  }

  @Test
  void timerAndDebouncerClocksAdvanceTogetherUnderSimHooks() {
    // Documents the test-clock relationship behind every timing assertion in the shooting tests:
    // Timer (RobotController.getTime) backs Timer.hasElapsed; MathSharedStore.getTimestamp backs
    // Debouncer. Both must advance with SimHooks.stepTiming or the debounce windows could not run
    // at all in-JVM. Any constant offset between them is fine (it shifts both anchors equally);
    // print it so the acceptance report can cite the measured value.
    double timer0 = Timer.getFPGATimestamp();
    double shared0 = MathSharedStore.getTimestamp();
    SimHooks.stepTiming(0.02);
    double timer1 = Timer.getFPGATimestamp();
    double shared1 = MathSharedStore.getTimestamp();
    assertEquals(0.02, timer1 - timer0, 1e-6, "Timer clock advanced by the step");
    assertEquals(0.02, shared1 - shared0, 1e-6, "Debouncer clock advanced by the same step");
    System.out.println(
        "[clock-diagnostic] Timer.getFPGATimestamp - MathSharedStore.getTimestamp = "
            + (timer1 - shared1)
            + " s");
  }
}

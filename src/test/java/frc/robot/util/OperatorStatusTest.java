package frc.robot.util;

import static frc.robot.util.DashboardControlsTestUtil.FLYWHEEL;
import static frc.robot.util.DashboardControlsTestUtil.HOOD;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Drum.DrumConstants;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodConstants;
import frc.robot.util.OperatorStatus.Action;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * OperatorStatus heartbeat and priority rules with an injected clock (same pattern as
 * BallFlowTrackerTest): expiry always falls back to IDLE — a released key, an interrupted command,
 * or a disable simply stops reporting. Active reversal/jam priorities are checked alongside the
 * rule that fresh post-jam/post-pause feeding supersedes stale blocked heartbeats. DriverStationSim
 * is explicitly reset to disabled after every test so enabled state cannot leak into other classes.
 */
class OperatorStatusTest {
  private final double[] now = {0.0};

  @BeforeAll
  static void setUpHal() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
  }

  @BeforeEach
  void injectClock() {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(() -> now[0]);
    // Explicit disabled baseline (isEnabled() is enabled && dsAttached; both default false).
    DriverStationSim.setEnabled(false);
    DriverStationSim.setDsAttached(false);
    DriverStationSim.notifyNewData();
  }

  @AfterEach
  void resetStatus() {
    OperatorStatus.resetForTest();
    OperatorStatus.setClockForTest(null);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setDsAttached(false);
    DriverStationSim.notifyNewData();
    DashboardControlsTestUtil.resetDefaults();
  }

  private void advanceTo(double t) {
    now[0] = t;
  }

  /** One robot loop for the sim mechanisms: advance time, then update mechanism inputs. */
  private static void stepMechanisms() {
    SimHooks.stepTiming(0.02);
    Drum.mInstance.periodic();
    Hood.mInstance.periodic();
  }

  @Test
  void noReportsIsIdle() {
    assertEquals(Action.IDLE, OperatorStatus.currentAction());
  }

  @Test
  void reportsExpireAfter150ms() {
    OperatorStatus.report(Action.FEEDING);
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());
    advanceTo(0.14);
    assertEquals(Action.FEEDING, OperatorStatus.currentAction(), "still within the expiry");
    advanceTo(0.16);
    assertEquals(Action.IDLE, OperatorStatus.currentAction(), "heartbeat expired");
  }

  @Test
  void jamOutranksFeedWhileClearingThenExpires() {
    OperatorStatus.report(Action.FEEDING);
    OperatorStatus.report(Action.CLEARING_JAM);
    assertEquals(Action.CLEARING_JAM, OperatorStatus.currentAction());

    advanceTo(0.10);
    assertEquals(Action.CLEARING_JAM, OperatorStatus.currentAction(), "jam still being handled");
    advanceTo(0.16);
    OperatorStatus.report(Action.FEEDING);
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());
  }

  @Test
  void highestPriorityFreshReportWins() {
    OperatorStatus.report(Action.INTAKING);
    OperatorStatus.report(Action.SPINNING_UP);
    OperatorStatus.report(Action.FEEDING);
    OperatorStatus.report(Action.WAITING_RECOVERY);
    assertEquals(Action.WAITING_RECOVERY, OperatorStatus.currentAction());
    OperatorStatus.report(Action.CLEARING_JAM);
    assertEquals(Action.CLEARING_JAM, OperatorStatus.currentAction());
    OperatorStatus.report(Action.REVERSING_LOCAL);
    assertEquals(Action.REVERSING_LOCAL, OperatorStatus.currentAction());
    OperatorStatus.report(Action.REVERSING_FULL);
    assertEquals(Action.REVERSING_FULL, OperatorStatus.currentAction());
  }

  @Test
  void disabledBlocksFeedRegardlessOfReports() {
    OperatorStatus.report(Action.FEEDING);
    OperatorStatus.reportFeedDecision(true, ShotReadiness.REASON_NONE);
    OperatorStatus.update();
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());
    assertEquals(ShotReadiness.REASON_DISABLED, OperatorStatus.feedBlockedReason());
    assertFalse(OperatorStatus.feedAllowed());
  }

  @Test
  void freshFeedingDecisionSupersedesStaleJamHeartbeat() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    OperatorStatus.report(Action.CLEARING_JAM);
    OperatorStatus.reportFeedDecision(true, ShotReadiness.REASON_NONE);
    assertEquals(Action.CLEARING_JAM, OperatorStatus.currentAction());
    assertEquals(ShotReadiness.REASON_CLEARING_JAM, OperatorStatus.feedBlockedReason());
    assertFalse(OperatorStatus.feedAllowed());

    // The command has finished UNJAM and resumed feeding; the old jam heartbeat is still inside
    // its 150 ms expiry but must not leave the panel blocked or reporting Clearing Jam.
    advanceTo(0.10);
    OperatorStatus.report(Action.FEEDING);
    OperatorStatus.reportFeedDecision(true, ShotReadiness.REASON_NONE);
    OperatorStatus.update();
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());
    assertTrue(OperatorStatus.feedAllowed());
    assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());
  }

  @Test
  void positiveFlywheelAllowsFallbackWhileMeasurementsAreNotReady() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    DashboardControlsTestUtil.set(HOOD, 100.0);
    Drum.mInstance.setVelocityRotPerSec(0.0);
    Hood.mInstance.setPositionRot(0.0);
    stepMechanisms();
    stepMechanisms();

    OperatorStatus.update();
    assertEquals(Action.IDLE, OperatorStatus.currentAction());
    assertTrue(
        OperatorStatus.feedAllowed(),
        "drum/hood measurements are diagnostics, not feed permission");
    assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());

    OperatorStatus.publish(0);
    assertTrue(
        NetworkTableInstance.getDefault()
            .getEntry("/SmartDashboard/Feed Allowed")
            .getBoolean(false));
    assertEquals(
        "",
        NetworkTableInstance.getDefault()
            .getEntry("/SmartDashboard/Feed Blocked Reason")
            .getString("missing"));
    assertEquals(
        "Idle",
        NetworkTableInstance.getDefault().getEntry("/SmartDashboard/Robot Action").getString(""));
  }

  @Test
  void zeroFlywheelBlocksAndPositiveRecoveryClearsDecision() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DashboardControlsTestUtil.set(FLYWHEEL, 0.0);
    OperatorStatus.report(Action.WAITING_RECOVERY);
    OperatorStatus.reportFeedDecision(false, ShotReadiness.REASON_FLYWHEEL_ZERO);
    OperatorStatus.update();
    assertFalse(OperatorStatus.feedAllowed());
    assertEquals(ShotReadiness.REASON_FLYWHEEL_ZERO, OperatorStatus.feedBlockedReason());

    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    advanceTo(0.02);
    OperatorStatus.report(Action.FEEDING);
    OperatorStatus.reportFeedDecision(true, ShotReadiness.REASON_NONE);
    OperatorStatus.update();
    assertEquals(Action.FEEDING, OperatorStatus.currentAction());
    assertTrue(OperatorStatus.feedAllowed());
    assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());
  }

  @Test
  void clearFeedDecisionRestoresFallbackImmediately() {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DashboardControlsTestUtil.set(FLYWHEEL, 100.0);
    DashboardControlsTestUtil.set(HOOD, 100.0);
    Drum.mInstance.setVelocityRotPerSec(DrumConstants.MaxShotVelocity);
    Hood.mInstance.setPositionRot(HoodConstants.MaxAngleDeg / 360.0);
    stepMechanisms();
    stepMechanisms();

    OperatorStatus.reportFeedDecision(false, ShotReadiness.REASON_FLYWHEEL_ZERO);
    assertFalse(OperatorStatus.feedAllowed());
    assertEquals(ShotReadiness.REASON_FLYWHEEL_ZERO, OperatorStatus.feedBlockedReason());

    OperatorStatus.clearFeedDecision();
    OperatorStatus.update();
    assertTrue(OperatorStatus.feedAllowed());
    assertEquals(ShotReadiness.REASON_NONE, OperatorStatus.feedBlockedReason());
  }

  @Test
  void publishWritesTheRobotActionKey() {
    OperatorStatus.report(Action.INTAKING);
    OperatorStatus.publish(0);
    assertEquals(
        "Intaking",
        NetworkTableInstance.getDefault().getEntry("/SmartDashboard/Robot Action").getString(""));
  }

  @Test
  void publishWaitingRecoveryUsesFlywheelLimitLabel() {
    OperatorStatus.report(Action.WAITING_RECOVERY);
    OperatorStatus.publish(0);
    assertEquals(
        "Waiting for Flywheel Limit",
        NetworkTableInstance.getDefault().getEntry("/SmartDashboard/Robot Action").getString(""));
  }
}

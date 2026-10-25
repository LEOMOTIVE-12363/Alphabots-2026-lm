package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Drum.DrumConstants;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Operator-dashboard status: which mechanism action is running, whether feeding is allowed, and the
 * human-readable reason when not.
 *
 * <p>The action comes from command heartbeats ({@link #report}); the feed decision comes from the
 * active shooting command itself ({@link #reportFeedDecision}) — the panel reports what the
 * mechanism is actually commanded to do, never guesses from buttons, and never recomputes a fixed
 * shot configuration while a command (e.g. Manual with its own speed/angle) is in charge. Both
 * expire {@value #REPORT_EXPIRY_SEC} seconds after the last heartbeat, so releasing a key, a
 * command interruption, or a disable returns the panel to Idle on its own — no stale status can
 * pin. Commands also call {@link #clearFeedDecision()} from {@code end()} so the panel drops a
 * finished command's decision immediately instead of waiting out the expiry.
 *
 * <p>When no command decision is fresh, the panel uses only {@code flywheelZero} from its own
 * {@link ShotReadiness} evaluation as the fallback permission input. Drum/hood/raw/stable readiness
 * remains available as diagnostics but does not change fallback feed permission or blocked reason.
 *
 * <p>The display publishes at 10 Hz from {@code Robot}'s dashboard-publish block via {@link
 * #publish} while control decisions ({@link #update}, {@link #feedAllowed}) run every loop. All
 * time comes from {@link Logger#getTimestamp()} (microseconds, replay-deterministic) unless a test
 * clock is injected via {@link #setClockForTest}; {@link #publish} writes nothing in replay.
 */
public final class OperatorStatus {
  /**
   * What a command reports while it runs. SPINNING_UP, FEEDING, WAITING_RECOVERY and CLEARING_JAM
   * are mutually exclusive shooter states: reporting one clears the other three timestamps so the
   * new state takes effect immediately. INTAKING and REVERSING_* retain their priority and expiry.
   */
  public enum Action {
    IDLE,
    INTAKING,
    SPINNING_UP,
    FEEDING,
    WAITING_RECOVERY,
    CLEARING_JAM,
    REVERSING_LOCAL,
    REVERSING_FULL
  }

  /** Seconds a report stays fresh after its last heartbeat. */
  private static final double REPORT_EXPIRY_SEC = 0.15;

  // Effective-action priority, highest first. Reversing overlays all other actions; the mutually
  // exclusive shooter states below cannot remain fresh together. IDLE is the no-reports fallback.
  private static final Action[] PRIORITY = {
    Action.REVERSING_FULL,
    Action.REVERSING_LOCAL,
    Action.CLEARING_JAM,
    Action.WAITING_RECOVERY,
    Action.FEEDING,
    Action.SPINNING_UP,
    Action.INTAKING
  };

  private static final Action[] SHOOTER_ACTIONS = {
    Action.SPINNING_UP, Action.FEEDING, Action.WAITING_RECOVERY, Action.CLEARING_JAM
  };

  // Last heartbeat timestamp (seconds) per Action, indexed by ordinal; NEGATIVE_INFINITY means
  // never reported, so boot and reset start from a clean IDLE with no special-casing.
  private static final double[] lastReportSec = new double[Action.values().length];

  private static DoubleSupplier clockForTest;

  // The dashboard's own diagnostic evaluator. Its flywheelZero field is the fallback permission
  // input when no shooting command reports a fresh decision; the other fields remain diagnostics.
  private static final ShotReadiness shotReadiness = new ShotReadiness();
  private static ShotReadiness.Evaluation latestEvaluation = notReadyPlaceholder();

  // The active shooting command's own feed decision (allowed + reason) and when it was last
  // reported; NEGATIVE_INFINITY means no command has reported (or the last one ended), so the
  // fallback evaluation takes over.
  private static boolean commandFeedAllowed = false;
  private static String commandFeedBlockedReason = ShotReadiness.REASON_NONE;
  private static double commandDecisionSec = Double.NEGATIVE_INFINITY;

  static {
    resetForTest();
  }

  private OperatorStatus() {}

  /**
   * Heartbeat: call every loop while the action holds; a report expires after {@value
   * #REPORT_EXPIRY_SEC} seconds. Reporting a shooter state clears the other shooter-state
   * heartbeats, but leaves INTAKING and REVERSING_* untouched.
   */
  public static void report(Action action) {
    switch (action) {
      case SPINNING_UP, FEEDING, WAITING_RECOVERY, CLEARING_JAM -> {
        for (Action shooterAction : SHOOTER_ACTIONS) {
          lastReportSec[shooterAction.ordinal()] = Double.NEGATIVE_INFINITY;
        }
      }
      default -> {}
    }
    lastReportSec[action.ordinal()] = nowSec();
  }

  /**
   * The active shooting command's own feed decision; call every loop from {@code execute()}. Like
   * an action report, it expires after {@value #REPORT_EXPIRY_SEC} seconds. This does not alter
   * action heartbeats: an allowed decision alone cannot clear an actively reported CLEARING_JAM.
   */
  public static void reportFeedDecision(boolean allowed, String blockedReason) {
    commandFeedAllowed = allowed;
    commandFeedBlockedReason = blockedReason;
    commandDecisionSec = nowSec();
  }

  /** Drops the command decision immediately; call from {@code Command.end()}. */
  public static void clearFeedDecision() {
    commandDecisionSec = Double.NEGATIVE_INFINITY;
  }

  private static boolean hasFreshCommandDecision() {
    return nowSec() - commandDecisionSec <= REPORT_EXPIRY_SEC;
  }

  /** Effective action: among fresh reports the highest priority wins; nothing fresh is IDLE. */
  public static Action currentAction() {
    double now = nowSec();
    for (Action action : PRIORITY) {
      if (now - lastReportSec[action.ordinal()] <= REPORT_EXPIRY_SEC) {
        return action;
      }
    }
    return Action.IDLE;
  }

  /**
   * Refreshes the fallback diagnostic evaluation against the dashboard shot configuration (full
   * shot speed, plus the Hood Angle (%) entry). Reads subsystem inputs and {@link
   * DashboardControls} only and touches no NetworkTables entries, so it is safe in every mode
   * including replay; call every robotPeriodic loop.
   */
  public static void update() {
    latestEvaluation =
        shotReadiness.update(
            DrumConstants.MaxShotVelocity, DashboardControls.getHoodTargetDeg() / 360.0);
  }

  /**
   * True while feeding is allowed: robot enabled, no jam clearing in progress, and the active
   * shooting command's own decision permitting. Without a fresh command decision, only a zero
   * flywheel limit blocks the fallback; drum/hood readiness does not.
   */
  public static boolean feedAllowed() {
    if (DriverStation.isDisabled() || currentAction() == Action.CLEARING_JAM) {
      return false;
    }
    if (hasFreshCommandDecision()) {
      return commandFeedAllowed;
    }
    return !latestEvaluation.flywheelZero();
  }

  /** Why feeding is blocked right now, or "" when it is allowed. */
  public static String feedBlockedReason() {
    if (DriverStation.isDisabled()) {
      return ShotReadiness.REASON_DISABLED;
    }
    if (currentAction() == Action.CLEARING_JAM) {
      return ShotReadiness.REASON_CLEARING_JAM;
    }
    if (hasFreshCommandDecision()) {
      return commandFeedBlockedReason;
    }
    return latestEvaluation.flywheelZero()
        ? ShotReadiness.REASON_FLYWHEEL_ZERO
        : ShotReadiness.REASON_NONE;
  }

  /**
   * Writes the operator panel entries to Elastic; call at 10 Hz. The key names are an external
   * contract consumed by elastic-layout-1022.json. No-op in replay.
   */
  public static void publish(int controllerPort) {
    if (Logger.hasReplaySource()) {
      return;
    }
    SmartDashboard.putString("Robot Action", displayName(currentAction()));
    SmartDashboard.putBoolean("Feed Allowed", feedAllowed());
    SmartDashboard.putString("Feed Blocked Reason", feedBlockedReason());
    SmartDashboard.putNumber(
        "Drum Target Speed (rot/s)", Drum.mInstance.getVelocitySetpointRotPerSec());
    SmartDashboard.putNumber("Drum Actual Speed (rot/s)", Drum.mInstance.getVelocityRotPerSec());
    SmartDashboard.putNumber("Battery Voltage (V)", RobotController.getBatteryVoltage());
    SmartDashboard.putBoolean(
        "Controller " + controllerPort + " Connected",
        DriverStation.isJoystickConnected(controllerPort));
  }

  /**
   * Clears every remembered heartbeat and command decision and restarts the diagnostic debounce;
   * restore an injected clock separately (null argument).
   */
  public static void resetForTest() {
    Arrays.fill(lastReportSec, Double.NEGATIVE_INFINITY);
    commandDecisionSec = Double.NEGATIVE_INFINITY;
    commandFeedAllowed = false;
    commandFeedBlockedReason = ShotReadiness.REASON_NONE;
    shotReadiness.reset();
    latestEvaluation = notReadyPlaceholder();
  }

  /** Test hook: replace the clock (seconds). Pass null to restore Logger-based time. */
  public static void setClockForTest(DoubleSupplier secondsSupplierOrNull) {
    clockForTest = secondsSupplierOrNull;
  }

  private static ShotReadiness.Evaluation notReadyPlaceholder() {
    return new ShotReadiness.Evaluation(
        false, false, false, false, false, ShotReadiness.REASON_DRUM_NOT_AT_SPEED);
  }

  private static String displayName(Action action) {
    return switch (action) {
      case IDLE -> "Idle";
      case INTAKING -> "Intaking";
      case SPINNING_UP -> "Spinning Up";
      case FEEDING -> "Feeding";
      case WAITING_RECOVERY -> "Waiting for Flywheel Limit";
      case CLEARING_JAM -> "Clearing Jam";
      case REVERSING_LOCAL -> "Reversing (Feeder/Indexer)";
      case REVERSING_FULL -> "Full Reverse (Eject)";
    };
  }

  private static double nowSec() {
    return clockForTest != null ? clockForTest.getAsDouble() : Logger.getTimestamp() / 1e6;
  }
}

package frc.robot.util;

import java.util.Arrays;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Heartbeat tracker for read-only semantic mechanism diagnostics consumed by the LED subsystem and
 * logs. Commands report what the mechanism is doing every loop while it holds ({@link #report});
 * consumers read the effective diagnostic state ({@link #effectiveState}) instead of watching
 * joystick buttons, so diagnostics follow the mechanism no matter which binding or command drives
 * it. This state is never an input to LED rendering.
 *
 * <p>Reports expire {@value #REPORT_EXPIRY_SEC} seconds after the last heartbeat, which makes
 * end/cancel/preemption, disable, and re-enable handling implicit: the moment a command stops
 * running it stops reporting, and the tracker falls back to IDLE on its own — no stale flag can pin
 * the diagnostic state forever.
 *
 * <p>Conflicting simultaneous reports are reported conservatively: a full-chain reverse fighting a
 * fresh feed/intake report, and any local (feeder/indexer-only) reverse, both produce IDLE with a
 * diagnostic note rather than a clean direction. The diagnostic state describes "the mechanism is
 * running this way", so it must never claim a direction the mechanism does not wholly have. It is
 * read-only and changes neither mechanical command behavior nor LED pixels.
 *
 * <p>All time comes from {@link Logger#getTimestamp()} (microseconds, replay-deterministic) unless
 * a test clock is injected via {@link #setClockForTest}. The tracker touches no NetworkTables
 * entries and no files, so it is safe in every mode including replay.
 */
public final class BallFlowTracker {
  /** What a command reports while it runs: which part of the ball path is moving, and which way. */
  public enum FlowKind {
    INTAKE,
    FEED,
    FULL_REVERSE,
    LOCAL_REVERSE
  }

  /**
   * The diagnostic direction reported by the LED subsystem; never used to choose rendered pixels.
   */
  public enum FlowState {
    IDLE,
    FORWARD,
    REVERSE
  }

  /** Seconds a report stays fresh after its last heartbeat. */
  private static final double REPORT_EXPIRY_SEC = 0.15;

  private static final String DIAGNOSTIC_CONFLICT = "conflict: reverse vs feed/intake";
  private static final String DIAGNOSTIC_LOCAL_REVERSE = "local reverse only";

  // Last heartbeat timestamp (seconds) per FlowKind, indexed by ordinal; NEGATIVE_INFINITY means
  // never reported, so boot and reset start from a clean IDLE with no special-casing.
  private static final double[] lastReportSec = new double[FlowKind.values().length];

  static {
    resetForTest();
  }

  private static DoubleSupplier clockForTest;

  private BallFlowTracker() {}

  /**
   * Heartbeat: call every loop while the condition holds; a report expires after {@value
   * #REPORT_EXPIRY_SEC} seconds.
   */
  public static void report(FlowKind kind) {
    lastReportSec[kind.ordinal()] = nowSec();
  }

  /**
   * Effective diagnostic state after the priority and conflict rules: fresh FULL_REVERSE together
   * with fresh FEED or INTAKE is a conflict (IDLE); fresh FULL_REVERSE alone is REVERSE; fresh FEED
   * or INTAKE is FORWARD; any fresh LOCAL_REVERSE stays IDLE (local motion only, never a whole-path
   * direction); no fresh reports is IDLE.
   */
  public static FlowState effectiveState() {
    return evaluate().state();
  }

  /**
   * Short human-readable note of a conflict or local reverse, "" when the state is clean; for logs
   * and Elastic.
   */
  public static String diagnostic() {
    return evaluate().diagnostic();
  }

  /** Clears every remembered heartbeat; restore an injected clock separately (null argument). */
  public static void resetForTest() {
    Arrays.fill(lastReportSec, Double.NEGATIVE_INFINITY);
  }

  /** Test hook: replace the clock (seconds). Pass null to restore Logger-based time. */
  public static void setClockForTest(DoubleSupplier secondsSupplierOrNull) {
    clockForTest = secondsSupplierOrNull;
  }

  private static double nowSec() {
    return clockForTest != null ? clockForTest.getAsDouble() : Logger.getTimestamp() / 1e6;
  }

  private static boolean isFresh(FlowKind kind, double nowSec) {
    return nowSec - lastReportSec[kind.ordinal()] <= REPORT_EXPIRY_SEC;
  }

  private record Evaluation(FlowState state, String diagnostic) {}

  private static Evaluation evaluate() {
    double now = nowSec();
    boolean intake = isFresh(FlowKind.INTAKE, now);
    boolean feed = isFresh(FlowKind.FEED, now);
    boolean fullReverse = isFresh(FlowKind.FULL_REVERSE, now);
    boolean localReverse = isFresh(FlowKind.LOCAL_REVERSE, now);

    // A clean direction is claimed only when every fresh report agrees on it; anything mixed is
    // shown as IDLE with a note instead of a direction the mechanism does not wholly have.
    if (fullReverse && (feed || intake)) {
      return new Evaluation(FlowState.IDLE, DIAGNOSTIC_CONFLICT);
    }
    if (localReverse) {
      // UNJAM reverses only the feeder/indexer while the intake stays stopped, so it is never a
      // whole-path reverse; combined with a full-reverse report the sources contradict each other.
      return new Evaluation(
          FlowState.IDLE, fullReverse ? DIAGNOSTIC_CONFLICT : DIAGNOSTIC_LOCAL_REVERSE);
    }
    if (fullReverse) {
      return new Evaluation(FlowState.REVERSE, "");
    }
    if (feed || intake) {
      return new Evaluation(FlowState.FORWARD, "");
    }
    return new Evaluation(FlowState.IDLE, "");
  }
}

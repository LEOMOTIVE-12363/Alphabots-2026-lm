package frc.robot.util;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.commands.ShootingCommand.ShootingConstants;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Hood.Hood;

/**
 * Measurement diagnostics for shot mechanism readiness. {@code drumReady} and {@code hoodReady}
 * compare actual mechanism inputs with their targets, {@code rawReady} is their instantaneous
 * conjunction, and {@code stableReady} is the debounced diagnostic. None of those four values is a
 * feed-permission decision: they must not delay or pause feeding. {@code flywheelZero} remains a
 * permission input alongside the disabled and clearing-jam overlays in {@link OperatorStatus}.
 *
 * <p>All tolerances and timings are the existing {@link ShootingConstants} values. An instance
 * reads subsystem inputs and {@link DashboardControls} only and touches no NetworkTables entries,
 * so it is safe in every mode including replay. Call {@link #reset()} for a fresh diagnostic
 * debounce baseline.
 */
public final class ShotReadiness {
  public static final String REASON_DISABLED = "Robot disabled";
  public static final String REASON_FLYWHEEL_ZERO = "Flywheel limit is 0%";
  public static final String REASON_DRUM_NOT_AT_SPEED = "Flywheel not at speed";
  public static final String REASON_HOOD_NOT_AT_ANGLE = "Hood not at angle";
  public static final String REASON_CLEARING_JAM = "Clearing indexer jam";
  public static final String REASON_NONE = "";

  /**
   * One diagnostic evaluation. {@code blockReason} describes the first unmet measurement condition
   * in priority order, or {@link #REASON_NONE} when raw-ready; it is not necessarily the operator
   * panel's actual feed-blocked reason. Disabled and clearing-jam overlays are applied by callers.
   */
  public record Evaluation(
      boolean flywheelZero,
      boolean drumReady,
      boolean hoodReady,
      boolean rawReady,
      boolean stableReady,
      String blockReason) {}

  // Requires raw readiness to hold continuously before stableReady turns true; recreated by
  // reset() because a stale debouncer that last saw "ready" would re-pass instantly on the first
  // sample instead of requiring the full debounce time again.
  private Debouncer readyDebouncer;

  public ShotReadiness() {
    reset();
  }

  /** Fresh diagnostic debounce baseline. */
  public void reset() {
    readyDebouncer = new Debouncer(ShootingConstants.READY_DEBOUNCE_SEC, DebounceType.kRising);
  }

  /**
   * Evaluates diagnostics against the given shot configuration: drum target in motor rot/s (the
   * Flywheel Speed Limit (%) is applied inside {@link Drum#isAtVelocity}), hood target in mechanism
   * rotations. The returned readiness values are measurements, not feed permission.
   */
  public Evaluation update(double drumTargetRotps, double hoodTargetRot) {
    boolean flywheelZero = DashboardControls.getFlywheelScale() <= 0.0;
    boolean drumReady =
        Drum.mInstance.isAtVelocity(
            drumTargetRotps, ShootingConstants.SHOOTER_VELOCITY_TOLERANCE_ROTPS);
    boolean hoodReady =
        Math.abs(Hood.mInstance.getPositionRot() - hoodTargetRot)
            < ShootingConstants.HOOD_ANGLE_TOLERANCE_ROT;
    boolean rawReady = drumReady && hoodReady;
    boolean stableReady = readyDebouncer.calculate(rawReady);
    String blockReason =
        flywheelZero
            ? REASON_FLYWHEEL_ZERO
            : !drumReady
                ? REASON_DRUM_NOT_AT_SPEED
                : !hoodReady ? REASON_HOOD_NOT_AT_ANGLE : REASON_NONE;
    return new Evaluation(flywheelZero, drumReady, hoodReady, rawReady, stableReady, blockReason);
  }
}

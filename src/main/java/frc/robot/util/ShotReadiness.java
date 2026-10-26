package frc.robot.util;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.commands.ShootingCommand.ShootingConstants;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Hood.Hood;

/**
 * Shot-readiness rule with the flywheel-speed feed gate. {@code drumReady} and {@code hoodReady}
 * compare actual mechanism inputs with their targets. {@code rawReady} is the instantaneous
 * flywheel-speed gate (drum only: the hood never permits or inhibits feeding), and {@code
 * stableReady} is its 0.15 s debounced form — Shooting and Manual feed only while the shot is
 * stable-ready, so a hopper full of balls cannot be pushed against a flywheel that is still
 * spinning up. {@code hoodReady} stays a logged diagnostic.
 *
 * <p>All tolerances and timings are the existing {@link ShootingConstants} values. An instance
 * reads subsystem inputs and {@link DashboardControls} only and touches no NetworkTables entries,
 * so it is safe in every mode including replay. Call {@link #reset()} for a fresh gate debounce
 * baseline.
 */
public final class ShotReadiness {
  public static final String REASON_DISABLED = "Robot disabled";
  public static final String REASON_FLYWHEEL_ZERO = "Flywheel limit is 0%";
  public static final String REASON_DRUM_NOT_AT_SPEED = "Flywheel not at speed";
  public static final String REASON_STABILIZING = "Stabilizing shot";
  public static final String REASON_CLEARING_JAM = "Clearing indexer jam";
  public static final String REASON_NONE = "";

  /**
   * One evaluation. {@code blockReason} describes the first unmet gate condition in priority order
   * (flywheel-zero, then drum speed), or {@link #REASON_NONE} when raw-ready; the hood never
   * contributes because it does not gate feeding. Disabled and clearing-jam overlays are applied by
   * callers.
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

  /** Fresh gate debounce baseline. */
  public void reset() {
    readyDebouncer = new Debouncer(ShootingConstants.READY_DEBOUNCE_SEC, DebounceType.kRising);
  }

  /**
   * Feeds the debounce a not-ready sample without reading the mechanism. Callers that deliberately
   * skip {@link #update} for a stretch (UNJAM's back-off) must call this each loop: a kRising
   * debouncer re-anchors only on baseline samples, so quiet time would otherwise let the timer
   * free-run and the first ready sample after the gap would pass instantly.
   */
  public void holdNotReady() {
    readyDebouncer.calculate(false);
  }

  /**
   * Evaluates the gate against the given shot configuration: drum target in motor rot/s (the
   * Flywheel Speed Limit (%) is applied inside {@link Drum#isAtVelocity}), hood target in mechanism
   * rotations. {@code rawReady}/{@code stableReady} are the feed-permission inputs; {@code
   * hoodReady} is a diagnostic only.
   */
  public Evaluation update(double drumTargetRotps, double hoodTargetRot) {
    boolean flywheelZero = DashboardControls.getFlywheelScale() <= 0.0;
    boolean drumReady =
        Drum.mInstance.isAtVelocity(
            drumTargetRotps, ShootingConstants.SHOOTER_VELOCITY_TOLERANCE_ROTPS);
    boolean hoodReady =
        Math.abs(Hood.mInstance.getPositionRot() - hoodTargetRot)
            < ShootingConstants.HOOD_ANGLE_TOLERANCE_ROT;
    boolean rawReady = drumReady;
    boolean stableReady = readyDebouncer.calculate(rawReady);
    String blockReason =
        flywheelZero ? REASON_FLYWHEEL_ZERO : !drumReady ? REASON_DRUM_NOT_AT_SPEED : REASON_NONE;
    return new Evaluation(flywheelZero, drumReady, hoodReady, rawReady, stableReady, blockReason);
  }
}

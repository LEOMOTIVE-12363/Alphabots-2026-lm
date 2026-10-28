package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Bounded startup steer-alignment gate for {@code Drive.runVelocity}.
 *
 * <p>When a translation command starts from measured rest (and the caller allows it — manual
 * rotation must never be delayed), the module drive output is held at zero until every steer angle
 * is within {@link #ALIGN_TOLERANCE_RAD} of its setpoint, so a full-speed start cannot drive the
 * chassis while the wheels still point somewhere else. The hold releases as soon as all four
 * modules are aligned, or unconditionally after {@link #ALIGN_TIMEOUT_SEC} so a failed steer motor
 * cannot wedge the drivetrain. While the translation command stays active the gate never re-arms —
 * a direction change at speed, even through a one-loop zero command, is handled by the ordinary
 * cosine-scaled steering with no added wait. Rotation-only commands never arm the gate, and {@link
 * #reset()} (called while disabled) drops all state so the next enable rebuilds it from the
 * actually measured rest state.
 */
public class StartupAlignGate {

  /** Maximum steer error (rad, drive-reversal aware) for a module to count as aligned. */
  public static final double ALIGN_TOLERANCE_RAD = Math.PI / 36.0; // 5 degrees

  /** Maximum time (s) the drive output may be held while waiting for steering. */
  public static final double ALIGN_TIMEOUT_SEC = 1.0;

  /**
   * Measured linear chassis speed (m/s) below which the robot counts as at rest for arming. A
   * direction-change slam enters its zero-command loop at full speed (orders of magnitude above
   * this), while a braked stop settles well below it.
   */
  public static final double REST_LINEAR_SPEED_MPS = 0.2;

  /** Measured angular chassis speed (rad/s) below which the robot counts as at rest. */
  public static final double REST_ANGULAR_SPEED_RADPS = 0.4;

  private boolean armed = false;
  private boolean commandedLastLoop = false;
  private double holdStartSec = 0.0;

  /**
   * Returns whether the module drive output should be held at zero this loop.
   *
   * @param timestampSec current timestamp (s)
   * @param translationCommanded whether this loop's commanded chassis speed has a translation
   *     component
   * @param allowStartupAlign false while the driver commands manual rotation (or any caller that
   *     must not be delayed)
   * @param measuredAtRest whether the measured chassis speeds are below the rest thresholds — a
   *     translation edge while the robot is still rolling (a mid-drive direction change through a
   *     one-loop zero command) must not arm
   * @param allAligned whether every steer angle is within tolerance of its setpoint
   */
  public boolean shouldHoldDrive(
      double timestampSec,
      boolean translationCommanded,
      boolean allowStartupAlign,
      boolean measuredAtRest,
      boolean allAligned) {
    if (!translationCommanded || !allowStartupAlign) {
      armed = false;
      commandedLastLoop = false;
      return false;
    }
    if (!commandedLastLoop) {
      // A genuine start from rest arms; a translation edge while still rolling never does.
      armed = measuredAtRest;
      holdStartSec = timestampSec;
    }
    commandedLastLoop = true;
    if (!armed) {
      return false;
    }
    if (allAligned || timestampSec - holdStartSec >= ALIGN_TIMEOUT_SEC) {
      armed = false;
      return false;
    }
    return true;
  }

  /** Drops all state (called while disabled so the next enable starts from a clean slate). */
  public void reset() {
    armed = false;
    commandedLastLoop = false;
  }

  /**
   * Steer error between a setpoint and a measured angle, aware that {@code Module.runSetpoint}
   * optimizes by reversing the drive direction: an error beyond 90 degrees is served by the
   * mirrored angle, so the effective error is the distance to the nearer of the two (an exactly
   * opposite wheel counts as aligned, driven backwards).
   */
  public static double optimizedSteerErrorRad(Rotation2d setpoint, Rotation2d measured) {
    // Rotation2d.minus already wraps the difference into [-pi, pi).
    double error = Math.abs(setpoint.minus(measured).getRadians());
    if (error > Math.PI / 2.0) {
      error = Math.PI - error;
    }
    return error;
  }
}

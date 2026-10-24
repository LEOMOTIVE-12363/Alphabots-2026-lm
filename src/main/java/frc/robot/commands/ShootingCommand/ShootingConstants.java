package frc.robot.commands.ShootingCommand;

import frc.robot.util.LoggedTunableNumber;

public final class ShootingConstants {
  // Feed speeds applied once we transition into the SHOOT state.
  public static final LoggedTunableNumber IndexerRotpsTunable =
      new LoggedTunableNumber("Shooting/IndexerRotps", 80);
  public static final LoggedTunableNumber IntakeRollerRotpsTunable =
      new LoggedTunableNumber("Shooting/IntakeRollerRotps", 40);
  public static final LoggedTunableNumber FeederRotpsTunable =
      new LoggedTunableNumber("Shooting/FeederRotps", 80);

  // Measurement-only readiness tolerances for diagnostics; they do not gate feeding.
  public static final double SHOOTER_VELOCITY_TOLERANCE_ROTPS = 1.;
  public static final double HOOD_ANGLE_TOLERANCE_ROT = 0.02;

  // Debounce window for the stableReady diagnostic only; it never delays or pauses feeding.
  public static final double READY_DEBOUNCE_SEC = 0.15;

  // Indexer jam detection during SHOOT/RETRACT: stator current continuously above this threshold
  // for the debounce time means a ball is stalled against the indexer (free-running feed draws far
  // less; the config's stator limit is 100 A so a true stall does reach 90). NOTE: only the normal
  // velocity-closed-loop feed can trip this — energy-save mode drives the indexer at 50 A torque
  // current, which caps stator current below the threshold.
  public static final double INDEXER_JAM_CURRENT_AMPS = 90.0;
  public static final double INDEXER_JAM_DEBOUNCE_SEC = 0.2;
  // Unjam response: run the indexer backwards at this speed for this long, then resume the volley.
  public static final LoggedTunableNumber UNJAM_INDEXER_Current =
      new LoggedTunableNumber("Shooting/UnjamIndexerCurrent", 50);
  public static final LoggedTunableNumber UNJAM_Feeder_Current =
      new LoggedTunableNumber("Shooting/UnjamFeederCurrent", 30);
  public static final LoggedTunableNumber UNJAM_DURATION_SEC =
      new LoggedTunableNumber("Shooting/UnjamDurationSec", 0.5);

  // Once SHOOT starts, wait this long (seconds) before retracting the intake.
  public static final LoggedTunableNumber RETRACT_DELAY_SEC =
      new LoggedTunableNumber("Shooting/retract_delay", 0.2);
  // Intake deploy position to retract to, in centimeters.
  public static final LoggedTunableNumber INTAKE_RETRACT_POSITION_CM =
      new LoggedTunableNumber("Shooting/retract", 35);

  // The hood mechanism zero is not horizontal: hood at 0 deg launches 78.2 deg above the
  // horizon, and increasing the hood angle flattens the shot.
  public static final double HOOD_ZERO_ELEVATION_DEG = 78.2;

  // The ball's exit point sits this far in front of robot center (measured 2026-07-08).
  public static final double EXIT_FORWARD_OFFSET_M = 0.17641;

  // Measured exit height used by maple-sim projectile launch.
  public static final double EXIT_HEIGHT_M = 0.4935;

  // Feeder wheel diameter (m), from the 120 mm feeder perimeter.
  public static final double FEEDER_WHEEL_DIAMETER_M = 0.120 / Math.PI;

  /** Feeder surface travel per motor rotation (m). */
  public static final double FEEDER_SURFACE_M_PER_MOTOR_ROT = Math.PI * FEEDER_WHEEL_DIAMETER_M;

  // Approximate spacing between balls in the feed path (m); one feeder revolution is a
  // reasonable first guess for sim launch cadence.
  public static final double FEED_BALL_SPACING_M = 0.120;

  // Maple-sim projectile: launch speed (m/s) = drum target rot/s × this constant.
  public static final LoggedTunableNumber SimLaunchSpeedPerDrumRotps =
      new LoggedTunableNumber("Shooting/SimLaunchSpeedPerDrumRotps", 0.12);

  // Maple-sim projectile: launch elevation (deg) = 90° − hood target (deg) − this offset.
  // Default offset (90 − 78.2 = 11.8) reproduces hood=0 → 78.2° elevation from measurement.
  public static final LoggedTunableNumber SimLaunchAngleOffsetDeg =
      new LoggedTunableNumber("Shooting/SimLaunchAngleOffsetDeg", 90.0 - HOOD_ZERO_ELEVATION_DEG);

  private ShootingConstants() {}
}

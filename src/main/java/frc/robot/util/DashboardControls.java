package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Hood.HoodConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Operator controls set from Elastic as percentages (0-100) — the drive train and shot settings
 * live under /SmartDashboard, the LED brightness under /Tuning.
 *
 * <p>Drive and Turn scale the chassis translational and rotational velocity independently inside
 * {@link frc.robot.subsystems.drive.Drive}. They are not applied during autonomous so auto paths
 * run at their tuned speeds.
 *
 * <p>Flywheel scales the drum velocity setpoint inside {@link frc.robot.subsystems.Drum.Drum} (so
 * the closed-loop velocity control is unchanged), and Hood Angle maps linearly onto the hood's
 * [{@link HoodConstants#MinAngleDeg}, {@link HoodConstants#MaxAngleDeg}] range. Together they are
 * the shot configuration used by {@link frc.robot.commands.ShootingCommand.Shooting}, in every
 * mode. LED Brightness scales the value channel of every {@link frc.robot.subsystems.LED.LED}
 * effect.
 *
 * <p>The entries are {@link LoggedNetworkNumber}s: the default is only set if the topic has no
 * value (so dashboard edits are never overwritten), and every value read is logged for replay.
 * Changed values are also persisted ({@link DashboardSettingsFile}) and restored on the next boot,
 * so settings survive a power cycle; the defaults below only apply until the first save, and to
 * keys with no saved value. Non-numeric Elastic settings (e.g. the LED pattern chooser) persist
 * through the same file via {@link #getSavedString} and {@link #persistStringIfChanged(String,
 * String)}.
 */
public final class DashboardControls {
  public static final double DEFAULT_SPEED_PERCENT = 100.0;
  public static final double DEFAULT_HOOD_PERCENT = 0.0;
  public static final double DEFAULT_BRIGHTNESS_PERCENT = 100.0;
  public static final double DEFAULT_FORCE_FEED_TIMEOUT_SEC = 3.0;
  static final double MIN_FORCE_FEED_TIMEOUT_SEC = 0.5;
  static final double MAX_FORCE_FEED_TIMEOUT_SEC = 10.0;

  static final String DRIVE_KEY = "/SmartDashboard/Drive Speed Limit (%)";
  static final String TURN_KEY = "/SmartDashboard/Turn Speed Limit (%)";
  static final String FLYWHEEL_KEY = "/SmartDashboard/Flywheel Speed Limit (%)";
  static final String HOOD_KEY = "/SmartDashboard/Hood Angle (%)";
  static final String LED_BRIGHTNESS_KEY = "/Tuning/LED Brightness (%)";
  static final String FORCE_FEED_TIMEOUT_KEY = "/SmartDashboard/Force Feed Timeout (s)";

  private static final LoggedNetworkNumber drivePercent =
      new LoggedNetworkNumber(DRIVE_KEY, DEFAULT_SPEED_PERCENT);
  private static final LoggedNetworkNumber turnPercent =
      new LoggedNetworkNumber(TURN_KEY, DEFAULT_SPEED_PERCENT);
  private static final LoggedNetworkNumber flywheelPercent =
      new LoggedNetworkNumber(FLYWHEEL_KEY, DEFAULT_SPEED_PERCENT);
  private static final LoggedNetworkNumber hoodPercent =
      new LoggedNetworkNumber(HOOD_KEY, DEFAULT_HOOD_PERCENT);
  private static final LoggedNetworkNumber ledBrightnessPercent =
      new LoggedNetworkNumber(LED_BRIGHTNESS_KEY, DEFAULT_BRIGHTNESS_PERCENT);
  private static final LoggedNetworkNumber forceFeedTimeoutSec =
      new LoggedNetworkNumber(FORCE_FEED_TIMEOUT_KEY, DEFAULT_FORCE_FEED_TIMEOUT_SEC);

  private static DashboardSettingsFile settingsFile;

  private DashboardControls() {}

  /**
   * Restores the values saved by the previous run, so Elastic settings survive a power cycle. Call
   * once at startup, before the controls are first read; keys with no saved value (or an unusable
   * saved file) keep their code defaults. Does nothing in replay, where values come from the log.
   */
  public static void applySavedSettings() {
    if (Logger.hasReplaySource()) {
      return;
    }
    DashboardSettingsFile saved = settingsFile();
    drivePercent.set(
        sanitizePercent(saved.getDouble(DRIVE_KEY, DEFAULT_SPEED_PERCENT), DEFAULT_SPEED_PERCENT));
    turnPercent.set(
        sanitizePercent(saved.getDouble(TURN_KEY, DEFAULT_SPEED_PERCENT), DEFAULT_SPEED_PERCENT));
    flywheelPercent.set(
        sanitizePercent(
            saved.getDouble(FLYWHEEL_KEY, DEFAULT_SPEED_PERCENT), DEFAULT_SPEED_PERCENT));
    hoodPercent.set(
        sanitizePercent(saved.getDouble(HOOD_KEY, DEFAULT_HOOD_PERCENT), DEFAULT_HOOD_PERCENT));
    ledBrightnessPercent.set(
        sanitizePercent(
            saved.getDouble(LED_BRIGHTNESS_KEY, DEFAULT_BRIGHTNESS_PERCENT),
            DEFAULT_BRIGHTNESS_PERCENT));
    forceFeedTimeoutSec.set(
        sanitizeForceFeedTimeoutSec(
            saved.getDouble(FORCE_FEED_TIMEOUT_KEY, DEFAULT_FORCE_FEED_TIMEOUT_SEC)));
  }

  /**
   * Persists changed control values (debounced, written off the real-time loop); call every loop.
   * Values are stored sanitized (clamped), matching what the robot actually applies.
   */
  public static void persistIfChanged() {
    if (Logger.hasReplaySource()) {
      return;
    }
    DashboardSettingsFile file = settingsFile();
    file.update(DRIVE_KEY, sanitizePercent(drivePercent.get(), DEFAULT_SPEED_PERCENT));
    file.update(TURN_KEY, sanitizePercent(turnPercent.get(), DEFAULT_SPEED_PERCENT));
    file.update(FLYWHEEL_KEY, sanitizePercent(flywheelPercent.get(), DEFAULT_SPEED_PERCENT));
    file.update(HOOD_KEY, sanitizePercent(hoodPercent.get(), DEFAULT_HOOD_PERCENT));
    file.update(
        LED_BRIGHTNESS_KEY,
        sanitizePercent(ledBrightnessPercent.get(), DEFAULT_BRIGHTNESS_PERCENT));
    file.update(FORCE_FEED_TIMEOUT_KEY, sanitizeForceFeedTimeoutSec(forceFeedTimeoutSec.get()));
  }

  private static DashboardSettingsFile settingsFile() {
    if (settingsFile == null) {
      settingsFile = DashboardSettingsFile.createDefault();
    }
    return settingsFile;
  }

  /**
   * Reads a saved non-numeric setting (e.g. the LED pattern name). Falls back when nothing valid is
   * saved; no-op in replay.
   */
  public static String getSavedString(String key, String fallback) {
    if (Logger.hasReplaySource()) {
      return fallback;
    }
    return settingsFile().getString(key, fallback);
  }

  /** Persists a changed non-numeric setting; call every loop (cheap no-op when unchanged). */
  public static void persistStringIfChanged(String key, String value) {
    if (Logger.hasReplaySource()) {
      return;
    }
    settingsFile().updateString(key, value);
  }

  /** Swaps the persistence backend for tests; pass null to restore the default. */
  static void setSettingsFileForTest(DashboardSettingsFile file) {
    settingsFile = file;
  }

  /**
   * Force-feed timeout in seconds: clamped to a sane range so a stray edit can never disable the
   * gate's fallback (0 would force-feed instantly) nor freeze the robot out for minutes. NaN/inf
   * fall back to the default.
   */
  public static double getForceFeedTimeoutSec() {
    return sanitizeForceFeedTimeoutSec(forceFeedTimeoutSec.get());
  }

  static double sanitizeForceFeedTimeoutSec(double sec) {
    if (!Double.isFinite(sec)) {
      return DEFAULT_FORCE_FEED_TIMEOUT_SEC;
    }
    return MathUtil.clamp(sec, MIN_FORCE_FEED_TIMEOUT_SEC, MAX_FORCE_FEED_TIMEOUT_SEC);
  }

  /** Clamps a dashboard percentage to [0, 100]; NaN/inf fall back to defaultPercent. */
  static double sanitizePercent(double percent, double defaultPercent) {
    if (!Double.isFinite(percent)) {
      percent = defaultPercent;
    }
    return MathUtil.clamp(percent, 0.0, 100.0);
  }

  /** Converts a dashboard speed percentage to a 0-1 scale: clamped, NaN/inf -> 100%. */
  static double percentToScale(double percent) {
    return sanitizePercent(percent, DEFAULT_SPEED_PERCENT) / 100.0;
  }

  /** Target Angle = Min + (Percentage / 100) * (Max - Min), with the percentage clamped. */
  static double hoodPercentToDeg(double percent) {
    double fraction = sanitizePercent(percent, DEFAULT_HOOD_PERCENT) / 100.0;
    return HoodConstants.MinAngleDeg
        + fraction * (HoodConstants.MaxAngleDeg - HoodConstants.MinAngleDeg);
  }

  /** Scales translation by driveScale and rotation by turnScale, independently. */
  static ChassisSpeeds scaleChassisSpeeds(
      ChassisSpeeds speeds, double driveScale, double turnScale) {
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond * driveScale,
        speeds.vyMetersPerSecond * driveScale,
        speeds.omegaRadiansPerSecond * turnScale);
  }

  private static double teleopScale(LoggedNetworkNumber percent) {
    return DriverStation.isAutonomous() ? 1.0 : percentToScale(percent.get());
  }

  /** Translational scale (0-1) currently applied to the drivetrain. */
  public static double getDriveScale() {
    return teleopScale(drivePercent);
  }

  /** Rotational scale (0-1) currently applied to the drivetrain. */
  public static double getTurnScale() {
    return teleopScale(turnPercent);
  }

  /** Scale (0-1) currently applied to the flywheel velocity setpoint. */
  public static double getFlywheelScale() {
    return percentToScale(flywheelPercent.get());
  }

  /** LED strip brightness percentage (0-100) currently applied by the LED subsystem. */
  public static double getLedBrightnessPercent() {
    return sanitizePercent(ledBrightnessPercent.get(), DEFAULT_BRIGHTNESS_PERCENT);
  }

  /** Hood target (mechanism degrees) from the Hood Angle (%) entry. */
  public static double getHoodTargetDeg() {
    return hoodPercentToDeg(hoodPercent.get());
  }

  /** Applies the Drive and Turn limits to a chassis speed command. */
  public static ChassisSpeeds applyDriveLimits(ChassisSpeeds speeds) {
    return scaleChassisSpeeds(speeds, getDriveScale(), getTurnScale());
  }

  /** Applies the Flywheel limit to a drum velocity setpoint. */
  public static double applyFlywheelLimit(double velocityRotPerSec) {
    return velocityRotPerSec * getFlywheelScale();
  }

  /** Nudges the Flywheel entry by deltaPercent (clamped); Elastic shows it on the next update. */
  public static void adjustFlywheelPercent(double deltaPercent) {
    flywheelPercent.set(
        sanitizePercent(flywheelPercent.get() + deltaPercent, DEFAULT_SPEED_PERCENT));
  }

  /** Logs the effective (clamped, currently applied) values for the dashboard and log review. */
  public static void logEffectiveValues() {
    Logger.recordOutput("DashboardControls/DriveEffectivePercent", getDriveScale() * 100.0);
    Logger.recordOutput("DashboardControls/TurnEffectivePercent", getTurnScale() * 100.0);
    Logger.recordOutput("DashboardControls/FlywheelEffectivePercent", getFlywheelScale() * 100.0);
    Logger.recordOutput("DashboardControls/HoodTargetDeg", getHoodTargetDeg());
  }
}

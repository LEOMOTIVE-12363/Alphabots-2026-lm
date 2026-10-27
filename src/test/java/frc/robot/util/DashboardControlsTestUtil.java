package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.reflect.Field;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Simulates Elastic edits: writes the NT entry, then refreshes the AdvantageKit input. */
public final class DashboardControlsTestUtil {
  private DashboardControlsTestUtil() {}

  public static final String DRIVE = "Drive Speed Limit (%)";
  public static final String TURN = "Turn Speed Limit (%)";
  public static final String FLYWHEEL = "Flywheel Speed Limit (%)";
  public static final String HOOD = "Hood Angle (%)";
  public static final String FORCE_FEED_TIMEOUT = "Force Feed Timeout (s)";

  /** Writes a value as Elastic would and makes it visible to the robot code. */
  public static void set(String label, double value) {
    setFullKey("/SmartDashboard/" + label, value);
  }

  /** Same as {@link #set(String, double)} for controls that do not live under /SmartDashboard. */
  public static void setFullKey(String fullKey, double value) {
    NetworkTableInstance.getDefault().getEntry(fullKey).setDouble(value);
    refresh();
  }

  /** Restores every entry to its default. */
  public static void resetDefaults() {
    set(DRIVE, DashboardControls.DEFAULT_SPEED_PERCENT);
    set(TURN, DashboardControls.DEFAULT_SPEED_PERCENT);
    set(FLYWHEEL, DashboardControls.DEFAULT_SPEED_PERCENT);
    set(HOOD, DashboardControls.DEFAULT_HOOD_PERCENT);
    setFullKey(DashboardControls.LED_BRIGHTNESS_KEY, DashboardControls.DEFAULT_BRIGHTNESS_PERCENT);
    setFullKey(
        DashboardControls.FORCE_FEED_TIMEOUT_KEY, DashboardControls.DEFAULT_FORCE_FEED_TIMEOUT_SEC);
  }

  /**
   * Runs each entry's periodic(), which Logger normally does every loop once started. Without the
   * Logger running this just reads the NT value.
   */
  public static void refresh() {
    try {
      for (String name :
          new String[] {
            "drivePercent",
            "turnPercent",
            "flywheelPercent",
            "hoodPercent",
            "ledBrightnessPercent",
            "forceFeedTimeoutSec"
          }) {
        Field field = DashboardControls.class.getDeclaredField(name);
        field.setAccessible(true);
        ((LoggedNetworkNumber) field.get(null)).periodic();
      }
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException(e);
    }
  }
}

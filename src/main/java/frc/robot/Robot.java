// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.rebuilt.ShiftUtil;
import frc.robot.simulation.FuelSimulation;
import frc.robot.simulation.MapleSimArena;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.util.DashboardControls;
import frc.robot.util.OperatorStatus;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private RobotContainer robotContainer;
  private int dashboardPublishCounter = 0;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Restore the Elastic dashboard settings saved by the previous run, so operator
    // percentages survive reboots and power cycles.
    DashboardControls.applySavedSettings();

    // Instantiate our RobotContainer. This will perform all our button bindings.
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Keep this disabled: at RT priority 99 any long loop (boot config flash, GC pause)
    // starves the NI NetComm and Phoenix threads — all CANivore signals go stale and the
    // Driver Station drops comms until the loop yields.
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Latched-intake edge polling runs after the scheduler so onlyWhile teardowns (priority loss,
    // disconnect, disable) land in the same loop they happen in; see RobotContainer's note for
    // why the latch cannot use a scheduler trigger edge.
    robotContainer.pollIntakeLatch();

    // Operator panel state (mechanism action + feed readiness) tracks the control decisions
    // every loop; only the SmartDashboard publish below is rate-limited to human frequency.
    OperatorStatus.update();

    // Dashboard data only needs human-display frequency. The RIO trace showed NetworkTables
    // serialization taking 20-40 ms, so publish these values at 10 Hz instead of every loop.
    if (++dashboardPublishCounter >= 5) {
      dashboardPublishCounter = 0;
      robotContainer.updateDashboardOutputs();
      // The operator status panel is also published here so it stays live no matter which
      // controller-binding scheme RobotContainer carries (the driver/demo controller sits on
      // DS port 1 in both).
      OperatorStatus.publish(1);
      ShiftUtil.publishShiftInfo();
    }

    // SHIFT tracking remains in the 50 Hz AdvantageKit log for replay review.
    ShiftUtil.ShiftInfo shiftInfo = ShiftUtil.getOfficialShiftInfo();
    Logger.recordOutput("Shift/CurrentShift", shiftInfo.currentShift());
    Logger.recordOutput("Shift/Active", shiftInfo.active());
    Logger.recordOutput("Shift/ElapsedTime", shiftInfo.elapsedTime());
    Logger.recordOutput("Shift/RemainingTime", shiftInfo.remainingTime());

    // Effective Elastic control values. Called every loop so the dashboard entries exist from boot.
    DashboardControls.logEffectiveValues();

    // Save changed Elastic settings so they survive the next power cycle. Debounced and written
    // on a background thread; a no-op when nothing changed.
    DashboardControls.persistIfChanged();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /**
   * This function is called once when the robot is disabled. Clear any standing drum request so
   * re-enabling can never spin the shooter back up on its own; the drum coasts out instead.
   */
  @Override
  public void disabledInit() {
    Drum.mInstance.stop();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** Demo robot: no autonomous programs remain, so autonomous is intentionally empty. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // Start the SHIFT timer at the beginning of teleop.
    ShiftUtil.initialize();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      MapleSimArena.getInstance().resetField();
    }
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      MapleSimArena.getInstance().simulationPeriodic();
      FuelSimulation fuelSimulation = FuelSimulation.getInstance();
      if (fuelSimulation != null) {
        fuelSimulation.simulationPeriodic();
      }
    }
  }
}
